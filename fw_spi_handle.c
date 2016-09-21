/* Copyright (c) 2016 COSCOI. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 */

#include <stdbool.h>
#include <string.h>

 #include "nrf_drv_spi.h"
 #include "app_error.h"
 #include "nrf_delay.h"
 #include "nrf_drv_spi.h"
 #include "nrf_log.h"
 #include "nrf_log_ctrl.h"
 #include "fw_mpu_6000.h"
 #include "fw_spi_handle.h"
 #include "main.h"

 #define SPI_INSTANCE                                    0                                                            /**< SPI instance index. */

static const   nrf_drv_spi_t      m_spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);         /**< SPI instance. */
static volatile bool                   m_spi_xfer_done;                                                            /**< Flag used to indicate that SPI instance completed the transfer. */

#define BUF_MAX_LENGTH                              8                                                          /**< SPI Buffer Max Length */

static uint8_t                          m_rx_buf[BUF_MAX_LENGTH];                                        /**< SPI RX buffer. */
static FW_SPI_EVENT_T        m_spi_event;                                                                    /**< Application SPI event*/
static uint8_t                          m_rx_cmp[2];                                                                  /**< Buffer to compare and check RX buffer*/
static uint8_t                          m_rx_copy[] = REG_ADDR_TO_READ;                             /**< Buffer to copy the RX value after reading module's data*/
// This array does not use the address value, it is just used for configuring the size in compile time

static bool received_invalid_rx; // true when invalid RX has arrived


void fw_spi_module_init(void){

		// SPI driver init
		/* data delivered MSB first as in MPU-6000 Product Specification
	   * latched on rising edge, transitioned on falling edge, active low - SPI mode 0
	   */
		nrf_drv_spi_config_t          spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin    = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;

		spi_config.frequency = (nrf_drv_spi_frequency_t)NRF_SPI_FREQ_1M;
    APP_ERROR_CHECK(nrf_drv_spi_init(&m_spi, &spi_config, fw_spi_event_handler));

		memset(m_rx_copy, 0x00, BUF_SIZE(m_rx_copy));

		fw_module_init();

		// Test
		/*for(uint8_t k=0; k< 8; k++){
				fw_spi_transfer(REG_WHO_AM_I, EXPECTED_VALUE_WHO_AM_I, SPI_READ,  SPI_EVENT_CHECK_RX,1000);
		}*/

}

void set_received_invalid_rx(bool b){
		received_invalid_rx = b;
}

bool is_invalid_rx_received(void){
		return received_invalid_rx;
}

/**
 * @brief SPI transfer (send data to read or write)
 * @param Module's register address
 * @param value to write, value to store in the TX buffer
 * @param Read/Write mode
 * @param Application SPI event type
 * @param amout of delay after transfering in ms
 */
void fw_spi_transfer(uint8_t reg_addr, uint8_t write_value,
	                              FW_SPI_READ_WRITE_T rw, FW_SPI_EVENT_T event,
                                uint32_t delay_ms){

		const uint8_t rw_byte[] = {0x00, 0x80}; // when reading, 0x80 has to be masked
		static uint8_t tx_buf[2];
		static const uint8_t length = BUF_SIZE(tx_buf);

		memset(tx_buf,0, length);
		set_received_invalid_rx(false); // init
		APP_ERROR_CHECK_BOOL(length % 2 == 0); // must be an even number

		fw_spi_trasfer_init();
		m_spi_event = event;

		tx_buf[0] = reg_addr;
		if(rw == SPI_WRITE){
				tx_buf[1] = write_value;
		}

		else if(rw == SPI_READ){

				if(event == SPI_EVENT_CHECK_RX){
						m_rx_cmp[1] = write_value;
				}

				for(uint8_t k=0; k<length; k += 2) {
						// mask registers, odd index (1, 3, 5...) are dummy packets
						tx_buf[k] |= rw_byte[rw];
				}
		}
		/*
		 * Ref. MPU-60X0 datasheet
		 * SPI read and write operations are completed in 16 or more clock cycles (two or more bytes).
		 * The first byte contains the SPI Address,
		 * and the following byte(s) contain(s) the SPI data.
		 * The first bit of the first byte contains the Read/Write bit and
		 * indicates the Read (1) or Write (0) operation.
		 */

		APP_ERROR_CHECK(nrf_drv_spi_transfer(&m_spi, tx_buf, length, m_rx_buf, length));

		while (!m_spi_xfer_done){
            // __WFE(); // Assembly Wait for event command
						power_manage();
		}

		nrf_delay_ms(delay_ms);
}



/**
 * @brief Init process before transfering SPI
 * Need to reset RX buffer and transfer done flag before sending buffers
 */
static void fw_spi_trasfer_init(void){

		memset(m_rx_cmp, 0, BUF_SIZE(m_rx_cmp));
		memset(m_rx_buf, 0, BUF_MAX_LENGTH);
		m_spi_xfer_done = false;
}


/**
 * @brief SPI user event handler.
 * @param event
 */
static void fw_spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    m_spi_xfer_done = true;
		if(m_spi_event == SPI_EVENT_IGNORE){
				return; // ignore this RX buffers
		}

    NRF_LOG_INFO("Transfer completed. Received: \r\n");
		NRF_LOG_HEXDUMP_INFO(m_rx_buf, 2);

		if(m_spi_event == SPI_EVENT_CHECK_RX){

				//int32_t result = memcmp(m_rx_buf, m_rx_cmp, BUF_SIZE(m_rx_cmp));
				if(m_rx_buf[1] != m_rx_cmp[1]){ // not equal to the expected value

						NRF_LOG_INFO("ERROR: Received invalid RX\r\n");
						set_received_invalid_rx(true);
				}
		}

		else if(m_spi_event == SPI_EVENT_COPY_RX){
				const uint8_t max_index = 1 + fw_get_data_reg_addr_array_length();
				static uint8_t index = 0;

				m_rx_copy[index] = m_rx_buf[1];

				switch(index){
						case 1: // accel x
						case 3: // accel y
						case 5: // accel z
						case 7: // gyro x
						case 9:  // gyro y
						case 11:  // gyro z
							fw_module_cal_accel_or_gyro_data(m_rx_copy[index-1], m_rx_copy[index], (index-1)/2, fw_get_module_data_addr());
							break;

						default:
							break;
				}

				index = (index + 1) % max_index;
		}

}
