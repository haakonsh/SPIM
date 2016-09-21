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
 
 #include <stdint.h>
 #include <stdio.h>
 #include <string.h>
 
 #include "nrf_delay.h"
 #include "fw_mpu_6000.h"
 #include "fw_spi_handle.h"
 
 #ifdef NRF_LOG_BACKEND_SERIAL_USES_RTT
#include <string.h> 
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#endif //NRF_LOG_BACKEND_SERIAL_USES_RTT
 
 static const uint8_t                   m_data_reg_addr_array[] = REG_ADDR_TO_READ;
 static module_cal_data_t         m_data;
  
 // ref. http://blog.daum.net/jeonggy/23
 
 /**
 * @brief Init the accel / gyro module
 */
 void fw_module_init(void){
		
		fw_module_reset();
	 
		// enable MPU6000's SPI interface (disable I2C), enable module's FIFO
		fw_spi_transfer(REG_USER_CTRL,
	                           USER_CTRL_SETTINGS,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
														 
		// Read and check USER_CTRL register
		fw_spi_transfer(REG_USER_CTRL,
	                           USER_CTRL_SETTINGS,
	                           SPI_READ, SPI_EVENT_CHECK_RX,
														 SPI_DEFAULT_DELAY);
	 
		
		// Configure Digital Low Pass Filter (DLPF) for both the gyroscopes and accelerometers
		// This also configures Gyro's data output rate (8kHz or 1kHz)
		fw_spi_transfer(REG_CONFIG,
	                           CONFIG_DLPF_CFG3,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
		
		// Set sample rate of the module
		fw_spi_transfer(REG_SMPLRT_DIV,
	                           FREQ_TO_REG_VAL(SAMPLING_FREQ_HZ),
														 // Sampling frequency is 100, corresponding value will be calculated
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
														 
		// Set Full Scale Range (sensitivity or scale) of gyro
		fw_spi_transfer(REG_GYRO_CONFIG,
	                           GYRO_CONFIG_FS_SEL0,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
														 
		// Set Full Scale Range (sensitivity or scale) of accel
		fw_spi_transfer(REG_ACCEL_CONFIG,
	                           ACCEL_CONFIG_FS_SEL0,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
		
		// Set FIFO
		fw_spi_transfer(REG_FIFO_EN,
	                           FIFO_EN_TEMP_FIFO_ENABLE | FIFO_EN_XG_FIFO_ENABLE |
														 FIFO_EN_YG_FIFO_ENABLE | FIFO_EN_ZG_FIFO_ENABLE |
														 FIFO_EN_ACCEL_FIFO_ENABLE,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
		
		//fw_enable_int();
		
		// reset MPU6000's signal path to use SPI interface
		fw_spi_transfer(REG_SIGNAL_PATH_RESET,
	                           SIGNAL_PATH_RESET_GYRO | SIGNAL_PATH_RESET_ACCEL | SIGNAL_PATH_RESET_TEMP,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 RESET_DELAY_SIGNAL_PATH_RESET);
		
		// check whether WHO AM I vaule is read normally
		fw_spi_transfer(REG_WHO_AM_I,
		                         EXPECTED_VALUE_WHO_AM_I,
														 SPI_READ, 
														 SPI_EVENT_CHECK_RX,
														 SPI_DEFAULT_DELAY);
		
		// init data for calibration values
		fw_module_data_init(fw_get_module_data_addr());
		
 }
 
 /**
 * @brief Reset the accel / gyro module
 */
void fw_module_reset(void){
		
		uint8_t reset_trial = 5; // try to reset the moudle n times
		bool is_invalid;
		
		while(--reset_trial != 0){
				NRF_LOG_INFO("Resetting module\r\n");
				
				// check whether WHO AM I vaule is read normally
				fw_spi_transfer(REG_WHO_AM_I,
																EXPECTED_VALUE_WHO_AM_I,
																SPI_READ, 
																SPI_EVENT_CHECK_RX,
																SPI_DEFAULT_DELAY);
	 
				// reset MPU6000 to use SPI interface
				fw_spi_transfer(REG_PWR_MGMT_1,
																PWR_MGMT_1_DEVICE_RESET,
																SPI_WRITE, SPI_EVENT_IGNORE,
																RESET_DELAY_PWR_MGMT_1);
				
				// Wake up device and select clock reference of the module - PLL with Z axis gyroscope
				fw_spi_transfer(REG_PWR_MGMT_1,
																PWR_MGMT_1_CLKSEL3,
																SPI_WRITE, SPI_EVENT_IGNORE,
																PWR_MGMT_1_CLK_SETTING_DELAY);
				
				// enable MPU6000's SPI interface (disable I2C), reset module's FIFO
				fw_spi_transfer(REG_USER_CTRL,
																USER_CTRL_I2C_IF_DIS | USER_CTRL_FIFO_RESET,
																SPI_WRITE, SPI_EVENT_IGNORE,
																SPI_DEFAULT_DELAY);
																
				// check written value, 
				fw_spi_transfer(REG_PWR_MGMT_1,
																PWR_MGMT_1_CLKSEL3,
																SPI_READ, SPI_EVENT_CHECK_RX,
																SPI_DEFAULT_DELAY);
																
				is_invalid = is_invalid_rx_received();
				if(!is_invalid){
						return;
				}
				nrf_delay_ms(50);
		}
		NRF_LOG_INFO("RESET FAILED!\r\n");
}
 
 /**
 * @brief Enable module's interrupt
 */
void fw_read_int_status(void){
		
		NRF_LOG_INFO("TX Read status reg: \r\n");
		// Read interrupt status register
		fw_spi_transfer(REG_INT_STATUS,
	                           INT_STATUS_DATA_READY,
	                           SPI_READ, SPI_EVENT_CHECK_RX,
														 SPI_DEFAULT_DELAY);
}
 
/**
 * @brief Enable module's interrupt
 */
void fw_enable_int(void){
		
		const uint8_t config_val = INT_PIN_CFG_LEVEL_ACTIVE_LOW | INT_PIN_PULSE_EMIT_ENABLE |
			                                      INT_PIN_INT_READ_CLEAR | INT_OPEN_PUSH_PULL;
		
		NRF_LOG_INFO("Enabling interrupt pin: \r\n");
		
		// Set module's interrupt pin configuration - active low, use 50us pulse
		// Interrupt status bits are cleared on any read operation.
		fw_spi_transfer(REG_INT_PIN_CFG,
	                           config_val,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
		
		// Enable module's interrupt (interrupt is created when data is ready)
		fw_spi_transfer(REG_INT_ENABLE,
	                           INT_ENABLE_DATA_READY,
	                           SPI_WRITE, SPI_EVENT_IGNORE,
														 SPI_DEFAULT_DELAY);
		
		// Check pin configuration register
		fw_spi_transfer(REG_INT_PIN_CFG,
	                           config_val,
	                           SPI_READ, SPI_EVENT_CHECK_RX,
														 SPI_DEFAULT_DELAY);
		
		// Check interrupt enable register
		fw_spi_transfer(REG_INT_ENABLE,
	                           INT_ENABLE_DATA_READY,
	                           SPI_READ, SPI_EVENT_CHECK_RX,
														 SPI_DEFAULT_DELAY);
														 
		fw_read_int_status();
}

/**
 * @brief Read module's accel, gyro, temperature value
 */
void fw_read_data(void){
		
		const uint8_t * p_reg_addr_array = fw_get_data_reg_addr_array();
		const uint8_t length = fw_get_data_reg_addr_array_length();
		
		for(uint8_t k=0; k < length; k++){
				// Read data
				fw_spi_transfer(p_reg_addr_array[k],
                                 0,
			                           SPI_READ, SPI_EVENT_COPY_RX,
			                           SPI_NO_DELAY);
		}
		
}

/**
 * @brief Get array which contains data register address
 */
const uint8_t * fw_get_data_reg_addr_array(void){
		return m_data_reg_addr_array;
}

/**
 * @brief Get the length of array which contains data register address
 */
uint8_t fw_get_data_reg_addr_array_length(void){
		return (uint8_t)BUF_SIZE(m_data_reg_addr_array);
}

/**
 * @brief Get the address of module's data which contains module's data
 */
module_cal_data_t * fw_get_module_data_addr(void){
		return &m_data;
}

/**
 * @brief Calibrate raw value to digital value
 * @param data high byte
 * @param data low byte
 * @param index (accel xyz or gyro xyz)
 * @param pointer (addresss of global variable)
 */
void fw_module_cal_accel_or_gyro_data(uint8_t byte_h, uint8_t byte_l, uint8_t index, module_cal_data_t * p_data){
		
		int32_t merge_byte = (int32_t) ((byte_h << 8) | byte_l);
		int32_t calibration;
		module_data_t * p_data_addr;
		
		switch(index){
				case 0: // accel x
				case 1: // accel y
				case 2: // accel z
					p_data_addr = &(p_data->a_x) + index;
					// In pointer arithmetic, adding 1 is adding size of data type to the address (which is 8)
					calibration = (int32_t) p_data->cal.sensitivity_accel;
					break;
				
				case 3: // gyro x
				case 4: // gyro y
				case 5: // gyro z
					p_data_addr = &(p_data->gyro_x) + (index - 3);
					calibration = (int32_t) p_data->cal.sensitivity_gyro;
					break;
				
				default:
					return;
		}
		
		(*p_data_addr) = (module_data_t) (merge_byte / calibration);
		
#ifdef NRF_LOG_BACKEND_SERIAL_USES_RTT
		uint8_t str[30] = {0,};
		uint8_t *axis[6] = {"ax", "ay", "az", "gx", "gy" , "gz"};
		sprintf(str, "%s %f\r\n", axis[index], (*p_data_addr));
		NRF_LOG_HEXDUMP_INFO(str, strlen(str));
		NRF_LOG_FLUSH();
#endif // NRF_LOG_BACKEND_SERIAL_USES_RTT
		
}



/**
 * @brief Init variable which contains data of the module
 * @param event
 */
static void fw_module_data_init(module_cal_data_t * p_data){
		
		memset(p_data, 0x00, sizeof(module_cal_data_t));
		
		// this will be changed by reading SPI RX
		p_data->cal.sensitivity_accel = 16384;
		p_data->cal.sensitivity_gyro = 131;
		
		p_data->cal.add_temp = 36.53;
		p_data->cal.denom_temp = 340;
}

