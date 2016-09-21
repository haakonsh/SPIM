#ifndef APP_SPI_HANDLE_H__
#define APP_SPI_HANDLE_H__

#include "nrf_drv_spi.h"

#define BUF_SIZE(X)             sizeof(X) / sizeof(X[0])

#define SPI_DEFAULT_DELAY              1      /* 1 ms*/
#define SPI_NO_DELAY                       0      /* 0 ms*/

// choose accel / gyro module using the preprocessors
#if defined(MODULE_MPU_6000)
	#include "fw_mpu_6000.h"
#else
	//#error "Define Accel Gyro module"
#endif // MODULE_MPU_6000, etc

typedef enum{
		SPI_WRITE = 0,
		SPI_READ,
}FW_SPI_READ_WRITE_T; // SPI read/write type. This is used as an array index
/*
 * The first bit of the first byte contains the Read/Write bit and
 * indicates the Read (1) or Write (0) operation.
 */

typedef enum{
		SPI_EVENT_IGNORE = 0,
		SPI_EVENT_CHECK_RX,
		SPI_EVENT_COPY_RX,	
}FW_SPI_EVENT_T; // SPI event type after receiving RX buffer



void fw_spi_module_init(void);

void set_received_invalid_rx(bool b);

bool is_invalid_rx_received(void);

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
                                uint32_t delay_ms);

/**
 * @brief Init process before transfering SPI
 * Need to reset RX buffer and transfer done flag before sending buffers
 */
static void fw_spi_trasfer_init(void);


/**
 * @brief SPI user event handler.
 * @param event
 */
static void fw_spi_event_handler(nrf_drv_spi_evt_t const * p_event);

#endif // APP_SPI_HANDLE_H__
