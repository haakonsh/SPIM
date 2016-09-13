/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "boards.h"
#include "app_error.h"
#include <string.h>
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
#define SPIS_INSTANCE 1 /**< SPIS instance index. */
static const nrf_drv_spis_t spis = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);/**< SPIS instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
static uint8_t       s_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       s_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t s_length = sizeof(m_tx_buf);        /**< Transfer length. */

#define TEST_STRING "Nordic"
static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

nrf_drv_spi_config_t spi_config = {
    .sck_pin      = SPI_SCK_PIN,
    .mosi_pin     = SPI_MOSI_PIN,
    .miso_pin     = SPI_MISO_PIN,
    .ss_pin       = SPI_SS_PIN,
    .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc          = 0xFF,
    .frequency    = NRF_DRV_SPI_FREQ_1M,
    .mode         = NRF_DRV_SPI_MODE_0,
    .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
};

nrf_drv_spis_config_t spis_config = {
    .csn_pin      = APP_SPIS_CS_PIN;
    .miso_pin     = APP_SPIS_MISO_PIN;
    .mosi_pin     = APP_SPIS_MOSI_PIN;
    .sck_pin      = APP_SPIS_SCK_PIN;
    .miso_drive   = NRF_DRV_SPIS_DEFAULT_MISO_DRIVE,
    .csn_pullup   = NRF_DRV_SPIS_DEFAULT_CSN_PULLUP,
    .orc          = SPIS_DEFAULT_ORC,
    .def          = SPIS_DEFAULT_DEF,
    .mode         = (nrf_drv_spis_mode_t)SPIS_DEFAULT_MODE,
    .bit_order    = (nrf_drv_spis_endian_t)SPIS_DEFAULT_BIT_ORDER,
    .irq_priority = SPIS_DEFAULT_CONFIG_IRQ_PRIORITY,
};

/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    NRF_LOG_INFO("Transfer completed.\r\n");
    if (m_rx_buf[0] != 0)
    {
        NRF_LOG_INFO(" Received: \r\n");
        NRF_LOG_HEXDUMP_INFO(m_rx_buf, strlen((const char *)m_rx_buf));
    }
}

void spis_event_handler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        spis_xfer_done = true;
        NRF_LOG_INFO(" Transfer completed. Received: %s\r\n",(uint32_t)m_rx_buf);
    }
}

int main(void)
{
    // Enable the constant latency sub power mode to minimize the time it takes
    // for the SPIS peripheral to become active after the CSN line is asserted
    // (when the CPU is in sleep mode).
    NRF_POWER->TASKS_CONSTLAT = 1;

    spi_config.ss_pin   = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler));

    while (1)
    {
        // Reset rx buffer and transfer done flag
        memset(m_rx_buf, 0, m_length);
        spi_xfer_done = false;

        APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, m_tx_buf, m_length, m_rx_buf, m_length));

        while (!spi_xfer_done)
        {
            __WFE();
        }
    }
}
