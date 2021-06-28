/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Bridge between I2C based IMU sensors and SPI since my LPC1768 currently only has SPI pins free.
#include "AdafruitPrecision9DoF.h"
#include "sdk_config.h"
#include "nrf_drv_spis.h"
#include "nrf_gpio.h"
#include "app_error.h"


// The pins used for communicating with the main LPC1768 board via SPI.
// The pins labelled SCK, MOSI, MISO on the Feather are used for that purpose.
#define SPIS_SCK_PIN    NRF_GPIO_PIN_MAP(0, 12)
#define SPIS_MISO_PIN   NRF_GPIO_PIN_MAP(0, 14)
#define SPIS_MOSI_PIN   NRF_GPIO_PIN_MAP(0, 13)
// Use A5 on the Feather as the chip select input.
#define SPIS_CS_PIN     NRF_GPIO_PIN_MAP(0, 29)

// The SPI Slave peripheral instance on the nRF52 to be used for communicating with the LPC1768.
#define SPIS_INSTANCE 1

// The pins used for communicating with the IMU sensors via I2C.
// The pins labelled SCL & SDA on the Feather are used for that purpose.
#define SENSOR_SDA_PIN     NRF_GPIO_PIN_MAP(0, 25)
#define SENSOR_SCL_PIN     NRF_GPIO_PIN_MAP(0, 26)
// Use the adjacent pin on the Feather as the interrupt pin.
#define SENSOR_INT_PIN      NRF_GPIO_PIN_MAP(0, 27)

// The rate at which the IMU sensors are to be sampled in Hz.
#define SENSOR_SAMPLE_RATE  200


// Global to indicate when the most recent packet has been transferred to the SPI master.
static volatile bool g_spiTransferDone = true;


// Forward Function Declarations
void spiSlaveEventHandler(nrf_drv_spis_event_t event);


int main(void)
{
    // Setup the SPI slave peripheral to which the latest sensor readings will be sent over to the main LPC1768 board.
    static const nrf_drv_spis_t spiSlave = NRF_DRV_SPIS_INSTANCE(SPIS_INSTANCE);
    nrf_drv_spis_config_t       spiSlaveConfig = NRF_DRV_SPIS_DEFAULT_CONFIG;
    spiSlaveConfig.csn_pin      = SPIS_CS_PIN;
    spiSlaveConfig.miso_pin     = SPIS_MISO_PIN;
    spiSlaveConfig.mosi_pin     = SPIS_MOSI_PIN;
    spiSlaveConfig.sck_pin      = SPIS_SCK_PIN;
    spiSlaveConfig.orc          = 0xFF;
    spiSlaveConfig.def          = 0xFF;
    spiSlaveConfig.mode         = NRF_DRV_SPIS_MODE_0;
    spiSlaveConfig.irq_priority = 7;

    ret_code_t result = nrf_drv_spis_init(&spiSlave, &spiSlaveConfig, spiSlaveEventHandler);
    APP_ERROR_CHECK(result);

    // Initialize GPIOTE as my InterruptIn hack requires it to properly setup the AdafruitPrecision9DoF object.
    result = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(result);

    // Setup to read the IMU sensor readings via I2C.
    static AdafruitPrecision9DoF sensors(SENSOR_SDA_PIN, SENSOR_SCL_PIN, SENSOR_INT_PIN, SENSOR_SAMPLE_RATE);

    if (sensors.didInitFail())
    {
        assert("Encountered I2C I/O error during IMU sensor init.\n" == NULL);
    }

    for (;;)
    {
        if (!sensors.wouldBlock())
        {
            SensorValues sensorValues = sensors.getRawSensorValues();
            if (sensors.didIoFail())
            {
                assert("Encountered I2C I/O error during fetch of IMU sensor readings.\n" == NULL);
            }

            // Wait for previous SPI transfer to complete before sending this one.
            // The previous SPI transfer should have completed by the time we get a new IMU sensor reading anyway.
            while (!g_spiTransferDone)
            {
                // Busy wait.
            }

            // Packet contains sensorValues + 1 byte pre-amble with the length of sensorValues.
            static uint8_t txBuffer[sizeof(sensorValues)+1];
            static uint8_t rxBuffer[sizeof(sensorValues)+1];
            txBuffer[0] = sizeof(sensorValues);
            memcpy(&txBuffer[1], &sensorValues, sizeof(sensorValues));

            g_spiTransferDone = false;
            result = nrf_drv_spis_buffers_set(&spiSlave, txBuffer, sizeof(txBuffer), rxBuffer, sizeof(rxBuffer));
            APP_ERROR_CHECK(result);
        }
    }

    return 0;
}

void spiSlaveEventHandler(nrf_drv_spis_event_t event)
{
    if (event.evt_type == NRF_DRV_SPIS_XFER_DONE)
    {
        // Let the main thread know that the last transfer is now complete.
        g_spiTransferDone = true;
    }
}

// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
extern "C" void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}
