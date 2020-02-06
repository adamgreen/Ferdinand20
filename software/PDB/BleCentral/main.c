/*  Copyright (C) 2020  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* The firmware for the Ferdinand20 Power Distribution Board. This board performs multiple duties:
   * Controls a relay that switches the robot's motor supply voltage on and off.
   * Only enables the motor supply voltage if a wireless BLE deadman switch is enaged.
   * Monitors the LiPo battery voltage and switches the motor supply relay off if undervoltage is detected.
   * Provides option to allow the user to take manual control with wireless BLE joystick in non-competition
     scenarios. A switch is provided on the PDB to enable this mode.
   * UART connectivity to the robot's main microcontroller
     * Sends joystick and deadman switch state from PDB to robot's main microcontroller.
     * Receives status text to be displayed on screen from robot's main microcontroller.
   * Updates the onboard OLED screen with:
     * LiPo battery voltage
     * Wireless BLE remote battery voltage
     * Manual/Auto mode
     * Motor relay on/off state
     * Status messages sent from robot's main microcontroller.

   The hardware pin connections are:
   nRF51 Pin Number     Pin Name            Pin Description

   This sample is heavily influenced by Nordic's BLE UART Service Client (ble_app_uart_c) SDK sample.
*/
#include <stdio.h>
#include <app_uart.h>
#include <app_timer.h>
#include <bsp.h>
#include <bsp_btn_ble.h>
#include <ble.h>
#include <ble_advdata.h>
#include <ble_db_discovery.h>
#include <ble_gap.h>
#include <ble_hci.h>
#include <ble_nus_c.h>
#include <softdevice_handler.h>
#include "../BleJoyShared.h"



// This firmware acts as a BLE central with a single link to the BleJoystick peripheral.
#define CENTRAL_LINK_COUNT      1
// This application doesn't need to act as a BLE peripheral.
#define PERIPHERAL_LINK_COUNT   0

// MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE    GATT_MTU_SIZE_DEFAULT
#endif

// The size of the transmit and receive arrays for buffering the UART data to and from the main robot uC.
#define UART_TX_BUF_SIZE        256
#define UART_RX_BUF_SIZE        256

// Value of the RTC1 PRESCALER register used by application timer.
#define APP_TIMER_PRESCALER     0
// Size of timer operation queues. Includes room for BSP specific timers.
#define APP_TIMER_OP_QUEUE_SIZE 2

// Parameters for configuring how this application scans for nearby BleJoystick peripherals.
// Scan interval between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s). 0xA0 is equal to 100 milliseconds.
// The central device will switch between advertising channels at this period.
#define SCAN_INTERVAL           0x00A0
// Scan window between 0x0004 and 0x4000 in 0.625ms units (2.5ms to 10.24s). 0x50 is equal to 50 milliseconds.
// The central device will actively scan a single advertising channel for this amount of time.
// Must be <= SCAN_INTERVAL. No scanning is performed for (SCAN_INTERVAL - SCAN_WINDOW) amount of time.
#define SCAN_WINDOW             0x0050
// Timeout when scanning between 0x0001 and 0xFFFF in seconds. 0 disables the timeout so that it keeps scanning as long
// as it is powered up.
#define SCAN_TIMEOUT            0x0000
// If 1, performs active scanning, meaning that it will send scan requests to each advertising device to learn more
// about it.
#define SCAN_ACTIVE             1
// If 1, ignore unknown devices (those not in the whitelist).
#define SCAN_SELECTIVE          0

// Timings for how often peripheral and central should communicate over the link. The shorter the interval, the more
// data that can be sent over the link but uses more CPU and power resources.
// Minimum acceptable connection interval in milliseconds. Connection interval uses 1.25 ms units.
#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(8, UNIT_1_25_MS)
// Maximum acceptable connection interval in milliseconds. Connection interval uses 1.25 ms units.
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(16, UNIT_1_25_MS)
// The number of the connection interval events that the peripheral can ignore before response is required.
#define SLAVE_LATENCY           0
// Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS)

// The size in bytes of the various UUID bit-sized types.
#define UUID16_SIZE             (16/8)
#define UUID32_SIZE             (32/8)
#define UUID128_SIZE            (128/8)



static ble_nus_c_t              g_nordicUartServiceClient;
static ble_db_discovery_t       g_bleDatabaseDiscovery;

static const ble_gap_conn_params_t g_bleConnectionParameters =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};

static const ble_gap_scan_params_t g_bleScanParameters =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
#if (NRF_SD_BLE_API_VERSION == 2)
    .selective   = SCAN_SELECTIVE,
    .p_whitelist = NULL,
#endif
#if (NRF_SD_BLE_API_VERSION == 3)
    .use_whitelist = SCAN_SELECTIVE,
#endif
};

// Only BleJoystick devices reporting this UUID in their advertisement scan data will be connected to.
static const ble_uuid_t g_bleJoystickUuid =
{
    .uuid = BLEJOY_ADVERTISE_UUID,
    .type = BLEJOY_ADVERTISE_UUID_TYPE
};



// Function Prototypes
static void initUart(void);
static void uartEventHandler(app_uart_evt_t* pEvent);
static void initButtonsAndLeds(void);
static void bspEventHandler(bsp_event_t event);
static void enterSleepMode(void);
static void initDatabaseDiscoveryModule(void);
static void databaseDiscoveryEventHandler(ble_db_discovery_evt_t* pEvent);
static void initBleStack(void);
static void bleEventDispatcher(ble_evt_t* pBleEvent);
static void bleEventHandler(ble_evt_t * pBleEvent);
static bool isUuidPresent(const ble_uuid_t* pTargetUuid, const ble_gap_evt_adv_report_t* pAdvertiseReport);
static void initNordicUartServiceClient(void);
static void nordicUartServiceClientEventHandler(ble_nus_c_t* pNordicUartServiceClient, const ble_nus_c_evt_t* pEvent);
static void dumpJoyData(const BleJoyData* pJoyData);
static void startScanningForNordicUartPeripherals(void);
static void enterLowPowerModeUntilNextEvent(void);



int main(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    initUart();
    initButtonsAndLeds();
    initDatabaseDiscoveryModule();
    initBleStack();
    initNordicUartServiceClient();

    startScanningForNordicUartPeripherals();

    for (;;)
    {
        enterLowPowerModeUntilNextEvent();
    }
}

static void initUart(void)
{
    uint32_t errorCode;

    const app_uart_comm_params_t uartCommParams =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = HWFC,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&uartCommParams,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uartEventHandler,
                       APP_IRQ_PRIORITY_LOWEST,
                       errorCode);
    APP_ERROR_CHECK(errorCode);
}

static void uartEventHandler(app_uart_evt_t* pEvent)
{
    uint8_t byte = 0;
    switch (pEvent->evt_type)
    {
        case APP_UART_DATA_READY:
            // UNDONE: Main robot uC will send status messages back this way.
            app_uart_get(&byte);
            break;
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(pEvent->data.error_communication);
            break;
        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(pEvent->data.error_code);
            break;
        default:
            break;
    }
}

static void initButtonsAndLeds(void)
{
    // UNDONE: Just need this for the micro:bit board.
    // Pull column1 of the LED matrix low so that LED will light up when Row1 goes high.
    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_clear(4);

    bsp_event_t startupEvent;
    uint32_t errorCode = bsp_init(BSP_INIT_LED,
                                  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                  bspEventHandler);
    APP_ERROR_CHECK(errorCode);

    errorCode = bsp_btn_ble_init(NULL, &startupEvent);
    APP_ERROR_CHECK(errorCode);
}

static void bspEventHandler(bsp_event_t event)
{
    uint32_t errorCode;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            enterSleepMode();
            break;
        case BSP_EVENT_DISCONNECT:
            errorCode = sd_ble_gap_disconnect(g_nordicUartServiceClient.conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (errorCode != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(errorCode);
            }
            break;
        default:
            break;
    }
}

static void enterSleepMode(void)
{
    uint32_t errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(errorCode);

    errorCode = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(errorCode);

    // This function won't return but will wakeup with a reset instead.
    errorCode = sd_power_system_off();
    APP_ERROR_CHECK(errorCode);
}

static void initDatabaseDiscoveryModule(void)
{
    uint32_t errorCode = ble_db_discovery_init(databaseDiscoveryEventHandler);
    APP_ERROR_CHECK(errorCode);
}

static void databaseDiscoveryEventHandler(ble_db_discovery_evt_t* pEvent)
{
    // As the discovery module finds UUIDs, it will call this function.
    // Pass them along to the Nordic UART Service module so that it can find the GATT IDs it needs for communicating
    // with the BLE UART based devices.
    ble_nus_c_on_db_disc_evt(&g_nordicUartServiceClient, pEvent);
}

static void initBleStack(void)
{
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t bleEnableParams;
    uint32_t errorCode = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                              PERIPHERAL_LINK_COUNT,
                                                              &bleEnableParams);
    APP_ERROR_CHECK(errorCode);

    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

#if (NRF_SD_BLE_API_VERSION == 3)
    bleEnableParams.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    errorCode = softdevice_enable(&bleEnableParams);
    APP_ERROR_CHECK(errorCode);

    errorCode = softdevice_ble_evt_handler_set(bleEventDispatcher);
    APP_ERROR_CHECK(errorCode);
}

static void bleEventDispatcher(ble_evt_t* pBleEvent)
{
    // First give this application itself an opportunity to handle BLE events of interest.
    bleEventHandler(pBleEvent);

    // Then dispatch the BLE events to the various modules we have instantiated so that they can process the ones they
    // are interested in.
    bsp_btn_ble_on_ble_evt(pBleEvent);
    ble_db_discovery_on_ble_evt(&g_bleDatabaseDiscovery, pBleEvent);
    ble_nus_c_on_ble_evt(&g_nordicUartServiceClient, pBleEvent);
}

static void bleEventHandler(ble_evt_t* pBleEvent)
{
    uint32_t              errorCode;
    const ble_gap_evt_t*  pGapEvent = &pBleEvent->evt.gap_evt;

    switch (pBleEvent->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            // This event will be sent during scanning of advertising peripherals.
            // Check to see if this is one advertising the desired Nordic UART UUID.
            const ble_gap_evt_adv_report_t* pAdvertiseReport = &pGapEvent->params.adv_report;

            if (isUuidPresent(&g_bleJoystickUuid, pAdvertiseReport))
            {
                // This is the device we are looking for so connect to it.
                // A successful connection will stop the scanning process.
                errorCode = sd_ble_gap_connect(&pAdvertiseReport->peer_addr,
                                               &g_bleScanParameters,
                                               &g_bleConnectionParameters);
                if (errorCode == NRF_SUCCESS)
                {
                    errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
                    APP_ERROR_CHECK(errorCode);
                    // UNDONE: Can get rid of later.
                    printf("Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
                             pAdvertiseReport->peer_addr.addr[0],
                             pAdvertiseReport->peer_addr.addr[1],
                             pAdvertiseReport->peer_addr.addr[2],
                             pAdvertiseReport->peer_addr.addr[3],
                             pAdvertiseReport->peer_addr.addr[4],
                             pAdvertiseReport->peer_addr.addr[5]
                             );
                }
            }
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
            errorCode = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(errorCode);

            // Start discovery of GATT services on this device. The Nordic UART Client waits for a discovery result
            // to find the services it is interested in for sending/receiving UART data.
            errorCode = ble_db_discovery_start(&g_bleDatabaseDiscovery, pBleEvent->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(errorCode);
            break;
        case BLE_GAP_EVT_TIMEOUT:
            if (pGapEvent->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                // Start scanning again if previous one timed out.
                startScanningForNordicUartPeripherals();
            }
            else if (pGapEvent->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                printf("Connection Request timed out.\r\n");
                // UNDONE: Might want to restart scanning process again here.
            }
            break;
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported.
            errorCode = sd_ble_gap_sec_params_reply(pBleEvent->evt.gap_evt.conn_handle,
                                                    BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(errorCode);
            break;
        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            errorCode = sd_ble_gap_conn_param_update(pGapEvent->conn_handle,
                                                     &pGapEvent->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(errorCode);
            break;
        case BLE_GATTC_EVT_TIMEOUT:
            printf("GATT Client timeout encountered so disconnecting.\r\n");
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gattc_evt.conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            // UNDONE: Might want to restart scanning process again here.
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            printf("GATT Server timeout encountered so disconnecting.\r\n");
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gatts_evt.conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            // UNDONE: Might want to restart scanning process again here.
            break;

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            errorCode = sd_ble_gatts_exchange_mtu_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                        NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(errorCode);
            break;
#endif

        default:
            break;
    }
}

// Reads an advertising report and checks if a uuid is present in the service list.
// This function is able to search for 16-bit, 32-bit and 128-bit service uuids.
// To see the format of a advertisement packet, see
//     https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
static bool isUuidPresent(const ble_uuid_t* pTargetUuid, const ble_gap_evt_adv_report_t* pAdvertiseReport)
{
    uint32_t   errorCode;
    uint32_t   index = 0;
    uint8_t*   pData = (uint8_t *)pAdvertiseReport->data;
    ble_uuid_t extractedUuid;

    while (index < pAdvertiseReport->dlen)
    {
        uint8_t field_length = pData[index];
        uint8_t field_type   = pData[index + 1];

        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID16_SIZE); u_index++)
            {
                errorCode = sd_ble_uuid_decode(  UUID16_SIZE,
                                                &pData[u_index * UUID16_SIZE + index + 2],
                                                &extractedUuid);
                if (errorCode == NRF_SUCCESS)
                {
                    if ((extractedUuid.uuid == pTargetUuid->uuid)
                        && (extractedUuid.type == pTargetUuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID32_SIZE); u_index++)
            {
                errorCode = sd_ble_uuid_decode(UUID16_SIZE,
                &pData[u_index * UUID32_SIZE + index + 2],
                &extractedUuid);
                if (errorCode == NRF_SUCCESS)
                {
                    if ((extractedUuid.uuid == pTargetUuid->uuid)
                        && (extractedUuid.type == pTargetUuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
            errorCode = sd_ble_uuid_decode(UUID128_SIZE,
                                          &pData[index + 2],
                                          &extractedUuid);
            if (errorCode == NRF_SUCCESS)
            {
                if ((extractedUuid.uuid == pTargetUuid->uuid)
                    && (extractedUuid.type == pTargetUuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

static void initNordicUartServiceClient(void)
{
    ble_nus_c_init_t initParams;

    initParams.evt_handler = nordicUartServiceClientEventHandler;

    uint32_t errorCode = ble_nus_c_init(&g_nordicUartServiceClient, &initParams);
    APP_ERROR_CHECK(errorCode);
}

static void nordicUartServiceClientEventHandler(ble_nus_c_t* pNordicUartServiceClient, const ble_nus_c_evt_t* pEvent)
{
    uint32_t errorCode;
    switch (pEvent->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            // This event is sent once the Nordic UART Service Client module detects the UUIDs it needs for sending
            // receiving data wirelessly via BLE.
            errorCode = ble_nus_c_handles_assign(pNordicUartServiceClient, pEvent->conn_handle, &pEvent->handles);
            APP_ERROR_CHECK(errorCode);

            errorCode = ble_nus_c_rx_notif_enable(pNordicUartServiceClient);
            APP_ERROR_CHECK(errorCode);
            printf("Found BLEJoystick device.\r\n");
            break;
        case BLE_NUS_C_EVT_NUS_RX_EVT:
            // This event is sent each time that the Nordic UART peripheral sends a packet of information to this
            // central device.
            if (pEvent->data_len == sizeof(BleJoyData))
            {
                BleJoyData joyData;
                memcpy(&joyData, pEvent->p_data, sizeof(BleJoyData));
                dumpJoyData(&joyData);
            }
            break;
        case BLE_NUS_C_EVT_DISCONNECTED:
            // Nordic UART peripheral has disconnected.
            // Start scanning for a reconnect.
            printf("Disconnected.\r\n");
            startScanningForNordicUartPeripherals();
            break;
    }
}

static void dumpJoyData(const BleJoyData* pJoyData)
{
    printf("% 4d,% 4d %s %u.%uV\r\n",
           pJoyData->x, pJoyData->y,
           (pJoyData->buttons & BLEJOY_BUTTONS_JOYSTICK) ? "JOY" : "   ",
           pJoyData->batteryVoltage / 10,
           pJoyData->batteryVoltage % 10);
}

static void startScanningForNordicUartPeripherals(void)
{
    printf("Searching for BleJoystick device...\r\n");

    uint32_t errorCode = sd_ble_gap_scan_start(&g_bleScanParameters);
    APP_ERROR_CHECK(errorCode);

    errorCode = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(errorCode);
}

static void enterLowPowerModeUntilNextEvent(void)
{
    uint32_t errorCode = sd_app_evt_wait();
    APP_ERROR_CHECK(errorCode);
}



// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}
