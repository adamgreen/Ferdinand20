/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
/* A Bluetooth Low Energy (BLE) to UART bridge to be used between the MRI debug monitor and the Macintosh running GDB.

   More information about MRI can be found here: https://github.com/adamgreen/mri
   The Macintosh running GDB also needs to run the macOS portion of BLEMRI to act as a bridge between BLE and the
   TCP/IP protocol since GDB knows how to use TCP/IP but not BLE for remote target debugging.

   Currently the hardware pin connections are:

   nRF51 Pin Number     Pin Name        Pin Description
          1             UART_RX_PIN     Connected to the MRI serial Tx line
          2             UART_TX_PIN     Connected to the MRI serial Rx line

   This sample is heavily influenced by Nordic's BLE UART Service (ble_app_uart) SDK sample.
*/
#include <app_button.h>
#include <app_scheduler.h>
#include <app_timer.h>
#include <app_uart.h>
#include <app_util_platform.h>
#include <ble_advdata.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_hci.h>
#include <ble_nus.h>
#include <bsp.h>
#include <bsp_btn_ble.h>
#include <nordic_common.h>
#include <nrf.h>
#include <softdevice_handler.h>



// UART Pin Connections
#define UART_RX_PIN                     1
#define UART_TX_PIN                     2

// The <= 20 byte chunks of UART data to be sent over BLE can be queued up if a BLE_ERROR_NO_TX_PACKETS is
// encountered when attempting the BLE send.
#define SERIAL_CHUNK_QUEUE_SIZE 64

// This is the vendor UUID offset to be advertised by BLEMRI devices.
#define BLEMRI_ADVERTISE 0xADA3

// The service database for this device can't be changed at runtime. Must be non-zero for DFU.
#define IS_SRVC_CHANGED_CHARACT_PRESENT 0

#if (NRF_SD_BLE_API_VERSION == 3)
// MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
#define NRF_BLE_MAX_MTU_SIZE            GATT_MTU_SIZE_DEFAULT
#endif

// Reply when unsupported features are requested.
#define APP_FEATURE_NOT_SUPPORTED       BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2

// This application doesn't need to act as a central device.
#define CENTRAL_LINK_COUNT              0
// This application is a peripheral and only hosts 1 peripheral link back to a central device.
#define PERIPHERAL_LINK_COUNT           1

#define DEVICE_NAME                     "BLEMRI"

// UUID type for the Nordic UART Service (vendor specific).
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN

// The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).
#define APP_ADV_INTERVAL                64
// The advertising timeout (in units of seconds).
#define APP_ADV_TIMEOUT_IN_SECONDS      180

// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         4

// Timings for how often peripheral and central should communicate over the link. The shorter the interval, the more
// data that can be sent over the link but uses more CPU and power resources.
// Minimum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units.
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
// Maximum acceptable connection interval (7.5 ms), Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
// The number of the connection interval events that the peripheral can ignore before response is required.
#define SLAVE_LATENCY                   0

// Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
// Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
// Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
// Number of attempts before giving up the connection parameter negotiation.
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

// Size of FIFOs used for UART transmission.
#define UART_TX_BUF_SIZE                256
#define UART_RX_BUF_SIZE                256



// Structures used to queue up serial chunks for later transmission via BLE.
typedef struct SerialChunk
{
    uint8_t length;
    uint8_t data[BLE_NUS_MAX_DATA_LEN];
} SerialChunk;

typedef struct SerialChunkCircularQueue
{
    SerialChunk      chunks[SERIAL_CHUNK_QUEUE_SIZE];
    volatile uint8_t pushIndex;
    volatile uint8_t popIndex;
} SerialChunkQueue;



// Nordic UART service.
static ble_nus_t                    g_nordicUartService;

// Current BLE connection.
static uint16_t                     g_currBleConnection = BLE_CONN_HANDLE_INVALID;

// UUIDS returned in advertising scan response.
static ble_uuid_t                   g_advertiseUuids[] = {{BLEMRI_ADVERTISE, NUS_SERVICE_UUID_TYPE}};

// Queue of serial packets to be sent to PC via BLE.
static SerialChunkQueue             g_chunkQueue;

// Global counters used to track serial packets queued up for transmission and packets that have been transmitted
// successfully. Each counter is incremented at a different IRQ priority level and when they are equal then there are
// no outstanding packets to be transmitted.
static volatile uint32_t            g_packetsQueued;
static volatile uint32_t            g_packetsTransmitted;



// Forward Function Declarations
static void initUart(void);
static void uartEventHandler(app_uart_evt_t * pEvent);
static void transmitFirstPacket(void * pData, uint16_t length);
static void interlockedIncrement(volatile uint32_t* pVal);
static void pushSerialChunk(const uint8_t* pData, uint8_t length);
static bool isSerialChunkQueueFull();
static uint32_t nextIndex(uint32_t index);
static void initButtonsAndLeds(bool * pEraseBonds);
static void bspEventHandler(bsp_event_t event);
static void enterDeepSleep(void);
static void initBleStack(void);
static void bleEventHandler(ble_evt_t * p_ble_evt);
static bool isSerialChunkQueueEmpty();
static void transmitNextChunk();
static void popSerialChunk(uint8_t* pDest, uint32_t* pLength);
static void handleBleEventsForApplication(ble_evt_t * pBleEvent);
static void initGapParams(void);
static void initBleUartService(void);
static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length);
static void initBleAdvertising(void);
static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent);
static void initConnectionParameters(void);
static void connectionParameterEventHandler(ble_conn_params_evt_t * pEvent);
static void connectionParameterErrorHandler(uint32_t errorCode);
static void enterLowPowerModeUntilNextEvent(void);



int main(void)
{
    APP_SCHED_INIT(BLE_NUS_MAX_DATA_LEN, 1);
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    initUart();

    bool eraseBonds;
    initButtonsAndLeds(&eraseBonds);
    initBleStack();
    initGapParams();
    initBleUartService();
    initBleAdvertising();
    initConnectionParameters();

    uint32_t errorCode = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(errorCode);

    // Enter main loop.
    for (;;)
    {
        enterLowPowerModeUntilNextEvent();
    }
}

static void initUart(void)
{
    uint32_t                     errorCode;
    const app_uart_comm_params_t commParams =
    {
        UART_RX_PIN,
        UART_TX_PIN,
        0xFF, // RTS not used
        0xFF, // CTS not used
        APP_UART_FLOW_CONTROL_DISABLED,
        false,
        UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&commParams,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uartEventHandler,
                       APP_IRQ_PRIORITY_HIGHEST,
                       errorCode);
    APP_ERROR_CHECK(errorCode);

    // Enable pull-up on Rx pin so that we don't get framing errors if nothing is connected.
    nrf_gpio_cfg_input(UART_RX_PIN, NRF_GPIO_PIN_PULLUP);
}

static void uartEventHandler(app_uart_evt_t * pEvent)
{
    static uint8_t data[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    static int32_t bytesLeftInPacket = -1;

    switch (pEvent->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data[index]));

            index++;
            if (bytesLeftInPacket >= 0)
            {
                bytesLeftInPacket--;
            }

            if (data[index - 1] == '#')
            {
                bytesLeftInPacket = 2;
            }
            if (bytesLeftInPacket == 0 || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                bool needToPushChunk = true;
                if (g_packetsQueued == g_packetsTransmitted)
                {
                    // Normally queued up serial chunks will be transmitted when the previous one completes.
                    // However if there isn't already a transmit in-flight, we need to queue up an event to
                    // have the main thread loop issue the first transmit.
                    uint32_t errorCode = app_sched_event_put(data, index, transmitFirstPacket);
                    if (errorCode == NRF_SUCCESS)
                    {
                        needToPushChunk = false;
                    }
                    // It will fail to schedule an event if there is already an event scheduled so it is ok to just add
                    // to chunk queue instead.
                }
                if (needToPushChunk)
                {
                    pushSerialChunk(data, index);
                }
                index = 0;
                if (bytesLeftInPacket == 0)
                {
                    bytesLeftInPacket = -1;
                }
            }
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

static void transmitFirstPacket(void * pData, uint16_t length)
{
    uint32_t errorCode  = ble_nus_string_send(&g_nordicUartService, pData, length);
    if (errorCode == NRF_SUCCESS)
    {
        interlockedIncrement(&g_packetsQueued);
    }
    else if (errorCode != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(errorCode);
    }
}

static void interlockedIncrement(volatile uint32_t* pVal)
{
    CRITICAL_REGION_ENTER();
        *pVal = *pVal + 1;
    CRITICAL_REGION_EXIT();
}

static void pushSerialChunk(const uint8_t* pData, uint8_t length)
{
    ASSERT ( !isSerialChunkQueueFull() );

    uint32_t pushIndex = g_chunkQueue.pushIndex;
    SerialChunk* pChunk = &g_chunkQueue.chunks[pushIndex];
    memcpy(pChunk->data, pData, length);
    pChunk->length = length;
    g_chunkQueue.pushIndex = nextIndex(pushIndex);
}

static bool isSerialChunkQueueFull()
{
    return nextIndex(g_chunkQueue.pushIndex) == g_chunkQueue.popIndex;
}

static uint32_t nextIndex(uint32_t index)
{
    return (index + 1) % SERIAL_CHUNK_QUEUE_SIZE;
}

static void initButtonsAndLeds(bool * pEraseBonds)
{
    bsp_event_t startupEvent;

    uint32_t errorCode = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                  bspEventHandler);
    APP_ERROR_CHECK(errorCode);

    errorCode = bsp_btn_ble_init(NULL, &startupEvent);
    APP_ERROR_CHECK(errorCode);

    *pEraseBonds = (startupEvent == BSP_EVENT_CLEAR_BONDING_DATA);
}

static void bspEventHandler(bsp_event_t event)
{
    uint32_t errorCode;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            enterDeepSleep();
            break;

        case BSP_EVENT_DISCONNECT:
            errorCode = sd_ble_gap_disconnect(g_currBleConnection, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (errorCode != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(errorCode);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (g_currBleConnection == BLE_CONN_HANDLE_INVALID)
            {
                errorCode = ble_advertising_restart_without_whitelist();
                if (errorCode != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(errorCode);
                }
            }
            break;

        default:
            break;
    }
}

static void enterDeepSleep(void)
{
    uint32_t errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(errorCode);

    // Prepare wakeup buttons.
    errorCode = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(errorCode);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    errorCode = sd_power_system_off();
    APP_ERROR_CHECK(errorCode);
}

static void initBleStack(void)
{
    uint32_t errorCode;

    nrf_clock_lf_cfg_t clockConfig = NRF_CLOCK_LFCLKSRC;

    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clockConfig, NULL);

    ble_enable_params_t defaultBleParams;
    errorCode = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &defaultBleParams);
    APP_ERROR_CHECK(errorCode);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    defaultBleParams.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    errorCode = softdevice_enable(&defaultBleParams);
    APP_ERROR_CHECK(errorCode);

    // Subscribe for BLE events.
    errorCode = softdevice_ble_evt_handler_set(bleEventHandler);
    APP_ERROR_CHECK(errorCode);
}

static void bleEventHandler(ble_evt_t * pBleEvent)
{
    ble_conn_params_on_ble_evt(pBleEvent);
    ble_nus_on_ble_evt(&g_nordicUartService, pBleEvent);
    handleBleEventsForApplication(pBleEvent);
    ble_advertising_on_ble_evt(pBleEvent);
    bsp_btn_ble_on_ble_evt(pBleEvent);
}

static void handleBleEventsForApplication(ble_evt_t * pBleEvent)
{
    uint32_t errorCode;

    switch (pBleEvent->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            errorCode = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(errorCode);
            g_currBleConnection = pBleEvent->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            errorCode = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(errorCode);
            g_currBleConnection = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_EVT_TX_COMPLETE:
            g_packetsTransmitted++;
            if (!isSerialChunkQueueEmpty())
            {
                transmitNextChunk();
            }
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            errorCode = sd_ble_gap_sec_params_reply(g_currBleConnection, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            errorCode = sd_ble_gatts_sys_attr_set(g_currBleConnection, NULL, 0, 0);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            errorCode = sd_ble_user_mem_reply(pBleEvent->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = pBleEvent->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    errorCode = sd_ble_gatts_rw_authorize_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(errorCode);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            errorCode = sd_ble_gatts_exchange_mtu_reply(pBleEvent->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(errorCode);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}

static bool isSerialChunkQueueEmpty()
{
    return g_chunkQueue.pushIndex == g_chunkQueue.popIndex;
}

static void transmitNextChunk()
{
    uint8_t data[BLE_NUS_MAX_DATA_LEN];
    uint32_t length = 0;

    popSerialChunk(data, &length);
    uint32_t errorCode  = ble_nus_string_send(&g_nordicUartService, data, length);
    if (errorCode == NRF_SUCCESS)
    {
        interlockedIncrement(&g_packetsQueued);
    }
    else if (errorCode != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(errorCode);
    }
}

static void popSerialChunk(uint8_t* pDest, uint32_t* pLength)
{
    ASSERT ( !isSerialChunkQueueEmpty() );

    uint32_t popIndex = g_chunkQueue.popIndex;
    const SerialChunk* pChunk = &g_chunkQueue.chunks[popIndex];
    uint32_t length = pChunk->length;
    memcpy(pDest, pChunk->data, length);
    *pLength = length;
    g_chunkQueue.popIndex = nextIndex(popIndex);
}

static void initGapParams(void)
{
    uint32_t                errorCode;
    ble_gap_conn_params_t   gapPreferredConnectionParams;
    ble_gap_conn_sec_mode_t securityMode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&securityMode);

    errorCode = sd_ble_gap_device_name_set(&securityMode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(errorCode);

    memset(&gapPreferredConnectionParams, 0, sizeof(gapPreferredConnectionParams));

    gapPreferredConnectionParams.min_conn_interval = MIN_CONN_INTERVAL;
    gapPreferredConnectionParams.max_conn_interval = MAX_CONN_INTERVAL;
    gapPreferredConnectionParams.slave_latency     = SLAVE_LATENCY;
    gapPreferredConnectionParams.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    errorCode = sd_ble_gap_ppcp_set(&gapPreferredConnectionParams);
    APP_ERROR_CHECK(errorCode);
}

static void initBleUartService(void)
{
    uint32_t       errorCode;
    ble_nus_init_t nordicUartServiceParams;

    memset(&nordicUartServiceParams, 0, sizeof(nordicUartServiceParams));

    nordicUartServiceParams.data_handler = nordicUartServiceHandler;

    errorCode = ble_nus_init(&g_nordicUartService, &nordicUartServiceParams);
    APP_ERROR_CHECK(errorCode);
}

static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while (app_uart_put(pData[i]) != NRF_SUCCESS);
    }
}

static void initBleAdvertising(void)
{
    uint32_t               errorCode;
    ble_advdata_t          advertisingData;
    ble_advdata_t          scanResponse;
    ble_adv_modes_config_t options;

    // Data to be sent with each advertising cycle.
    memset(&advertisingData, 0, sizeof(advertisingData));
    advertisingData.name_type          = BLE_ADVDATA_FULL_NAME;
    advertisingData.include_appearance = false;
    advertisingData.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // Data to be sent back to central device if it sends a scan request.
    memset(&scanResponse, 0, sizeof(scanResponse));
    scanResponse.uuids_complete.uuid_cnt = sizeof(g_advertiseUuids) / sizeof(g_advertiseUuids[0]);
    scanResponse.uuids_complete.p_uuids  = g_advertiseUuids;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    errorCode = ble_advertising_init(&advertisingData, &scanResponse, &options, bleAdvertisingEventHandler, NULL);
    APP_ERROR_CHECK(errorCode);
}

static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent)
{
    uint32_t errorCode;

    switch (bleAdvertisingEvent)
    {
        case BLE_ADV_EVT_FAST:
            errorCode = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(errorCode);
            break;
        case BLE_ADV_EVT_IDLE:
            enterDeepSleep();
            break;
        default:
            break;
    }
}

static void initConnectionParameters(void)
{
    uint32_t               errorCode;
    ble_conn_params_init_t initParams;

    memset(&initParams, 0, sizeof(initParams));

    initParams.p_conn_params                  = NULL;
    initParams.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    initParams.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    initParams.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    initParams.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    initParams.disconnect_on_fail             = false;
    initParams.evt_handler                    = connectionParameterEventHandler;
    initParams.error_handler                  = connectionParameterErrorHandler;

    errorCode = ble_conn_params_init(&initParams);
    APP_ERROR_CHECK(errorCode);
}

static void connectionParameterEventHandler(ble_conn_params_evt_t * pEvent)
{
    uint32_t errorCode;

    if (pEvent->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        errorCode = sd_ble_gap_disconnect(g_currBleConnection, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(errorCode);
    }
}

static void connectionParameterErrorHandler(uint32_t errorCode)
{
    APP_ERROR_HANDLER(errorCode);
}

static void enterLowPowerModeUntilNextEvent(void)
{
    uint32_t errorCode = sd_app_evt_wait();
    APP_ERROR_CHECK(errorCode);
    app_sched_execute();
}



// Break into debugger if any errors/asserts are detected at runtime.
// This handler is called when an application error is encountered in BLE stack or application code.
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    { __asm volatile ("bkpt #0"); }
}
