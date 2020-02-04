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
/* A Bluetooth Low Energy (BLE) Deadman Switch with Joystick to be used with the Ferdinand20 Power
   Distribution Board. The Deadman Switch has to be engaed for the motor power to be enabled on the Ferdinand20
   rover. The Joystick allows for manual driving control to be used in non-competition testing scenarios.

   The hardware pin connections are:
   nRF51 Pin Number     Pin Name            Pin Description
          1             JOYSTICK_PRESS_PIN  Connected to the joystick's button. Active low w/ software pull-up.
          3 (AIN4)      JOYSTICK_X_PIN      Connected to the X potentiometer wiper of the joystick.
          2 (AIN3)      JOYSTICK_Y_PIN      Connected to the Y potentiometer wiper of the joystick.

   This sample is heavily influenced by Nordic's BLE UART Service (ble_app_uart) SDK sample.
*/
#include <app_scheduler.h>
#include <app_timer_appsh.h>
#include <ble_advertising.h>
#include <ble_conn_params.h>
#include <ble_hci.h>
#include <ble_nus.h>
#include <bsp_btn_ble.h>
#include <nrf_adc.h>
#include <softdevice_handler_appsh.h>
#include "BleJoyShared.h"



// The joystick's X and Y pins are connected to these analog pins.
#define JOYSTICK_X_PIN      NRF_ADC_CONFIG_INPUT_4
#define JOYSTICK_Y_PIN      NRF_ADC_CONFIG_INPUT_3
// The switch that goes low when the joystick itself is depressed is connected to this pin. It will be pulled high in
// software.
#define JOYSTICK_PRESS_PIN  1



// The name of this device.
#define DEVICE_NAME                     "BleJoystick"

// How often the battery voltage level should be measured. 1/2 minute.
#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
// How often the joystick state and deadman switch state should be sampled (60Hz).
#define JOYSTICK_MEAS_INTERVAL          APP_TIMER_TICKS(16, APP_TIMER_PRESCALER)

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


// UUID type for the Nordic UART Service (vendor specific).
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN

// The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms).
#define APP_ADV_INTERVAL                64
// The advertising timeout (in units of seconds).
#define APP_ADV_TIMEOUT_IN_SECONDS      180

// UNDONE: Can this queue size be decreased to 2 instead of 4?
// Value of the RTC1 PRESCALER register.
#define APP_TIMER_PRESCALER             0
// Size of timer operation queues.
#define APP_TIMER_OP_QUEUE_SIZE         4

// Timings for how often peripheral and central should communicate over the link. The shorter the interval, the more
// data that can be sent over the link but uses more CPU and power resources.
// Minimum acceptable connection interval. Connection interval uses 1.25 ms units.
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)
// Maximum acceptable connection interval. Connection interval uses 1.25 ms units.
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(16, UNIT_1_25_MS)
// The number of the connection interval events that the peripheral can ignore before response is required.
#define SLAVE_LATENCY                   20

// Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units.
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)
// Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds).
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)
// Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds).
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER)
// Number of attempts before giving up the connection parameter negotiation.
#define MAX_CONN_PARAMS_UPDATE_COUNT    3

// Maximum size of scheduled events - Timer & BLE Stack events are placed in this queue.
#define SCHED_MAX_EVENT_DATA_SIZE       MAX(APP_TIMER_SCHED_EVT_SIZE, BLE_STACK_HANDLER_SCHED_EVT_SIZE)
// Maximum number of events to be schedule at once.
// 10 for BLE + 2 timers.
#define SCHED_QUEUE_SIZE                (10 + 2)



// Nordic UART service.
static ble_nus_t                    g_nordicUartService;

// Current BLE connection.
static uint16_t                     g_currBleConnection = BLE_CONN_HANDLE_INVALID;

// UUIDS returned in advertising scan response.
static ble_uuid_t                   g_advertiseUuids[] = {{BLEJOY_ADVERTISE, NUS_SERVICE_UUID_TYPE}};

// Latest battery voltage measurement.
static uint8_t                      g_latestBatteryMeasurement;

// Timer objects used for measuring battery voltage, joystick pots, and switch states.
APP_TIMER_DEF(g_batteryMeasurementTimer);
APP_TIMER_DEF(g_joystickMeasurementTimer);



// Forward Function Declarations
static void initTimers(void);
static void batteryMeasurementTimeoutHandler(void* pvContext);
static void joystickMeasurementTimeoutHandler(void* pvContext);
static void initButtonsAndLeds(bool * pEraseBonds);
static void bspEventHandler(bsp_event_t event);
static void enterDeepSleep(void);
static void initBleStack(void);
static void bleEventHandler(ble_evt_t * p_ble_evt);
static void handleBleEventsForApplication(ble_evt_t * pBleEvent);
static void initGapParams(void);
static void initBleUartService(void);
static void nordicUartServiceHandler(ble_nus_t * pNordicUartService, uint8_t * pData, uint16_t length);
static void initBleAdvertising(void);
static void bleAdvertisingEventHandler(ble_adv_evt_t bleAdvertisingEvent);
static void initConnectionParameters(void);
static void connectionParameterEventHandler(ble_conn_params_evt_t * pEvent);
static void connectionParameterErrorHandler(uint32_t errorCode);
static void startTimers(void);
static void enterLowPowerModeUntilNextEvent(void);



int main(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    initTimers();

    bool eraseBonds;
    initButtonsAndLeds(&eraseBonds);
    initBleStack();
    initGapParams();
    initBleUartService();
    initBleAdvertising();
    initConnectionParameters();

    startTimers();
    uint32_t errorCode = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(errorCode);

    // Enter main loop.
    for (;;)
    {
        enterLowPowerModeUntilNextEvent();
    }
}

static void initTimers(void)
{
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create battery and joystick reading timers.
    uint32_t errorCode = app_timer_create(&g_batteryMeasurementTimer,
                                          APP_TIMER_MODE_REPEATED,
                                          batteryMeasurementTimeoutHandler);
    APP_ERROR_CHECK(errorCode);
    errorCode = app_timer_create(&g_joystickMeasurementTimer,
                                 APP_TIMER_MODE_REPEATED,
                                 joystickMeasurementTimeoutHandler);
    APP_ERROR_CHECK(errorCode);
}

static void batteryMeasurementTimeoutHandler(void* pvContext)
{
    // Setup to sample 1/3Vcc and compare to 1.2V VBG.
    const nrf_adc_config_t adcConfig = { .resolution = NRF_ADC_CONFIG_RES_10BIT,
                                         .scaling = NRF_ADC_CONFIG_SCALING_SUPPLY_ONE_THIRD,
                                         .reference = NRF_ADC_CONFIG_REF_VBG };
    nrf_adc_configure((nrf_adc_config_t *)&adcConfig);

    // Perform a blocking ADC read and convert to percentage.
    uint32_t adcReading = nrf_adc_convert_single(ADC_CONFIG_PSEL_Disabled);
    g_latestBatteryMeasurement = adcReading * 36 / 1023;
    ASSERT ( g_latestBatteryMeasurement <= 36 );
}

static void joystickMeasurementTimeoutHandler(void* pvContext)
{
    BleJoyData joyData;

    // Just return without making a reading if there is no BLE connection.
    if (g_currBleConnection == BLE_CONN_HANDLE_INVALID)
    {
        return;
    }

    const nrf_adc_config_t adcConfig = { .resolution = NRF_ADC_CONFIG_RES_10BIT,
                                         .scaling = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD,
                                         .reference = NRF_ADC_CONFIG_REF_SUPPLY_ONE_THIRD };

    // Setup to sample 1/3 analog input and compare to 1/3Vdd.
    nrf_adc_configure((nrf_adc_config_t *)&adcConfig);

    // Perform a blocking ADC reads and convert to -2048 to 2047 range expected for mouse.
    int32_t horizontalReading = nrf_adc_convert_single(JOYSTICK_X_PIN);
    joyData.x = horizontalReading - 512;
    ASSERT ( joyData.x >= -512 && joyData.x <= 511 );

    int32_t verticalReading = nrf_adc_convert_single(JOYSTICK_Y_PIN);
    joyData.y = verticalReading - 512;
    ASSERT ( joyData.y >= -512 && joyData.y <= 511 );

    // UNDONE: Check the deadman switch as well.
    joyData.buttons = 0;
    uint32_t joystickPressed = nrf_gpio_pin_read(JOYSTICK_PRESS_PIN);
    if (joystickPressed == 0)
    {
        joyData.buttons |= BLEJOY_BUTTONS_JOYSTICK;
    }

    // Copy latest battery voltage measurement over to send as well.
    joyData.batteryVoltage = g_latestBatteryMeasurement;

    uint32_t errorCode  = ble_nus_string_send(&g_nordicUartService, (void*)&joyData, sizeof(joyData));
    if (errorCode != NRF_SUCCESS &&
        errorCode != NRF_ERROR_INVALID_STATE &&
        errorCode != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
    {
        APP_ERROR_CHECK(errorCode);
    }
}

static void initButtonsAndLeds(bool * pEraseBonds)
{
    bsp_event_t startupEvent;

    // UNDONE: Just need this for the micro:bit board.
    // Pull column1 of the LED matrix low so that LED will light up when Row1 goes high.
    nrf_gpio_cfg_output(4);
    nrf_gpio_pin_clear(4);

    uint32_t errorCode = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                  APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                  bspEventHandler);
    APP_ERROR_CHECK(errorCode);

    errorCode = bsp_btn_ble_init(NULL, &startupEvent);
    APP_ERROR_CHECK(errorCode);

    // Configure the pin connected to the joystick switch as an input with pull-up.
    nrf_gpio_cfg_input(JOYSTICK_PRESS_PIN, NRF_GPIO_PIN_PULLUP);

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

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module to use the application scheduler for handling BLE events from main().
    SOFTDEVICE_HANDLER_APPSH_INIT(&clock_lf_cfg, true);

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
    // This application just ignores any data sent from the central.
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

static void startTimers(void)
{
    // Start the timers used to measure the battery voltage and joystick state at regular intervals.
    uint32_t errorCode = app_timer_start(g_batteryMeasurementTimer, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(errorCode);

    errorCode = app_timer_start(g_joystickMeasurementTimer, JOYSTICK_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(errorCode);

    // Force the battery voltage measurement to take place immediately so that we don't need to wait for the first
    // timeout to occur before having a valid measurement.
    batteryMeasurementTimeoutHandler(NULL);
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
