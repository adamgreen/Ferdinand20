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
     * Sends battery voltages, joystick and deadman switch state from PDB to robot's main microcontroller.
     * Receives status text to be displayed on screen from robot's main microcontroller.
   * Updates the onboard LCD display with:
     * LiPo battery voltage
     * Wireless BLE remote battery voltage
     * Manual/Auto mode
     * Motor relay on/off state
     * Status messages sent from robot's main microcontroller.

   The hardware pin connections are:
   nRF51 Pin Number     Pin Name            Pin Description
    3                   RX_PIN_NUMBER       UART Receive Input
    5                   TX_PIN_NUMBER       UART Transmit Output
    24                  MOTOR_RELAY_PIN     Relay Enable Output (Pulls ground down to Gnd via NPN transistor)
    15                  MANUAL_SWITCH_PIN   Manual Switch Input (Pulled low when user flips manual switch on)
    12                  LCD_SCK_PIN         LCD SPI Clock Output
    11                  LCD_MOSI_PIN        LCD SPI MOSI Output
    2                   LCD_TFTDC_PIN       LCD Data/Command Output
    8                   LCD_TFTCS_PIN       LCD SPI Chip Select Output
    23                  LCD_TFTRST_PIN      LCD Reset Output
    6 (AIN7)            BATTERY_VOLTAGE_PIN LiPo Battery Voltage Analog Input (via 300/600 ohm voltage divider)

   This sample is heavily influenced by Nordic's BLE UART Service Client (ble_app_uart_c) SDK sample.
*/
#include <stdio.h>
#include <app_uart.h>
#include <app_scheduler.h>
#include <app_timer_appsh.h>
#include <ble.h>
#include <ble_advdata.h>
#include <ble_db_discovery.h>
#include <ble_gap.h>
#include <ble_hci.h>
#include <ble_nus_c.h>
#include <softdevice_handler.h>
#include <nrf_adc.h>
#include <nrf_delay.h>
#include <nrf_gpio.h>
#include <nrf_drv_wdt.h>
#include "../../include/BleJoyShared.h"
#include "../../include/PdbPacket.h"
#include "Screen.h"


// The UART pins.
#define RX_PIN_NUMBER           3
#define TX_PIN_NUMBER           5

// The output pin used to turn control the motor power relay.
#define MOTOR_RELAY_PIN         24

// The input pin used to read the state of the manual override switch.
#define MANUAL_SWITCH_PIN       15

// The pins used to communicate with the LCD.
#define LCD_SCK_PIN             12
#define LCD_MOSI_PIN            11
#define LCD_TFTDC_PIN           2
#define LCD_TFTCS_PIN           8
#define LCD_TFTRST_PIN          23

// The analog input pin used to read the current LiPo battery voltage through 1/3 voltage divider.
#define BATTERY_VOLTAGE_PIN     NRF_ADC_CONFIG_INPUT_7  // P0.06 is AIN7 on nRF51422.

// The resistors used in battery voltage divider.
#define BATTERY_VOLTAGE_DIVIDER_BOTTOM  327
#define BATTERY_VOLTAGE_DIVIDER_TOP     681
#define BATTERY_VOLTAGE_DIVIDER_TOTAL   (BATTERY_VOLTAGE_DIVIDER_BOTTOM + BATTERY_VOLTAGE_DIVIDER_TOP)

// Low frequency clock source to be used by the SoftDevice
#define NRF_CLOCK_LFCLKSRC      {.source        = NRF_CLOCK_LF_SRC_XTAL,           \
                                 .rc_ctiv       = 0,                              \
                                 .rc_temp_ctiv  = 0,                                \
                                 .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM}

// Motors will be disabled once the 2S LiPo battery reaches this voltage to protect the battery from under voltage.
//  64 = 6.4V
#define ROBOT_LOW_BATTERY       64

// The LCD update timer must feed the watchdog in this time interval (msec) or the PDB will be reset.
#define WATCHDOG_TIMEOUT        250

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
#define UART_RX_BUF_SIZE        1

// Value of the RTC1 PRESCALER register used by application timer.
#define APP_TIMER_PRESCALER     0
// Size of timer operation queues.
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

// If the lcdTimeoutHandler() routine doesn't see new BLE JoyData packets for this many iterations then it considers
// the remote control to have been disconnected.
#define MAX_MISSED_BLE_PACKETS  30

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

// Maximum size of scheduled events - Only timer events are placed in this queue.
#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVT_SIZE
// Maximum number of events to be schedule at once.
#define SCHED_QUEUE_SIZE                2

// The delay to be used between LCD/UART updates.
#define LCD_TIMER_DELAY         APP_TIMER_TICKS(16, APP_TIMER_PRESCALER)
// The number of ticks in a second.
#define TICKS_PER_SECOND        APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)
//Interval to be used between battery voltage level measurements.
#define BATTERY_TIMER_INTERVAL  APP_TIMER_TICKS((60000), APP_TIMER_PRESCALER)



// Format of full packet when in manual driving mode. Contains header and joystick info.
struct ManualPacket
{
    PdbSerialPacketHeader header;
    PdbSerialManualPacket manual;
};

// Format of full packet when in auto mode.
struct AutoPacket
{
    PdbSerialPacketHeader header;
    PdbSerialAutoPacket   _auto;
};

// Circular queue class used for serial data received from robot microcontroller for display on LCD.
template <uint32_t SIZE>
class CircularQueue
{
    public:
        CircularQueue()
        {
            m_pushIndex = 0;
            m_popIndex = 0;
        }

        bool isFull()
        {
            return m_pushIndex + 1 == m_popIndex;
        }

        bool isEmpty()
        {
            return m_pushIndex == m_popIndex;
        }

        uint32_t freeByteCount()
        {
            uint32_t pushIndex = m_pushIndex;
            uint32_t popIndex = m_popIndex;

            if (pushIndex < popIndex)
            {
                return popIndex - pushIndex - 1;
            }
            else
            {
                return (SIZE - 1) - (pushIndex - popIndex);
            }
        }

        void push(uint8_t byte)
        {
            ASSERT ( !isFull() );
            m_data[m_pushIndex] = byte;
            m_pushIndex = nextIndex(m_pushIndex);
        }

        uint8_t pop()
        {
            ASSERT ( !isEmpty() );
            uint8_t byte = m_data[m_popIndex];
            m_popIndex = nextIndex(m_popIndex);
            return byte;
        }


    protected:
        uint32_t nextIndex(uint32_t index)
        {
            return (index + 1) % SIZE;
        }

        volatile uint32_t m_pushIndex;
        volatile uint32_t m_popIndex;
        volatile uint8_t  m_data[SIZE];
};


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
#if (NRF_SD_BLE_API_VERSION == 2)
    .selective   = SCAN_SELECTIVE,
    .p_whitelist = NULL,
#endif
#if (NRF_SD_BLE_API_VERSION == 3)
    .use_whitelist = SCAN_SELECTIVE,
#endif
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
};

// Only BleJoystick devices reporting this UUID in their advertisement scan data will be connected to.
static const ble_uuid_t g_bleJoystickUuid =
{
    .uuid = BLEJOY_ADVERTISE_UUID,
    .type = BLEJOY_ADVERTISE_UUID_TYPE
};

// Timer object used for updating LCD and sending UART packets.
APP_TIMER_DEF(g_lcdTimer);
// Timer object used for reading the battery voltage.
APP_TIMER_DEF(g_batteryTimer);

// Objects used to control the LCD over SPI.
static Screen  g_screen(TICKS_PER_SECOND, LCD_MOSI_PIN, LCD_SCK_PIN, LCD_TFTCS_PIN, LCD_TFTDC_PIN, LCD_TFTRST_PIN);

// This global is set if the robot's LiPo battery drops below the minimum allowed voltage.
// Once set, the motors can no longer be enabled with the deadman switch without power cycling the PDB.
static volatile bool g_isBatteryTooLow = false;

// Global object used to track the PDB state. Used for updating the LCD.
static Screen::PdbState g_state =
{
    .isManualMode = false,
    .isRemoteConnected = false,
    .areMotorsEnabled = false,
    .robotBattery = 0,
    .remoteBattery = 0
};

// Global object used to record the most recent joystick state returned from remote control.
static BleJoyData   g_joyData;

// Count of BLE packets that have been received from the remote control.
static volatile uint32_t    g_blePacketCount = 0;

// The last count seen in g_blePacketCount by lcdTimeoutHandler().
static uint32_t             g_lastBlePacketCount = 0;

// The number of times lcdTimeoutHandler() has seen no new BLE packets.
static uint32_t             g_missingBlePacketCount = 0;

// The packet sent when in manual driving mode.
static ManualPacket g_manualPacket =
{
    .header =
    {
        .signature = PDBSERIAL_PACKET_SIGNATURE,
        .length = sizeof(PdbSerialManualPacket)
    },
    .manual = {
        .x = 0,
        .y = 0,
        .robotBattery = 0,
        .remoteBattery = 0,
        .flags = 0
    }
};

// The packet sent when in auto driving mode.
static AutoPacket g_autoPacket =
{
    .header =
    {
        .signature = PDBSERIAL_PACKET_SIGNATURE,
        .length = sizeof(PdbSerialAutoPacket)
    },
    ._auto = {
        .robotBattery = 0,
        .remoteBattery = 0,
        .flags = 0
    }
};

// The states that the code can be in when parsing UART data sent back from the main robot microcontroller.
static enum UartRecvState
{
    UART_RECV_PACKET_START,
    UART_RECV_PACKET_LENGTH,
    UART_RECV_PACKET_DATA,
    UART_RECV_IGNORE_DATA
} g_recvPacketState = UART_RECV_PACKET_START;

// Number of received packets that had to be dropped because of queue overflow.
static uint32_t g_recvPacketIgnoredCount = 0;

// Number of currently available packets to be pulled from queue.
static volatile uint32_t g_recvPacketQueueCount = 0;

// Length of packet data currently being placed in queue or being dropped.
static uint32_t g_recvPacketLength = 0;

// Progress of transferring packet data into queue or being dropped.
static uint32_t g_recvPacketIndex = 0;

// Circular queue used for serial data received from robot microcontroller for display on LCD.
static CircularQueue<256> g_recvPacketQueue;

// Watchdog channel to be fed by lcdTimeoutHandler().
static nrf_drv_wdt_channel_id g_watchdogChannel;



// Function Prototypes
static void initTimers(void);
static void lcdTimeoutHandler(void* pvContext);
static void checkForMissingBlePackets(Screen::PdbState* pState, BleJoyData* pJoyData);
static void sendSerialPacket(const Screen::PdbState* pState, const BleJoyData* pJoyData);
static void sendBufferToUart(const void* pvData, size_t length);
static uint32_t interlockedDecrement(volatile uint32_t* pVal);
static void startLcdTimer(void);
static void batteryTimeoutHandler(void* pvContext);
static void readBatteryVoltage(void);
static void initUart(void);
static void uartEventHandler(app_uart_evt_t* pEvent);
static void parseReceivedPacketByte(uint8_t byte);
static uint32_t interlockedIncrement(volatile uint32_t* pVal);
static void initButtonsAndLeds(void);
static void initBatteryVoltageReading(void);
static void initDatabaseDiscoveryModule(void);
static void databaseDiscoveryEventHandler(ble_db_discovery_evt_t* pEvent);
static void initBleStack(void);
static void bleEventDispatcher(ble_evt_t* pBleEvent);
static void bleEventHandler(ble_evt_t * pBleEvent);
static bool isUuidPresent(const ble_uuid_t* pTargetUuid, const ble_gap_evt_adv_report_t* pAdvertiseReport);
static void initNordicUartServiceClient(void);
static void nordicUartServiceClientEventHandler(ble_nus_c_t* pNordicUartServiceClient, const ble_nus_c_evt_t* pEvent);
static void startWatchdog(void);
static void watchdogTimeoutHandler(void);
static void startBatteryTimer(void);
static void startScanningForNordicUartPeripherals(void);
static void enterLowPowerModeUntilNextEvent(void);



int main(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
    initTimers();

    initUart();
    initButtonsAndLeds();
    initBatteryVoltageReading();

    initDatabaseDiscoveryModule();
    initBleStack();
    initNordicUartServiceClient();

    startWatchdog();
    startLcdTimer();
    startBatteryTimer();
    startScanningForNordicUartPeripherals();

    for (;;)
    {
        enterLowPowerModeUntilNextEvent();
    }
}


static void initTimers(void)
{
    // Initialize timer module, making it use the scheduler.
    APP_TIMER_APPSH_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, true);

    // Create LCD and UART update timer.
    uint32_t errorCode = app_timer_create(&g_lcdTimer, APP_TIMER_MODE_SINGLE_SHOT, lcdTimeoutHandler);
    APP_ERROR_CHECK(errorCode);

    // Create battery voltage sampling timer.
    errorCode = app_timer_create(&g_batteryTimer, APP_TIMER_MODE_REPEATED, batteryTimeoutHandler);
    APP_ERROR_CHECK(errorCode);
}

static void lcdTimeoutHandler(void* pvContext)
{
    // Take a snapshot of the remote and PDB state as they exist at the beginning of this timer. That way one part of
    // this handler doesn't see a different state than another part.
    Screen::PdbState state = g_state;
    BleJoyData       joyData = g_joyData;

    // Mark remote as not being connected if we haven't received any packets from it for a few iterations of this
    // callback.
    checkForMissingBlePackets(&state, &joyData);

    // Make sure that motors get disabled once the LiPo battery voltage gets too low.
    if (g_isBatteryTooLow)
    {
        joyData.buttons &= ~BLEJOY_BUTTONS_DEADMAN;
    }

    // Set motor relay enable pin based on latest remote dead man switch state just received.
    nrf_gpio_pin_write(MOTOR_RELAY_PIN, joyData.buttons & BLEJOY_BUTTONS_DEADMAN);

    // Read the state of the manual override switch.
    uint32_t manualButtonPressed = nrf_gpio_pin_read(MANUAL_SWITCH_PIN);
    state.isManualMode = (manualButtonPressed == 1);

    // Set PDB state fields from most recent joystick data.
    state.remoteBattery = joyData.batteryVoltage;
    state.areMotorsEnabled = joyData.buttons & BLEJOY_BUTTONS_DEADMAN;

    g_screen.update(&state);
    if (g_recvPacketQueueCount > 0)
    {
        char buffer[Screen::TEXT_LINE_LENGTH + 1];
        char* pBuffer = buffer;

        uint8_t textLength = g_recvPacketQueue.pop();
        for (uint8_t i = 0 ; i < textLength ; i++)
        {
            uint8_t byte = g_recvPacketQueue.pop();
            if (i < sizeof(buffer) - 1)
            {
                *pBuffer++ = byte;
            }
        }
        *pBuffer = '\0';
        g_screen.updateText(buffer);
        interlockedDecrement(&g_recvPacketQueueCount);
    }

    // Send serial packet to robot microcontroller.
    sendSerialPacket(&state, &joyData);

    // Feed the watchdog timer.
    nrf_drv_wdt_channel_feed(g_watchdogChannel);

    // Restart the timer again.
    startLcdTimer();
}

static void checkForMissingBlePackets(Screen::PdbState* pState, BleJoyData* pJoyData)
{
    if (g_blePacketCount == g_lastBlePacketCount)
    {
        // No new BLE packets from the remote control since the last time this function ran.
        g_missingBlePacketCount++;
        if (g_missingBlePacketCount > MAX_MISSED_BLE_PACKETS)
        {
            // Not receiving BLE packets from remote control so force deadman switch status to unpressed state.
            pJoyData->buttons &= ~BLEJOY_BUTTONS_DEADMAN;
            pState->isRemoteConnected = false;
        }
    }
    else
    {
        // Remember that we have seen this latest BLE packet.
        g_lastBlePacketCount = g_blePacketCount;
        g_missingBlePacketCount = 0;
    }

}

static void sendSerialPacket(const Screen::PdbState* pState, const BleJoyData* pJoyData)
{
    if (pState->isManualMode)
    {
        g_manualPacket.manual.x = pJoyData->x;
        g_manualPacket.manual.y = pJoyData->y;
        g_manualPacket.manual.robotBattery = pState->robotBattery;
        g_manualPacket.manual.remoteBattery = pState->remoteBattery;
        g_manualPacket.manual.flags = ((pJoyData->buttons & BLEJOY_BUTTONS_JOYSTICK) ? PDBSERIAL_FLAGS_JOYSTICK_BUTTON : 0) |
                                      (pState->areMotorsEnabled ? PDBSERIAL_FLAGS_MOTORS_ENABLED : 0) |
                                      (pState->isRemoteConnected ? PDBSERIAL_FLAGS_REMOTE_CONNECTED : 0);
        sendBufferToUart(&g_manualPacket, sizeof(g_manualPacket));
    }
    else
    {
        g_autoPacket._auto.robotBattery = pState->robotBattery;
        g_autoPacket._auto.remoteBattery = pState->remoteBattery;
        g_autoPacket._auto.flags = (pState->areMotorsEnabled ? PDBSERIAL_FLAGS_MOTORS_ENABLED : 0) |
                                   (pState->isRemoteConnected ? PDBSERIAL_FLAGS_REMOTE_CONNECTED : 0);
        sendBufferToUart(&g_autoPacket, sizeof(g_autoPacket));
    }
}

static void sendBufferToUart(const void* pvData, size_t length)
{
    const uint8_t* pData = (const uint8_t*)pvData;
    while (length-- > 0)
    {
        uint32_t errorCode = app_uart_put(*pData++);
        APP_ERROR_CHECK(errorCode);
    }
}

static uint32_t interlockedDecrement(volatile uint32_t* pVal)
{
    uint32_t newValue;
    CRITICAL_REGION_ENTER();
        newValue = *pVal - 1;
        *pVal = newValue;
    CRITICAL_REGION_EXIT();
    return newValue;
}

static void startLcdTimer(void)
{
    // Start the timer used to send UART packets and update LCD at regular intervals.
    uint32_t errorCode = app_timer_start(g_lcdTimer, LCD_TIMER_DELAY, NULL);
    APP_ERROR_CHECK(errorCode);
}

static void batteryTimeoutHandler(void* pvContext)
{
    // Process the most recent battery voltage ADC conversion if ready.
    if (nrf_adc_conversion_finished())
    {
        readBatteryVoltage();
    }
}

static void readBatteryVoltage(void)
{
    // Read the last ADC battery voltage read.
    uint32_t adcReading = nrf_adc_result_get();

    // Convert to 10 X Volts.
    g_state.robotBattery = adcReading * 36 * BATTERY_VOLTAGE_DIVIDER_TOTAL / (1023 * BATTERY_VOLTAGE_DIVIDER_BOTTOM);

    // Set global flag if the battery level ever gets too low so that motors can be disabled to protect the battery.
    if (g_state.robotBattery <= ROBOT_LOW_BATTERY)
    {
        g_isBatteryTooLow = true;
    }

    // Start the next conversion.
    nrf_adc_conversion_event_clean();
    nrf_adc_start();
}

static void initUart(void)
{
    uint32_t errorCode;

    const app_uart_comm_params_t uartCommParams =
    {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = UART_PIN_DISCONNECTED,
        .cts_pin_no   = UART_PIN_DISCONNECTED,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
    };

    APP_UART_FIFO_INIT(&uartCommParams,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uartEventHandler,
                       APP_IRQ_PRIORITY_HIGHEST,
                       errorCode);
    APP_ERROR_CHECK(errorCode);

    // Enable pull-up on Rx pin so that we don't get framing errors if nothing is connected.
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_PULLUP);
}

static void uartEventHandler(app_uart_evt_t* pEvent)
{
    uint8_t byte = 0;
    switch (pEvent->evt_type)
    {
        case APP_UART_DATA_READY:
            app_uart_get(&byte);
            parseReceivedPacketByte(byte);
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

static void parseReceivedPacketByte(uint8_t byte)
{
    uint32_t length = 0;

    switch (g_recvPacketState)
    {
        case UART_RECV_PACKET_START:
            if (byte == PDBSERIAL_PACKET_SIGNATURE)
            {
                g_recvPacketState = UART_RECV_PACKET_LENGTH;
            }
            break;
        case UART_RECV_PACKET_LENGTH:
            length = byte;
            if (length == 0)
            {
                // Received a packet with no data so setup to wait for start of next packet.
                g_recvPacketState = UART_RECV_PACKET_START;
            }
            else if (g_recvPacketQueue.freeByteCount() < length + 1) // +1 for length prefix.
            {
                // Queue doesn't contain enough room for this packet so ignore it.
                g_recvPacketLength = length;
                g_recvPacketIndex = 0;
                g_recvPacketState = UART_RECV_IGNORE_DATA;
            }
            else
            {
                g_recvPacketQueue.push(length);
                g_recvPacketLength = length;
                g_recvPacketIndex = 0;
                g_recvPacketState = UART_RECV_PACKET_DATA;
            }
            break;
        case UART_RECV_PACKET_DATA:
            g_recvPacketQueue.push(byte);
            g_recvPacketIndex++;
            if (g_recvPacketIndex >= g_recvPacketLength)
            {
                // Have finished receiving packet data.
                interlockedIncrement(&g_recvPacketQueueCount);
                g_recvPacketState = UART_RECV_PACKET_START;
            }
            break;
        case UART_RECV_IGNORE_DATA:
            g_recvPacketIndex++;
            if (g_recvPacketIndex >= g_recvPacketLength)
            {
                // Have finished receiving packet data.
                g_recvPacketIgnoredCount++;
                g_recvPacketState = UART_RECV_PACKET_START;
            }
            break;
    }
}

static uint32_t interlockedIncrement(volatile uint32_t* pVal)
{
    uint32_t newValue;
    CRITICAL_REGION_ENTER();
        newValue = *pVal + 1;
        *pVal = newValue;
    CRITICAL_REGION_EXIT();
    return newValue;
}

static void initButtonsAndLeds(void)
{
    // Configure the output pin to drive the motor power relay via a NPN BJT.
    // Needs to be able to source 5mA to the BJT gate when high.
    nrf_gpio_pin_clear(MOTOR_RELAY_PIN);
    nrf_gpio_cfg(MOTOR_RELAY_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,
                 NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);

    // Configure the pin connected to the manual mode override switch as an input with pull-down.
    nrf_gpio_cfg_input(MANUAL_SWITCH_PIN, NRF_GPIO_PIN_PULLDOWN);
}

static void initBatteryVoltageReading(void)
{
    // Setup to sample 1/3Vbatt and compare to 1.2V VBG.
    const nrf_adc_config_t adcConfig = { .resolution = NRF_ADC_CONFIG_RES_10BIT,
                                         .scaling = NRF_ADC_CONFIG_SCALING_INPUT_ONE_THIRD,
                                         .reference = NRF_ADC_CONFIG_REF_VBG };
    nrf_adc_configure((nrf_adc_config_t *)&adcConfig);

    // Start the first battery voltage read.
    nrf_adc_input_select(BATTERY_VOLTAGE_PIN);
    nrf_adc_start();

    // Wait for the first battery level translation to complete.
    while (!nrf_adc_conversion_finished())
    {
    }
    // Read the first battery voltage sample and queue up the next one to be read in the battery timer.
    readBatteryVoltage();
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
            }
            break;
        }
        case BLE_GAP_EVT_CONNECTED:
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
            errorCode = sd_ble_gap_disconnect(pBleEvent->evt.gattc_evt.conn_handle,
                                              BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(errorCode);
            // UNDONE: Might want to restart scanning process again here.
            break;
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
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
            break;
        case BLE_NUS_C_EVT_NUS_RX_EVT:
            // This event is sent each time that the Nordic UART peripheral sends a packet of information to this
            // central device.
            if (pEvent->data_len == sizeof(BleJoyData))
            {
                // Copy the most recent joystick state into the g_joyData global to be used from the LCD timer.
                memcpy(&g_joyData, pEvent->p_data, sizeof(g_joyData));

                // The remote isn't considered connected until we successfully receive its first packet. This stops
                // a remote battery voltage of 0.0V being displayed on the LCD or sent out the UART.
                g_state.isRemoteConnected = true;

                // Let the LCD update timer know that we have received a BLE packet recently.
                g_blePacketCount++;
            }
            break;
        case BLE_NUS_C_EVT_DISCONNECTED:
            // Nordic UART peripheral has disconnected.
            // Record that it has disconnected.
            g_state.isRemoteConnected = false;

            // Forget joystick state once no longer connected.
            memset(&g_joyData, 0, sizeof(g_joyData));

            // Start scanning for a reconnect.
            startScanningForNordicUartPeripherals();
            break;
    }
}

static void startScanningForNordicUartPeripherals(void)
{
    uint32_t errorCode = sd_ble_gap_scan_start(&g_bleScanParameters);
    APP_ERROR_CHECK(errorCode);
}

static void startWatchdog(void)
{
    nrf_drv_wdt_config_t config =
    {
        .behaviour = NRF_WDT_BEHAVIOUR_RUN_SLEEP,
        .reload_value = WATCHDOG_TIMEOUT,
        .interrupt_priority = APP_IRQ_PRIORITY_LOWEST
    };

    uint32_t errorCode = nrf_drv_wdt_init(&config, watchdogTimeoutHandler);
    APP_ERROR_CHECK(errorCode);

    errorCode = nrf_drv_wdt_channel_alloc(&g_watchdogChannel);
    APP_ERROR_CHECK(errorCode);

    nrf_drv_wdt_enable();
}

static void watchdogTimeoutHandler(void)
{
    while (true)
    {
        // Do nothing and just wait for the microcontroller to reset.
    }
}

static void startBatteryTimer(void)
{
    // Start the timer used to measure the battery voltage.
    uint32_t errorCode = app_timer_start(g_batteryTimer, BATTERY_TIMER_INTERVAL, NULL);
    APP_ERROR_CHECK(errorCode);
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

// Required to get C++ code to build.
extern "C" void __cxa_pure_virtual()
{
    ASSERT ( 0 );
    abort();
}
