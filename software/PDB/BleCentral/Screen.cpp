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
/* Updates the LCD screen attached to the Ferdinand20 Power Distribution Board.
   Updates the screen with:
     * LiPo battery voltage
     * Wireless BLE remote battery voltage
     * Manual/Auto mode
     * Motor relay on/off state
     * Status messages sent from robot's main microcontroller.
*/
#include <stdio.h>
#include <string.h>
#include <app_timer.h>
#include <nrf_gpio.h>
#include "Screen.h"



// Orientation of the screen.
#define SCREEN_ORIENTATION 3

// Size to use for normal text on screen.
static const uint8_t normalTextSize = 2;
// Size to use for tall double height text on screen (ie. Robot Voltage).
static const uint8_t tallTextSize = 4;
// The width of characters on the screen when text size is set to 1.
static const uint16_t charWidth = 6;
// The height of character on the screen when text size is set to 1.
static const uint16_t charHeight = 8;
// The amount of room to leave at top of screen above the scrolling text.
static const uint16_t scrollingTopMargin = tallTextSize * charHeight + 5;



Screen::Screen(uint32_t ticksPerSecond,
               uint8_t mosiPin, uint8_t sckPin, uint8_t csPin, uint8_t dcPin, uint8_t rstPin) :
    m_ticksPerSecond(ticksPerSecond),
    m_tft(SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0, mosiPin, sckPin, dcPin, rstPin, csPin)
{
    m_lastBlinkStartTicks = 0;
    m_blinkManual = false;
    m_blinkManualState = false;
    m_blinkBle = false;
    m_blinkBleState = false;
    m_currentTextLine = 0;

    memset(&m_text, 0, sizeof(m_text));

    // Can just pull CS low once here instead of multiple times in the LCD driver.
    if (CS_ALWAYS_LOW)
    {
        nrf_gpio_pin_clear(csPin);
        nrf_gpio_cfg_output(csPin);
    }

    // Initialize current state to invalid values so that first update will update everything.
    memset(&m_currentState, 0xFF, sizeof(m_currentState));

    m_tft.init();

    // Rotate the screen output to the correction orientation for the screen mounting.
    m_tft.setRotation(SCREEN_ORIENTATION);

    m_tft.clearScreen();
}

void Screen::update(const PdbState* pState)
{
    PdbState state = *pState;

    drawManualAutoMode(state.isManualMode);
    drawRemoteIcon(state.isRemoteConnected);
    drawRemoteVoltage(state.isRemoteConnected, state.remoteBattery);
    drawMotorIcon(state.areMotorsEnabled);
    drawRobotVoltage(state.robotBattery);

    // Remember the new state.
    m_currentState = state;

    // See if we are ready to start a new 1-second blink interval.
    uint32_t currentTicks = app_timer_cnt_get();
    uint32_t elapsedTicks;
    app_timer_cnt_diff_compute(currentTicks, m_lastBlinkStartTicks, &elapsedTicks);
    if (elapsedTicks >= m_ticksPerSecond)
    {
        m_lastBlinkStartTicks = currentTicks;
    }
}

void Screen::drawManualAutoMode(bool isManualMode)
{
    if (m_blinkManual)
    {
        // Turn the "Manual" text on at the beginning of a 1 second interval
        //      - and -
        // Turn the "Manual" text off 3/4 of the way through the 1 second interval.
        uint32_t currentTicks = app_timer_cnt_get();
        uint32_t tickThreshold = (m_ticksPerSecond * 3) / 4;
        uint32_t elapsedTicks;
        app_timer_cnt_diff_compute(currentTicks, m_lastBlinkStartTicks, &elapsedTicks);

        uint16_t color = TFT_WHITE;
        if (!m_blinkManualState && elapsedTicks < tickThreshold)
        {
            color = TFT_RED;
            m_blinkManualState = true;
        }
        else if (m_blinkManualState && elapsedTicks >= tickThreshold)
        {
            color = TFT_BLACK;
            m_blinkManualState = false;
        }

        if (color != TFT_WHITE)
        {
            // Write "Manual" in red or black.
            m_tft.setCursor(0, 0);
            m_tft.setTextSize(normalTextSize);
            m_tft.setTextColor(color);
            m_tft.print("Manual");
        }
    }

    if (isManualMode != m_currentState.isManualMode)
    {
        // Manual/Auto mode has changed so update it on the screen.
        const char* pCurrText;
        const char* pNewText;
        uint16_t newColor;
        if (isManualMode)
        {
            pCurrText = "Auto";
            pNewText = "Manual";
            newColor = TFT_RED;
            m_blinkManualState = true;
        }
        else
        {
            pCurrText = "Manual";
            pNewText = "Auto";
            newColor = TFT_WHITE;
        }

        // Erase current mode text.
        m_tft.setCursor(0, 0);
        m_tft.setTextSize(normalTextSize);
        m_tft.setTextColor(TFT_BLACK);
        m_tft.print(pCurrText);

        // Write next mode text;
        m_tft.setCursor(0, 0);
        m_tft.setTextSize(normalTextSize);
        m_tft.setTextColor(newColor);
        m_tft.print(pNewText);

        m_blinkManual = isManualMode;
    }
}

void Screen::drawRemoteIcon(bool isRemoteConnected)
{
    const uint8_t bluetoothIcon[] = { 0x0c, 0x00, // 00001100 00000000
                                        0x0a, 0x00, // 00001010 00000000
                                        0x49, 0x00, // 01001001 00000000
                                        0x28, 0x80, // 00101000 10000000
                                        0x19, 0x00, // 00011001 00000000
                                        0x0a, 0x00, // 00001010 00000000
                                        0x0c, 0x00, // 00001100 00000000
                                        0x0a, 0x00, // 00001010 00000000
                                        0x19, 0x00, // 00011001 00000000
                                        0x28, 0x80, // 00101000 10000000
                                        0x49, 0x00, // 01001001 00000000
                                        0x0a, 0x00, // 00001010 00000000
                                        0x0c, 0x00};// 00001100 00000000

    if (m_blinkBle)
    {
        // Turn the BLE icon on at the beginning of a 1 second interval
        //      - and -
        // Turn it off 1/2 of the way through the 1 second interval.
        uint32_t currentTicks = app_timer_cnt_get();
        uint32_t tickThreshold = m_ticksPerSecond / 2;
        uint32_t elapsedTicks;
        app_timer_cnt_diff_compute(currentTicks, m_lastBlinkStartTicks, &elapsedTicks);

        uint16_t color = TFT_WHITE;
        if (!m_blinkBleState && elapsedTicks < tickThreshold)
        {
            color = TFT_BLUE;
            m_blinkBleState = true;
        }
        else if (m_blinkBleState && elapsedTicks >= tickThreshold)
        {
            color = TFT_BLACK;
            m_blinkBleState = false;
        }

        if (color != TFT_WHITE)
        {
            // Set BLE icon to blue or black.
            m_tft.drawBitmap(SCREEN_WIDTH - 5.5*normalTextSize*charWidth, 0, bluetoothIcon, 16, 13, color);
        }
    }

    if (isRemoteConnected != m_currentState.isRemoteConnected)
    {
        // Remote connection state has changed so update it on the screen.
        uint16_t newColor = TFT_BLUE;
        if (!isRemoteConnected)
        {
            newColor = TFT_BLACK;
            // Erase last remote battery voltage displayed when remote it no longer connected.
            drawBatteryVoltage(SCREEN_WIDTH - 4*normalTextSize*charWidth, 0, normalTextSize, TFT_BLACK, m_currentState.remoteBattery);
        }
        else
        {
            // Force battery level to be displayed below by making the old and new voltages mismatch.
            m_currentState.remoteBattery = 0xFF;
        }

        m_tft.drawBitmap(SCREEN_WIDTH - 5.5*normalTextSize*charWidth, 0, bluetoothIcon, 16, 13, newColor);

        m_blinkBle = !isRemoteConnected;
        m_blinkBleState = m_blinkBle;
    }
}

void Screen::drawRemoteVoltage(bool isRemoteConnected, uint8_t remoteBattery)
{
    if (isRemoteConnected && remoteBattery != m_currentState.remoteBattery)
    {
        // When remote is connected, draw when remote's battery voltage has changed.
        uint16_t batteryColor = TFT_GREEN;
        if (remoteBattery <= REMOTE_LOW_BATTERY)
        {
            batteryColor = TFT_RED;
        }
        // Erase old remote battery voltage before displaying new voltage.
        drawBatteryVoltage(SCREEN_WIDTH - 4*normalTextSize*charWidth, 0, normalTextSize, TFT_BLACK, m_currentState.remoteBattery);
        drawBatteryVoltage(SCREEN_WIDTH - 4*normalTextSize*charWidth, 0, normalTextSize, batteryColor, remoteBattery);
    }
}

void Screen::drawMotorIcon(bool areMotorsEnabled)
{
    if (areMotorsEnabled != m_currentState.areMotorsEnabled)
    {
        // Dead man switch state has changed so update its icon.
        const uint8_t motorIcon[] = { 0x0f, 0xfc, // 00001111 11111100
                                      0x14, 0x02, // 00010100 00000010
                                      0x24, 0x01, // 00100100 00000001
                                      0x27, 0xc1, // 00100111 11000001
                                      0x24, 0x01, // 00100100 00000001
                                      0xe7, 0xc1, // 11100111 11000001
                                      0x24, 0x01, // 00100100 00000001
                                      0x27, 0xc1, // 00100111 11000001
                                      0x24, 0x01, // 00100100 00000001
                                      0x14, 0x02, // 00010100 00000010
                                      0x0f, 0xfc};// 00001111 11111100
        uint16_t newColor;
        if (areMotorsEnabled)
        {
            newColor = TFT_WHITE;
        }
        else
        {
            newColor = TFT_BLACK;
        }
        m_tft.drawBitmap(SCREEN_WIDTH - 3*normalTextSize*charWidth, normalTextSize*charHeight + 2, motorIcon, 16, 11, newColor);
    }
}

void Screen::drawRobotVoltage(uint8_t robotBattery)
{
    if (robotBattery != m_currentState.robotBattery)
    {
        // Robot's battery voltage has changed so update it.
        uint16_t batteryColor = TFT_GREEN;
        if (robotBattery <= ROBOT_ERROR_BATTERY)
        {
            batteryColor = TFT_RED;
        }
        else if (robotBattery <= ROBOT_WARN_BATTERY)
        {
            batteryColor = TFT_ORANGE;
        }
        // Erase old robot battery voltage before displaying new voltage.
        drawBatteryVoltage(SCREEN_WIDTH/2 - 2*tallTextSize*charWidth, 0, tallTextSize, TFT_BLACK, m_currentState.robotBattery);
        drawBatteryVoltage(SCREEN_WIDTH/2 - 2*tallTextSize*charWidth, 0, tallTextSize, batteryColor, robotBattery);
    }
}

void Screen::drawBatteryVoltage(uint16_t x, uint16_t y, uint8_t size, uint16_t color, uint8_t voltage)
{
    m_tft.setCursor(x, y);
    m_tft.setTextColor(color);
    m_tft.setTextSize(size);

    char buffer[6];
    snprintf(buffer, sizeof(buffer), "%u.%uV", voltage / 10, voltage % 10);
    m_tft.print(buffer);
}

void Screen::updateText(const char* pText)
{
    // Truncate the string to fit on this screen so that it doesn't overflow the buffers.
    size_t textLength = strlen(pText);
    if (textLength > TEXT_LINE_LENGTH)
    {
        textLength = TEXT_LINE_LENGTH;
    }

    if (SCREEN_ORIENTATION == 0)
    {
        // Can use hardware vertical scrolling when in this orientation.
        // Roll the topmost line of text to the bottom of the text window.
        m_tft.setScrollArea(scrollingTopMargin, 320 - (scrollingTopMargin + TEXT_LINES * TEXT_SIZE * charHeight));
        m_tft.setScroll(scrollingTopMargin + (m_currentTextLine+1) * TEXT_SIZE * charHeight);

        // Erase the bottommost line of text.
        eraseTextLine(m_currentTextLine);
        // Replace with new text.
        memcpy(m_text[m_currentTextLine], pText, textLength);
        m_text[m_currentTextLine][textLength] = '\0';
        drawTextLine(m_currentTextLine, TFT_WHITE);

        m_currentTextLine++;
        if (m_currentTextLine == TEXT_LINES)
        {
            m_currentTextLine = 0;
        }
    }
    else
    {
        // Scroll all of the text up by one line, making room for the latest text at the bottom of the screen.
        for (uint8_t i = 0 ; i < TEXT_LINES ; i++)
        {
            // Erase what is currently on this line.
            eraseTextLine(i);

            // Draw the next text to go on this line.
            if (i < TEXT_LINES - 1)
            {
                strcpy(m_text[i], m_text[i+1]);
            }
            else
            {
                memcpy(m_text[i], pText, textLength);
                m_text[i][textLength] = '\0';
            }
            drawTextLine(i, TFT_WHITE);
        }
    }
}

void Screen::eraseTextLine(uint8_t line)
{
    size_t lineLength = strlen(m_text[line]);
    m_tft.fillRect(0, line * TEXT_SIZE * charHeight + scrollingTopMargin,
                   lineLength * TEXT_SIZE * charWidth, TEXT_SIZE * charHeight,
                   TFT_BLACK);
}

void Screen::drawTextLine(uint8_t line, uint16_t color)
{
    m_tft.setCursor(0, line * TEXT_SIZE * charHeight + scrollingTopMargin);
    m_tft.setTextColor(color);
    m_tft.setTextSize(TEXT_SIZE);
    m_tft.print(m_text[line]);
}
