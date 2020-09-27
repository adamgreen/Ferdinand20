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
#define SCREEN_ORIENTATION 0

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



Screen::Blinker::Blinker(uint32_t offTime, uint16_t foregroundColor, uint16_t backgroundColor) :
    m_offTime(offTime),
    m_backgroundColor(backgroundColor),
    m_foregroundColor(foregroundColor),
    m_isOn(true),
    m_isBlinking(false)
{
}

void Screen::Blinker::calcColor(uint32_t time, bool* pColorChanged, uint16_t* pNewColor)
{
    // Return early if blink isn't enabled.
    if (!m_isBlinking)
    {
        *pColorChanged = false;
        return;
    }

    bool isNowOn;
    if (time < m_offTime)
    {
        isNowOn = true;
    }
    else
    {
        isNowOn = false;
    }

    if (isNowOn == m_isOn)
    {
        // Let the caller know that the colour hasn't changed since the last call.
        *pColorChanged = false;
        return;
    }

    // Get here if the color has just changed.
    *pColorChanged = true;
    if (isNowOn)
    {
        *pNewColor = m_foregroundColor;
    }
    else
    {
        *pNewColor = m_backgroundColor;
    }
    m_isOn = isNowOn;
}



Screen::Screen(uint32_t ticksPerSecond,
               uint8_t mosiPin, uint8_t sckPin, uint8_t csPin, uint8_t dcPin, uint8_t rstPin) :
    m_ticksPerSecond(ticksPerSecond),
    m_manualBlinker((ticksPerSecond * 3) / 4, TFT_RED, TFT_BLACK),
    m_bleBlinker(ticksPerSecond / 2, TFT_BLUE, TFT_BLACK),
    m_robotVoltageBlinker(ticksPerSecond / 2, TFT_RED, TFT_BLACK),
    m_remoteVoltageBlinker(ticksPerSecond / 2, TFT_RED, TFT_BLACK),
    m_tft(SCREEN_WIDTH, SCREEN_HEIGHT, 0, 0, mosiPin, sckPin, dcPin, rstPin, csPin)
{
    m_lastBlinkStartTicks = 0;
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
    uint32_t currentTicks = app_timer_cnt_get();
    uint32_t elapsedTicks;
    app_timer_cnt_diff_compute(currentTicks, m_lastBlinkStartTicks, &elapsedTicks);

    drawManualAutoMode(elapsedTicks, pState->isManualMode);
    drawRemoteIcon(elapsedTicks, pState->isRemoteConnected);
    drawRemoteVoltage(elapsedTicks, pState->isRemoteConnected, pState->remoteBattery);
    drawMotorIcon(pState->areMotorsEnabled);
    drawRobotVoltage(elapsedTicks, pState->robotBattery);

    // Remember the new state.
    m_currentState = *pState;

    // See if we are ready to start a new 1-second blink interval.
    if (elapsedTicks >= m_ticksPerSecond)
    {
        m_lastBlinkStartTicks = currentTicks;
    }
}

void Screen::drawManualAutoMode(uint32_t time, bool isManualMode)
{
    bool     hasNewColor = false;
    uint16_t newColor = TFT_BLACK;
    m_manualBlinker.calcColor(time, &hasNewColor, &newColor);
    if (hasNewColor)
    {
        // Write "Manual" in red or black.
        m_tft.setCursor(0, 0);
        m_tft.setTextSize(normalTextSize);
        m_tft.setTextColor(newColor);
        m_tft.print("Manual");
    }

    if (isManualMode != m_currentState.isManualMode)
    {
        // Manual/Auto mode has changed so update it on the screen.
        const char* pCurrText;
        const char* pNewText;
        if (isManualMode)
        {
            pCurrText = "Auto";
            pNewText = "Manual";
            newColor = TFT_RED;
            m_manualBlinker.enable();
        }
        else
        {
            pCurrText = "Manual";
            pNewText = "Auto";
            newColor = TFT_WHITE;
            m_manualBlinker.disable();
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
    }
}

void Screen::drawRemoteIcon(uint32_t time, bool isRemoteConnected)
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

    bool     hasNewColor = false;
    uint16_t newColor = TFT_BLACK;
    m_bleBlinker.calcColor(time, &hasNewColor, &newColor);
    if (hasNewColor)
    {
        // Set BLE icon to blue or black.
        m_tft.drawBitmap(SCREEN_WIDTH - 5.5*normalTextSize*charWidth, 0, bluetoothIcon, 16, 13, newColor);
    }

    if (isRemoteConnected != m_currentState.isRemoteConnected)
    {
        // Remote connection state has changed so update it on the screen.
        if (!isRemoteConnected)
        {
            newColor = TFT_BLACK;
            // Erase last remote battery voltage displayed when remote it no longer connected.
            drawBatteryVoltage(SCREEN_WIDTH - 4*normalTextSize*charWidth, 0, normalTextSize, TFT_BLACK, m_currentState.remoteBattery);
            m_remoteVoltageBlinker.disable();
            m_bleBlinker.enable();
        }
        else
        {
            newColor = TFT_BLUE;
            // Force battery level to be displayed below by making the old and new voltages mismatch.
            m_currentState.remoteBattery = 0xFF;
            m_bleBlinker.disable();
        }

        m_tft.drawBitmap(SCREEN_WIDTH - 5.5*normalTextSize*charWidth, 0, bluetoothIcon, 16, 13, newColor);
    }
}

void Screen::drawRemoteVoltage(uint32_t time, bool isRemoteConnected, uint8_t remoteBattery)
{
    bool     hasNewColor = false;
    uint16_t newColor = TFT_BLACK;
    m_remoteVoltageBlinker.calcColor(time, &hasNewColor, &newColor);
    if (hasNewColor)
    {
        drawBatteryVoltage(SCREEN_WIDTH - 4*normalTextSize*charWidth, 0, normalTextSize, newColor, remoteBattery);
    }

    if (isRemoteConnected && remoteBattery != m_currentState.remoteBattery)
    {
        // When remote is connected, draw when remote's battery voltage has changed.
        uint16_t batteryColor;
        if (remoteBattery <= REMOTE_LOW_BATTERY)
        {
            batteryColor = TFT_RED;
            m_remoteVoltageBlinker.enable();
        }
        else
        {
            batteryColor = TFT_GREEN;
            m_remoteVoltageBlinker.disable();
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

void Screen::drawRobotVoltage(uint32_t time, uint8_t robotBattery)
{
    bool     hasNewColor = false;
    uint16_t newColor = TFT_BLACK;
    m_robotVoltageBlinker.calcColor(time, &hasNewColor, &newColor);
    if (hasNewColor)
    {
        drawBatteryVoltage(SCREEN_WIDTH/2 - 2*tallTextSize*charWidth, 0, tallTextSize, newColor, robotBattery);
    }

    if (robotBattery != m_currentState.robotBattery)
    {
        // Robot's battery voltage has changed so update it.
        uint16_t batteryColor = TFT_GREEN;
        if (robotBattery <= ROBOT_ERROR_BATTERY)
        {
            batteryColor = TFT_RED;
            m_robotVoltageBlinker.setForegroundColor(TFT_RED);
            m_robotVoltageBlinker.enable();
        }
        else if (robotBattery <= ROBOT_WARN_BATTERY)
        {
            batteryColor = TFT_ORANGE;
            m_robotVoltageBlinker.setForegroundColor(TFT_ORANGE);
            m_robotVoltageBlinker.enable();
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
