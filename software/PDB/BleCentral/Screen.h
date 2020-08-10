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
#ifndef SCREEN_H_
#define SCREEN_H_

#include <Adafruit_GFX.h>
#include <Arduino_ST7789_Fast.h>



class Screen
{
    public:
        Screen(uint32_t ticksPerSecond,
               uint8_t mosiPin, uint8_t sckPin, uint8_t csPin, uint8_t dcPin,
               uint8_t rstPin = NRF_DRV_SPI_PIN_NOT_USED);

        struct PdbState
        {
            // Is the PDB set to run in manual or auto mode?
            bool    isManualMode;
            // Is remote connected over BLE?
            bool    isRemoteConnected;
            // Is dead man switch on remote control enaged to enable motors?
            bool    areMotorsEnabled;
            // Robot's battery level.
            //  Divide by 10 to get actual voltage range of 0.0V to 8.0V+.
            uint8_t robotBattery;
            // Remote control's battery level.
            //  0 to 36 - Divide by 10 to get actual voltage range of 0.0V to 3.6V.
            uint8_t remoteBattery;
        };

        void update(const PdbState* pState);
        void updateText(const char* pText);

        // Class specific constants for our screen dimensions.
        enum
        {
            SCREEN_WIDTH = 240,
            SCREEN_HEIGHT = 240
        };

    protected:
        // Size of characters to use for information scrolling text.
        enum
        {
            TEXT_SIZE = 2
        };

    public:
        // Maximum number of characters to be displayed on a line.
        enum
        {
            TEXT_LINE_LENGTH = 40 / TEXT_SIZE
        };

    protected:
        // Number of text lines that can be displayed on the screen at once.
        enum
        {
            TEXT_LINES = 24 / TEXT_SIZE
        };

        // Helper class for blinking indicators and text.
        class Blinker
        {
            public:
                Blinker(uint32_t offTime, uint16_t backgroundColor, uint16_t foregroundColor);

                void calcColor(uint32_t time, bool* pColorChanged, uint16_t* pNewColor);
                void setForegroundColor(uint16_t foregroundColor) { m_foregroundColor = foregroundColor; }
                void enable() { m_isBlinking = true; m_isOn = true; }
                void disable() { m_isBlinking = false; }

            protected:
                uint32_t    m_offTime;
                uint16_t    m_backgroundColor;
                uint16_t    m_foregroundColor;
                bool        m_isOn;
                bool        m_isBlinking;
        };

        uint32_t        m_ticksPerSecond;
        uint32_t        m_lastBlinkStartTicks;
        Blinker         m_manualBlinker;
        Blinker         m_bleBlinker;
        Blinker         m_robotVoltageBlinker;
        Blinker         m_remoteVoltageBlinker;
        Arduino_ST7789  m_tft;
        PdbState        m_currentState;
        uint8_t         m_currentTextLine;
        char            m_text[TEXT_LINES][TEXT_LINE_LENGTH+1];

        void drawManualAutoMode(uint32_t time, bool isManualMode);
        void drawRemoteIcon(uint32_t time, bool isRemoteConnected);
        void drawRemoteVoltage(uint32_t time, bool isRemoteConnected, uint8_t remoteVoltage);
        void drawMotorIcon(bool areMotorsEnabled);
        void drawRobotVoltage(uint32_t time, uint8_t robotVoltage);
        void drawBatteryVoltage(uint16_t x, uint16_t y, uint8_t size, uint16_t color, uint8_t voltage);
        void eraseTextLine(uint8_t line);
        void drawTextLine(uint8_t line, uint16_t color);

        // NiMH voltage levels below this value in the remote control should be shown in red text.
        enum
        {
            // 20 = 2.0V
            REMOTE_LOW_BATTERY = 20
        };

        // 2S LiPo voltage levels that generate warning and error messages for main robot battery.
        enum
        {
            // Battery voltage will be displayed as orange text one it reaches this voltage.
            ROBOT_WARN_BATTERY = 70,    // 7.0V
            // Battery voltage will be display as red text once it reaches this voltage.
            ROBOT_ERROR_BATTERY = 67    // 6.7V
        };
};

#endif // SCREEN_H_
