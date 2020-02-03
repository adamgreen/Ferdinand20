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
/* This header contains the information about the BleJoy firmware that needs to be shared with the
   Central code that connects to it.
*/
#ifndef BLE_JOY_SHARED_H_
#define BLE_JOY_SHARED_H_

// This is the vendor UUID offset to be advertised by BLEJOY devices.
#define BLEJOY_ADVERTISE                0xADA4

// Bits that can be set in BleJoyData::buttons.
//  Joystick stick is pressed.
#define BLEJOY_BUTTONS_JOYSTICK         (1 << 0)
//  Deadman switch is pressed.
#define BLEJOY_BUTTONS_DEADMAN          (1 << 1)


// This structure defines the data that is sent from the BLEJOY peripheral to the central. The Ferdinand20
// project will use the same little-endian Cortex-M devices on both ends so the data will be provided as raw
// little endian data in this structure.
typedef struct BleJoyData
{
    // X coordinate of joystick: -512 to 511.
    int16_t x;
    // Y coordinate of joystick: -512 to 511.
    int16_t y;
    // Bitfield to indicate which buttons are being pressed. One of the BLEJOY_BUTTONS_* bits from above.
    uint8_t buttons;
    // Battery voltage: 0 to 36 - Divide by 10 to get actual voltage range of 0.0V to 3.6V.
    uint8_t batteryVoltage;
} BleJoyData;

#endif // BLE_JOY_SHARED_H_
