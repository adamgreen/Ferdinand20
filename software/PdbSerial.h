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
/* This header contains the data structure used to transfer information between the Power Distribution Board and the
   main robot microcontroller.
*/
#ifndef PDB_SERIAL_H_
#define PDB_SERIAL_H_

#include <stdint.h>



// PdbSerialPacketHeader::signature byte will always be set to this value to indicate the start of packet.
#define PDBSERIAL_PACKET_SIGNATURE      0xA5

// Bits that can be set in PdbSerialManualPacket/PdbSerialAutoPacket flags field.
//  Joystick button is pressed.
#define PDBSERIAL_FLAGS_JOYSTICK_BUTTON     (1 << 0)
//  Deadman switch is pressed to enable motors.
#define PDBSERIAL_FLAGS_MOTORS_ENABLED      (1 << 1)
//  BLE remote control is connected.
#define PDBSERIAL_FLAGS_REMOTE_CONNECTED    (1 << 2)



//  The PDB packets sent in both directions contain a 2 byte header.
typedef struct PdbSerialPacketHeader
{
    // This will always be set to PDBSERIAL_PACKET_SIGNATURE (0xA5).
    uint8_t signature;
    // This will be the length of the rest of the packet data.
    uint8_t length;
} PdbSerialPacketHeader;

// Packet sent from PDB when in manual driving mode.
typedef struct PdbSerialManualPacket
{
    // X coordinate of joystick: -512 to 511.
    // Will be zero when PDBSERIAL_FLAGS_REMOTE_CONNECTED isn't set.
    int16_t x;
    // Y coordinate of joystick: -512 to 511.
    // Will be zero when PDBSERIAL_FLAGS_REMOTE_CONNECTED isn't set.
    int16_t y;
    // Battery voltage of robot's main LiPo battery. Divide by 10 to get actual voltage.
    uint8_t robotBattery;
    // Battery voltage of BLE remote control's battery. Divide by 10 to get actual voltage.
    // Will be zero when PDBSERIAL_FLAGS_REMOTE_CONNECTED isn't set.
    uint8_t remoteBattery;
    // Bitfield of flags. One of the PDBSERIAL_FLAGS_* bits from above.
    uint8_t flags;
} PdbSerialManualPacket;

// Packet sent from PDB when in auto mode.
typedef struct PdbSerialAutoPacket
{
    // Battery voltage levels. Divide by 10 to get actual voltage.
    uint8_t robotBattery;
    // Battery voltage of BLE remote control's battery. Divide by 10 to get actual voltage.
    // Will be zero when PDBSERIAL_FLAGS_REMOTE_CONNECTED isn't set.
    uint8_t remoteBattery;
    // Bitfield of flags. One of the PDBSERIAL_FLAGS_* bits from above.
    uint8_t flags;
} PdbSerialAutoPacket;


#endif // PDB_SERIAL_H_
