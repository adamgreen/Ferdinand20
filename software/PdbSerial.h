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

// Bits that can be set in PdbSerialManualPacket::buttons.
//  Joystick stick is pressed.
#define PDBSERIAL_BUTTONS_JOYSTICK      (1 << 0)
//  Deadman switch is pressed.
#define PDBSERIAL_BUTTONS_DEADMAN       (1 << 1)


//  The PDB packets sent in both directions contain a 2 byte header.
typedef struct PdbSerialPacketHeader
{
    // This will always be set to PDBSERIAL_PACKET_SIGNATURE (0xA5).
    uint8_t signature;
    // This will be the length of the rest of the packet data.
    // This can be 0 if just a heartbeat being sent from PDB in auto mode or ACK being sent back to PDB.
    uint8_t length;
} PdbSerialPacketHeader;

// Packet sent from PDB when in manual driving mode.
typedef struct PdbSerialManualPacket
{
    // X coordinate of joystick: -512 to 511.
    int16_t x;
    // Y coordinate of joystick: -512 to 511.
    int16_t y;
    // Bitfield to indicate which buttons are being pressed. One of the PDBSERIAL_BUTTONS_* bits from above.
    uint8_t buttons;
} PdbSerialManualPacket;

#endif // PDB_SERIAL_H_
