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
// Test program to echo serial data received from Power Distribution Board to stdout.
#include <mbed.h>
#include "../PdbSerial.h"


RawSerial g_pc(USBTX, USBRX);
RawSerial g_pdb(p13, p14);


static void waitForStartOfPacket();
static uint8_t getPacketLength();
static void readPacket(void* pvPacket, size_t packetLength);
static void sendPacket(const void* pvPacket, size_t length);
static void sendBytes(const void* pvData, size_t length);


int main(void)
{
    uint32_t iteration = 0;

    g_pc.baud(230400);
    g_pdb.baud(115200);

    while (true)
    {
        waitForStartOfPacket();
        uint8_t packetLength = getPacketLength();
        switch (packetLength)
        {
            case 0:
                g_pc.printf("Auto mode heartbeat\r\n");
                break;
            case sizeof(PdbSerialManualPacket):
            {
                PdbSerialManualPacket manualPacket;
                readPacket(&manualPacket, sizeof(manualPacket));
                g_pc.printf("% 4d,% 4d %s %s\r\n",
                            manualPacket.x, manualPacket.y,
                            (manualPacket.buttons & PDBSERIAL_BUTTONS_JOYSTICK) ? "JOY" : "   ",
                            (manualPacket.buttons & PDBSERIAL_BUTTONS_DEADMAN) ? "DEADMAN" : "       ");
                break;
            }
            default:
                g_pc.printf("Unknown packet of length %u.\r\n", packetLength);
                break;
        }

        if ((iteration & 1) == 0)
        {
            // Just ack the even packets.
            sendPacket(NULL, 0);
        }
        else
        {
            // The odd packets will send back a string to display.
            char buffer[256];
            int length = snprintf(buffer, sizeof(buffer), "Test #%lu", iteration);
            sendPacket(buffer, length);
        }
        iteration++;
    }

    return 0;
}

static void waitForStartOfPacket()
{
    while (g_pdb.getc() != PDBSERIAL_PACKET_SIGNATURE)
    {
    }
}

static uint8_t getPacketLength()
{
    return g_pdb.getc();
}

static void readPacket(void* pvPacket, size_t packetLength)
{
    uint8_t* pPacket = (uint8_t*)pvPacket;
    while (packetLength-- > 0)
    {
        *pPacket++ = g_pdb.getc();
    }
}

static void sendPacket(const void* pvPacket, size_t length)
{
    PdbSerialPacketHeader header =
    {
        .signature = PDBSERIAL_PACKET_SIGNATURE,
        .length = length
    };

    sendBytes(&header, sizeof(header));
    sendBytes(pvPacket, length);
}

static void sendBytes(const void* pvData, size_t length)
{
    const uint8_t* pData = (const uint8_t*)pvData;
    while (length-- > 0)
    {
        g_pdb.putc(*pData++);
    }
}
