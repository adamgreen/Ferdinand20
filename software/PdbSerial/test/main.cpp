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
// Program to test serial connection to Power Distribution Board.
#include <mbed.h>
#include <PdbSerial.h>

RawSerial g_pc(USBTX, USBRX);
PdbSerial g_pdb(p13, p14);


int main(void)
{
    g_pc.baud(230400);

    g_pc.printf("Starting...\r\n");

    uint32_t iteration = 0;
    uint32_t lastPacketTimestamp = 0;
    while (true)
    {
        PdbSerial::PdbSerialPacket packet = g_pdb.getLatestPacket();
        if (packet.timestamp == lastPacketTimestamp)
        {
            continue;
        }
        lastPacketTimestamp = packet.timestamp;

        switch (packet.type)
        {
            case PdbSerial::PDBSERIAL_AUTO_PACKET:
                g_pc.printf("A %u", packet.data._auto.robotBattery);
                if (packet.data._auto.flags & PDBSERIAL_FLAGS_REMOTE_CONNECTED)
                {
                    g_pc.printf(" %u R%s",
                                packet.data._auto.remoteBattery,
                                (packet.data._auto.flags & PDBSERIAL_FLAGS_MOTORS_ENABLED)  ? "M" : " ");
                }
                g_pc.printf("\r\n");
                break;
            case PdbSerial::PDBSERIAL_MANUAL_PACKET:
                g_pc.printf("M %u", packet.data._manual.robotBattery);
                if (packet.data._manual.flags & PDBSERIAL_FLAGS_REMOTE_CONNECTED)
                {
                    g_pc.printf(" %u R%s%s %d,%d",
                                packet.data._manual.remoteBattery,
                                (packet.data._manual.flags & PDBSERIAL_FLAGS_MOTORS_ENABLED)  ? "M" : " ",
                                (packet.data._manual.flags & PDBSERIAL_FLAGS_JOYSTICK_BUTTON) ? "J" : " ",
                                 packet.data._manual.x, packet.data._manual.y);
                }
                g_pc.printf("\r\n");
                break;
            default:
                g_pc.printf("Unknown packet type.\r\n");
                break;
        }

        if ((iteration & 0x7) == 0)
        {
            // Only send back some text on every 16th frame.
            g_pdb.printf("Test #%lu", iteration);
        }
        iteration++;
    }

    return 0;
}
