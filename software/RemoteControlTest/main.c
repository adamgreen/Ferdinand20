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
/* This is a test program to run on macOS that verifies that the BleJoystick firmware is streaming the expected
   data over BLE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "bleuart.h"
#include "../../include/BleJoyShared.h"



// The Control+C handler and workerMain both need to access these globals.
static volatile int      g_controlCdetected;



static void controlCHandler(int arg);
static void dumpJoyData(const BleJoyData* pJoyData);



int main(int argc, char *argv[])
{
    /*
       Initialize the Core Bluetooth stack on this the main thread and start the worker robot thread to run the
       code found in workerMain() below.
    */
    bleuartInitAndRun();
    return 0;
}



void workerMain(void)
{
    int                   isBleConnected = 0;

    signal(SIGINT, controlCHandler);

    while (!g_controlCdetected)
    {
        int                result = -1;
        size_t             bytesRead;
        BleJoyData         joyData;

        if (!isBleConnected)
        {
            printf("Attempting to connect to BleJoystick device...\n");
            result = bleuartConnect(NULL);
            if (result)
            {
                fprintf(stderr, "error: Failed to connect to remote BleJoystick device.\n");
                goto Error;
            }
            printf("BleJoystick device connected!\n");
            isBleConnected = 1;
        }

        while (isBleConnected && !g_controlCdetected)
        {
            /* Forward data received from BLEUART to socket. */
            result = bleuartReceiveData(&joyData, sizeof(joyData), &bytesRead);
            if (result == BLEUART_ERROR_NOT_CONNECTED)
            {
                printf("BLE connection lost!\n");
                isBleConnected = 0;
                break;
            }
            else if (bytesRead == sizeof(joyData))
            {
                dumpJoyData(&joyData);
            }
            else if (bytesRead != 0)
            {
                printf("Received truncated BLE packet of %lu bytes!\n", bytesRead);
            }

            /* The joystick should only send packets every 16ms so we will sample at twice that rate for safety. */
            usleep(8000);
        }
    }

Error:
    printf("Shutting down\n");
    bleuartDisconnect();
}

static void controlCHandler(int arg)
{
    g_controlCdetected = 1;
}

static void dumpJoyData(const BleJoyData* pJoyData)
{
    printf("% 4d,% 4d  %s %s %.1fV\n",
           pJoyData->x, pJoyData->y,
           (pJoyData->buttons & BLEJOY_BUTTONS_JOYSTICK) ? "JOYSTICK_BUTTON" : "               ",
           (pJoyData->buttons & BLEJOY_BUTTONS_DEADMAN) ? "DEADMAN_SWITCH" : "              ",
           pJoyData->batteryVoltage / 10.0f);
}
