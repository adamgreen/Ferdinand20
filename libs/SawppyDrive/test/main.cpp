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
// Test program to allow manual control of the Sawppy rover.
// It is based off of Roger Cheng's original Sawppy Arduino sketch code:
//      https://github.com/Roger-random/Sawppy_Rover/blob/master/arduino_sawppy/arduino_sawppy.ino
#include <mbed.h>
#include <stdarg.h>
#include <DebugSerial.h>
#include <PdbSerial.h>
#include <SawppyDrive.h>


// Set to non-zero to have SawppyDrive object log the servo settings on each update to GDB.
#define LOG_SAWPPY_DRIVE 0

// Number of milliseconds from g_stateChageStartTime to ensure we're stopped.
#define WAIT_FOR_STOP 600
// Number of milliseconds from g_stateChageStartTime for a transition. Should be greater than WAIT_FOR_STOP.
#define WAIT_TRANSITION 1000



// This is the DebugSerial object which can asynchronously log text to GDB without blocking the CPU like printf() does.
static DebugSerial<256> g_debugSerial;

#if LOG_SAWPPY_DRIVE
    static int DebugSerialPrintF(const char* pFormat, ...)
    {
        va_list argList;
        char    buffer[128];

        va_start(argList, pFormat);
            vsnprintf(buffer, sizeof(buffer), pFormat, argList);
            g_debugSerial.outputStringToGdb(buffer);
        va_end(argList);
    }
    static SawppyDrive      g_drive(p9, p10, DebugSerialPrintF);
#else
    static SawppyDrive      g_drive(p9, p10);
#endif // LOG_SAWPPY_DRIVE

static bool             g_haveGlobalConstructorsRun = false;


// Enumeration tracking current state of rover control
enum RoverState { driving, enterTurnInPlace, turnInPlace, enterDriving };

enum RoverState g_state = driving;
uint32_t        g_stateChageStartTime;
Timer           g_timer;
PdbSerial       g_pdb(p13, p14);
uint32_t        g_lastPacketTimestamp = 0;



// Function Prototypes
static void setup();
static void loop();


int main(void)
{
    g_haveGlobalConstructorsRun = true;
    setup();
    while (true)
    {
        loop();
    }
}

// Runs once upon startup.
static void setup()
{
    g_timer.start();
    g_state = driving;
}

// Runs regularly as long as there is power to microcontroller.
static void loop()
{
    PdbSerial::PdbSerialPacket pdbPacket = g_pdb.getLatestPacket();
    if (pdbPacket.timestamp == g_lastPacketTimestamp ||
        pdbPacket.type == PdbSerial::PDBSERIAL_AUTO_PACKET ||
        (pdbPacket.data._manual.flags & PDBSERIAL_FLAGS_MOTORS_ENABLED) == 0)
    {
        // Haven't received a new manual packet with the deadman switch pressed so ignore this packet.
        return;
    }
    g_lastPacketTimestamp = pdbPacket.timestamp;

    PdbSerialManualPacket* pManual = &pdbPacket.data._manual;
    int32_t steering = ((int32_t)pManual->x * 100) / 512;
    int32_t velocity = ((int32_t)pManual->y * 100) / 512;
    bool    isJoystickPressed = pManual->flags & PDBSERIAL_FLAGS_JOYSTICK_BUTTON;

    uint32_t currTime = g_timer.read_ms();
    // State changes can only be triggered when going slowly or stopped.
    if (isJoystickPressed && abs(velocity) < 10 && abs(steering) < 10)
    {
        if (g_state == driving)
        {
            // If we were slowed, command a stop now, and start our transition timer.
            velocity = 0;
            g_stateChageStartTime = currTime;
            g_state = enterTurnInPlace;
            g_pdb.outputStringToPDB("TurnInPlace...");
        }
        else if (g_state == turnInPlace)
        {
            // If we were slowed, command a stop now, and start our transition timer.
            velocity = 0;
            g_stateChageStartTime = currTime;
            g_state = enterDriving;
            g_pdb.outputStringToPDB("Drive...");
        }
        g_drive.stopAll();
    }

    uint32_t elapsedTime = currTime - g_stateChageStartTime;
    if (g_state == enterTurnInPlace)
    {
        velocity = 0;
        if (elapsedTime > WAIT_TRANSITION && !isJoystickPressed)
        {
            // We've waited long enough for everything to settle
            // and button has been released.
            g_state = turnInPlace;
            g_pdb.outputStringToPDB("TurningInPlace");
        }
        else if(elapsedTime > WAIT_FOR_STOP)
        {
            // We should be at full stop now, turn corner wheels.
            g_drive.turnInPlace(0);
        }
    }

    if (g_state == enterDriving)
    {
        velocity = 0;
        if (elapsedTime > WAIT_TRANSITION && !isJoystickPressed)
        {
            // We've waited long enough for everything to settle
            // and button has been released.
            g_state = driving;
            g_pdb.outputStringToPDB("Driving");
        }
        else if(elapsedTime > WAIT_FOR_STOP)
        {
            // We should be at full stop now, turn corner wheels to desired steering angle.
            g_drive.drive(0, 0);
        }
    }

    if (g_state == turnInPlace)
    {
        g_drive.turnInPlace(steering);
    }

    if (g_state == driving)
    {
        g_drive.drive(steering, velocity);
    }
}



// These are hooks for the MRI debug monitor to make sure that the motors are stopped when halting into the debugger
// so that the rover doesn't continue to run off when it hits a breakpoint or crashes.
extern "C" void __mriPlatform_EnteringDebuggerHook(void)
{
    // Don't do anything until the global constructors have been executed to setup the servos and debugger serial port.
    if (!g_haveGlobalConstructorsRun)
    {
        return;
    }
    // Allow any outbound DMA traffic on debug serial port to complete before passing control to MRI.
    g_debugSerial.flush();

    g_drive.stopAll();
}

extern "C" void __mriPlatform_LeavingDebuggerHook(void)
{
    // Don't do anything until the global constructors have been executed to setup the servos and debugger serial port.
    if (!g_haveGlobalConstructorsRun)
    {
        return;
    }

    // Just let the main loop recover and startup the motors again.

    // Clear any interrupts that might have been marked as pending against the debug serial port while MRI was
    // communicating with GDB.
    g_debugSerial.clearPendingInterrupt();
}
