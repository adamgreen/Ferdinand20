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
#include <SawppyDrive.h>


// Set to non-zero to have SawppyDrive object log the servo settings on each update to GDB.
#define LOG_SAWPPY_DRIVE 0

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

enum RoverState state = driving;
uint32_t stateChangeStart; // track time in transition for enterX states
#define WAIT_FOR_STOP 600 // number of milliseconds from stateChangeStart to ensure we're stopped.
#define WAIT_TRANSITION 1000 // number of milliseconds from stateChangeStart for a transition. Should be greater than WAIT_FOR_STOP
Timer g_timer;


// Function Prototypes
static void setup();
static void loop();
static int32_t getSelection(const char* pDisplay, int32_t min, int32_t max);


int main(void)
{
    g_haveGlobalConstructorsRun = true;
    setup();
    while (1)
    {
        loop();
    }
}

// Runs once upon startup.
static void setup()
{
    // Start in driving state
    state = driving;
}

// Runs regularly as long as there is power to microcontroller.
static void loop()
{
    int32_t steering = 0; // -100 to 100, retrieved by JoyDrive::getSteering()
    int32_t velocity = 0; // -100 to 100, retrieved by JoyDrive::getVelocity()
    int32_t duration = 0;

    // UNDONE: Save joystick code for later.
    bool triggerStateChange = false;
#ifdef UNDONE
    float invert = 1.0f; // Multiplier for implementing RoverWheel.rollServoInverted
    bool triggerStateChange = (digitalRead(INPLACE_BUTTON) == LOW); // Read button that commands turn-in-place mode.
    delay(100);
    steering = jd.getSteering(); // Read steering potentiometer
    delay(100);
    velocity = jd.getVelocity(); // Read velocity potentiometer
#endif // UNDONE

    // Prompt user for next motion to execute.
    printf("\n");
    printf("1. Drive\n");
    printf("2. Turn In Place\n");
    int32_t selection = getSelection("Selection", 1, 2);
    switch (selection)
    {
        case 1:
            steering = getSelection("Steering (%)", -100, 100);
            velocity = getSelection("Velocity (%)", -100, 100);
            duration = getSelection("Duration (msec)", 0, 60000);
            state = driving;
            break;
        case 2:
            steering = getSelection("Steering (%)", -100, 100);
            duration = getSelection("Duration (msec)", 0, 60000);
            state = turnInPlace;
            break;
    }

    uint32_t currTime = g_timer.read_ms();
    // State changes can only be triggered when going slowly or stopped
    if (triggerStateChange && abs(velocity) < 10 && abs(steering) < 10)
    {
        if (state == driving)
        {
            // If we were slowed, command a stop now, and start our transition timer.
            velocity = 0;

            stateChangeStart = currTime;

            // UNDONE: Change code around once have joystick.
            wait_ms(WAIT_TRANSITION);
            state = turnInPlace;
            // UNDONE: state = enterTurnInPlace;
        }
        else if (state == turnInPlace)
        {
            // If we were slowed, command a stop now, and start our transition timer.
            velocity = 0;

            stateChangeStart = currTime;

            // UNDONE: Change code around once have joystick.
            wait_ms(WAIT_TRANSITION);
            state = driving;
            // UNDONE: state = enterDriving;
        }
        g_drive.stopAll();
    }

    uint32_t elapsedTime = currTime - stateChangeStart;
    if (state == enterTurnInPlace)
    {
        velocity = 0;
        if (elapsedTime > WAIT_TRANSITION && !triggerStateChange)
        {
            // We've waited long enough for everything to settle
            // and button has been released.
            state = turnInPlace;
        }
        else if(elapsedTime > WAIT_FOR_STOP)
        {
            // We should be at full stop now, turn corner wheels.
            g_drive.turnInPlace(0);
        }
    }

    if (state == enterDriving)
    {
        velocity = 0;
        if (elapsedTime > WAIT_TRANSITION && !triggerStateChange)
        {
            // We've waited long enough for everything to settle
            // and button has been released.
            state = driving;
        }
        else if(elapsedTime > WAIT_FOR_STOP)
        {
            // We should be at full stop now, turn corner wheels to desired steering angle.
            g_drive.drive(0, 0);
        }
    }

    if (state == turnInPlace)
    {
        g_drive.turnInPlace(steering);
    }

    if (state == driving)
    {
        g_drive.drive(steering, velocity);
    }

    wait_ms(duration);
    // UNDONE: Don't want this stop for joystick mode.
    g_drive.stopAll();
}

static int32_t getSelection(const char* pDisplay, int32_t min, int32_t max)
{
    while (true)
    {
        printf("%s [%ld to %ld]: ", pDisplay, min, max);

        char lineBuffer[16];
        char* pResult = fgets(lineBuffer, sizeof(lineBuffer), stdin);
        if (pResult == NULL)
        {
            printf("error: Invalid selection.\n");
            continue;
        }
        long selection = strtol(lineBuffer, NULL, 10);
        if (selection < min || selection > max)
        {
            printf("error: Invalid selection.\n");
            continue;
        }
        return selection;
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
