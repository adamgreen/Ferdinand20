/*  Copyright (C) 2021  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Test for using LX-16A potentiometer readings as encoder.
#include <ctype.h>
#include <DriveMotor.h>



enum StepState
{
    STARTING,
    HALF_SPEED,
    FULL_SPEED
};



static Timer            g_timer;
static LX16A_ServoBus   g_servoBus(p9, p10);
static LX16A_DriveMotor g_drive(g_servoBus, 1);
volatile char           g_cmd = '\0';
#if !MRI_ENABLE
static Serial           g_serial(USBTX, USBRX);
#endif

// Step response globals.
static StepState    g_stepState = STARTING;
static Timer        g_stepStateTimer;



static void displayState();
static void stepResponse();
static void nothing();
#if !MRI_ENABLE
static void serialReceiveISR();
#endif

static void (*g_pProcessRoutine)(void) = nothing;



int main()
{
#if !MRI_ENABLE
    g_serial.baud(230400);
    g_serial.attach(serialReceiveISR);
#endif // !MRI_ENABLE

    uint8_t servoId = g_servoBus.discoverServoId();
    printf("Servo ID = %u\n", servoId);
    g_drive.setServoId(servoId);
    g_drive.setPower(0);
    g_timer.start();
    g_stepStateTimer.start();

    while (true)
    {
        switch (tolower(g_cmd))
        {
            case 'i':
                printf("Idling...\n");
                g_drive.setPower(0);
                g_pProcessRoutine = nothing;
                g_cmd = '\0';
                break;
            case 's':
                printf("Stop...\n");
                g_drive.setSpeed(0.0f);
                g_pProcessRoutine = displayState;
                g_cmd = '\0';
                break;
            case 'f':
                printf("Forward...\n");
                g_timer.reset();
                g_drive.reset();
                g_drive.setSpeed(1.5f);
                g_pProcessRoutine = displayState;
                g_cmd = '\0';
                break;
            case 'r':
                printf("Reverse...\n");
                g_timer.reset();
                g_drive.reset();
                g_drive.setSpeed(-1.5f);
                g_pProcessRoutine = displayState;
                g_cmd = '\0';
                break;
            case 't':
                printf("Step response measurement...\n");
                g_stepState = STARTING;
                g_pProcessRoutine = stepResponse;
                g_cmd = '\0';
                break;
            case '\0':
                break;
            default:
                printf("'%c' is an unknown command.\n", g_cmd);
                g_pProcessRoutine = nothing;
                g_cmd = '\0';
            break;
        }

        g_pProcessRoutine();
    }
}

static void displayState()
{
#if !MRI_ENABLE
    // Add in a bit of delay if not already slown down by MRI debug output handling.
    wait_ms(5);
#endif
    EncoderState currState = g_drive.getState();
    uint32_t elapsedTime = g_timer.read_us();
    g_timer.reset();

    printf("%lu,%d,%f,%f\n",
        elapsedTime,
        g_drive.getPower(),
        currState.angle*LX16A_RADIAN_TO_DEGREE, currState.velocity);
}

static void stepResponse()
{
    switch (g_stepState)
    {
        case STARTING:
            printf("time,setpoint,angle,velocity\n");
            g_stepStateTimer.reset();
            g_drive.reset();
            g_drive.setPower(500);
            g_stepState = HALF_SPEED;
            break;
        case HALF_SPEED:
            if (g_stepStateTimer.read_ms() >= 3000)
            {
                g_stepStateTimer.reset();
                g_drive.setPower(1000);
                g_stepState = FULL_SPEED;
            }
            break;
        case FULL_SPEED:
            if (g_stepStateTimer.read_ms() >= 3000)
            {
                g_drive.setPower(0);
                g_pProcessRoutine = nothing;
            }
            break;
    }

    EncoderState currState = g_drive.getState();
    int16_t motorPower = g_drive.getPower();
    uint32_t currSampleTime = g_stepStateTimer.read_us();

    printf("%lu,%d,%f,%f\n",
        currSampleTime,
        motorPower,
        currState.angle, currState.velocity);
}

static void nothing()
{
}


#if !MRI_ENABLE
static void serialReceiveISR()
{
    while (g_serial.readable())
    {
        g_cmd = g_serial.getc();
    }
}
#endif // !MRI_ENABLE
