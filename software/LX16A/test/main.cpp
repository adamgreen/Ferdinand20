/*  Copyright (C) 2019  Adam Green (https://github.com/adamgreen)

    This program is free software; you can redistribute it and/or
    modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation; either version 2
    of the License, or (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*/
// Tests for the LewanSoul LX-16A Serial Bus Servo driver.
#include <LX16A.h>



LX16A_ServoBus g_servoBus(p9, p10);
LX16A_Servo    g_servo(g_servoBus, 1);



static int32_t getSelection(const char* pDisplay, int32_t min, int32_t max);
static void setRotationSpeed(int16_t speed, int32_t duration, bool readPosition);
static void displayPosition();
static void displayAngleOffset();
static void displayAngleLimits();
static void displayVoltageLimits();
static void displayTemperatureLimit();
static void displayTemperature();
static void displayVoltage();
static void displayMotorState();
static void displayLedState();
static void displayLedFaultMask();



int main()
{
    while (true)
    {
        printf("\n");
        printf("1. Discover Servo ID when only 1 servo is attached.\n");
        printf("2. Change active Servo Id.\n");
        printf("3. Change the ID on the active servo.\n");
        printf("4. Set rotation speed.\n");
        printf("5. Set rotation speed and read position.\n");
        printf("6. Set angle.\n");
        printf("7. Set angle for synchronized start.\n");
        printf("8. Broadcast synchronized start.\n");
        printf("9. Send rotation stop.\n");
        printf("10. Broadcast rotation stop.\n");
        printf("11. Disable motor.\n");
        printf("12. Adjust current angle offset.\n");
        printf("13. Persist current angle offset.\n");
        printf("14. Set angle limits.\n");
        printf("15. Set voltage limits.\n");
        printf("16. Set temperature limit.\n");
        printf("17. Modify LED settings.\n");
        printf("18. Set LED fault mask.\n");
        printf("19. Display servo settings.\n");
        int32_t selection = getSelection("Selection", 1, 19);
        switch (selection)
        {
            case 1:
            {
                uint8_t servoId = g_servoBus.discoverServoId();
                printf("  Servo ID = %u\n", servoId);
                break;
            }
            case 2:
            {
                int32_t servoId = getSelection("ServoId", 0, 253);
                g_servo.setServoId(servoId);
                break;
            }
            case 3:
            {
                int32_t newServoId = getSelection("New ServoId", 0, 253);
                printf("  Switching servo to Id #%ld\n", newServoId);
                g_servo.setIdOnServo(newServoId);
                g_servo.setServoId(newServoId);
                break;
            }
            case 4:
            case 5:
            {
                int32_t speed = getSelection("Speed", -1000, 1000);
                int32_t duration = getSelection("Duration (msec)", 0, 60000);
                setRotationSpeed(speed, duration, selection == 5);
                break;
            }
            case 6:
            {
                int32_t angle = getSelection("Angle", 0, 1000);
                int32_t duration = getSelection("Duration (msec)", 0, 30000);
                g_servo.setAngle(angle, duration);
                break;
            }
            case 7:
            {
                int32_t angle = getSelection("Angle", 0, 1000);
                int32_t duration = getSelection("Duration (msec)", 0, 30000);
                g_servo.queueSetAngle(angle, duration);
                break;
            }
            case 8:
            {
                g_servoBus.broadcastStart();
                printf("  Synchronized start sent!\n");
                break;
            }
            case 9:
            {
                g_servo.stop();
                printf("  Stop sent!\n");
                break;
            }
            case 10:
            {
                g_servoBus.broadcastStop();
                printf("  Synchronized stop sent!\n");
                break;
            }
            case 11:
            {
                g_servo.disableMotor();
                printf("  Motor disabled!\n");
                break;
            }
            case 12:
            {
                int32_t angle = getSelection("Angle offset", -125, 125);
                printf("  Setting servo offset to %ld (%.2f degrees)\n", angle, angle * 0.24f);
                g_servo.setAngleOffset(angle);
                break;
            }
            case 13:
            {
                g_servo.saveCurrentAngleOffset();
                printf("  Saving current angle offset in non-volatile memory of servo.\n");
                break;
            }
            case 14:
            {
                int32_t min = getSelection("Minimum Angle", 0, 1000);
                int32_t max = getSelection("Maximum Angle", min+1, 1000);
                printf("  Setting angle limits to %ld - %ld (%.2f - %.2f degrees)\n",
                        min, max, min * 0.24f, max * 0.24f);
                LX16A_ServoAngleLimits limits = { (uint16_t)min, (uint16_t)max };
                g_servo.setAngleLimits(limits);
                break;
            }
            case 15:
            {
                int32_t min = getSelection("Minimum Voltage", 4500, 12000);
                int32_t max = getSelection("Maximum Voltage", min+1, 12000);
                printf("  Setting voltage limits to %ld - %ld mV (%.3f - %.3f V)\n",
                        min, max, min / 1000.0f, max / 1000.0f);
                LX16A_ServoVoltageLimits limits = { (uint16_t)min, (uint16_t)max };
                g_servo.setVoltageLimits(limits);
                break;
            }
            case 16:
            {
                int32_t max = getSelection("Max Temperature", 50, 100);
                printf("  Setting temperature limit to %ldC\n", max);
                g_servo.setTemperatureLimit(max);
                break;
            }
            case 17:
            {
                int32_t setting = getSelection("LED Always On", 0, 1);
                printf("  Setting LED Always On to %ld\n", setting);
                g_servo.setLed(setting ? SERVO_LED_ALWAYS_ON : SERVO_LED_OFF);
                break;
            }
            case 18:
            {
                int32_t temperature = getSelection("Alarm on over temperature", 0, 1);
                int32_t voltage = getSelection("Alarm on over voltage", 0, 1);
                int32_t stall = getSelection("Alarm on motor stall", 0, 1);
                uint8_t mask = (stall << 2) | (voltage << 1) | temperature;
                printf("  Setting LED Fault Mask to %u\n", mask);
                g_servo.setLedFaultMask(mask);
                break;
            }
            case 19:
            {
                displayPosition();
                displayAngleOffset();
                displayAngleLimits();
                displayVoltage();
                displayVoltageLimits();
                displayTemperature();
                displayTemperatureLimit();
                displayMotorState();
                displayLedState();
                displayLedFaultMask();
                break;
            }
            default:
            {
                printf("error: Invalid selection\n");
                break;
            }
        }
    }
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

static void setRotationSpeed(int16_t speed, int32_t duration, bool readPosition)
{
    printf("  Spinning motor...\n");
    g_servo.setRotationSpeed(speed);

    if (readPosition)
    {
        Timer timer;
        timer.start();
        while (timer.read_ms() < duration)
        {
            printf("  ");
            displayPosition();
        }
    }
    else
    {
        wait_ms(duration);
    }

    printf("  Stopping motor!\n");
    g_servo.setRotationSpeed(0);
}

static void displayPosition()
{
    int16_t position = g_servo.getPosition();
    printf("  Position = %d (%.2f degrees)\n", position, position * 0.24f);
}

static void displayAngleOffset()
{
    int8_t angleOffset = g_servo.getAngleOffset();
    printf("  Angle offset = %d (%.2f degrees)\n", angleOffset, angleOffset * 0.24f);
}

static void displayAngleLimits()
{
    LX16A_ServoAngleLimits limits = g_servo.getAngleLimits();
    printf("  Min Angle = %d (%.2f degrees)\n", limits.minimum, limits.minimum * 0.24f);
    printf("  Max Angle = %d (%.2f degrees)\n", limits.maximum, limits.maximum * 0.24f);
}

static void displayVoltageLimits()
{
    LX16A_ServoVoltageLimits limits = g_servo.getVoltageLimits();
    printf("  Min Voltage = %dmV (%.3fV)\n", limits.minimum, limits.minimum / 1000.0f);
    printf("  Max Voltage = %dmV (%.3fV)\n", limits.maximum, limits.maximum / 1000.0f);
}

static void displayTemperatureLimit()
{
    uint8_t limit = g_servo.getTemperatureLimit();
    printf("  Max Temperature = %uC\n", limit);
}

static void displayTemperature()
{
    uint8_t temp = g_servo.getTemperature();
    printf("  Temperature = %uC\n", temp);
}

static void displayVoltage()
{
    uint16_t voltage = g_servo.getVoltage();
    printf("  Voltage = %umV (%.3fV)\n", voltage, voltage / 1000.0f);
}

static void displayMotorState()
{
    uint8_t motor = g_servo.isMotorEnabled();
    printf("  Motor On/Off = %u\n", motor);
}

static void displayLedState()
{
    LX16A_ServoLedSettings led = g_servo.getLedSetting();
    printf("  LED Setting = %s\n", led == SERVO_LED_ALWAYS_ON ? "Always On" : "Off");
}

static void displayLedFaultMask()
{
    uint8_t faultMask = g_servo.getLedFaultMask();
    printf("  LED Fault on over temperature = %s\n", (faultMask & 1) ? "Enabled" : "Disabled");
    printf("  LED Fault on over voltage = %s\n", (faultMask & 2) ? "Enabled" : "Disabled");
    printf("  LED Fault on stall = %s\n", (faultMask & 4) ? "Enabled" : "Disabled");
}
