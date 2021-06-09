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
// 6 Wheel drive control for the Sawppy Rover.
// It is based off of Roger Cheng's original Sawppy Arduino sketch code:
//      https://github.com/Roger-random/Sawppy_Rover/blob/master/arduino_sawppy/arduino_sawppy.ino
#ifndef SAWPPY_DRIVE_H_
#define SAWPPY_DRIVE_H_

#include <LX16A.h>
#include <DriveMotor.h>

class SawppyDrive : protected LX16A_ServoBus
{
    public:
        // Construct the SawppyDrive object for controlling the 6 wheels of the rover.
        //  txPin - The pin used for the HalfDuplex UART transmission to the LX-16A servos.
        //  rxPin - The pin used for the HalfDuplex UART transmission from the LX-16A servos.
        //          NOTE: Both txPin and rxPin should be connected to the same data line of the LX-16A servos.
        //  pLogFormatted - An optional printf() like callback which can be used for verbose logging of the
        //                  servo commands for user inspection.
        SawppyDrive(PinName txPin, PinName rxPin, int (*pLogFormatted)(const char*, ...) = NULL);

        // UNDONE: Track the current drive state and delay transition as necessary.

        // Turn in place by rotating around the center of the robot.
        //  speed - Valid values are -100 to 100 percent.
        void turnInPlace(int32_t speed);

        // Drive forward/backward with the specified turn angle.
        //  steering - Valid values are -100 to 100 percent of the maximum turn angle.
        //  speed - Valid values are -100 to 100 percent of the maximum speed.
        void drive(int32_t steering, int32_t speed);

        // Stops all drive motion on Sawppy.
        void stopAll();

        // Reset the encoder/PID state of the 6 drive motors.
        void reset();

        // The Sawppy rover has 6 wheels.
        enum { WHEEL_COUNT = 6 };

    protected:
        static bool isRightWheel(size_t index);
        static bool isMiddleWheel(size_t index);

        void calculateWheelAnglesAndRadiiToTurnInPlace();
        void calculateWheelAnglesAndSpeedsForDrive(int32_t steering, int32_t speed);
        void setWheelAnglesAndSpeedsToDriveStraight(int32_t speed);
        void calculateWheelSpeedsFromRadii(int32_t maxSpeed);
        void sendCommandsToServos();

        int (*m_pLogFormatted)(const char*, ...);
        float m_maxTurnAngle;
        struct WheelUpdate
        {
            LX16A_Servo         steeringServo;
            LX16A_DriveMotor    rollServo;
            float               angle;
            float               speed;
            float               radius;
        } m_wheelUpdates[WHEEL_COUNT];
};

#endif // SAWPPY_DRIVE_H_
