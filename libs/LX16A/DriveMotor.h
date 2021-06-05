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
// Library to apply closed loop control of the LX-16A servo.
#ifndef DRIVE_MOTOR_H_
#define DRIVE_MOTOR_H_

#include <LX16A.h>
#include <PID.h>


struct EncoderState
{
    float angle;
    float velocity;
};

class LX16A_DriveMotor
{
    public:
        LX16A_DriveMotor(LX16A_ServoBus& servoBus, uint8_t servoId);

        void reset();
        void setServoId(uint8_t servoId);
        void setSpeed(float radiansPerSec);
        void setPower(int16_t speed);
        int16_t getPower();
        EncoderState getState();

    protected:
        void resetWithoutServo();
        void updateState();
        void updatePID(float dt);
        bool ignoringNonDeadZoneSamples(int16_t servoPos);
        static bool isServoInDeadZone(int16_t servoPos);
        static float constrainAngle(float angle);

        LX16A_ServoBus& m_servoBus;
        LX16A_Servo     m_servo;
        PID             m_pid;
        Timer           m_timer;
        uint32_t        m_lastSampleTime;
        uint32_t        m_ignoredSamples;
        EncoderState    m_currState;
        int16_t         m_lastControlOutput;
        bool            m_wasInDeadZone;
};

#endif // DRIVE_MOTOR_H_
