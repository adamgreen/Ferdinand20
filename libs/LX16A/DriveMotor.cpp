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
#include <DriveMotor.h>
#include <assert.h>

// PID parameters for controlling the LX16A servos in continous rotation mode.
#define PID_Kc  200.0f
#define PID_Ti  0.2909f
#define PID_Td  0.01332f

// The latest velocity measurement gets this weight while the rest get 1 minus this value.
#define VELOCITY_AVERAGING_WEIGHT 0.05f

// The number of non-deadzone samples required to snap the filter out of interpolation mode.
// This causes the filter to ignore the random samples that occur when the pot wiper if left floating.
#define SAMPLES_TO_IGNORE 1



LX16A_DriveMotor::LX16A_DriveMotor(LX16A_ServoBus& servoBus, uint8_t servoId) :
    m_servoBus(servoBus),
    m_servo(servoBus, servoId),
    m_pid(PID_Kc, PID_Ti, PID_Td, 0.0f, LX16A_ROTATION_SPEED_MIN, LX16A_ROTATION_SPEED_MAX, 0.010f)
{
    m_timer.start();
    resetWithoutServo();
}

void LX16A_DriveMotor::resetWithoutServo()
{
    m_lastSampleTime = 0;
    m_timer.reset();
    m_currState.angle = 0.0f;
    m_currState.velocity = 0.0f;
    m_ignoredSamples = 0;
    m_wasInDeadZone = false;
}

void LX16A_DriveMotor::reset()
{
    resetWithoutServo();
    m_currState.angle = constrainAngle(LX16A_SERVO_TO_RADIAN_VALUE * m_servo.getPosition());
    setPower(0);
}


float LX16A_DriveMotor::constrainAngle(float angle)
{
    float a = angle;
    if (angle < -(float)M_PI)
        a = angle + 2.0f*(float)M_PI;
    else if (angle > (float)M_PI)
        a = angle - 2.0f*(float)M_PI;

    // UNDONE: Need to handle more than +/- 2pi?
    assert ( a >= -(float)M_PI && a <= (float)M_PI );
    return a;
}

void LX16A_DriveMotor::setServoId(uint8_t servoId)
{
    m_servo.setServoId(servoId);
}

void LX16A_DriveMotor::setSpeed(float radiansPerSec)
{
    // Just force output to 0 if desired speed is 0 and disable PID loop to let wheels coast down to 0.
    if (radiansPerSec == 0.0f)
    {
        setPower(0);
        return;
    }

    // Switch out of manual motor speed mode if necessary.
    if (!m_pid.isAutomaticModeEnabled())
    {
        m_pid.setOutputManually(0.0f);
        m_servo.setRotationSpeed(0);
        m_lastControlOutput = 0;
        m_pid.enableAutomaticMode();
    }
    m_pid.updateSetPoint(radiansPerSec);
}

void LX16A_DriveMotor::setPower(int16_t speed)
{
    m_pid.setOutputManually(speed);
    updatePID(0.010f);
}

int16_t LX16A_DriveMotor::getPower()
{
    return (int16_t)(m_pid.getControlOutput() + 0.5f);
}

EncoderState LX16A_DriveMotor::getState()
{
    updateState();
    return m_currState;
}

void LX16A_DriveMotor::updateState()
{
    // Calculate time since last running of the filter and grab the current servo position in close temporal
    // relation to the time capture.
    int16_t servoPos = m_servo.getPosition();
    uint32_t currSampleTime = m_timer.read_us();
    uint32_t elapsedMicroSec = currSampleTime - m_lastSampleTime;
    m_lastSampleTime = currSampleTime;
    float dt = (float)elapsedMicroSec / 1000000.0f;

    EncoderState prevState = m_currState;
    if (ignoringNonDeadZoneSamples(servoPos) || isServoInDeadZone(servoPos))
    {
        // The servo position could be off by as much as +/-20 degrees when in dead zone of the pot.
        // Assume that the motor is still spinning at the same rate when in dead zone.
        m_currState.angle = constrainAngle(prevState.angle + prevState.velocity * dt);
        m_currState.velocity = prevState.velocity;
        m_wasInDeadZone = true;
    }
    else
    {
        // The pot reading should be accurate so use it.
        m_currState.angle = constrainAngle(servoPos * LX16A_SERVO_TO_RADIAN_VALUE);

        if (m_wasInDeadZone)
        {
            // Don't want to base current velocity off of previously interpolated position.
            m_currState.velocity = prevState.velocity;
        }
        else
        {
            // Calculate velocity from change in angle and then use weighted average so that it doesn't snap too much.
            const float weight = VELOCITY_AVERAGING_WEIGHT;
            float angleDelta = constrainAngle(fmodf(m_currState.angle - prevState.angle, 2.0f*(float)M_PI));
            float velocity = angleDelta / dt;
            m_currState.velocity = (1.0f-weight)*m_currState.velocity + weight*velocity;
        }
        m_wasInDeadZone = false;
    }

    updatePID(dt);
}

bool LX16A_DriveMotor::ignoringNonDeadZoneSamples(int16_t servoPos)
{
    if (isServoInDeadZone(servoPos))
    {
        m_ignoredSamples = 0;
        return false;
    }
    else if (m_ignoredSamples < SAMPLES_TO_IGNORE)
    {
        m_ignoredSamples++;
        return true;
    }
    return false;
}

bool LX16A_DriveMotor::isServoInDeadZone(int16_t servoPos)
{
    return (servoPos < (int16_t)(LX16A_DEGREE_TO_SERVO_VALUE * -40.0f) ||
            servoPos > (int16_t)(LX16A_DEGREE_TO_SERVO_VALUE * 279.0f));
}

void LX16A_DriveMotor::updatePID(float dt)
{
    m_pid.setSampleTime(dt);
    m_pid.compute(m_currState.velocity);
    int16_t newControlOutput = getPower();
    if (newControlOutput == m_lastControlOutput)
    {
        // Don't waste serial output time if the motor is already at the correct output power level.
        return;
    }

    m_servo.setRotationSpeed(newControlOutput);
    m_lastControlOutput = newControlOutput;
}
