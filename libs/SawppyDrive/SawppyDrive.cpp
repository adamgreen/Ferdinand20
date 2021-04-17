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
// 6 Wheel drive control for the Sawppy Rover.
// It is based off of Roger Cheng's original Sawppy Arduino sketch code:
//      https://github.com/Roger-random/Sawppy_Rover/blob/master/arduino_sawppy/arduino_sawppy.ino
#include "SawppyDrive.h"


// Indices into the wheel arrays (m_wheelUpdates & g_dimensions).
#define WHEEL_FRONT_LEFT        0
#define WHEEL_FRONT_RIGHT       1
#define WHEEL_MIDDLE_LEFT       2
#define WHEEL_MIDDLE_RIGHT      3
#define WHEEL_REAR_LEFT         4
#define WHEEL_REAR_RIGHT        5


struct SawppyDimension
{
    // Dimensions are relative to the midpoint between the two middle wheels.
    float x;
    float y;
};

static const SawppyDimension g_dimensions[SawppyDrive::WHEEL_COUNT] =
{
  // WHEEL_FRONT_LEFT
  {
    -9.125f,
    11.375f
  },
  // WHEEL_FRONT_RIGHT
  {
     9.125f,
    11.375f
  },
  // WHEEL_MIDDLE_LEFT
  {
    -10.375f,
    0.0f
  },
  // WHEEL_MIDDLE_RIGHT
  {
    10.375f,
    0.0f
  },
  // WHEEL_REAR_LEFT
  {
    -9.0f,
    -10.0f
  },
  // WHEEL_REAR_RIGHT
  {
     9.0f,
    -10.0f,
  }
};


// The middle wheels don't have a steering servo so set to an unused servo id.
#define SERVOID_UNUSED              1

// The LX16A servo IDs for each of Sawppy's wheels.
//  Servos on left side.
#define SERVOID_FRONT_LEFT_STEER    11
#define SERVOID_FRONT_LEFT_ROLL     12
#define SERVOID_MIDDLE_LEFT_STEER   SERVOID_UNUSED
#define SERVOID_MIDDLE_LEFT_ROLL    13
#define SERVOID_REAR_LEFT_ROLL      14
#define SERVOID_REAR_LEFT_STEER     15
//  Servos on right side.
#define SERVOID_FRONT_RIGHT_STEER   91
#define SERVOID_FRONT_RIGHT_ROLL    92
#define SERVOID_MIDDLE_RIGHT_STEER  SERVOID_UNUSED
#define SERVOID_MIDDLE_RIGHT_ROLL   93
#define SERVOID_REAR_RIGHT_ROLL     94
#define SERVOID_REAR_RIGHT_STEER    95

SawppyDrive::SawppyDrive(PinName txPin, PinName rxPin, int (*pLogFormatted)(const char*, ...) /* = NULL */) :
    LX16A_ServoBus(txPin, rxPin),
    m_pLogFormatted(pLogFormatted),
    m_wheelUpdates
    {
        {
            LX16A_Servo(*this, SERVOID_FRONT_LEFT_STEER),   // steeringServo
            LX16A_Servo(*this, SERVOID_FRONT_LEFT_ROLL),    // rollServo
            0.0f,                                           // angle
            0.0f,                                           // speed
            0.0f                                            // radius
        },
        {
            LX16A_Servo(*this, SERVOID_FRONT_RIGHT_STEER),  // steeringServo
            LX16A_Servo(*this, SERVOID_FRONT_RIGHT_ROLL),   // rollServo
            0.0f,                                           // angle
            0.0f,                                           // speed
            0.0f                                            // radius
        },
        {
            LX16A_Servo(*this, SERVOID_MIDDLE_LEFT_STEER),  // steeringServo
            LX16A_Servo(*this, SERVOID_MIDDLE_LEFT_ROLL),   // rollServo
            0.0f,                                           // angle
            0.0f,                                           // speed
            0.0f                                            // radius
        },
        {
            LX16A_Servo(*this, SERVOID_MIDDLE_RIGHT_STEER), // steeringServo
            LX16A_Servo(*this, SERVOID_MIDDLE_RIGHT_ROLL),  // rollServo
            0.0f,                                           // angle
            0.0f,                                           // speed
            0.0f                                            // radius
        },
        {
            LX16A_Servo(*this, SERVOID_REAR_LEFT_STEER),    // steeringServo
            LX16A_Servo(*this, SERVOID_REAR_LEFT_ROLL),     // rollServo
            0.0f,                                           // angle
            0.0f,                                           // speed
            0.0f                                            // radius
        },
        {
            LX16A_Servo(*this, SERVOID_REAR_RIGHT_STEER),   // steeringServo
            LX16A_Servo(*this, SERVOID_REAR_RIGHT_ROLL),    // rollServo
            0.0f,                                           // angle
            0.0f,                                           // speed
            0.0f                                            // radius
        }
    }
{
    // Calculate maximum turning angle.
    // If the front left wheel is turned left to this angle, then the point of rotation will be the left middle wheel
    // itself. You can't turn any sharper than this.
    float adjacent = g_dimensions[WHEEL_MIDDLE_LEFT].x - g_dimensions[WHEEL_FRONT_LEFT].x;
    float opposite = g_dimensions[WHEEL_FRONT_LEFT].y;
    m_maxTurnAngle = (int32_t)fabsf(atanf(opposite/adjacent));
}

void SawppyDrive::turnInPlace(int32_t speed)
{
    calculateWheelAnglesAndRadiiToTurnInPlace();
    calculateWheelSpeedsFromRadii(speed);
    sendCommandsToServos();
}

void SawppyDrive::drive(int32_t steering, int32_t speed)
{
    calculateWheelAnglesAndSpeedsForDrive(steering, speed);
    sendCommandsToServos();
}

void SawppyDrive::calculateWheelAnglesAndRadiiToTurnInPlace()
{
    for (size_t wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        // Calculate angles to point at center of robot.
        if (isMiddleWheel(wheel))
        {
            // Can't turn middle wheels. Turning radius is its x coordinate.
            m_wheelUpdates[wheel].angle = 0.0f;
            m_wheelUpdates[wheel].radius = fabsf(g_dimensions[wheel].x);
        }
        else
        {
            // Calculate angle between this wheel and center of robot.
            m_wheelUpdates[wheel].angle = -atanf(g_dimensions[wheel].y / g_dimensions[wheel].x);
            // Use Pythagorean theorem to calculate distance between this wheel and center of robot. This is the
            // turning radius.
            m_wheelUpdates[wheel].radius = sqrtf(g_dimensions[wheel].x * g_dimensions[wheel].x +
                                                 g_dimensions[wheel].y * g_dimensions[wheel].y);
        }

        // If left wheels are turning forward then right wheels need to turn backward and vice versa to spin in place.
        if (isRightWheel(wheel))
        {
            m_wheelUpdates[wheel].radius = -m_wheelUpdates[wheel].radius;
        }
    }
}

void SawppyDrive::calculateWheelAnglesAndSpeedsForDrive(int32_t steering, int32_t speed)
{
    // It is pretty simple to calculate the wheel steering angles and rolling speeds when the desired steering angle
    // is 0 so special case it.
    if (steering == 0)
    {
        setWheelAnglesAndSpeedsToDriveStraight(speed);
        return;
    }

    // When not turning in place, Sawppy will turn about a point along a line which passes through both of the middle
    // wheels. The following code will choose a reference wheel and then calculate the point of rotation along that line.
    size_t referenceWheel;
    if (steering > 0)
    {
        referenceWheel = WHEEL_FRONT_RIGHT;
    }
    else
    {
        referenceWheel = WHEEL_FRONT_LEFT;
    }

    // Rover motion is based on angle of the reference wheel (right or left). The angle to turn this reference wheel
    // is determined as the specified percentage of the maximum turning angle calculated in the constructor.
    float angle = (float)(m_maxTurnAngle * steering) / 100.0f;
    m_wheelUpdates[referenceWheel].angle = angle;

    // Use this angle to calculate the X coordinate of the point of rotation.
    float pointOfRotation = g_dimensions[referenceWheel].x + (g_dimensions[referenceWheel].y / tanf(angle));

    // Calculate the hypotenuse which will give us the distance between the reference wheel and the point of rotation.
    // Record it as the turning radius for this wheel.
    m_wheelUpdates[referenceWheel].radius = fabsf(g_dimensions[referenceWheel].y / sinf(angle));

    // Calculate other wheel angles and turn radii based on the just calculated point of rotation.
    for (size_t wheel = 0 ; wheel < 6 ; wheel++)
    {
        // Can skip the refrence wheel since we already calculated it above.
        if (wheel != referenceWheel)
        {
            // The X-axis distance between this wheel and the point of rotation.
            float wheelToCenter = pointOfRotation - g_dimensions[wheel].x;

            if (isMiddleWheel(wheel))
            {
                // No steering servo, but good to blank out steering angle anyway.
                m_wheelUpdates[wheel].angle = 0.0f;

                // The point of rotation lies on the same line as the two middle wheels.
                m_wheelUpdates[wheel].radius = fabsf(wheelToCenter);
            }
            else
            {
                // Calculate wheel angle.
                angle = atanf(g_dimensions[wheel].y / wheelToCenter);
                m_wheelUpdates[wheel].angle = angle;

                // Store hypotenuse (turning radius) for later speed calculation.
                m_wheelUpdates[wheel].radius = abs(g_dimensions[wheel].y / sin(angle));
            }
        }
    }

    // Use the turning radii to calculate the actual rolling speed for each wheel.
    calculateWheelSpeedsFromRadii(speed);
}

void SawppyDrive::setWheelAnglesAndSpeedsToDriveStraight(int32_t speed)
{
    for (size_t wheel = 0; wheel < 6; wheel++)
    {
        m_wheelUpdates[wheel].angle = 0.0f;
        m_wheelUpdates[wheel].speed = speed;
    }
}

void SawppyDrive::calculateWheelSpeedsFromRadii(int32_t maxSpeed)
{
    size_t wheel;

    // Find maximum radius of all wheels since it will need to rotate the fastest and the rest will be scaled down
    // relative to it.
    float maxRadius = 0.0f;
    for (wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        float radius = fabsf(m_wheelUpdates[wheel].radius);
        if (radius > maxRadius)
        {
            maxRadius = radius;
        }
    }

    // Scale each of the wheel speeds based on wheel's turning radius. The larger the radius, the faster it needs
    // to spin to cover the same amount of arc angle.
    for (wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        m_wheelUpdates[wheel].speed = (m_wheelUpdates[wheel].radius / maxRadius) * (float)maxSpeed;
    }
}

void SawppyDrive::sendCommandsToServos()
{
    size_t wheel;

    // All angles and speeds are now calculated for each wheel so send corresponding commands to LX-16A servos.
    for (wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        // Middle wheels don't have a turning servo so this step can be skipped for them.
        if (!isMiddleWheel(wheel))
        {
            // UNDONE: Maybe set the duration to 0.5 to 1.0 of the loop interval.
            // LX-16A servos require angle values between 0 and 1000 with 500 being the centered value.
            int32_t servoAngle = LX16A_RADIAN_TO_SERVO_VALUE * m_wheelUpdates[wheel].angle + 500.0f;
            m_wheelUpdates[wheel].steeringServo.setAngle(servoAngle, 0);
        }

        int32_t servoSpeed = m_wheelUpdates[wheel].speed * 10.0f;
        if (isRightWheel(wheel))
        {
            servoSpeed = -servoSpeed;
        }
        m_wheelUpdates[wheel].rollServo.setRotationSpeed(servoSpeed);
    }

    if (m_pLogFormatted == NULL)
    {
        // Not setup for verbose logging.
        return;
    }

    // Print out the user friendly angles and speeds (degrees and %).
    for (wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        m_pLogFormatted(" W%u ", wheel);
        if (!isMiddleWheel(wheel))
        {
            m_pLogFormatted("%.2fdeg ", LX16A_RADIAN_TO_DEGREE * m_wheelUpdates[wheel].angle);
        }

        m_pLogFormatted("%.2f%% ", isRightWheel(wheel) ? -m_wheelUpdates[wheel].speed :
                                                          m_wheelUpdates[wheel].speed);
    }
    m_pLogFormatted("\n");

    // Print out the raw angles and rotation speeds (-1000 to 1000).
    for (wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        m_pLogFormatted(" w%u ", wheel);
        if (!isMiddleWheel(wheel))
        {
            int32_t servoAngle = LX16A_RADIAN_TO_SERVO_VALUE * m_wheelUpdates[wheel].angle + 500.0f;
            m_pLogFormatted("%ld ", servoAngle);
        }

        int32_t servoSpeed = m_wheelUpdates[wheel].speed * 10.0f;
        m_pLogFormatted("%ld ", isRightWheel(wheel) ? -servoSpeed : servoSpeed);
    }
    m_pLogFormatted("\n");
}

void SawppyDrive::stopAll()
{
    // Stops all of the steering servos.
    broadcastStop();
    // Stops all of the rolling servos.
    for (size_t wheel = 0 ; wheel < WHEEL_COUNT ; wheel++)
    {
        m_wheelUpdates[wheel].rollServo.setRotationSpeed(0);
    }
}

// *** Static Methods ***
// Returns true if the specified index refers to a wheel on the right side of Sawppy.
bool SawppyDrive::isRightWheel(size_t index)
{
    // Right side wheels have an odd index.
    return (index & 1) == 1;
}

// Returns true if the specified index refers to a middle wheel.
 bool SawppyDrive::isMiddleWheel(size_t index)
{
    return index == WHEEL_MIDDLE_LEFT || index == WHEEL_MIDDLE_RIGHT;
}
