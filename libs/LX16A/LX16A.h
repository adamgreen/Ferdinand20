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
// LewanSoul LX-16A Serial Bus Servo mbed driver.
#ifndef LX16A_H_
#define LX16A_H_

#include <mbed.h>
#include <assert.h>
#include <HalfDuplexSerial.h>



// Constants used to convert angle in degrees or radians to the corresponding LX-16A servo angle integer value.
#define LX16A_SERVO_TO_DEGREE_VALUE (240.0f / 1000.0f)
#define LX16A_DEGREE_TO_SERVO_VALUE (1.0f / LX16A_SERVO_TO_DEGREE_VALUE)
#define LX16A_SERVO_TO_RADIAN_VALUE ((4.0f / 3.0f * (float)M_PI) / 1000.0f)
#define LX16A_RADIAN_TO_SERVO_VALUE (1.0f / LX16A_SERVO_TO_RADIAN_VALUE)
#define LX16A_RADIAN_TO_DEGREE (180.0f/(float)M_PI)
#define LX16A_DEGREE_TO_RADIAN ((float)M_PI/180.0f)

// Minimum and maximum rotation speeds.
#define LX16A_ROTATION_SPEED_MIN -1000
#define LX16A_ROTATION_SPEED_MAX 1000



struct LX16A_ServoAngleLimits
{
    uint16_t minimum;
    uint16_t maximum;
};

struct LX16A_ServoVoltageLimits
{
    uint16_t minimum;
    uint16_t maximum;
};

enum LX16A_ServoLedSettings
{
    SERVO_LED_ALWAYS_ON = 0,
    SERVO_LED_OFF = 1
};



class LX16A_ServoBus
{
    public:
        // Construct object to represent the LPC1768 UART used to communicate with the LX-16A servos.
        // Note: The specified LPC1768 UART Tx and Rx pins should both be connected to the same LX-16A signal wire.
        //       Half Duplex communication is used so that both ends don't attempt to Tx on the wire at the same time.
        LX16A_ServoBus(PinName txPin, PinName rxPin);

        void sendWriteCommand(uint8_t servoId, const uint8_t* pCommand, size_t commandSize);
        bool sendReadCommand(uint8_t servoId, uint8_t readCommand, void* pResponse, size_t responseSize);

        // Broadcast start command to all attached servos. Will start all rotations previously queued up on individual
        // servos via LX16A_Servo::queueSetAngle() calls. This way all servos start moving at exactly the same time.
        void broadcastStart();
        // Command all attached servos to stop any in-progress motions.
        void broadcastStop();
        // When only one servo is attached to the bus, this method can be called to obtain its ID.
        uint8_t discoverServoId();

    protected:
        void sendCommand(uint8_t servoId, const uint8_t* pCommand, size_t commandSize);
        bool readResponse(uint8_t expectedServoId, uint8_t expectedCommand, void* pResponse, size_t responseSize);
        uint8_t checksum(uint8_t* pBuffer, size_t bufferLength);
        void insertInterCmdDelay();

        HalfDuplexSerial m_serial;
        Timer            m_timer;
        uint32_t         m_lastSendTime;
};


class LX16A_Servo
{
    public:
        // Construct object to represent a single LX-16A servo.
        //  servoBus is a reference to the LX16A_ServoBus object configured for the serial connection to the LX-16A
        //           servos.
        //  servoId is the ID of the LX-16A servo this object should be used to communicate with. A valid servo ID is
        //          between 0 and 253.
        //  maxRetry is the number of times read commands should be sent in an attempt to resolve I/O errors.
        LX16A_Servo(LX16A_ServoBus& servoBus, uint8_t servoId, uint8_t maxRetry = 2);

        // Changes the ID of the LX-16A servo to which this object should communicate.
        // A valid servo ID is between 0 and 253.
        // Note: This doesn't change the ID on the LX16-A servo itself. The setIdOnServo() method is used to make such
        //       ID changes.
        void setServoId(uint8_t servoId)
        {
            assert ( servoId <= 253 );
            m_servoId = servoId;
        }
        // Returns the servo ID to which this object has been set.
        uint8_t getServoId()
        {
            return m_servoId;
        }
        // Returns the result of the last get*() method call.
        //  Returns false if the last read/get attempt call failed and true if it was successful.
        bool getLastReadResult()
        {
            return m_lastReadResult;
        }

        // Changes the ID on the LX-16A servo hardware itself.
        // A valid servo ID is between 0 and 253.
        void setIdOnServo(uint8_t newServoId);

        // Sets the servo to spin in continuous rotation at the specified speed.
        //  speed determines the speed/direction of the rotation. It can be set to a value between -1000 and 1000.
        void setRotationSpeed(int16_t speed);
        // Sets servo to a specified angle and disable any continuous rotation.
        //  angle is a value between 0 and 1000, where 0 sets it to 0 degrees and 1000 sets it to 240 degrees. A value
        //        of 0.24 can be used to convert this angle to/from degrees.
        //  msecTime is the amount of time the servo will take to transition from its current angle to the new angle.
        //           It is specified in the units of milliseconds.
        void setAngle(uint16_t angle, uint16_t msecTime = 0);
        // Returns the current angle of the servo.
        //  A return value of 0 corresponds to 0 degrees.
        //  A return value of 1000 corresponds to 240 degrees.
        //  As a servo can rotate 360 degrees in continuous mode, it can return values outside of this 0 to 1000 range.
        //  It can even be a bit negative.
        int16_t getPosition();

        // Queues up a request for the servo to move to a specified angle and disable any continuous rotation. The
        // actual rotation doesn't start until the LX16A_ServoBus::broadcastStart() method is called.
        //  angle is a value between 0 and 1000, where 0 sets it to 0 degrees and 1000 sets it to 240 degrees. A value
        //        of 0.24 can be used to convert this angle to/from degrees.
        //  msecTime is the amount of time the servo will take to transition from its current angle to the new angle.
        //           It is specified in the units of milliseconds.
        void queueSetAngle(uint16_t angle, uint16_t msecTime = 0);
        // Commands the servo to stop any in-progress motions.
        void stop();

        // Disable motor by removing current so that it provides no torque.
        void disableMotor();
        // Returns whether the motor is enabled or not.
        //  A return value of 0 means that the motor is disabled and providing no torque.
        //  A return value of 1 means that the motor is enabled and providing torque.
        uint8_t isMotorEnabled();

        // Sets the angle offset to fine tune the angle output of the servo.
        //  angle is a value between -125 and 125, corresponding to -30 and 30 degrees.
        // Note: This adjusts the live offset but will be lost after a power cycle. Call saveCurrentAngleOffset() to
        //       have it persist across power cycles.
        void setAngleOffset(int8_t angle);
        // Persists the current angle offset settings in the servo so that it is used after power cycling.
        void saveCurrentAngleOffset();
        // Returns the current angle offset setting on the servo.
        int8_t getAngleOffset();

        // Sets the minimum and maximum soft limits of the servo.
        // These limits can be set to values between 0 and 1000 (0 to 240 degrees) with the maximum limit greater than
        // the minimum. These settings are persisted across power cycles.
        void setAngleLimits(LX16A_ServoAngleLimits& limits);
        // Returns the current minimum and maximum soft limit settings of the servo.
        LX16A_ServoAngleLimits  getAngleLimits();

        // Returns the current servo power supply voltage.
        // The return is in units of mV.
        uint16_t getVoltage();
        // Sets the minimum and maximum voltage limits. If the power supply falls outside of these limits then the
        // motor will generate a voltage fault and disable itself.
        // These limits can be set to values between 4500mV and 12000mV with the maximum limit greater than the minimum.
        // These settings are persisted across power cycles.
        void setVoltageLimits(LX16A_ServoVoltageLimits& limits);
        // Returns the current minimum and maximum voltage limits.
        LX16A_ServoVoltageLimits getVoltageLimits();

        // Returns the current servo temperature.
        // The return is in units of Celcius.
        uint8_t getTemperature();
        // Sets the maximum temperature limit. If the servo temperature raises above this limit then the motor will
        // generate a temperature fault and disable itself until the temperature drops below this limit again. These
        // settings are persisted across power cycles.
        // This limit can be set to a value between 50 and 100C.
        void setTemperatureLimit(uint8_t limit);
        // Returns the current maximum temperature limit.
        uint8_t getTemperatureLimit();

        // Sets the default LED operation while no fault is occuring.
        // Note: The LED always flashes when an enabled fault occurs.
        void setLed(LX16A_ServoLedSettings setting);
        // Returns the current default LED operation setting.
        LX16A_ServoLedSettings getLedSetting();

        // Sets the mask to determine which faults should cause the LED to flash when they occur.
        // Bit 0 is set high to enable faults on over temperature.
        // Bit 1 is set high to enable faults on over voltage.
        // Bit 2 is set high to enable faults on motor stall.
        void setLedFaultMask(uint8_t faultMask);
        // Returns the current LED fault mask.
        uint8_t getLedFaultMask();

    protected:
        void setToPositionMode();
        void sendWriteCommand(const uint8_t* pCommand, size_t commandSize);
        bool sendReadCommand(uint8_t readCommand, void* pResponse, size_t responseSize);

        LX16A_ServoBus& m_servoBus;
        enum { UNKNOWN, POSITION, CONTINUOUS } m_state;
        bool            m_lastReadResult;
        uint8_t         m_servoId;
        uint8_t         m_maxRetry;
};

#endif // LX16A_H_
