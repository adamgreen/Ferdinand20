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
// LewanSoul LX-16A Serial Bus Servo mbed driver.
#include "LX16A.h"



// Servo ID to use for broadcasting command to all servos.
#define SERVOID_BROADCAST           0xFE

// Commands that can be sent to the LX-16A Servo.
#define SERVO_MOVE_TIME_WRITE       1
#define SERVO_MOVE_TIME_READ        2
#define SERVO_MOVE_TIME_WAIT_WRITE  7
#define SERVO_MOVE_TIME_WAIT_READ   8
#define SERVO_MOVE_START            11
#define SERVO_MOVE_STOP             12
#define SERVO_ID_WRITE              13
#define SERVO_ID_READ               14
#define SERVO_ANGLE_OFFSET_ADJUST   17
#define SERVO_ANGLE_OFFSET_WRITE    18
#define SERVO_ANGLE_OFFSET_READ     19
#define SERVO_ANGLE_LIMIT_WRITE     20
#define SERVO_ANGLE_LIMIT_READ      21
#define SERVO_VIN_LIMIT_WRITE       22
#define SERVO_VIN_LIMIT_READ        23
#define SERVO_TEMP_MAX_LIMIT_WRITE  24
#define SERVO_TEMP_MAX_LIMIT_READ   25
#define SERVO_TEMP_READ             26
#define SERVO_VIN_READ              27
#define SERVO_POS_READ              28
#define SERVO_OR_MOTOR_MODE_WRITE   29
#define SERVO_OR_MOTOR_MODE_READ    30
#define SERVO_LOAD_OR_UNLOAD_WRITE  31
#define SERVO_LOAD_OR_UNLOAD_READ   32
#define SERVO_LED_CTRL_WRITE        33
#define SERVO_LED_CTRL_READ         34
#define SERVO_LED_ERROR_WRITE       35
#define SERVO_LED_ERROR_READ        36

// Servo modes for SERVO_OR_MOTOR_MODE_WRITE/READ.
#define SERVO_MODE_POSITION_CONTROL 0
#define SERVO_MODE_MOTOR_CONTROL    1

// Set to false to disable logging of read errors to stdout.
#define LOG_READ_ERRORS             true

#define LOG_READ_ERROR(...) if (LOG_READ_ERRORS) printf("LX-16A_Error: " __VA_ARGS__);

// Macros to split 16-bit servo parameters into upper and lower bytes.
#define HIGH_BYTE(U16) ((uint8_t)(((U16) >> 8) & 0xFF))
#define LOW_BYTE(U16)  ((uint8_t)((U16) & 0xFF))

// Timeout to use when waiting for a serial response from the LX-16A servo.
#define RESPONSE_TIMEOUT_MSEC 250


LX16A_ServoBus::LX16A_ServoBus(PinName txPin, PinName rxPin) :
    m_serial(txPin, rxPin, 115200)
{
}

void LX16A_ServoBus::sendWriteCommand(uint8_t servoId, const uint8_t* pCommand, size_t commandSize)
{
    sendCommand(servoId, pCommand, commandSize);
}

bool LX16A_ServoBus::sendReadCommand(uint8_t servoId, uint8_t readCommand, void* pResponse, size_t responseSize)
{
    uint8_t command[1] = { readCommand };

    sendCommand(servoId, command, sizeof(command));
    return readResponse(servoId, readCommand, pResponse, responseSize);
}

void LX16A_ServoBus::broadcastStart()
{
    uint8_t command[] = { SERVO_MOVE_START };
    sendWriteCommand(SERVOID_BROADCAST, command, sizeof(command));
}

void LX16A_ServoBus::broadcastStop()
{
    uint8_t command[] = { SERVO_MOVE_STOP };
    sendWriteCommand(SERVOID_BROADCAST, command, sizeof(command));
}

uint8_t LX16A_ServoBus::discoverServoId()
{
    uint8_t servoId = SERVOID_BROADCAST;
    sendReadCommand(SERVOID_BROADCAST, SERVO_ID_READ, &servoId, sizeof(servoId));
    return servoId;
}

void LX16A_ServoBus::sendCommand(uint8_t servoId, const uint8_t* pCommand, size_t commandSize)
{
    uint8_t buffer[10];

    // Will add two bytes of header sync bytes (0x55 0x55), 1 byte servo ID, 1 byte length, and 1 byte checksum.
    assert ( commandSize + 2 + 1 + 1 + 1 <= sizeof(buffer) );
    buffer[0] = 0x55;
    buffer[1] = 0x55;
    buffer[2] = servoId;
    buffer[3] = commandSize + 2;
    memcpy(&buffer[4], pCommand, commandSize);
    buffer[4 + commandSize] = checksum(&buffer[2], commandSize + 2);

    size_t bytesToSend = commandSize + 5;
    m_serial.write(buffer, bytesToSend);
}

bool LX16A_ServoBus::readResponse(uint8_t expectedServoId, uint8_t expectedCommand, void* pResponse, size_t responseSize)
{
    uint8_t buffer[10];

    // Servo will send two bytes of header sync bytes (0x55 0x55), 1 byte servo ID, 1 byte length, 1 byte command id and
    // 1 byte checksum in addition to data.
    size_t bytesToRead = responseSize + 2 + 1 + 1 + 1 + 1;
    assert ( bytesToRead <= sizeof(buffer) );
    size_t bytesRead = m_serial.read(buffer, bytesToRead, RESPONSE_TIMEOUT_MSEC);
    if (bytesRead != bytesToRead ||
        buffer[0] != 0x55 ||
        buffer[1] != 0x55 ||
        (expectedServoId != SERVOID_BROADCAST && buffer[2] != expectedServoId) ||
        buffer[3] != responseSize + 3 ||
        buffer[4] != expectedCommand ||
        buffer[5 + responseSize] != checksum(&buffer[2], responseSize + 3))
    {
        return false;
    }
    memcpy(pResponse, &buffer[5], responseSize);
    return true;
}

uint8_t LX16A_ServoBus::checksum(uint8_t* pBuffer, size_t bufferLength)
{
    uint8_t sum = 0;
    for (uint8_t i = 0 ; i < bufferLength ; i++)
    {
        sum = sum + *pBuffer++;
    }
    return ~sum;
}



LX16A_Servo::LX16A_Servo(LX16A_ServoBus& servoBus, uint8_t servoId, uint8_t maxRetry /* = 2 */) :
    m_servoBus(servoBus)
{
    assert ( servoId <= 253 );
    assert ( maxRetry >= 1 );
    m_servoId = servoId;
    m_maxRetry = maxRetry;
    m_lastReadResult = true;
    m_state = UNKNOWN;
}

void LX16A_Servo::setAngle(uint16_t angle, uint16_t msecTime /* = 0 */)
{
    assert ( angle <= 1000 && msecTime <= 30000 );

    setToPositionMode();
    uint8_t command[] = { SERVO_MOVE_TIME_WRITE,
                          LOW_BYTE(angle), HIGH_BYTE(angle),
                          LOW_BYTE(msecTime), HIGH_BYTE(msecTime) };
    sendWriteCommand(command, sizeof(command));
}

void LX16A_Servo::setToPositionMode()
{
    if (m_state == POSITION)
    {
        return;
    }
    uint8_t command[] = { SERVO_OR_MOTOR_MODE_WRITE, SERVO_MODE_POSITION_CONTROL , 0, 0, 0 };
    sendWriteCommand(command, sizeof(command));
    m_state = POSITION;
}

void LX16A_Servo::setRotationSpeed(int16_t speed)
{
    assert ( speed >= -1000 && speed <= 1000 );

    uint8_t command[] = { SERVO_OR_MOTOR_MODE_WRITE, SERVO_MODE_MOTOR_CONTROL, 0, LOW_BYTE(speed), HIGH_BYTE(speed) };
    sendWriteCommand(command, sizeof(command));
    m_state = CONTINUOUS;
}

void LX16A_Servo::disableMotor()
{
    uint8_t command[] = { SERVO_LOAD_OR_UNLOAD_WRITE, 0 };
    sendWriteCommand(command, sizeof(command));
}

void LX16A_Servo::setIdOnServo(uint8_t newServoId)
{
    assert ( newServoId <= 253 );
    uint8_t command[] = { SERVO_ID_WRITE, newServoId };
    sendWriteCommand(command, sizeof(command));
}

void LX16A_Servo::queueSetAngle(uint16_t angle, uint16_t msecTime /* = 0 */)
{
    assert ( angle <= 1000 && msecTime <= 30000 );

    setToPositionMode();
    uint8_t command[] = { SERVO_MOVE_TIME_WAIT_WRITE,
                          LOW_BYTE(angle), HIGH_BYTE(angle),
                          LOW_BYTE(msecTime), HIGH_BYTE(msecTime) };
    sendWriteCommand(command, sizeof(command));
}

void LX16A_Servo::stop()
{
    uint8_t command[] = { SERVO_MOVE_STOP };
    sendWriteCommand(command, sizeof(command));
}

void LX16A_Servo::setAngleOffset(int8_t angle)
{
    assert ( angle >= -125 && angle <= 125 );

    uint8_t command[] = { SERVO_ANGLE_OFFSET_ADJUST, (uint8_t)angle };
    sendWriteCommand(command, sizeof(command));
}

void LX16A_Servo::saveCurrentAngleOffset()
{
    uint8_t command[] = { SERVO_ANGLE_OFFSET_WRITE };
    sendWriteCommand(command, sizeof(command));
}

int8_t LX16A_Servo::getAngleOffset()
{
    int8_t angleOffset = 0;
    bool result = sendReadCommand(SERVO_ANGLE_OFFSET_READ, &angleOffset, sizeof(angleOffset));
    if (!result)
    {
        LOG_READ_ERROR("getAngleOffset() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return angleOffset;
}

int16_t LX16A_Servo::getPosition()
{
    int16_t position = 0;
    bool result = sendReadCommand(SERVO_POS_READ, &position, sizeof(position));
    if (!result)
    {
        LOG_READ_ERROR("readPosition() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return position;
}

void LX16A_Servo::setAngleLimits(LX16A_ServoAngleLimits& limits)
{
    assert ( limits.minimum >= 0 && limits.minimum <= 1000 );
    assert ( limits.maximum > limits.minimum && limits.maximum <= 1000 );

    uint8_t command[] = { SERVO_ANGLE_LIMIT_WRITE, LOW_BYTE(limits.minimum), HIGH_BYTE(limits.minimum),
                                                   LOW_BYTE(limits.maximum), HIGH_BYTE(limits.maximum) };
    sendWriteCommand(command, sizeof(command));
}

LX16A_ServoAngleLimits LX16A_Servo::getAngleLimits()
{
    LX16A_ServoAngleLimits limits = { 0, 0 };
    bool result = sendReadCommand(SERVO_ANGLE_LIMIT_READ, &limits, sizeof(limits));
    if (!result)
    {
        LOG_READ_ERROR("getAngleLimits() failed for Servo #%u.\n", m_servoId);
    }
    return limits;
}

void LX16A_Servo::setVoltageLimits(LX16A_ServoVoltageLimits& limits)
{
    assert ( limits.minimum >= 4500 && limits.minimum <= 12000 );
    assert ( limits.maximum > limits.minimum && limits.maximum <= 12000 );

    uint8_t command[] = { SERVO_VIN_LIMIT_WRITE, LOW_BYTE(limits.minimum), HIGH_BYTE(limits.minimum),
                                                 LOW_BYTE(limits.maximum), HIGH_BYTE(limits.maximum) };
    sendWriteCommand(command, sizeof(command));
}

LX16A_ServoVoltageLimits LX16A_Servo::getVoltageLimits()
{
    LX16A_ServoVoltageLimits limits = { 0, 0 };
    bool result = sendReadCommand(SERVO_VIN_LIMIT_READ, &limits, sizeof(limits));
    if (!result)
    {
        LOG_READ_ERROR("getVoltageLimits() failed for Servo #%u.\n", m_servoId);
    }
    return limits;
}

void LX16A_Servo::setTemperatureLimit(uint8_t limit)
{
    assert ( limit >= 50 && limit <= 100 );

    uint8_t command[] = { SERVO_TEMP_MAX_LIMIT_WRITE, limit };
    sendWriteCommand(command, sizeof(command));
}

uint8_t LX16A_Servo::getTemperatureLimit()
{
    uint8_t limit = 0;
    bool result = sendReadCommand(SERVO_TEMP_MAX_LIMIT_READ, &limit, sizeof(limit));
    if (!result)
    {
        LOG_READ_ERROR("getTemperatureLimit() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return limit;
}

uint8_t LX16A_Servo::getTemperature()
{
    uint8_t temp = 0;
    bool result = sendReadCommand(SERVO_TEMP_READ, &temp, sizeof(temp));
    if (!result)
    {
        LOG_READ_ERROR("getTemperature() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return temp;
}

uint16_t LX16A_Servo::getVoltage()
{
    uint16_t voltage = 0;
    bool result = sendReadCommand(SERVO_VIN_READ, &voltage, sizeof(voltage));
    if (!result)
    {
        LOG_READ_ERROR("getVoltage() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return voltage;
}

uint8_t LX16A_Servo::isMotorEnabled()
{
    uint8_t motor = 0;
    bool result = sendReadCommand(SERVO_LOAD_OR_UNLOAD_READ, &motor, sizeof(motor));
    if (!result)
    {
        LOG_READ_ERROR("isMotorEnabled() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return motor;
}

void LX16A_Servo::setLed(LX16A_ServoLedSettings setting)
{
    uint8_t command[] = { SERVO_LED_CTRL_WRITE, (uint8_t)setting };
    sendWriteCommand(command, sizeof(command));
}

LX16A_ServoLedSettings LX16A_Servo::getLedSetting()
{
    uint8_t led = 0;
    bool result = sendReadCommand(SERVO_LED_CTRL_READ, &led, sizeof(led));
    if (!result)
    {
        LOG_READ_ERROR("getLedSetting() failed for Servo #%u.\n", m_servoId);
        return SERVO_LED_OFF;
    }
    if (led > SERVO_LED_OFF)
    {
        LOG_READ_ERROR("getLedSetting() returned invalid value for Servo #%u.\n", m_servoId);
        m_lastReadResult = false;
        return SERVO_LED_OFF;
    }
    return (LX16A_ServoLedSettings)led;
}

void LX16A_Servo::setLedFaultMask(uint8_t faultMask)
{
    uint8_t command[] = { SERVO_LED_ERROR_WRITE, (uint8_t)faultMask };
    sendWriteCommand(command, sizeof(command));
}

uint8_t LX16A_Servo::getLedFaultMask()
{
    uint8_t error = 0;
    bool result = sendReadCommand(SERVO_LED_ERROR_READ, &error, sizeof(error));
    if (!result)
    {
        LOG_READ_ERROR("getLedFaultMask() failed for Servo #%u.\n", m_servoId);
        return 0;
    }
    return error;
}


void LX16A_Servo::sendWriteCommand(const uint8_t* pCommand, size_t commandSize)
{
    m_servoBus.sendWriteCommand(m_servoId, pCommand, commandSize);
}

bool LX16A_Servo::sendReadCommand(uint8_t readCommand, void* pResponse, size_t responseSize)
{
    for (uint8_t retry = 1 ; retry <= m_maxRetry ; retry++)
    {
        if (m_servoBus.sendReadCommand(m_servoId, readCommand, pResponse, responseSize))
        {
            m_lastReadResult = true;
            return true;
        }
    }
    m_lastReadResult = false;
    return false;
}
