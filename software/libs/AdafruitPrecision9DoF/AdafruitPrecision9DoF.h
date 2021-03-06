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
#ifndef ADAFRUIT_PRECISION_9DOF_H_
#define ADAFRUIT_PRECISION_9DOF_H_

#include <mbed.h>
#include "FXOS8700CQ.h"
#include "FXAS21002C.h"
#include "Quaternion.h"
#include "Vector.h"
#include "Matrix4x4.h"

struct SensorValues
{
    Vector<int16_t> accel;
    Vector<int16_t> mag;
    Vector<int16_t> gyro;
    int16_t         gyroTemperature;
};

struct SensorCalibratedValues
{
    Vector<float> accel;
    Vector<float> mag;
    Vector<float> gyro;
};

struct SensorCalibration
{
    float             initialVariance;
    float             gyroVariance;
    float             accelMagVariance;
    Vector<float>     gyroCoefficientA;
    Vector<float>     gyroCoefficientB;
    Vector<float>     gyroScale;
    Vector<float>     declinationCorrection;
    Vector<float>     mountingCorrection;
    Vector<int16_t>   accelMin;
    Vector<int16_t>   accelMax;
    Vector<int16_t>   magMin;
    Vector<int16_t>   magMax;
    Vector<int16_t>   accelSwizzle;
    Vector<int16_t>   magSwizzle;
    Vector<int16_t>   gyroSwizzle;
};


class AdafruitPrecision9DoF
{
public:
    AdafruitPrecision9DoF(PinName sdaPin, PinName sclPin, PinName int1Pin, int32_t sampleRateHz,
                          const SensorCalibration* pCalibration = NULL);

    void calibrate(const SensorCalibration* pCalibration);

    bool                   wouldBlock() { return m_lastSample == m_currentSample; }
    SensorValues           getRawSensorValues();
    SensorCalibratedValues calibrateSensorValues(const SensorValues* pRawValues);
    Quaternion             getOrientation(SensorCalibratedValues* pCalibratedValues);
    float                  getHeading(Quaternion* pOrientation);
    float                  getYaw(Quaternion* pOrientation);
    float                  getPitch(Quaternion* pOrientation);
    float                  getRoll(Quaternion* pOrientation);
    static float           constrainAngle(float angle);

    void  reset() { m_resetRequested = true; }
    int   didInitFail() { return m_failedInit; }
    int   didIoFail() { return m_failedIo; }
    float getIdleTimePercent() { return m_idleTimePercent; }

protected:
    void         resetKalmanFilter(SensorCalibratedValues* pCalibratedValues);
    Quaternion   getOrientationFromAccelerometerMagnetometerMeasurements(SensorCalibratedValues* pCalibratedValues);
    static float angleFromDegreeMinuteSecond(Vector<float>* pAngle);
    void         interruptHandler();

    Timer                   m_totalTimer;
    Timer                   m_idleTimer;
    SensorCalibration       m_calibration;
    I2C                     m_i2c;
    InterruptIn             m_int1Pin;
    FXOS8700CQ              m_accelMag;
    FXAS21002C              m_gyro;
    SensorCalibratedValues  m_midpoints;
    SensorCalibratedValues  m_scales;
    Quaternion              m_currentOrientation;
    Matrix4x4               m_kalmanP;
    Matrix4x4               m_kalmanQ;
    Matrix4x4               m_kalmanR;
    float                   m_gyroTimeScaleFactor;
    float                   m_idleTimePercent;
    float                   m_declinationCorrection;
    float                   m_mountingCorrection;
    volatile int            m_failedInit;
    volatile int            m_failedIo;
    volatile uint32_t       m_currentSample;
    uint32_t                m_lastSample;
    SensorValues            m_sensorValues;
    bool                    m_resetRequested;
};

#endif /* ADAFRUIT_PRECISION_9DOF_H_ */
