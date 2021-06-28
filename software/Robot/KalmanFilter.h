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
#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_

#include <mbed.h>
#include "AdafruitPrecision9DoF.h"


class KalmanFilter
{
public:
    KalmanFilter(int32_t sampleRateHz, const SensorCalibration* pCalibration = NULL);

    void calibrate(const SensorCalibration* pCalibration);

    SensorCalibratedValues calibrateSensorValues(const SensorValues* pRawValues);
    Quaternion             getOrientation(SensorCalibratedValues* pCalibratedValues);
    float                  getHeading(Quaternion* pOrientation);
    float                  getYaw(Quaternion* pOrientation);
    float                  getPitch(Quaternion* pOrientation);
    float                  getRoll(Quaternion* pOrientation);
    static float           constrainAngle(float angle);

    void  reset() { m_resetRequested = true; }

protected:
    void         resetKalmanFilter(SensorCalibratedValues* pCalibratedValues);
    Quaternion   getOrientationFromAccelerometerMagnetometerMeasurements(SensorCalibratedValues* pCalibratedValues);
    static float angleFromDegreeMinuteSecond(Vector<float>* pAngle);
    void         interruptHandler();

    SensorCalibration       m_calibration;
    SensorCalibratedValues  m_midpoints;
    SensorCalibratedValues  m_scales;
    Quaternion              m_currentOrientation;
    Matrix4x4               m_kalmanP;
    Matrix4x4               m_kalmanQ;
    Matrix4x4               m_kalmanR;
    float                   m_gyroTimeScaleFactor;
    float                   m_declinationCorrection;
    float                   m_mountingCorrection;
    bool                    m_resetRequested;
};

#endif /* KALMAN_FILTER_H_ */
