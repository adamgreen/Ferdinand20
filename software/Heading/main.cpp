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
/* Test harness for my heading class. */
#include <assert.h>
#include <mbed.h>
#include "ConfigFile.h"
#include "DmaSerial.h"
#include "FlashFileSystem.h"
#include "files.h"
#include "AdafruitPrecision9DoF.h"

enum OUTPUT_MODE
{
    // Original output mode which just returned raw sensor readings, temperature, and sample interval.
    // Start up in this mode on reset.
    OUTPUT_ORIGINAL,

    // Original output + Kalman calculated quaternion + compass heading + idle CPU %.
    // Switch to this mode once we have received a reset request.
    OUTPUT_KALMAN
};

static volatile bool        g_resetRequested = false;
static volatile OUTPUT_MODE g_outputMode = OUTPUT_ORIGINAL;

#if !MRI_ENABLE
static DmaSerial g_serial(USBTX, USBRX);
#endif // !MRI_ENABLE


// Function prototypes.
#if !MRI_ENABLE
static void serialReceiveISR();
#endif
static void readConfigurationFile(int32_t* pSampleRateHz, SensorCalibration* pCalibration);


int main()
{
    static Timer timer;
    timer.start();
#if !MRI_ENABLE
    g_serial.baud(230400);
    g_serial.attach(serialReceiveISR);
#endif // !MRI_ENABLE

    static FlashFileSystem fileSystem("flash", g_fileSystemData);
    if (!fileSystem.IsMounted())
        error("Encountered error mounting FLASH file system.\n");


    SensorCalibration calibration;
    int32_t           sampleRateHz;
    readConfigurationFile(&sampleRateHz, &calibration);
    static AdafruitPrecision9DoF sensors(p9, p10, p30, sampleRateHz, &calibration);
    if (sensors.didInitFail())
        error("Encountered I2C I/O error during IMU sensor init.\n");

    for (;;)
    {
        char buffer[256];
        int  length;
        bool wasResetRequested = g_resetRequested;

        if (wasResetRequested)
        {
            sensors.reset();
            g_resetRequested = false;
        }

        SensorValues sensorValues = sensors.getRawSensorValues();
        if (sensors.didIoFail())
            error("Encountered I2C I/O error during fetch of IMU sensor readings.\n");
        SensorCalibratedValues calibratedValues = sensors.calibrateSensorValues(&sensorValues);
        Quaternion orientation = sensors.getOrientation(&calibratedValues);
        float headingAngle = sensors.getHeading(&orientation);

        int elapsedTime = timer.read_us();
        timer.reset();

        switch (g_outputMode)
        {
        case OUTPUT_ORIGINAL:
            length = snprintf(buffer, sizeof(buffer), "%s%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                              wasResetRequested ? "R," : "",
                              sensorValues.accel.x, sensorValues.accel.y, sensorValues.accel.z,
                              sensorValues.mag.x, sensorValues.mag.y, sensorValues.mag.z,
                              sensorValues.gyro.x, sensorValues.gyro.y, sensorValues.gyro.z,
                              sensorValues.gyroTemperature,
                              elapsedTime);
            break;
        case OUTPUT_KALMAN:
        default:
            length = snprintf(buffer, sizeof(buffer), "%s%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.1f,%f,%f,%f,%f,%f\n",
                              wasResetRequested ? "R," : "",
                              sensorValues.accel.x, sensorValues.accel.y, sensorValues.accel.z,
                              sensorValues.mag.x, sensorValues.mag.y, sensorValues.mag.z,
                              sensorValues.gyro.x, sensorValues.gyro.y, sensorValues.gyro.z,
                              sensorValues.gyroTemperature,
                              elapsedTime,
                              sensors.getIdleTimePercent(),
                              orientation.w, orientation.x, orientation.y, orientation.z,
                              headingAngle);
            break;
        }
        assert( length < (int)sizeof(buffer) );

#if MRI_ENABLE
        (void)length;
        printf("%s", buffer);
#else
        g_serial.dmaTransmit(buffer, length);
#endif
    }

    return 0;
}

#if !MRI_ENABLE
static void serialReceiveISR()
{
    while (g_serial.readable())
    {
        int byte = g_serial.getc();
        if (byte == 'R')
        {
            g_resetRequested = true;
            g_outputMode = OUTPUT_KALMAN;
        }
    }
}
#endif // !MRI_ENABLE

static void readConfigurationFile(int32_t* pSampleRateHz, SensorCalibration* pCalibration)
{
    ConfigFile configFile;
    if (configFile.open("/flash/config.ini"))
        error("Encountered error opening /flash/config.ini\n");

    if (!configFile.getIntVector("compass.accelerometer.min", &pCalibration->accelMin))
        error("Failed to read compass.accelerometer.min\n");
    if (!configFile.getIntVector("compass.accelerometer.max", &pCalibration->accelMax))
        error("Failed to read compass.accelerometer.max\n");
    if (!configFile.getIntVector("compass.magnetometer.min", &pCalibration->magMin))
        error("Failed to read compass.magnetometer.min\n");
    if (!configFile.getIntVector("compass.magnetometer.max", &pCalibration->magMax))
        error("Failed to read compass.magnetometer.max\n");
    if (!configFile.getFloatVector("compass.gyro.coefficient.A", &pCalibration->gyroCoefficientA))
        error("Failed to read compass.gyro.coefficient.A\n");
    if (!configFile.getFloatVector("compass.gyro.coefficient.B", &pCalibration->gyroCoefficientB))
        error("Failed to read compass.gyro.coefficient.B\n");
    if (!configFile.getFloatVector("compass.gyro.scale", &pCalibration->gyroScale))
        error("Failed to read compass.gyro.scale\n");
    if (!configFile.getIntVector("compass.accelerometer.swizzle", &pCalibration->accelSwizzle))
        error("Failed to read compass.accelerometer.swizzle\n");
    if (!configFile.getIntVector("compass.magnetometer.swizzle", &pCalibration->magSwizzle))
        error("Failed to read compass.magnetometer.swizzle\n");
    if (!configFile.getIntVector("compass.gyro.swizzle", &pCalibration->gyroSwizzle))
        error("Failed to read compass.gyro.swizzle\n");
    if (!configFile.getFloat("compass.initial.variance", &pCalibration->initialVariance))
        error("Failed to read compass.initial.variance\n");
    if (!configFile.getFloat("compass.gyro.variance", &pCalibration->gyroVariance))
        error("Failed to read compass.gyro.variance\n");
    if (!configFile.getFloat("compass.accelerometer.magnetometer.variance", &pCalibration->accelMagVariance))
        error("Failed to read compass.accelerometer.magnetometer.variance\n");
    if (!configFile.getFloatVector("compass.declinationCorrection", &pCalibration->declinationCorrection))
        error("Failed to read compass.declinationCorrection\n");
    if (!configFile.getFloatVector("compass.mountingCorrection", &pCalibration->mountingCorrection))
        error("Failed to read compass.mountingCorrection\n");
    if (!configFile.getInt("compass.rate", pSampleRateHz))
        error("Failed to read compass.rate\n");
}
