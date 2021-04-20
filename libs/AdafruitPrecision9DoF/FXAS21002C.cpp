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
#include "FXAS21002C.h"


/* I2C Register Addresses */
#define STATUS          0x00
#define OUT_X_MSB       0x01
#define OUT_X_LSB       0x02
#define OUT_Y_MSB       0x03
#define OUT_Y_LSB       0x04
#define OUT_Z_MSB       0x05
#define OUT_Z_LSB       0x06
#define DR_STATUS       0x07
#define F_STATUS        0x08
#define F_SETUP         0x09
#define F_EVENT         0x0A
#define INT_SRC_FLAG    0x0B
#define WHO_AM_I        0x0C
#define CTRL_REG0       0x0D
#define RT_CFG          0x0E
#define RT_SRC          0x0F
#define RT_THS          0x10
#define RT_COUNT        0x11
#define TEMP            0x12
#define CTRL_REG1       0x13
#define CTRL_REG2       0x14
#define CTRL_REG3       0x15

/* CTRL_REG0 bits */
/*  Low pass filter cutoff frequency selection, varies by ODR setting. */
#define BW_SHIFT            6
#define BW_HIGHEST_FREQ     (0 << BW_SHIFT)
#define BW_MEDIUM_FREQ      (1 << BW_SHIFT)
#define BW_LOWEST_FREQ      (3 << BW_SHIFT)
/*  SPI Interface Mode */
#define SPIW_3_WIRE         (1 << 5)
/*  High pass filter cutoff frequency selection, varies by ODR setting. */
#define SEL_SHIFT           3
#define SEL_HIGHEST         (0 << SEL_SHIFT)
#define SEL_LOWEST          (3 << SEL_SHIFT)
/*  High pass filter enabled when set to 1. */
#define HPF_EN              (1 << 2)
/*  Full range selection: 250/500/1000/2000 deg/s */
#define FS_2000_DPS         0
#define FS_1000_DPS         1
#define FS_500_DPS          2
#define FS_250_DPS          3

/* CTRL_REG1 bits */
/*  Software reset started when set to 1. */
#define RST                 (1 << 6)
/*  Self test enabled when set to 1. */
#define ST                  (1 << 5)
/*  Output data rate. */
#define DR_SHIFT            2
#define DR_800_HZ           (0 << DR_SHIFT)
#define DR_400_HZ           (1 << DR_SHIFT)
#define DR_200_HZ           (2 << DR_SHIFT)
#define DR_100_HZ           (3 << DR_SHIFT)
#define DR_50_HZ            (4 << DR_SHIFT)
#define DR_25_HZ            (5 << DR_SHIFT)
#define DR_12_5_HZ          (6 << DR_SHIFT)
/*  Active mode enabled when set to 1. */
#define ACTIVE              (1 << 1)
/*  Ready mode enabled when set to 1. */
#define READY               (1 << 0)


FXAS21002C::FXAS21002C(I2C* pI2C, int address /* = (0x21<<1) */) : SensorBase(pI2C, address)
{
    initGyro();
}

void FXAS21002C::initGyro()
{
    // Assume that init has failed until proven wrong.
    m_failedInit = 1;

    // Reset to make sure that device is in standby mode.
    // NOTE: Ignore any I2C error as a reset will truncate the ACK.
    writeRegister(CTRL_REG1, RST);

    // Wait a bit for reset to occur.
    wait_ms(1);

    // Configure for highest low pass filter frequency setting, disable high pass filter, and
    // set full scale resolution to 2000 degrees/s.
    writeRegister(CTRL_REG0, BW_HIGHEST_FREQ | FS_2000_DPS);
    if (m_failedIo)
        return;
    // UNDONE: Hardcoded for 100Hz right now.
    // Set sampling rate to 100Hz.
    // Also switches device back into active mode.
    writeRegister(CTRL_REG1, DR_100_HZ | ACTIVE);
    if (m_failedIo)
        return;

    // Takes 60ms to enter active mode.
    wait_ms(60);

    // This part of init was successful.
    m_failedInit = 0;
}

void FXAS21002C::getVector(Vector<int16_t>* pVector, int16_t* pTemperature)
{
    char bigEndianData[2*3];

    readRegisters(OUT_X_MSB, bigEndianData, sizeof(bigEndianData));
    if (m_failedIo)
        return;

    // Data returned is big endian so byte swap.
    pVector->x = (bigEndianData[0] << 8) | bigEndianData[1];
    pVector->y = (bigEndianData[2] << 8) | bigEndianData[3];
    pVector->z = (bigEndianData[4] << 8) | bigEndianData[5];

    int8_t temp;
    readRegister(TEMP, (char*)&temp);
    if (m_failedIo)
        return;
    *pTemperature = temp;
}
