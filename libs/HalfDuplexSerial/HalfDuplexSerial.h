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
// Half Duplex Serial mbed driver for the LPC1768.
// The user just needs to connect the single serial wire from the half-duplex device to both the Rx and Tx pins.
#ifndef HALF_DUPLEX_SERIAL_H_
#define HALF_DUPLEX_SERIAL_H_

#include <mbed.h>


class HalfDuplexSerial : private SerialBase
{
    public:
        HalfDuplexSerial(PinName tx, PinName rx, int baud = MBED_CONF_PLATFORM_DEFAULT_SERIAL_BAUD_RATE);

        void write(const void* pWrite, size_t writeSize);
        size_t read(void* pRead, size_t readSize, int msecTimeout);

    protected:
        enum State { TRANSMITTING, RECEIVING };

        void initGpioForTx(PinName pin);
        void emptyRxOfEchoedBytes();
        void setState(State newState);
        void switchToReceive();
        bool isLastTransmitComplete();
        void switchToTransmit();
        void emptyRx();
        int readByteWithTimeout(uint32_t msecTimeout);
        void verifyNoErrors();

        gpio_t m_gpioForTx;
        Timer  m_timer;
        State  m_state;
        uint32_t m_bytesOutstanding;
};

#endif // HALF_DUPLEX_SERIAL_H_
