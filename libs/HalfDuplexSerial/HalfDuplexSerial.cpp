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
#include <pinmap.h>
#include <assert.h>
#include "HalfDuplexSerial.h"



HalfDuplexSerial::HalfDuplexSerial(PinName tx, PinName rx, int baud) :
    SerialBase(tx, rx, baud)
{
    initGpioForTx(tx);
    m_state = TRANSMITTING;
    m_bytesOutstanding = 0;
    m_timer.start();
}

void HalfDuplexSerial::initGpioForTx(PinName pin)
{
    m_gpioForTx.pin = pin;
    if (pin == (PinName)NC)
        return;

    m_gpioForTx.mask = 1 << ((int)pin & 0x1F);
    LPC_GPIO_TypeDef *port_reg = (LPC_GPIO_TypeDef *)((int)pin & ~0x1F);
    m_gpioForTx.reg_set = &port_reg->FIOSET;
    m_gpioForTx.reg_clr = &port_reg->FIOCLR;
    m_gpioForTx.reg_in  = &port_reg->FIOPIN;
    m_gpioForTx.reg_dir = &port_reg->FIODIR;
}

void HalfDuplexSerial::write(const void* pWrite, size_t writeSize)
{
    setState(TRANSMITTING);

    const uint8_t* pSrc = (uint8_t*)pWrite;
    while (writeSize-- > 0)
    {
        emptyRxOfEchoedBytes();
        _base_putc(*pSrc++);
        m_bytesOutstanding++;
    }
}

void HalfDuplexSerial::setState(State newState)
{
    if (newState == m_state)
    {
        // Already in the correct state so just return.
        return;
    }

    if (newState == RECEIVING)
    {
        switchToReceive();
    }
    else
    {
        switchToTransmit();
    }
}

void HalfDuplexSerial::switchToReceive()
{
    while (!isLastTransmitComplete() || m_bytesOutstanding > 0)
    {
        emptyRxOfEchoedBytes();
    }

    // Switch UART Tx pin to become a GPIO input pin to place it in high impedance mode.
    gpio_dir(&m_gpioForTx, PIN_INPUT);
    pin_function(m_gpioForTx.pin, 0);

    m_state = RECEIVING;
}

bool HalfDuplexSerial::isLastTransmitComplete()
{
    const uint32_t TEMT = 1 << 6;

    return (_serial.uart->LSR & TEMT) != 0;
}

void HalfDuplexSerial::switchToTransmit()
{
    static const PinMap PinMap_UART_TX[] = {
        {P0_0,  UART_3, 2},
        {P0_2,  UART_0, 1},
        {P0_10, UART_2, 1},
        {P0_15, UART_1, 1},
        {P0_25, UART_3, 3},
        {P2_0 , UART_1, 2},
        {P2_8 , UART_2, 2},
        {P4_28, UART_3, 3},
        {NC   , NC    , 0}
    };

    emptyRx();
    pinmap_pinout(m_gpioForTx.pin, PinMap_UART_TX);

    m_state = TRANSMITTING;
}

void HalfDuplexSerial::emptyRx()
{
    while (readable())
    {
        _base_getc();
    }
}

void HalfDuplexSerial::emptyRxOfEchoedBytes()
{
    assert ( m_state == TRANSMITTING );

    // Discard echoed bytes while in transmit mode.
    while (readable() && m_bytesOutstanding > 0)
    {
        _base_getc();
        m_bytesOutstanding--;
    }
}

size_t HalfDuplexSerial::read(void* pRead, size_t readSize, int msecTimeout)
{
    setState(RECEIVING);

    uint8_t* pDest = (uint8_t*)pRead;
    size_t bytesRead = 0;
    while (bytesRead < readSize)
    {
        int byte = readByteWithTimeout(msecTimeout);
        if (byte == -1)
        {
            break;
        }
        *pDest++ = byte;
        bytesRead++;
    }
    return bytesRead;
}

int HalfDuplexSerial::readByteWithTimeout(uint32_t msecTimeout)
{
    m_timer.reset();
    while (!readable() && (uint32_t)m_timer.read_ms() < msecTimeout)
    {
        // Wait for byte to be available for reading...
    }
    if (readable())
    {
        return _base_getc();
    }

    // Get here if we timed out waiting for data to read.
    return -1;
}
