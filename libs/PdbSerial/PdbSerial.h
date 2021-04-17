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
// Library to handle serial connection between robot microcontroller and Power Distribution Board.
//  Receives manual and auto packets from PDB. These are parsed byte by byte as they are received by UART ISR.
//  Sends informational text to be displayed on PDB's LCD. DMA is used to send this text in the background.
#ifndef PDB_SERIAL_H_
#define PDB_SERIAL_H_

#include <assert.h>
#include <stdarg.h>
#include <mbed.h>
#include <mri.h>
#include <GPDMA.h>
#include "../../include/PdbPacket.h"


class PdbSerial : protected RawSerial
{

public:
    PdbSerial(PinName tx, PinName rx);

    enum PacketType
    {
        PDBSERIAL_AUTO_PACKET,
        PDBSERIAL_MANUAL_PACKET
    };

    struct PdbSerialPacket
    {
        uint32_t    timestamp;
        PacketType  type;
        union PacketData
        {
            PdbSerialManualPacket _manual;
            PdbSerialAutoPacket   _auto;
        } data;
    };

    void outputStringToPDB(const char* pOutput);
    void printf(const char* pFormat, ...);
    void flush();
    void waitForNewPacket();
    PdbSerialPacket getLatestPacket();


protected:
    void serialReceive();
    void parseReceivedPacketByte(uint8_t byte);
    void dmaTransmit(const void* pData, size_t dataLength);

    Timer           m_timer;

    // The UART Rx interrupt handler fills in this packet structure, byte by byte.
    PdbSerialPacket m_dirtyPacket;
    // Once the whole packet has been received, it is copied into this one.
    PdbSerialPacket m_packet;

    // These members are used by the state machine which fills in m_dirtyPacket byte by byte as they are received in
    // UART Rx interrupt handler.
    enum UartRecvState
    {
        UART_RECV_PACKET_START,
        UART_RECV_PACKET_LENGTH,
        UART_RECV_PACKET_DATA,
        UART_RECV_IGNORE_DATA
    } m_recvPacketState;
    uint32_t m_recvPacketLength;
    uint32_t m_recvPacketIndex;

    // Number of received packets that were dropped because they weren't of a recognized type.
    uint32_t m_recvPacketIgnoredCount;


    LPC_GPDMACH_TypeDef*    m_pChannel;
    uint32_t                m_channelMask;
    bool                    m_transferInProgress;
    uint8_t                 m_buffer[128];

    // Packets that have a timestamp older than the following in milliseconds aren't considered new by waitForNewPacket().
    enum
    {
        RECENT_TIMESTAMP_THRESHOLD = 2
    };
};


PdbSerial::PdbSerial(PinName tx, PinName rx) : RawSerial(tx, rx, 115200)
{
    memset(&m_packet, 0, sizeof(m_packet));
    m_recvPacketState = UART_RECV_PACKET_START;
    m_recvPacketIgnoredCount = 0;
    m_recvPacketLength = 0;
    m_recvPacketIndex = 0;

    m_timer.start();

    static const uint32_t fifoEnable = (1 << 0);
    static const uint32_t dmaEnable = (1 << 3);

    // Allocate a GPDMA channel to use for this UART.
    int channelIndex = allocateDmaChannel(GPDMA_CHANNEL_LOW);
    assert ( channelIndex != -1 );
    m_pChannel = dmaChannelFromIndex(channelIndex);
    m_channelMask = 1 << channelIndex;

    // Enable DMA for UART.
    _serial.uart->FCR = fifoEnable | dmaEnable;
    m_transferInProgress = false;

    // Make sure that GPDMA block is enabled.
    enableGpdmaPower();
    enableGpdmaInLittleEndianMode();

    // Make sure that GPDMA is configured for UART and not TimerMatch.
    LPC_SC->DMAREQSEL &= ~(3 << (_serial.index * 2));

    // Configure interrupt handler for this UART.
    attach(callback(this, &PdbSerial::serialReceive), Serial::RxIrq);
}

void PdbSerial::serialReceive(void)
{
    while (readable())
    {
        parseReceivedPacketByte(getc());
    }
}

void PdbSerial::parseReceivedPacketByte(uint8_t byte)
{
    uint32_t length = 0;

    switch (m_recvPacketState)
    {
        case UART_RECV_PACKET_START:
            if (byte == PDBSERIAL_PACKET_SIGNATURE)
            {
                m_recvPacketState = UART_RECV_PACKET_LENGTH;
            }
            break;
        case UART_RECV_PACKET_LENGTH:
            length = byte;
            if (length == 0)
            {
                // Received a packet with no data so setup to wait for start of next packet.
                m_recvPacketState = UART_RECV_PACKET_START;
            }
            else if (length == sizeof(PdbSerialManualPacket) || length == sizeof(PdbSerialAutoPacket))
            {
                m_dirtyPacket.type = (length == sizeof(PdbSerialManualPacket)) ? PacketType::PDBSERIAL_MANUAL_PACKET :
                                                                                 PacketType::PDBSERIAL_AUTO_PACKET;
                m_recvPacketLength = length;
                m_recvPacketIndex = 0;
                m_recvPacketState = UART_RECV_PACKET_DATA;
            }
            else
            {
                // Unknown packet length/type so ignore it.
                m_recvPacketLength = length;
                m_recvPacketIndex = 0;
                m_recvPacketState = UART_RECV_IGNORE_DATA;
            }
            break;
        case UART_RECV_PACKET_DATA:
            {
                uint8_t* pData = (uint8_t*) &m_dirtyPacket.data;
                pData[m_recvPacketIndex++] = byte;
                if (m_recvPacketIndex >= m_recvPacketLength)
                {
                    // Have finished receiving packet data.
                    m_dirtyPacket.timestamp = m_timer.read_ms();
                    m_packet = m_dirtyPacket;
                    m_recvPacketState = UART_RECV_PACKET_START;
                }
            }
            break;
        case UART_RECV_IGNORE_DATA:
            m_recvPacketIndex++;
            if (m_recvPacketIndex >= m_recvPacketLength)
            {
                // Have finished receiving packet data.
                m_recvPacketIgnoredCount++;
                m_recvPacketState = UART_RECV_PACKET_START;
            }
            break;
    }
}

void PdbSerial::outputStringToPDB(const char* pOutput)
{
    size_t length = strlen(pOutput);

    // Packet contains overhead of PDBSERIAL_PACKET_SIGNATURE + length.
    if (length > sizeof(m_buffer) - 2)
    {
        // Truncate the text to what will fit in our buffer.
        length = sizeof(m_buffer) - 2;
    }

    // Wait for previous transmit (if any) to complete since we are using the same m_buffer.
    flush();

    m_buffer[0] = PDBSERIAL_PACKET_SIGNATURE;
    m_buffer[1] = (uint8_t)length;
    memmove(&m_buffer[2], pOutput, length);
    dmaTransmit(m_buffer, length + 2);
}

void PdbSerial::printf(const char* pFormat, ...)
{
    va_list argList;

    va_start(argList, pFormat);
        vsnprintf((char*)&m_buffer[2], sizeof(m_buffer)-2, pFormat, argList);
        outputStringToPDB((char*)&m_buffer[2]);
    va_end(argList);
}

void PdbSerial::dmaTransmit(const void* pData, size_t dataLength)
{
    uint32_t uartTx = DMA_PERIPHERAL_UART0TX_MAT0_0 + (_serial.index * 2);

    // Clear error and terminal complete interrupts for this UART's GPDMA channel.
    LPC_GPDMA->DMACIntTCClear = m_channelMask;
    LPC_GPDMA->DMACIntErrClr  = m_channelMask;

    // Prep GPDMA channel to send bytes via the UART.
    m_pChannel->DMACCSrcAddr  = (uint32_t)pData;
    m_pChannel->DMACCDestAddr = (uint32_t)&_serial.uart->THR;
    m_pChannel->DMACCLLI      = 0;
    m_pChannel->DMACCControl  = DMACCxCONTROL_I | DMACCxCONTROL_SI |
                     (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_SBSIZE_SHIFT) |
                     (DMACCxCONTROL_BURSTSIZE_1 << DMACCxCONTROL_DBSIZE_SHIFT) |
                     (dataLength & DMACCxCONTROL_TRANSFER_SIZE_MASK);

    // Enable GPDMA channel.
    m_pChannel->DMACCConfig = DMACCxCONFIG_ENABLE |
                   (uartTx << DMACCxCONFIG_DEST_PERIPHERAL_SHIFT) |
                   DMACCxCONFIG_TRANSFER_TYPE_M2P |
                   DMACCxCONFIG_IE |
                   DMACCxCONFIG_ITC;

    // Flag that we have a transaction in progress.
    m_transferInProgress = true;
}

void PdbSerial::flush()
{
    while (m_transferInProgress && (LPC_GPDMA->DMACIntStat & m_channelMask) == 0)
    {
    }
    m_transferInProgress = false;
}

void PdbSerial::waitForNewPacket()
{
    while (m_timer.read_ms() - m_packet.timestamp >= RECENT_TIMESTAMP_THRESHOLD)
    {
    }
}

PdbSerial::PdbSerialPacket PdbSerial::getLatestPacket()
{
    PdbSerialPacket packet;

    // Disable interrupts so that ISR doesn't overwrite part of it while we are copying it out.
    __disable_irq();
        packet = m_packet;
    __enable_irq();

    return packet;
}

#endif // PDB_SERIAL_H_
