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
/* Acts as a bridge between local TCP/IP and remote blemri devices.
   An application like GDB can open a socket to which this server is listening
   and be connected to a remote serial device over Bluetooth Low Energy.
*/
#include <errno.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "bleuart.h"

static int initListenSocket(uint16_t portNumber);
static int socketHasDataToRead(int socket);

int main(int argc, char *argv[])
{
    /*
       Initialize the Core Bluetooth stack on this the main thread and start the worker robot thread to run the
       code found in workerMain() below.
    */
    bleuartInitAndRun();
    return 0;
}

void workerMain(void)
{
    static const uint16_t portNumber = 3333;
    int                   isBleConnected = 0;
    int                   result = -1;
    int                   listenSocket = -1;
    int                   socket = -1;

    listenSocket = initListenSocket(portNumber);
    if (listenSocket == -1)
    {
        printf("error: Failed to initialize socket. %s\n", strerror(errno));
        goto Error;
    }
    while (1)
    {
        size_t             bytesRead;
        struct sockaddr_in remoteAddress;
        socklen_t          remoteAddressSize = sizeof(remoteAddress);
        uint8_t            buffer[4096];

        if (!isBleConnected)
        {
            printf("Connecting to BLEUART device.\n");
            result = bleuartConnect(NULL);
            if (result)
            {
                printf("error: Failed to connect to remote BLEUART device.\n");
                goto Error;
            }
            printf("Connected...\n");
            isBleConnected = 1;
        }
        if (socket == -1)
        {
            printf("Waiting for TCP/IP connection on port %d...\n", portNumber);
            socket = accept(listenSocket, (struct sockaddr*)&remoteAddress, &remoteAddressSize);
            if (socket == -1)
            {
                printf("error: Failed to accept TCP/IP connection. %s\n", strerror(errno));
                goto Error;
            }
            printf("Connected...\n");
        }

        while (socket != -1 && isBleConnected)
        {
            /* Forward data received from BLEUART to socket. */
            result = bleuartReceiveData(buffer, sizeof(buffer), &bytesRead);
            if (result == BLEUART_ERROR_NOT_CONNECTED)
            {
                printf("BLE no longer connected.\n");
                isBleConnected = 0;
                break;
            }
            if (bytesRead > 0)
            {
                printf("tcp<-ble:%.*s\n", (int)bytesRead, buffer);
                result = send(socket, buffer, bytesRead, 0);
                if (result == -1)
                {
                    printf("TCP/IP connection lost.\n");
                    close(socket);
                    socket = -1;
                    break;
                }
            }

            /* Forward data received from socket to BLEUART. */
            if (socketHasDataToRead(socket))
            {
                result = recv(socket, buffer, sizeof(buffer), 0);
                if (result < 1)
                {
                    printf("TCP/IP connection lost.\n");
                    close(socket);
                    socket = -1;
                    break;
                }
                printf("tcp->ble:%.*s\n", result, buffer);
                result = bleuartTransmitData(buffer, result);
                if (result == BLEUART_ERROR_NOT_CONNECTED)
                {
                    printf("BLE no longer connected.\n");
                    isBleConnected = 0;
                    break;
                }
            }

            /* This is a little more time than it takes to transmit one byte at 115200. */
            usleep(100);
        }
    }

Error:
    if (socket != -1)
        close (socket);
    if (listenSocket != -1)
        close(listenSocket);
    bleuartDisconnect();
}

static int initListenSocket(uint16_t portNumber)
{
    int optionValue = 1;
    int listenSocket = -1;
    int result = -1;
    struct sockaddr_in bindAddress;

    listenSocket = socket(PF_INET, SOCK_STREAM, 0);
    if (listenSocket == -1)
        goto Error;
    setsockopt(listenSocket, SOL_SOCKET, SO_REUSEADDR, &optionValue, sizeof(optionValue));

    memset(&bindAddress, 0, sizeof(bindAddress));
    bindAddress.sin_family = AF_INET;
    bindAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    bindAddress.sin_port = htons(portNumber);
    result = bind(listenSocket, (struct sockaddr *)&bindAddress, sizeof(bindAddress));
    if (result == -1)
        goto Error;

    result = listen(listenSocket, 0);
    if (result == -1)
        goto Error;

    return listenSocket;

Error:
    if (listenSocket != -1)
        close(listenSocket);
    return -1;
}

static int socketHasDataToRead(int socket)
{
    int            result = -1;
    struct timeval zeroTimeout = {0, 0};
    fd_set         readSet;

    FD_ZERO(&readSet);
    FD_SET(socket, &readSet);
    result = select(socket + 1, &readSet, NULL, NULL, &zeroTimeout);
    if (result == -1)
        return 0;
    return result;
}
