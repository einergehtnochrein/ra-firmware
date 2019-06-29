/* Copyright (c) 2018, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions
 * and the following disclaimer in the documentation and/or other materials provided with the
 * distribution.
 * Neither the name of the author nor the names of its contributors may be used to endorse
 * or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "config.h"

#include "bl652.h"



struct BL652_Context {
    UART_Handle uart;
    GPIO_Pin gpioNAUTORUN;
    GPIO_Pin gpioSIO02;
    GPIO_Pin gpioNRESET;
    uint32_t baudrate;
    struct {
        bool success;
        char firmwareVersion[40];
        char deviceName[80];
        char macAddress[40];
    } response;
} _bl652Context;


static void _BL652_processRx (BL652_Handle handle)
{
    char s[80];
    int code;
    int param;
    int readPos;

    if (UART_readLine(handle->uart, s, sizeof(s)) > 0) {
        if (sscanf(s, "%d", &code) == 1) {
            switch (code) {
                case 0:
                    handle->response.success = true;
                    break;

                case 10:
                    if (sscanf(s, "%*d %d", &param) == 1) {
                        switch (param) {
                            case 3:
                                sscanf(s, "%*d %*d %s", handle->response.firmwareVersion);
                                break;
                            case 4:
                                sscanf(s, "%*d %*d %*s %s", handle->response.macAddress);
                                break;
                        }
                    }
                    break;

                case 27:
                    sscanf(s, "%*d %s%n", handle->response.deviceName, &readPos);
                    strncat(handle->response.deviceName, &s[readPos], sizeof(handle->response.deviceName));
                    /* Remove trailing LF */
                    if (strlen(handle->response.deviceName) >= 1) {
                        handle->response.deviceName[strlen(handle->response.deviceName) - 1] = 0;
                    }
                    break;
            }
        }
    }
}


static void _BL652_initResponse (BL652_Handle handle)
{
    memset(&handle->response, 0, sizeof(handle->response));
}


LPCLIB_Result BL652_setMode (BL652_Handle handle, int mode)
{
    switch (mode) {
        case BL652_MODE_COMMAND:
            GPIO_writeBit(handle->gpioSIO02, 0);
            GPIO_writeBit(handle->gpioNAUTORUN, 1);
            GPIO_writeBit(handle->gpioNRESET, 0);
            osDelay(100);
            GPIO_writeBit(handle->gpioNRESET, 1);
            break;

        case BL652_MODE_VSP_COMMAND:
            GPIO_writeBit(handle->gpioSIO02, 1);
            GPIO_writeBit(handle->gpioNAUTORUN, 0);
            GPIO_writeBit(handle->gpioNRESET, 0);
            osDelay(100);
            GPIO_writeBit(handle->gpioNRESET, 1);
            break;

        case BL652_MODE_VSP_BRIDGE:
            GPIO_writeBit(handle->gpioSIO02, 1);
            GPIO_writeBit(handle->gpioNAUTORUN, 1);
            GPIO_writeBit(handle->gpioNRESET, 0);
            osDelay(100);
            GPIO_writeBit(handle->gpioNRESET, 1);
            break;
    }

    return LPCLIB_SUCCESS;
}


LPCLIB_Result BL652_open (
    UART_Handle uart,
    GPIO_Pin gpioNAUTORUN,
    GPIO_Pin gpioSIO02,
    GPIO_Pin gpioNRESET,
    BL652_Handle *pHandle)
{
    BL652_Handle handle = &_bl652Context;
    *pHandle = handle;

    handle->uart = uart;
    handle->gpioNAUTORUN = gpioNAUTORUN;
    handle->gpioSIO02 = gpioSIO02;
    handle->gpioNRESET = gpioNRESET;

    return LPCLIB_SUCCESS;
}


static UART_Config uartConfigBaudrate[] = {
    {.opcode = UART_OPCODE_SET_BAUDRATE,
        {.baudrate = 115200,}},

    UART_CONFIG_END
};


static bool _BL652_testBaudrate (BL652_Handle handle, uint32_t baudrate)
{
    handle->baudrate = baudrate;

    uartConfigBaudrate[0].baudrate = baudrate;
    UART_ioctl(handle->uart, uartConfigBaudrate);
    osDelay(10);
    _BL652_initResponse(handle);
    UART_write(handle->uart, "\r", 1);
    for (int delay = 0; delay < 10; delay++) {
        osDelay(10);
        _BL652_processRx(handle);
        if (handle->response.success) {
            return true;
        }
    }

    return false;
}


LPCLIB_Result BL652_findBaudrate (BL652_Handle handle)
{
    BL652_setMode(handle, BL652_MODE_COMMAND);

    if (!_BL652_testBaudrate(handle, 115200)) {
        if (!_BL652_testBaudrate(handle, 230400)) {
            if (!_BL652_testBaudrate(handle, 460800)) {
                return LPCLIB_ERROR;
            }
        }
    }

    return LPCLIB_SUCCESS;
}


#define BL652_REQ_FIRMWARE_VERSION          "AT I 3\r"
#define BL652_REQ_MAC_ADDRESS               "AT I 4\r"
#define BL652_REQ_GET_DEVICE_NAME           "AT+CFGEX 117 ?\r"
#define BL652_REQ_SET_DEVICE_NAME           "AT+CFGEX 117 \""


LPCLIB_Result BL652_readParameters (BL652_Handle handle)
{
    int delay;

    BL652_setMode(handle, BL652_MODE_COMMAND);
    _BL652_initResponse(handle);

    if (_BL652_testBaudrate(handle, handle->baudrate)) {
        UART_write(handle->uart, BL652_REQ_MAC_ADDRESS, strlen(BL652_REQ_MAC_ADDRESS));
        for (delay = 0; delay < 10; delay++) {
            osDelay(10);
            _BL652_processRx(handle);
        }

        UART_write(handle->uart, BL652_REQ_FIRMWARE_VERSION, strlen(BL652_REQ_FIRMWARE_VERSION));
        for (delay = 0; delay < 10; delay++) {
            osDelay(10);
            _BL652_processRx(handle);
        }

        UART_write(handle->uart, BL652_REQ_GET_DEVICE_NAME, strlen(BL652_REQ_GET_DEVICE_NAME));
        for (delay = 0; delay < 10; delay++) {
            osDelay(10);
            _BL652_processRx(handle);
        }

        return LPCLIB_SUCCESS;
    }

    return LPCLIB_ERROR;
}



LPCLIB_Result BL652_updateParameters (BL652_Handle handle)
{
    char name[80];

    /* Update device name?
     * Were we able to read it from the module? (If not we can't update)
     * Is there a name in the configuration? (If not, leave default)
     * Is the actual device name different from the requested one? (If so, do the update)
     */
    if (strlen(handle->response.deviceName) > 0) {
        strncpy(name, config_g->nameBluetooth, sizeof(name));
        if (strlen(name) > 0) {
            if (strcmp(name, handle->response.deviceName) != 0) {
                UART_write(handle->uart, BL652_REQ_SET_DEVICE_NAME, strlen(BL652_REQ_SET_DEVICE_NAME));
                UART_write(handle->uart, name, strlen(name));
                UART_write(handle->uart, "\"\r", 2);
                osDelay(100);
            }
        }
    }

    return LPCLIB_SUCCESS;
}


LPCLIB_Result BL652_getFirmwareVersion (BL652_Handle handle, uint32_t *pFirmwareVersion)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (pFirmwareVersion == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *pFirmwareVersion = 0;

    int versionBytes[4];
    if (sscanf(handle->response.firmwareVersion,
                "%d.%d.%d.%d",
                &versionBytes[0],
                &versionBytes[1],
                &versionBytes[2],
                &versionBytes[3]) == 4) {

        *pFirmwareVersion = 0
            | (versionBytes[0] << 24)
            | (versionBytes[1] << 16)
            | (versionBytes[2] << 8)
            | (versionBytes[3] << 0)
            ;
    }

    return LPCLIB_SUCCESS;
}