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


#define BL652_FIRMWARE_VERSION(a,b,c,d) \
    (16777216*(a) + 65536*(b) + 256*(c) + (d))


struct BL652_Context {
    UART_Handle uart;
    GPIO_Pin gpioNAUTORUN;
    GPIO_Pin gpioSIO02;
    GPIO_Pin gpioNRESET;
    uint32_t baudrate;
    int *cfg_response_ptr;
    struct {
        bool success;
        uint32_t firmwareVersion;
        char deviceName[80];
        char macAddress[40];
        int advertInterval;
        int att_mtu;
        int att_data_length;
    } response;
} _bl652Context;


static void _BL652_processRx (BL652_Handle handle)
{
    char s[80];
    int code;
    int param;
    int readPos;
    int versionBytes[4];

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
                                if (sscanf(s, "%*d %*d %d.%d.%d.%d",
                                            &versionBytes[0],
                                            &versionBytes[1],
                                            &versionBytes[2],
                                            &versionBytes[3]) == 4) {
                                    handle->response.firmwareVersion = 0
                                        | (versionBytes[0] << 24)
                                        | (versionBytes[1] << 16)
                                        | (versionBytes[2] << 8)
                                        | (versionBytes[3] << 0)
                                        ;
                                }
                                break;
                            case 4:
                                sscanf(s, "%*d %*d %*s %s", handle->response.macAddress);
                                break;
                        }
                    }
                    break;

                case 27:
                    /* Try reading a numerical response */
                    if (handle->cfg_response_ptr != NULL) {
                        sscanf(s, "%*d %*X (%d)", handle->cfg_response_ptr);
                    } else {
                        /* Otherwise assume it's the response to the device name command */
                        sscanf(s, "%*d %s%n", handle->response.deviceName, &readPos);
                        strncat(handle->response.deviceName, &s[readPos], sizeof(handle->response.deviceName)-1);
                        /* Remove trailing LF */
                        if (strlen(handle->response.deviceName) >= 1) {
                            handle->response.deviceName[strlen(handle->response.deviceName) - 1] = 0;
                        }
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
            break;

        case BL652_MODE_VSP_COMMAND:
            GPIO_writeBit(handle->gpioSIO02, 1);
            GPIO_writeBit(handle->gpioNAUTORUN, 0);
            break;

        case BL652_MODE_VSP_BRIDGE:
            GPIO_writeBit(handle->gpioSIO02, 1);
            GPIO_writeBit(handle->gpioNAUTORUN, 1);
            break;
    }
    GPIO_writeBit(handle->gpioNRESET, 0);
    osDelay(5);
    GPIO_writeBit(handle->gpioNRESET, 1);
    osDelay(500);
    GPIO_writeBit(handle->gpioNAUTORUN, 0);
    GPIO_writeBit(handle->gpioSIO02, 0);

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
    handle->cfg_response_ptr = NULL;

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
#if (BOARD_RA == 1)     /* Don't try with Ra1. Leave it @ 115k2 */
        return LPCLIB_ERROR;
#else
        if (!_BL652_testBaudrate(handle, 230400)) {
            if (!_BL652_testBaudrate(handle, 460800)) {
                return LPCLIB_ERROR;
            }
        }
#endif
    }

    return LPCLIB_SUCCESS;
}


#define BL652_REQ_FIRMWARE_VERSION          "AT I 3\r"
#define BL652_REQ_MAC_ADDRESS               "AT I 4\r"
#define BL652_REQ_GET_DEVICE_NAME           "AT+CFGEX 117 ?\r"
#define BL652_REQ_SET_DEVICE_NAME           "AT+CFGEX 117 \""
#define BL652_ADVERT_INTERVAL               1000
#define BL652_REQ_GET_ADVERT_INTERVAL       "AT+CFG 113 ?\r"
#define BL652_REQ_SET_ADVERT_INTERVAL(x)    "AT+CFG 113 " #x "\r"
#define BL652_REQ_SET_ADVERT_INTERVAL2(x)   "AT+CFG 102 " #x "\r"
#define BL652_ATT_MTU                       247
#define BL652_REQ_GET_ATT_MTU               "AT+CFG 211 ?\r"
#define BL652_REQ_SET_ATT_MTU(x)            "AT+CFG 211 " #x "\r"
#define BL652_ATT_DATA_LENGTH               244
#define BL652_REQ_GET_ATT_DATA_LENGTH       "AT+CFG 212 ?\r"
#define BL652_REQ_SET_ATT_DATA_LENGTH(x)    "AT+CFG 212 " #x "\r"


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

        handle->cfg_response_ptr = NULL;
        UART_write(handle->uart, BL652_REQ_GET_DEVICE_NAME, strlen(BL652_REQ_GET_DEVICE_NAME));
        for (delay = 0; delay < 10; delay++) {
            osDelay(10);
            _BL652_processRx(handle);
        }

        handle->cfg_response_ptr = &handle->response.advertInterval;
        UART_write(handle->uart, BL652_REQ_GET_ADVERT_INTERVAL, strlen(BL652_REQ_GET_ADVERT_INTERVAL));
        for (delay = 0; delay < 10; delay++) {
            osDelay(10);
            _BL652_processRx(handle);
        }

        /* Data length extensions require a minimum firmware 28.6.2.0 */
        if (handle->response.firmwareVersion >= BL652_FIRMWARE_VERSION(28,6,2,0)) {
            handle->cfg_response_ptr = &handle->response.att_mtu;
            UART_write(handle->uart, BL652_REQ_GET_ATT_MTU, strlen(BL652_REQ_GET_ATT_MTU));
            for (delay = 0; delay < 10; delay++) {
                osDelay(10);
                _BL652_processRx(handle);
            }

            handle->cfg_response_ptr = &handle->response.att_data_length;
            UART_write(handle->uart, BL652_REQ_GET_ATT_DATA_LENGTH, strlen(BL652_REQ_GET_ATT_DATA_LENGTH));
            for (delay = 0; delay < 10; delay++) {
                osDelay(10);
                _BL652_processRx(handle);
            }
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

    if (handle->response.advertInterval != 1000) {
        UART_write(handle->uart, BL652_REQ_SET_ADVERT_INTERVAL(BL652_ADVERT_INTERVAL),
                                 strlen(BL652_REQ_SET_ADVERT_INTERVAL(BL652_ADVERT_INTERVAL)));
        osDelay(100);
        UART_write(handle->uart, BL652_REQ_SET_ADVERT_INTERVAL2(BL652_ADVERT_INTERVAL),
                                 strlen(BL652_REQ_SET_ADVERT_INTERVAL2(BL652_ADVERT_INTERVAL)));
        osDelay(100);
    }

    /* Data length extensions require a minimum firmware 28.6.2.0 */
    if (handle->response.firmwareVersion >= BL652_FIRMWARE_VERSION(28,6,2,0)) {
        if (handle->response.att_mtu != BL652_ATT_MTU) {
            UART_write(handle->uart, BL652_REQ_SET_ATT_MTU(BL652_ATT_MTU),
                                    strlen(BL652_REQ_SET_ATT_MTU(BL652_ATT_MTU)));
            osDelay(100);
        }

        if (handle->response.att_mtu != BL652_ATT_DATA_LENGTH) {
            UART_write(handle->uart, BL652_REQ_SET_ATT_DATA_LENGTH(BL652_ATT_DATA_LENGTH),
                                    strlen(BL652_REQ_SET_ATT_DATA_LENGTH(BL652_ATT_DATA_LENGTH)));
            osDelay(100);
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

    *pFirmwareVersion = handle->response.firmwareVersion;

    return LPCLIB_SUCCESS;
}
