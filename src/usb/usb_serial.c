/*
 * @brief This file contains USB HID Consumer device example using USB ROM Drivers.
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include <string.h>

#include "lpclib.h"

#include "usbuser_config.h"


#define TX_BUFFER_SIZE1                     96
#define RX_BUFFER_SIZE1                     64
#define TX_BUFFER_SIZE2                     2048
#define RX_BUFFER_SIZE2                     64

struct _USBSerial_Context {
    USBD_HANDLE_T hUsb;
    USBD_HANDLE_T hCdc;

    uint8_t cif_if;
    uint8_t int_ep;
    uint8_t in_ep;
    uint8_t out_ep;

    uint8_t *pTxBuffer;
    uint32_t txBufferSize;
    int txWriteIndex;
    volatile int txReadIndex;
    int txPendingIndex;
    bool txBusy;
    uint8_t *pRxBuffer;
    uint32_t rxBufferSize;

    uint16_t controlLineState;
    bool sendNotification;
    struct {
        CDC_NOTIFICATION_HEADER header;
        uint16_t payload;
    } notification;
} usbSerialContext[1];


/* Set line coding call back routine */
static ErrorCode_t USBSerial_SetControlLineState (USBD_HANDLE_T hCDC, uint16_t state)
{
    struct _USBSerial_Context *handle = &usbSerialContext[0];
//    int count;


    if (hCDC == usbSerialContext[0].hCdc) {
        handle = &usbSerialContext[0];
    }
    else {
        return ERR_USBD_UNHANDLED;
    }

    /* Any change? */
    if (state ^ handle->controlLineState) {
        handle->controlLineState = state;

        handle->notification.payload = 0
                    | ((handle->controlLineState & 2) ? CDC_SERIAL_STATE_TX_CARRIER : 0)
                    | CDC_SERIAL_STATE_RX_CARRIER
                    ;
        handle->notification.header.bmRequestType.BM.Recipient = REQUEST_TO_INTERFACE;
        handle->notification.header.bmRequestType.BM.Type = REQUEST_CLASS;
        handle->notification.header.bmRequestType.BM.Dir = REQUEST_DEVICE_TO_HOST;
        handle->notification.header.bRequest = CDC_NOTIFICATION_SERIAL_STATE;
        handle->notification.header.wIndex.W = handle->cif_if;
        handle->notification.header.wLength = sizeof(handle->notification.payload);
        /*count =*/ pRom->pUsbd->hw->WriteEP(handle->hUsb, handle->int_ep, (uint8_t *)&handle->notification, sizeof(handle->notification));
    }

    return LPC_OK;
}





/* Set line coding call back routine */
static ErrorCode_t USBSerial_SetLineCode(USBD_HANDLE_T hCDC, CDC_LINE_CODING *line_coding)
{
#if 0
    uint32_t config_data = 0;

    switch (line_coding->bDataBits) {
    case 5:
        config_data |= UART0_LCR_WLEN5;
        break;
    case 6:
        config_data |= UART0_LCR_WLEN6;
        break;
    case 7:
        config_data |= UART0_LCR_WLEN7;
        break;
    case 8:
    default:
        config_data |= UART0_LCR_WLEN8;
        break;
    }

    switch (line_coding->bCharFormat) {
    case 1: /* 1.5 Stop Bits */
        /* In the UART hardware 1.5 stop bits is only supported when using 5
         * data bits. If data bits is set to 5 and stop bits is set to 2 then
         * 1.5 stop bits is assumed. Because of this 2 stop bits is not support
         * when using 5 data bits.
         */
        if (line_coding->bDataBits == 5) {
            config_data |= UART0_LCR_SBS_2BIT;
        }
        else {
            return ERR_USBD_UNHANDLED;
        }
        break;

    case 2: /* 2 Stop Bits */
        /* In the UART hardware if data bits is set to 5 and stop bits is set to 2 then
         * 1.5 stop bits is assumed. Because of this 2 stop bits is
         * not support when using 5 data bits.
         */
        if (line_coding->bDataBits != 5) {
            config_data |= UART0_LCR_SBS_2BIT;
        }
        else {
            return ERR_USBD_UNHANDLED;
        }
        break;

    default:
    case 0: /* 1 Stop Bit */
        config_data |= UART0_LCR_SBS_1BIT;
        break;
    }

    switch (line_coding->bParityType) {
    case 1:
        config_data |= (UART0_LCR_PARITY_EN | UART0_LCR_PARITY_ODD);
        break;

    case 2:
        config_data |= (UART0_LCR_PARITY_EN | UART0_LCR_PARITY_EVEN);
        break;

    case 3:
        config_data |= (UART0_LCR_PARITY_EN | UART0_LCR_PARITY_F_1);
        break;

    case 4:
        config_data |= (UART0_LCR_PARITY_EN | UART0_LCR_PARITY_F_0);
        break;

    default:
    case 0:
        config_data |= UART0_LCR_PARITY_DIS;
        break;
    }

    if (line_coding->dwDTERate < 3125000) {
        Chip_UART0_SetBaud(LPC_USART0, line_coding->dwDTERate);
    }
    Chip_UART0_ConfigData(LPC_USART0, config_data);
#else
    (void)hCDC;
    (void)line_coding;
#endif
    return LPC_OK;
}



/* UCOM bulk EP_IN and EP_OUT endpoints handler */
static ErrorCode_t USBSerial_endpointHandler (USBD_HANDLE_T hUsb, void *data, uint32_t event)
{
    struct _USBSerial_Context *handle = (struct _USBSerial_Context *) data;
    int count = 0;
    int wi, ri;

    switch (event) {
    case USB_EVT_IN:
        handle->txReadIndex = handle->txPendingIndex;

        /* More data to send? */
        wi = handle->txWriteIndex;
        ri = handle->txReadIndex;
        count = (wi >= ri) ? wi - ri : (int)handle->txBufferSize - ri;
        if (count > 64) {
            count = 64;
        }
        handle->txBusy = (count > 0);

        if (count) {
            count = pRom->pUsbd->hw->WriteEP(hUsb, handle->in_ep, &handle->pTxBuffer[handle->txReadIndex], count);
            handle->txPendingIndex = (handle->txReadIndex + count) % handle->txBufferSize;
        }
        break;

    case USB_EVT_OUT:
        //TODO just drop RX data for now
        pRom->pUsbd->hw->ReadEP(hUsb, handle->out_ep, handle->pRxBuffer);
        break;

    default:
        break;
    }

    return LPC_OK;
}



ErrorCode_t USBSerial_init(int port,
                           USBD_HANDLE_T hUsb,
                           const USB_INTERFACE_DESCRIPTOR *pCifIntfDesc,
                           const USB_INTERFACE_DESCRIPTOR *pDifIntfDesc,
                           uint32_t *mem_base,
                           uint32_t *mem_size)
{
    struct _USBSerial_Context *handle = &usbSerialContext[port];
    USBD_CDC_INIT_PARAM_T cdcParam;
    ErrorCode_t ret = LPC_OK;

    /* Do a quick check of if the interface descriptor passed is the right one. */
    if ((pCifIntfDesc == 0) || (pCifIntfDesc->bInterfaceClass != USB_DEVICE_CLASS_COMMUNICATIONS)) {
        return ERR_FAILED;
    }

    handle->hUsb = hUsb;

    /* Init CDC params */
    memset((void *) &cdcParam, 0, sizeof(cdcParam));
    cdcParam.mem_base = *mem_base;
    cdcParam.mem_size = *mem_size;
    cdcParam.cif_intf_desc = (uint8_t *)pCifIntfDesc;
    cdcParam.dif_intf_desc = (uint8_t *)pDifIntfDesc;
    cdcParam.SetCtrlLineState = USBSerial_SetControlLineState;
    cdcParam.SetLineCode = USBSerial_SetLineCode;
    ret = pRom->pUsbd->cdc->init(handle->hUsb, &cdcParam, &handle->hCdc);

    if (ret == LPC_OK) {
        if (port == 0) {
            /* Allocate buffers from USB memory */
            //TODO: check...
            handle->pTxBuffer = (uint8_t *)cdcParam.mem_base;
            handle->txBufferSize = TX_BUFFER_SIZE1;
            cdcParam.mem_base += TX_BUFFER_SIZE1;
            cdcParam.mem_size -= TX_BUFFER_SIZE1;
            handle->pRxBuffer = (uint8_t *)cdcParam.mem_base;
            handle->rxBufferSize = RX_BUFFER_SIZE1;
            cdcParam.mem_base += RX_BUFFER_SIZE1;
            cdcParam.mem_size -= RX_BUFFER_SIZE1;

            handle->cif_if = pCifIntfDesc->bInterfaceNumber;
            handle->int_ep = USBCONFIG_SERIAL_CIF_EP_INT;
            handle->in_ep = USBCONFIG_SERIAL_DIF_EP_IN;
            handle->out_ep = USBCONFIG_SERIAL_DIF_EP_OUT;

            /* Register IN endpoint handler */
            ret = pRom->pUsbd->core->RegisterEpHandler(handle->hUsb, 5, USBSerial_endpointHandler, handle); //TODO

            if (ret == LPC_OK) {
                /* Register OUT endpoint handler */
                ret = pRom->pUsbd->core->RegisterEpHandler(handle->hUsb, 4, USBSerial_endpointHandler, handle); //TODO
            }
        }
    }

    *mem_base = cdcParam.mem_base;
    *mem_size = cdcParam.mem_size;

    return ret;
}




void USBSerial_write (int port, const void *message, int len)
{
    struct _USBSerial_Context *handle = &usbSerialContext[port];
    int i;
    int wi;
    int ri;
    int count;
    int bytesFree;
    int requiredLen;

    if (!USBUSER_isConfigured()) {
        return;
    }

    /* Find out how many bytes are free in the TX buffer */
    wi = handle->txWriteIndex;
    ri = handle->txReadIndex;
    bytesFree = (ri > wi) ? ri - wi - 1 : (int)handle->txBufferSize - (wi - ri);

    /* Port numbers 2 and higher are virtual ports muxed on physical port 1. Header takes three bytes. */
    requiredLen = len;
    if (port >= 2) {
        requiredLen += 3;
    }

    /* Drop data if not enough space left */
    if (bytesFree < requiredLen) {
        return;
    }

    /* Copy into TX buffer */
    if (port >= 2) {
        handle->pTxBuffer[wi] = port - 2;
        wi = (wi + 1) % handle->txBufferSize;
        handle->pTxBuffer[wi] = len % 256;
        wi = (wi + 1) % handle->txBufferSize;
        handle->pTxBuffer[wi] = len / 256;
        wi = (wi + 1) % handle->txBufferSize;
    }
    for (i = 0; i < len; i++) {
        handle->pTxBuffer[wi] = ((const uint8_t *)message)[i];
        wi = (wi + 1) % handle->txBufferSize;
    }
    handle->txWriteIndex = wi;

    /* If TX machine is no longer running, kick-start it now */
    if (!handle->txBusy) {
        /* Calculate amount of data to send (max: 64 bytes) */
        wi = handle->txWriteIndex;
        ri = handle->txReadIndex;
        count = (wi >= ri) ? wi - ri : (int)handle->txBufferSize - ri;
        if (count > 64) {
            count = 64;
        }
        handle->txBusy = (count > 0);

        if (count) {
            count = pRom->pUsbd->hw->WriteEP(handle->hUsb, handle->in_ep, &handle->pTxBuffer[handle->txReadIndex], count);
            handle->txPendingIndex = (handle->txReadIndex + count) % handle->txBufferSize;
        }
    }
}

