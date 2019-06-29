/* Copyright (c) 2017, DF9DQ
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

#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>

#include "lpclib.h"

#include "usb_audio.h"



struct _USBAUDIO_Context {
    USBD_HANDLE_T hUsb;
    const USBAUDIO_FunctionDeclaration *pFunction;
    const USBAUDIO_MemoryModel *memModel;
    void *memModelData;
} usbaudio;


/** Determine the index of the given control in the list.
 *
 *  Search the list of known controls, and return the index of the given control into that list.
 *
 *  \param [in] unit Index of the unit (feature, mixer, terminal, ...)
 *  \param [in] control Identifies the control (volume, mute, ...)
 *  \param [out] index Index in the control list
 *
 *  \return true=control found; false=control not found
 */
static _Bool USBAUDIO_findControl (USBAUDIO_Handle handle,
                                   uint8_t unit,
                                   uint8_t controlId,
                                   USBAUDIO_ControlRange **control)
{
    uint8_t n;


    /* Search the list */
    for (n = 0; n < handle->pFunction->controlInterface->numControls; n++) {
        if ((handle->pFunction->controlInterface->controls[n].unit_id == unit) &&
            (handle->pFunction->controlInterface->controls[n].control_id == controlId)) {
            /* Found it! */
            *control = &(handle->pFunction->controlInterface->controls[n]);
            return true;
        }
    }

    /* This is an unknown unit/control. */
    return false;
}



/** Handle audio class requests to the control interface.
 *
 *  \param [in] instance The instance of this audio class (starts with 0).
 *  \param [in] setup The setup packet
 *  \param [out] length Length of data to be returned. Maximum possible length is in setup->wLength.
 *  \param [out] data Points to the buffer that can take the data. Buffer must be able to take
 *                    setup->wLength bytes.
 *  \return
 */
static ErrorCode_t _USBAUDIO_handleControlRequests (
                        USBAUDIO_Handle handle,
                        USB_SETUP_PACKET *pSetup)
{
    USBAUDIO_ControlRange *pControl;
    struct _USB_CORE_CTRL_T *core = (struct _USB_CORE_CTRL_T *)(handle->hUsb);
    ErrorCode_t ret = LPC_OK;
    LPCLIB_Event event;


    /* Prepare event */
    event.id = LPCLIB_EVENTID_USBAUDIO;

    /* Is there a control like this? */
    if (USBAUDIO_findControl(handle,
                             pSetup->wIndex.WB.H,
                             pSetup->wValue.WB.H,
                             &pControl)) {
        switch (pSetup->bRequest) {
        case GET_MIN:
            core->EP0Data.pData = (uint8_t *)&pControl->min;
            core->EP0Data.Count = pControl->length;
            pRom->pUsbd->core->DataInStage(handle->hUsb);
            break;

        case GET_MAX:
            core->EP0Data.pData = (uint8_t *)&pControl->max;
            core->EP0Data.Count = pControl->length;
            pRom->pUsbd->core->DataInStage(handle->hUsb);
            break;

        case GET_CUR:
            core->EP0Data.pData = (uint8_t *)&pControl->current;
            core->EP0Data.Count = pControl->length;
            pRom->pUsbd->core->DataInStage(handle->hUsb);
            break;

        case GET_RES:
            core->EP0Data.pData = (uint8_t *)&pControl->resolution;
            core->EP0Data.Count = pControl->length;
            pRom->pUsbd->core->DataInStage(handle->hUsb);
            break;

        case SET_CUR:
            /* Wait until all data has arrived */
            if (core->EP0Data.Count == 0) {
                if (pControl->length == 1) {
                    pControl->current = core->EP0Buf[0];
                }
                else {
                    pControl->current = 0
                            | (core->EP0Buf[0] << 0)
                            | (core->EP0Buf[1] << 8)
                            ;
                }

                pRom->pUsbd->core->StatusInStage(handle->hUsb);

                /* Check if there is a callback */
                if (pControl->callback) {
                    event.opcode = USBAUDIO_EVENT_SET_CONTROL;
                    event.block = pSetup->wIndex.WB.H;
                    event.channel = pControl->control_id;
                    event.parameter = (void *)((int32_t)pControl->current);
                    pControl->callback(event);
                }
            }
            else {
                core->EP0Data.pData = core->EP0Buf;
            }
            break;

        case SET_RES:
            /* Wait until all data has arrived */
            if (core->EP0Data.Count == 0) {
                /* Value ignored */
                ;

                pRom->pUsbd->core->StatusInStage(handle->hUsb);
            }
            else {
                core->EP0Data.pData = core->EP0Buf;
            }
            break;

        default:
//            result = LPCLIB_ILLEGAL_PARAMETER;
            break;
        }
    }
    else {
        ret = ERR_USBD_UNHANDLED;   //TODO: error
    }

    return ret;
}



static const LPCLIB_Event eventSamplerate = {
    .id = LPCLIB_EVENTID_USBAUDIO,
    .opcode = USBAUDIO_EVENT_ENDPOINT,
    .channel = USBAC_CS_EP_SAMPLING_FREQ_CONTROL,
};



/** Handle audio class requests to an endpoint..
 *
 *  \param [in] instance The instance of this audio class (starts with 0).
 *  \param [in] setup The setup packet
 *  \param [out] length Length of data to be returned. Maximum possible length is in setup->wLength.
 *  \param [out] data Points to the buffer that can take the data. Buffer must be able to take
 *                    setup->wLength bytes.
 *  \return
 */
static ErrorCode_t _USBAUDIO_handleEndpointRequests (
            USBAUDIO_Handle handle,
            USB_SETUP_PACKET *pSetup)
{
    uint8_t n, m;
//    uint32_t temp;
    USBAUDIO_StreamingInterface *streaming;
    bool found;
    struct _USB_CORE_CTRL_T *core = (struct _USB_CORE_CTRL_T *)(handle->hUsb);
    LPCLIB_Event event;
    ErrorCode_t ret = ERR_USBD_UNHANDLED;


    /* Determine the interface to which this endpoint belongs. */
    streaming = NULL;
    found = false;
    for (n = 0; n < handle->pFunction->numStreamingInterfaces; n++) {
        streaming = &(handle->pFunction->streamingInterfaces[n]);
        for (m = 0; m < streaming->numAltSettings; m++) {
            if (streaming->params[m].numEndpoints > 0) {
                if (streaming->params[m].endpointNumber == pSetup->wIndex.WB.L) {
                    found = true;
                    break;
                }
            }
        }

        if (found ) {
            break;
        }

    }

    if (!found) {
        /* Unknown endpoint! */
        return ERR_USBD_UNHANDLED;  //TODO error
    }

    switch (pSetup->bRequest) {
    case SET_CUR:
        /* Wait until all data has arrived */
        if (core->EP0Data.Count == 0) {
            if ((pSetup->wLength == 3) && (pSetup->wValue.WB.H == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
                streaming->currentSamplerate = 0
                        | (core->EP0Buf[0] << 0)
                        | (core->EP0Buf[1] << 8)
                        | (core->EP0Buf[2] << 16)
                        ;
                pRom->pUsbd->core->StatusInStage(handle->hUsb);
                ret = LPC_OK;

                if (handle->pFunction->callback) {
                    event = eventSamplerate;
        //            event.block = pSetup->endpoint;
//                    event.parameter = (void *)((uint32_t)temp);
        //event.channel = pSetup->wValue >> 8;
                    handle->pFunction->callback(event);
                }
            }
        }
        else {
            /* Check for supported requests */
            if ((pSetup->wLength == 3) && (pSetup->wValue.WB.H == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
                core->EP0Data.pData = core->EP0Buf;
                ret = LPC_OK;
            }
        }
        break;

    case GET_CUR:
        if ((pSetup->wLength == 3) && (pSetup->wValue.WB.H == USBAC_CS_EP_SAMPLING_FREQ_CONTROL)) {
            core->EP0Data.pData = (uint8_t *)&streaming->currentSamplerate;
            core->EP0Data.Count = 3;
            pRom->pUsbd->core->DataInStage(handle->hUsb);
            ret = LPC_OK;
        }
        else {
            ret = ERR_USBD_UNHANDLED;          /* TODO unsupported */
        }
        break;
    }

    return ret;
}






/** Handle audio class requests.
 *
 *  \param [in] classInstance The instance of this audio class (starts with 0).
 *  \param [in] setup The setup packet
 *  \param [in] buffer Data buffer
 *  \return
 */
static ErrorCode_t USBAUDIO_endpoint0Handler (USBD_HANDLE_T hUsb, void *data, uint32_t usbdEvent)
{
    USBAUDIO_Handle handle = (USBAUDIO_Handle) data;
    struct _USB_CORE_CTRL_T *core = (struct _USB_CORE_CTRL_T *)hUsb;
    USB_SETUP_PACKET *pSetup = &core->SetupPacket;
    ErrorCode_t ret = ERR_USBD_UNHANDLED;
    LPCLIB_Event event;
    int n;

    LPCLIB_initEvent(&event, LPCLIB_EVENTID_USBAUDIO);

    if (usbdEvent == USB_EVT_SETUP) {
        /* Filter standard requests. This gives us a chance to follow changes in alternate settings. */
        if ((pSetup->bmRequestType.BM.Type == REQUEST_STANDARD) &&
            (pSetup->bRequest == USB_REQUEST_SET_INTERFACE)) {
            for (n = 0; n < handle->pFunction->numStreamingInterfaces; n++) {
                if (handle->pFunction->streamingInterfaces[n].interfaceNumber == pSetup->wIndex.WB.L) {
                    handle->pFunction->streamingInterfaces[n].activeSetting = pSetup->wValue.WB.L;


if (handle->pFunction->streamingInterfaces[n].activeSetting == 0) {
    LPC_USB0->INTEN &= ~(1u << 30);
}
else {
    LPC_USB0->INTEN |= (1u << 30);
}

                    /* Inform the application */
                    if (handle->pFunction->callback) {
#if 1
                        event.opcode = USBAUDIO_EVENT_INTERFACE_CHANGE;
                        event.block = pSetup->wIndex.WB.L;
                        event.channel = pSetup->wValue.WB.L;
//                        event.parameter = pBuffer;
                        LPCLIB_Result callbackResult = handle->pFunction->callback(event);
                        if (callbackResult != LPCLIB_SUCCESS) {
    //                        result = callbackResult;
                        }
#endif
                    }
                }
            }
        }
        if ((pSetup->bmRequestType.BM.Type == REQUEST_STANDARD) &&
            (pSetup->bRequest == USB_REQUEST_SET_CONFIGURATION)) {

            /* Inform the application */
            if (handle->pFunction->callback) {
                event.opcode = USBAUDIO_EVENT_CONFIGURATION_CHANGE;
                event.channel = pSetup->wValue.WB.L;
//                event.parameter = pBuffer;
//                callbackResult = handle->pFunction->callback(event);
//                if (callbackResult != LPCLIB_SUCCESS) {
    //                result = callbackResult;
//                }
            }
        }

        /* Events handled by audio class */
        if ((pSetup->bmRequestType.BM.Type == REQUEST_CLASS) &&
            (pSetup->bmRequestType.BM.Recipient == REQUEST_TO_INTERFACE) &&
            (pSetup->wIndex.WB.L == handle->pFunction->controlInterface->interfaceNumber)) {

            ret = _USBAUDIO_handleControlRequests(handle, pSetup);
        }
        if ((pSetup->bmRequestType.BM.Type == REQUEST_CLASS) &&
            (pSetup->bmRequestType.BM.Recipient == REQUEST_TO_ENDPOINT)) {
            for (n = 0; n < handle->pFunction->numEndpoints; n++) {
                if (handle->pFunction->pEndpointList[n] == pSetup->wIndex.WB.L) {
                    ret = _USBAUDIO_handleEndpointRequests(handle, pSetup);
                }
            }
        }
    }

    if (usbdEvent == USB_EVT_OUT) {
        /* Events handled by audio class */
        if ((pSetup->bmRequestType.BM.Type == REQUEST_CLASS) &&
            (pSetup->bmRequestType.BM.Recipient == REQUEST_TO_INTERFACE) &&
            (pSetup->wIndex.WB.L == handle->pFunction->controlInterface->interfaceNumber)) {

            ret = _USBAUDIO_handleControlRequests(handle, pSetup);
        }
        if ((pSetup->bmRequestType.BM.Type == REQUEST_CLASS) &&
            (pSetup->bmRequestType.BM.Recipient == REQUEST_TO_ENDPOINT)) {
            for (n = 0; n < handle->pFunction->numEndpoints; n++) {
                if (handle->pFunction->pEndpointList[n] == pSetup->wIndex.WB.L) {
                    ret = _USBAUDIO_handleEndpointRequests(handle, pSetup);
                }
            }
        }
    }

    return ret;
}


static ErrorCode_t USBAUDIO_endpointInHandler (USBD_HANDLE_T hUsb, void *data, uint32_t usbdEvent)
{
    (void)hUsb;
    (void)data;
    (void)usbdEvent;

    return LPC_OK;
}


void USBAUDIO_frameHandler (USBAUDIO_Handle handle, int frameNumber)
{
    (void) frameNumber;

    if (!handle->memModel) {
        return;
    }

    if (!handle->memModel->getNextBuffer) {
        return;
    }

    uint32_t address;
    uint32_t length;
    handle->memModel->getNextBuffer(handle->memModelData, &address, &length);
    if (length > 0) {
        //TODO
        uint32_t temp1, temp2;
        bool active1, active2;

        temp1 = ((volatile uint32_t *)0x04000000)[3*4+2];
        temp2 = ((volatile uint32_t *)0x04000000)[3*4+3];

        active1 = (temp1 & 0x80000000) != 0;
        active2 = (temp2 & 0x80000000) != 0;

        if (!active1) {
            temp1 = (temp1 & ~0x83FFFFFF)
                | ((address >> 6) & 0xFFFF)
                | (length << 16)
                | (1u << 31);
            ((volatile uint32_t *)0x04000000)[3*4+2] = temp1;
        }
        if (!active2) {
            temp2 = (temp2 & ~0x83FFFFFF)
                | ((address >> 6) & 0xFFFF)
                | (length << 16)
                | (1u << 31);
            ((volatile uint32_t *)0x04000000)[3*4+3] = temp2;
        }
    }
}


/* Open a USB Audio Class instance. */
ErrorCode_t USBAUDIO_init(USBD_HANDLE_T hUsb,
                          const USBAUDIO_FunctionDeclaration *pFunction,
                          const USBAUDIO_MemoryModel *pMemModel,
                          void *memModelData,
                          USBAUDIO_Handle *pHandle)
{
    USBAUDIO_Handle handle = &usbaudio;
    ErrorCode_t ret = LPC_OK;

    handle->pFunction = pFunction;
    handle->hUsb = hUsb;
    handle->memModel = pMemModel;
    handle->memModelData = memModelData;

    if (handle->memModel) {
        if (handle->memModel->init) {
            handle->memModel->init(handle->memModelData);
        }
    }

    /* Register audio class EP0 handler */
    if (ret == LPC_OK) {
        ret = pRom->pUsbd->core->RegisterClassHandler(handle->hUsb, USBAUDIO_endpoint0Handler, handle);
    }

    if (ret == LPC_OK) {
//TODO hard-coded...
        ret = pRom->pUsbd->core->RegisterEpHandler(handle->hUsb, 7, USBAUDIO_endpointInHandler, handle);
    }

    if (ret == LPC_OK) {
        if (pHandle) {
            *pHandle = handle;
        }
    }

    return ret;
}


