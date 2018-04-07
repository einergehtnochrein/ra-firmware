/* Copyright (c) 2016, DF9DQ
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


#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "app.h"
#include "scanner.h"




#define SCANNER_QUEUE_LENGTH                10

typedef enum {
    SCANNER_MODE_OFF = 0,
    SCANNER_MODE_MANUAL,                        /* Manually controlled continuous operation on a single frequency */
    SCANNER_MODE_LIST,                          /* Walk through list of discrete frequencies */
} SCANNER_Mode;


typedef struct {
    uint8_t opcode;
    union {
        LPCLIB_Event event;
        int manualMode;
    };
} SCANNER_Message;


/* Message opcodes */
enum {
    SCANNER_OPCODE_EVENT,
    SCANNER_OPCODE_MANUAL_MODE,
};


/* Entry in scan list */
typedef struct _SCANNER_Item {
    struct _SCANNER_Item *next;
    float frequency;
    SONDE_Detector detector;
    uint32_t lastHeard;
} SCANNER_Item;



struct SCANNER_Context {
    struct pt pt;
    osMailQId queue;                                    /**< Task message queue */
    osEvent rtosEvent;

    SCANNER_Item *scanList;                             /* Preferred scan list */
    SCANNER_Item *scanCurrent;                          /* Current item in scan list */

    float manualFrequency;                              /* Frequency for manual operation */
    SONDE_Detector manualSondeDetector;
    bool scanner;
    bool scannerNeedInit;

    int preferredIndex;

    SCANNER_Mode mode;
    osTimerId scanTick;
    bool scanTickTimeout;

    uint32_t spectrumFrequency;
} scannerContext;




static void _SCANNER_getSpectrum (void)
{
    SCANNER_Handle handle = &scannerContext;
    const int N = 10;
    const int OVER = 100;
    const uint32_t grid = 10000;
    int i;
    int n;
    int o;
    float level[OVER];
    float avgLevel;
    char s[20 + 4*N];

    /* Prepare result string (start frequency) */
    sprintf(s, "%.3f,%.3f", handle->spectrumFrequency / 1e6f, grid / 1e6f);

    for (n = 0; n < N; n++) {
        /* Adjust next RX frequency */
        if ((handle->spectrumFrequency < 400000000) || (handle->spectrumFrequency >= 406100000)) {
            handle->spectrumFrequency = 400000000;
            break;
        }
 
        /* Set radio frequency */
        if (ADF7021_setPLL(radio, handle->spectrumFrequency - 100000) == LPCLIB_SUCCESS) {
            /* Measure channel power */
            avgLevel = 0;
            for (i = 0; i < OVER; i++) {
                SYS_readRssi(sys, &level[i]);
                avgLevel += level[i];
            }
            avgLevel /= OVER;

            o = 0;
            float denoised = 0;
            for (i = 0; i < OVER; i++) {
                if (fabs(level[i] - avgLevel) < 4.0f) {
                    denoised += level[i];
                    ++o;
                }
            }
            if (o > 0) {
                denoised /= o;
            }
            else {
                denoised = -140.0f;
            }
            sprintf(&s[strlen(s)], ",%.0f", (denoised + 140.0f) * 10.0f);
        }
        else {
            strcat(s, ",");
        }

        /* Next frequency */
        handle->spectrumFrequency += grid;
    }
 
    /* Send result */
    SYS_send2Host(HOST_CHANNEL_SPECTRUM, s);
}


/* Find the next QRG and sonde type to listen to. */
static bool _SCANNER_getNextQrg (SONDE_Detector *sondeDetector, float *frequency, uint32_t *durationMs)
{
    SCANNER_Handle handle = &scannerContext;
    bool result = true;


    switch (handle->mode) {
        case SCANNER_MODE_OFF:
            ; /* no action */
            break;

        case SCANNER_MODE_MANUAL:
            *sondeDetector = handle->manualSondeDetector;
            *frequency = handle->manualFrequency;
            *durationMs = 1000;
            break;

        case SCANNER_MODE_LIST:
            /* Are we at the end of the active sondes list?
            * Go to start of list or select next scan frequency (if band scan enabled) //TODO
            */
            if (!handle->scanCurrent) {
                handle->scanCurrent = handle->scanList;
            }
            if (handle->scanCurrent) {
                *sondeDetector = handle->scanCurrent->detector;
                *frequency = handle->scanCurrent->frequency;
                *durationMs = 2200;

                handle->scanCurrent = handle->scanCurrent->next;
            }
            else {
                result = false;
            }
            break;
    }

    return result;
}



static void _SCANNER_rtosCallback (void const *argument)
{
    (void) argument;

    scannerContext.scanTickTimeout = true;
}


static bool _SCANNER_checkEvent (SCANNER_Handle handle)
{
    bool haveEvent = false;


    /* Something in the mailbox? */
    handle->rtosEvent = osMailGet(handle->queue, 0);
    haveEvent |= handle->rtosEvent.status == osEventMail;

    haveEvent |= handle->scanTickTimeout;

    return haveEvent;
}


osMailQDef(scannerQueueDef, SCANNER_QUEUE_LENGTH, SCANNER_Message);



LPCLIB_Result SCANNER_open (SCANNER_Handle *pHandle)
{
    *pHandle = &scannerContext;
//    SCANNER_Handle handle = *pHandle;

//    SCANNER_addListenFrequency(handle, 402.9e6f, SONDE_RS92);
//    SCANNER_addListenFrequency(handle, 405.5e6f, SONDE_RS41);
//    SCANNER_addListenFrequency(handle, 402.5e6f, SONDE_RS92);

    return LPCLIB_SUCCESS;
}


void SCANNER_setManualMode (SCANNER_Handle handle, bool enable)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    if (enable) {
        handle->mode = SCANNER_MODE_MANUAL;
    }
    else {
        handle->mode = SCANNER_MODE_LIST;
    }

    /* Force checking for new frequency/type settings */
    handle->scanTickTimeout = true;
}


bool SCANNER_getManualMode (SCANNER_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return false;
    }

    return handle->mode == SCANNER_MODE_MANUAL;
}


void SCANNER_setScannerMode (SCANNER_Handle handle, bool enable)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    handle->scanner = enable;
    handle->scannerNeedInit = enable;
}


bool SCANNER_getScannerMode (SCANNER_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return false;
    }

    return handle->scanner;
}


void SCANNER_setManualFrequency (SCANNER_Handle handle, float frequency)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    handle->manualFrequency = frequency;

    /* Force checking for new frequency/type settings */
    handle->scanTickTimeout = true;
}


void SCANNER_setManualSondeDetector (SCANNER_Handle handle, SONDE_Detector sondeDetector)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    handle->manualSondeDetector = sondeDetector;

    /* Force checking for new frequency/type settings */
    handle->scanTickTimeout = true;
}


SONDE_Detector SCANNER_getManualSondeDetector (SCANNER_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return SONDE_UNDEFINED;
    }

    return handle->manualSondeDetector;
}


void SCANNER_addListenFrequency (SCANNER_Handle handle, float frequency, SONDE_Detector sondeDetector)
{
    SCANNER_Item *lastItem;
    SCANNER_Item *item;
    bool found;


    /* See if this entry already exists */
    found = false;
    item = handle->scanList;
    while (item && !found) {
        if ((item->frequency == frequency) && (item->detector == sondeDetector)) {
            found = true;
            item->lastHeard = os_time;          /* Mark as updated */
        }

        item = item->next;
    }

    /* Create new entry if not yet in list */
    if (!found) {
        item = (SCANNER_Item *)calloc(1, sizeof(SCANNER_Item));
        if (item) {
            item->frequency = frequency;
            item->detector = sondeDetector;
            item->lastHeard = os_time;

            /* Find out where to add this entry */
            if (!handle->scanList) {
                handle->scanList = item;
            }
            else {
                lastItem = handle->scanList;
                while (lastItem->next) {
                    lastItem = lastItem->next;
                }
                lastItem->next = item;
            }
        }
    }
}


void SCANNER_removeListenFrequency (SCANNER_Handle handle, float frequency)
{
    float frequencyKhz = roundf(frequency / 1e3f);

    /* Find matching list entry */
    SCANNER_Item **parent = &handle->scanList;
    SCANNER_Item *item = handle->scanList;
    while (item) {
        if (roundf(item->frequency / 1000.0f) == frequencyKhz) {
            *parent = item->next;           /* Remove from chain */
            free(item);
            break;
        }

        parent = &item->next;
        item = item->next;
    }
}


void SCANNER_notifyValidFrame (SCANNER_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    handle->scanTickTimeout = true; //TODO...
    printf("valid frame\r\n");
}


osTimerDef(scannerTickDef, _SCANNER_rtosCallback);


PT_THREAD(SCANNER_thread (SCANNER_Handle handle))
{
    PT_BEGIN(&handle->pt);

    scannerContext.queue = osMailCreate(osMailQ(scannerQueueDef), NULL);

    handle->manualFrequency = 405.1e6f;
    handle->manualSondeDetector = SONDE_DETECTOR_RS41_RS92;
//handle->mode = SCANNER_MODE_SEARCH;
handle->mode = SCANNER_MODE_MANUAL;

    handle->scanTick = osTimerCreate(osTimer(scannerTickDef), osTimerOnce, (void *)handle);
    osTimerStart(handle->scanTick, 10);

    while (1) {
        /* Wait for an event */
        PT_WAIT_UNTIL(&handle->pt, _SCANNER_checkEvent(handle));

        if (handle->scanTickTimeout) {
            osTimerStop(handle->scanTick);
            handle->scanTickTimeout = false;

            if (handle->scanner) {
                /* One-time init for scanner mode */
                if (handle->scannerNeedInit) {
                    //TODO must disable AFC. SRSC detector does this...
                    SYS_enableDetector(sys, 400e6f, SONDE_DETECTOR_C34_C50);
                    handle->scannerNeedInit = false;
                }

                _SCANNER_getSpectrum();
//                osTimerStart(handle->scanTick, 10);
                handle->scanTickTimeout = true;
                PT_YIELD(&handle->pt);
            }
            else {
                SONDE_Detector sondeDetector = SONDE_DETECTOR_RS41_RS92;
                float frequency = 0;
                uint32_t durationMs = 1;
                if (_SCANNER_getNextQrg(&sondeDetector, &frequency, &durationMs)) {
                    SYS_enableDetector(sys, frequency, sondeDetector);
                    if ((handle->mode == SCANNER_MODE_MANUAL) && !handle->scanner) {
                        handle->manualFrequency = SYS_getCurrentFrequency(sys);
                    }
//durationMs = 0xFFFFFFFF;
                    osTimerStart(handle->scanTick, durationMs);
printf("scanner @ %.3f\r\n", frequency / 1e6f);
                }
            }
        }
    }

    PT_END(&handle->pt);
}


