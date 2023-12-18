/* Copyright (c) 2023, DF9DQ
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


#include <inttypes.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"

#include "sys.h"
#include "mon.h"
#include "monprivate.h"


#define MON_QUEUE_LENGTH        10

typedef struct {
    uint8_t opcode;

    const char *txData;
} MON_Message;


/** Message opcodes. */
enum {
    MON_OPCODE_SEND_AUDIO,
};


struct MON_Context {
    struct pt pt;

    osMailQId queue;                                    /**< Task message queue */
    osEvent rtosEvent;
} monContext;


osMailQDef(monQueueDef, MON_QUEUE_LENGTH, MON_Message);


LPCLIB_Result MON_open (MON_Handle *pHandle)
{
    *pHandle = &monContext;

    return LPCLIB_SUCCESS;
}


/* Called from DSP interrupt context with the BASE64 encoded audio samples.
 * Send message to thread layer to have it sent out to the host.
 */
void _MON_sendAudio (const char *txData)
{
    MON_Message *pMessage = NULL;

    if (txData) {
        if (strlen(txData) > 0) {
            pMessage = osMailAlloc(monContext.queue, 0);
            if (pMessage) {
                pMessage->opcode = MON_OPCODE_SEND_AUDIO;
                pMessage->txData = txData;
            }
        }
    }

    if (pMessage) {
        osMailPut(monContext.queue, pMessage);
    }
}


static bool _MON_checkEvent (MON_Handle handle)
{
    bool haveEvent = false;

    /* Something in the mailbox? */
    handle->rtosEvent = osMailGet(handle->queue, 0);
    haveEvent |= handle->rtosEvent.status == osEventMail;

    return haveEvent;
}


PT_THREAD(MON_thread (MON_Handle handle))
{
    static MON_Message *pMessage;

    PT_BEGIN(&handle->pt);

    monContext.queue = osMailCreate(osMailQ(monQueueDef), NULL);

    while (1) {
        /* Wait for an event */
        PT_WAIT_UNTIL(&handle->pt, _MON_checkEvent(handle));

        /* Is there a new message? */
        if (handle->rtosEvent.status == osEventMail) {
            pMessage = (MON_Message *)handle->rtosEvent.value.p;
            switch (pMessage->opcode) {
            case MON_OPCODE_SEND_AUDIO:
                SYS_send2Host(HOST_CHANNEL_AUDIO, pMessage->txData);
                break;
            }

            osMailFree(handle->queue, pMessage);
        }
    }

    PT_END(&handle->pt);
}
