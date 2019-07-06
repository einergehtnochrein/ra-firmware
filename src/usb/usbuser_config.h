/* Copyright (c) 2013, DF9DQ
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

#ifndef __USBUSER_CONFIG_H
#define __USBUSER_CONFIG_H

#include "lpclib.h"

#include "usbd_rom_api.h"
#include "usb_audio.h"
#include "usb_serial.h"



/** Number of configurations. */
#define USBCONFIG_NUM_CONFIGURATIONS                    1

/** Interface numbers for configuration 1. */
#define USBCONFIG_NUM_INTERFACES                        4
#define USBCONFIG_INTERFACE_AUDIO_CONTROL               0
#define USBCONFIG_INTERFACE_AUDIO_STREAM                1
#define USBCONFIG_INTERFACE_SERIAL_CIF                  2
#define USBCONFIG_INTERFACE_SERIAL_DIF                  3

/** Endpoints */
#define USBCONFIG_SERIAL_CIF_EP_INT                     0x81
#define USBCONFIG_SERIAL_DIF_EP_IN                      0x82
#define USBCONFIG_SERIAL_DIF_EP_OUT                     0x02
#define USBCONFIG_SERIAL_CIF_EP_INT_SIZE                16
#define USBCONFIG_SERIAL_DIF_EP_IN_SIZE                 64
#define USBCONFIG_SERIAL_DIF_EP_OUT_SIZE                64
#define USBCONFIG_AUDIO_IN_EP                           0x83
#define USBCONFIG_AUDIO_IN_EP_SIZE                      72


/* Audio unit numbers */
#define USBCONFIG_UNIT_TERMINAL_IN                      1
#define USBCONFIG_UNIT_FEATURE                          2
#define USBCONFIG_UNIT_TERMINAL_OUT                     3


/*  WORKAROUND for artf44835 ROM driver BUG:
    Code clearing STALL bits in endpoint reset routine corrupts memory area
    next to the endpoint control data. For example When EP0, EP1_IN, EP1_OUT,
    EP2_IN are used we need to specify 3 here. But as a workaround for this
    issue specify 4. So that extra EPs control structure acts as padding buffer
    to avoid data corruption. Corruption of padding memory doesn’t affect the
    stack/program behaviour.
 */
#define USBCONFIG_MAX_NUM_EP                            5

void USBUSER_initDescriptors (void);
void USBUSER_open (void);
void USBUSER_worker (void);
bool USBUSER_isConfigured (void);
void USBUSER_writeAudioStereo_i32_i32 (const int32_t *buffer1, const int32_t *buffer2, int nSamples);
void USBUSER_writeAudioStereo_i32_float (const int32_t *buffer1, const float *buffer2, int nSamples);
void USBUSER_writeAudioStereo_float_float (const float *buffer1, const float *buffer2, int nSamples);

#endif


