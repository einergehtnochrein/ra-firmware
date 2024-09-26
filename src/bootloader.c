/* Copyright (c) 2024, DF9DQ
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

#include <string.h>

#include "bootloader.h"
#include "lpclib.h"


const uint32_t SIGNATURE_RA2_V2a[] = {0x2000FFD0,0x00002C59,0x00002C95,0x00002C99};
const uint32_t SIGNATURE_RA2_V2b[] = {0x2000FFD0,0x00002FCD,0x00003009,0x0000300D};
const uint32_t SIGNATURE_RA2_V2c[] = {0x2000FFD0,0x00002FA9,0x00002FE5,0x00002FE9};
const uint32_t SIGNATURE_RA2_V4a[] = {0x2000FFD0,0x0000324D,0x00003289,0x0000328D};
const uint32_t SIGNATURE_RA2_V4b[] = {0x2000FFD0,0x0000418D,0x000041C9,0x000041CD};


int BOOTLOADER_getVersion (void)
{
    int version = 0;

    /* See if there is a valid info record at the end of the bootloader sector */
    uint16_t loader_version = ((volatile uint16_t *)0x7F00)[0];
    uint16_t loader_version_inverted = ((volatile uint16_t *)0x7F00)[1] ^ 0xFFFF;
    if (loader_version == loader_version_inverted) {
        version = loader_version;
    } else {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wnonnull"

#if (BOARD_RA == 2)
        /* Are there known memory signatures at the beginning of the bootloader sector? */
        if (memcmp((void *)0, SIGNATURE_RA2_V2a, sizeof(SIGNATURE_RA2_V2a)) == 0) {
            version = 2;
        }
        if (memcmp((void *)0, SIGNATURE_RA2_V2b, sizeof(SIGNATURE_RA2_V2b)) == 0) {
            version = 2;
        }
        if (memcmp((void *)0, SIGNATURE_RA2_V2c, sizeof(SIGNATURE_RA2_V2c)) == 0) {
            version = 2;
        }
        if (memcmp((void *)0, SIGNATURE_RA2_V4a, sizeof(SIGNATURE_RA2_V4a)) == 0) {
            version = 4;
        }
        if (memcmp((void *)0, SIGNATURE_RA2_V4b, sizeof(SIGNATURE_RA2_V4b)) == 0) {
            version = 4;
        }
#endif

#pragma GCC diagnostic pop
    }

    return version;
}

