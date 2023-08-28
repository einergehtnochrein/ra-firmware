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

        .syntax     unified
        .arch       armv7e-m

/** This function encodes a single sample (sign + 13 bits) into an ADPCM output stream (16 kbit/s).
 *  The context is defined in the calling C module.
 *
 *  Context:
 *      int32_t yl;         // 0
 *      int32_t yu;         // 4
 *      int16_t dq[6];      // 8
 *      int32_t a[2];       // 20
 *      int32_t b[6];       // 28
 *      int32_t ap;         // 52
 *      int32_t dms;        // 56
 *      int32_t dml;        // 60
 *      int32_t td;         // 64
 *      int32_t pk[2];      // 68
 *      int32_t sr[2];      // 76
 *
 */

        .set        DUMMY, 2

        .bss

__bss_dummy:        .long 0

        .text

        .thumb_func
_ADPCM_fmult:
        push    {r4, r5}

        /* R2 = anmag = |an| & 0x1FFF */
        movs    r2, r0
        it      mi
        negmi   r2, r2
        bfc     r2, #13, #32-13

        /* R0 = an ^ srn */
        eor     r0, r1

        /* R3 = anexp = quan(anmag) - 6 */
        clz     r3, r2
        rsbs    r3, r3, #26     // Set flags!

        /* R4 = anmant */
        itte    mi
        negmi   r4, r3
        lslmi   r4, r2, r4
        lsrpl   r4, r2, r3
        movs    r2, r2
        it      eq
        moveq   r4, #32

        /* R5 = wanexp = anexp + ((srn >> 6) & 0xF) - 17 */
        lsr     r5, r1, #6
        bfc     r5, #4, #32-4
        add     r5, r3
        sub     r5, #17

        /* R1 = wanmant = anmant * (srn & 0x3F) + 48 */
        and     r1, #0x3F
        mul     r1, r4
        add     r1, #48

        /* R1 = retval */
        movs    r5, r5
        itte    mi
        negmi   r5, r5
        lsrmi   r1, r1, r5
        lslpl   r1, r1, r5
        bfc     r1, #15, #32-15

        movs    r0, r0
        ite     mi
        negsmi  r0, r1
        movpl   r0, r1

        pop     {r4, r5}
        bx      lr


        .thumb_func
        .global ADPCM_predictor_zero
ADPCM_predictor_zero:
        push    {r4-r6,lr}

        mov     r4, r0
        mov     r5, r1
        mov     r6, #0

        ldr     r0, [r4, #0]
        asr     r0, #2
        ldrsh   r1, [r5, #0]
        bl      _ADPCM_fmult
        add     r6, r0

        ldr     r0, [r4, #4]
        asr     r0, #2
        ldrsh   r1, [r5, #2]
        bl      _ADPCM_fmult
        add     r6, r0

        ldr     r0, [r4, #8]
        asr     r0, #2
        ldrsh   r1, [r5, #4]
        bl      _ADPCM_fmult
        add     r6, r0

        ldr     r0, [r4, #12]
        asr     r0, #2
        ldrsh   r1, [r5, #6]
        bl      _ADPCM_fmult
        add     r6, r0

        ldr     r0, [r4, #16]
        asr     r0, #2
        ldrsh   r1, [r5, #8]
        bl      _ADPCM_fmult
        add     r6, r0

        ldr     r0, [r4, #20]
        asr     r0, #2
        ldrsh   r1, [r5, #10]
        bl      _ADPCM_fmult
        add     r0, r6

        pop     {r4-r6,lr}
        bx      lr


        .thumb_func
        .global ADPCM_predictor_pole
ADPCM_predictor_pole:
        push    {r4-r6,lr}

        mov     r4, r0
        mov     r5, r1
        mov     r6, #0

        ldr     r0, [r4, #0]
        asr     r0, #2
        ldr     r1, [r5, #0]
        bl      _ADPCM_fmult
        add     r6, r0

        ldr     r0, [r4, #4]
        asr     r0, #2
        ldr     r1, [r5, #4]
        bl      _ADPCM_fmult
        add     r0, r6

        pop     {r4-r6,lr}
        bx      lr

        .end
