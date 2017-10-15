/* Copyright (c) 2016, DF9DQ
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 * Neither the name of the author nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file
 *  \brief TIMER driver interface.
 *
 *  This file contains the API for the TIMER block.
 */

#ifndef __LPC54XXX_TIMER_H__
#define __LPC54XXX_TIMER_H__

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup TIMER
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"
#include "lpclib_types.h"


/** \defgroup TIMER_Public_Types TIMER Types, enums, macros
 *  @{
 */


#define TIMER_NUM_TIMERS    5

typedef enum TIMER_Name {
    TIMER0 = 0,                             /**< Timer 0 */
    TIMER1 = 1,                             /**< Timer 1 */
    TIMER2 = 2,                             /**< Timer 2 */
    TIMER3 = 3,                             /**< Timer 3 */
    TIMER4 = 4,                             /**< Timer 4 */
} TIMER_Name;


/** Handle for a timer block */
typedef struct TIMER_Context *TIMER_Handle;


typedef enum TIMER_MatchChannel {
    TIMER_MATCH0 = 0,                       /**< Match channel 0 */
    TIMER_MATCH1 = 1,                       /**< Match channel 1 */
    TIMER_MATCH2 = 2,                       /**< Match channel 2 */
    TIMER_MATCH3 = 3,                       /**< Match channel 3 */
    TIMER_MAT0 = TIMER_MATCH0,
    TIMER_MAT1 = TIMER_MATCH1,
    TIMER_MAT2 = TIMER_MATCH2,
    TIMER_MAT3 = TIMER_MATCH3,
} TIMER_MatchChannel;

typedef enum TIMER_CaptureChannel {
    TIMER_CAP0 = 0,                           /**< Capture channel 0 */
    TIMER_CAP1 = 1,                           /**< Capture channel 1 */
} TIMER_CaptureChannel;


typedef enum TIMER_MatchOutputFunction {
    TIMER_MATCH_OUTPUT_NONE = 0,            /**< Match output: no action */
    TIMER_MATCH_OUTPUT_CLEAR = 1,           /**< Match output: clear */
    TIMER_MATCH_OUTPUT_SET = 2,             /**< Match output: set */
    TIMER_MATCH_OUTPUT_TOGGLE = 3,          /**< Match output: toggle */
} TIMER_MatchOutputFunction;


typedef enum TIMER_Mode {
    TIMER_MODE_TIMER = 0,                   /**< Mode: Timer */
    TIMER_MODE_COUNT_RISING = 1,            /**< Mode: Counter rising edge */
    TIMER_MODE_COUNT_FALLING = 2,           /**< Mode: Counter falling edge */
    TIMER_MODE_COUNT_BOTH = 3,              /**< Mode: Counter both edges */
} TIMER_Mode;


typedef enum TIMER_Opcode {
    TIMER_OPCODE_INVALID = 0,               /**< (List terminator) */
    TIMER_OPCODE_CONFIG_MATCH,              /**< Control action: Configure match */
    TIMER_OPCODE_CONFIG_CAPTURE,            /**< Control action: Configure capture */
    TIMER_OPCODE_MODE,                      /**< Control action: Set mode */
    TIMER_OPCODE_SET_CALLBACK,              /**< Control action: Install handler */
} TIMER_Opcode;


struct TIMER_ConfigMatch {
    TIMER_MatchChannel channel;             /**< Match channel */
    LPCLIB_Switch intOnMatch;               /**< Flag: Enable interrupt on match */
    LPCLIB_Switch resetOnMatch;             /**< Flag: Enable reset on match */
    LPCLIB_Switch stopOnMatch;              /**< Flag: Enable stop on match */
    TIMER_MatchOutputFunction function;     /**< Match output action */
    LPCLIB_Switch pwm;                      /**< Flag: Enable PWM mode */
};

struct TIMER_ConfigCapture {
    TIMER_CaptureChannel channel;           /**< Capture channel */
    LPCLIB_Switch risingEdge;               /**< Flag: Capture on rising edge */
    LPCLIB_Switch fallingEdge;              /**< Flag: Capture on falling edge */
    LPCLIB_Switch intOnCapture;             /**< Flag: Capture triggers interrupt */
};

struct TIMER_ConfigMode {
    TIMER_Mode mode;                        /**< Timer/Counter mode */
    TIMER_CaptureChannel channel;           /**< Capture channel used as counter input */
};


typedef struct TIMER_Config {
    TIMER_Opcode opcode;                    /**< Control action */

    union {
        struct TIMER_ConfigMatch match;     /**< Match configuration */
        struct TIMER_ConfigCapture capture; /**< Capture configuration */
        struct TIMER_ConfigMode mode;       /**< Mode configuration */
        LPCLIB_Callback callback;           /**< Callback in interrupt context */
    };
} TIMER_Config;

/** Config list terminator. */
#define TIMER_CONFIG_END \
    {.opcode = TIMER_OPCODE_INVALID}



typedef enum TIMER_Event {
    TIMER_EVENT_MATCH = 1,                  /**< Match event has occurred */
    TIMER_EVENT_CAPTURE = 2,                /**< Capture event has occurred */
} TIMER_Event;


/** @} TIMER types, enums, macros */


/** \defgroup TIMER_Public_Functions TIMER API Functions
 *  @{
 */


/** Open a Timer block for use.
 *
 *  \param[in] timer Name of the timer
 *  \param[out] pHandle Handle to be used in future API calls to the TIMER module.
 *  \retval LPCLIB_SUCCESS Access to timer module granted.
 *  \retval LPCLIB_BUSY Timer is in use
 */
LPCLIB_Result TIMER_open (TIMER_Name timer, TIMER_Handle *pHandle);


/** Close a timer channel.
 *
 *  \param[in] pHandle Timer handle
 *  \retval LPCLIB_SUCCESS Ok
 *  \retval LPCLIB_ILLEGAL_PARAMETER Invalid handle, or timer not open.
 */
LPCLIB_Result TIMER_close (TIMER_Handle *pHandle);


/** Adjust the configuration of the timer block.
 *
 *  \param[in] handle Timer handle
 *  \param[in] pConfig Describes the configuration change
 */
LPCLIB_Result TIMER_ioctl (TIMER_Handle handle, const TIMER_Config *pConfig);


/** Start a timer.
 *
 *  The timer's RUN bit is set.
 *
 *  \param[in] handle Timer handle
 */
static void TIMER_run (TIMER_Handle handle);


/** Stop a timer.
 *
 *  The timer's RUN bit is cleared. The timer's current value is preserved,
 *  and it can be restarted from there with \ref TIMER_run.
 *
 *  \param[in] handle Timer handle
 */
static void TIMER_stop (TIMER_Handle handle);


/** Stop a timer.
 *
 *  The timer's RUN bit is cleared, and the current timer value is cleared.
 *
 *  \param[in] handle Timer handle
 */
static void TIMER_stopAndReset (TIMER_Handle handle);


static void TIMER_write(TIMER_Handle handle, uint32_t value);
static uint32_t TIMER_read(TIMER_Handle handle);


static void TIMER_writePrescaler (TIMER_Handle handle, uint32_t value);


static void TIMER_writeMatch (TIMER_Handle handle, TIMER_MatchChannel channel, uint32_t value);


/** Read value from capture register.
 *
 *  Read the current value from the addressed capture register.
 *
 *  \param[in] handle Timer handle
 *  \param[in] channel Capture register selector
 *  \returns Timer value at last capture event.
 */
static uint32_t TIMER_readCapture (TIMER_Handle handle, TIMER_CaptureChannel channel);





__FORCEINLINE(uint32_t TMR_MakeCaptureConfig(
            LPCLIB_Switch RisingEdge,
            LPCLIB_Switch FallingEdge,
            LPCLIB_Switch IntOnCaption))
{
    return ((RisingEdge & 1) << 0)  |
           ((FallingEdge & 1) << 1) |
           ((IntOnCaption & 1) << 2);
}



__FORCEINLINE(LPC_TIMER_Type * __TIMER_getHardwarePtr (TIMER_Handle handle))
{
    return *((LPC_TIMER_Type **)handle);
}


__FORCEINLINE(uint32_t TIMER_read(TIMER_Handle handle))
{
    return __TIMER_getHardwarePtr(handle)->TC;
}

__FORCEINLINE(void TIMER_write(TIMER_Handle handle, uint32_t value))
{
    __TIMER_getHardwarePtr(handle)->TC = value;
}

__FORCEINLINE(uint32_t TIMER_readCapture(TIMER_Handle handle, TIMER_CaptureChannel channel))
{
    return __TIMER_getHardwarePtr(handle)->CR[channel];
}

__FORCEINLINE(void TIMER_writeMatch(TIMER_Handle handle, TIMER_MatchChannel channel, uint32_t value))
{
    __TIMER_getHardwarePtr(handle)->MR[channel] = value;
}

__FORCEINLINE(void TIMER_writePrescaler(TIMER_Handle handle, uint32_t value))
{
    __TIMER_getHardwarePtr(handle)->PR = value;
}

__FORCEINLINE(void TIMER_run(TIMER_Handle handle))
{
    __TIMER_getHardwarePtr(handle)->TCR = 1;
}

__FORCEINLINE(void TIMER_stop(TIMER_Handle handle))
{
    __TIMER_getHardwarePtr(handle)->TCR = 0;
}

__FORCEINLINE(void TIMER_stopAndReset(TIMER_Handle handle))
{
    __TIMER_getHardwarePtr(handle)->TCR = 2;
}


/** @} TIMER API Functions */

/** @} TIMER */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __LPC54XXX_TIMER_H__ */

