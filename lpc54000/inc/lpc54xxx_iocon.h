/* Copyright (c) 2014, Df9DQ
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
 *  \brief IOCON driver interface.
 *
 *  This file contains the API for the IOCON block.
 */


#ifndef __LPC54XXX_IOCON_H__
#define __LPC54XXX_IOCON_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** \defgroup IOCON
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"

#include "lpc54xxx_clkpwr.h"


/** \defgroup IOCON_Public_Types IOCON Types, enums, macros
 *  @{
 */


enum IOCON_PinType {
    IOCON_PINTYPE_D = (1u << 28),           /**< Normal digital pin */
    IOCON_PINTYPE_A = (2u << 28),
    IOCON_PINTYPE_I = (4u << 28),
};


typedef enum IOCON_PinName {
    PIN_P0_0    =   0 | IOCON_PINTYPE_D,
    PIN_P0_1    =   1 | IOCON_PINTYPE_D,
    PIN_P0_2    =   2 | IOCON_PINTYPE_D,
    PIN_P0_3    =   3 | IOCON_PINTYPE_D,
    PIN_P0_4    =   4 | IOCON_PINTYPE_D,
    PIN_P0_5    =   5 | IOCON_PINTYPE_D,
    PIN_P0_6    =   6 | IOCON_PINTYPE_D,
    PIN_P0_7    =   7 | IOCON_PINTYPE_D,
    PIN_P0_8    =   8 | IOCON_PINTYPE_D,
    PIN_P0_9    =   9 | IOCON_PINTYPE_D,
    PIN_P0_10   =  10 | IOCON_PINTYPE_D,
    PIN_P0_11   =  11 | IOCON_PINTYPE_D,
    PIN_P0_12   =  12 | IOCON_PINTYPE_D,
    PIN_P0_13   =  13 | IOCON_PINTYPE_D,
    PIN_P0_14   =  14 | IOCON_PINTYPE_D,
    PIN_P0_15   =  15 | IOCON_PINTYPE_D,
    PIN_P0_16   =  16 | IOCON_PINTYPE_D,
    PIN_P0_17   =  17 | IOCON_PINTYPE_D,
    PIN_P0_18   =  18 | IOCON_PINTYPE_D,
    PIN_P0_19   =  19 | IOCON_PINTYPE_D,
    PIN_P0_20   =  20 | IOCON_PINTYPE_D,
    PIN_P0_21   =  21 | IOCON_PINTYPE_D,
    PIN_P0_22   =  22 | IOCON_PINTYPE_D,
    PIN_P0_23   =  23 | IOCON_PINTYPE_I,
    PIN_P0_24   =  24 | IOCON_PINTYPE_I,
    PIN_P0_25   =  25 | IOCON_PINTYPE_I,
    PIN_P0_26   =  26 | IOCON_PINTYPE_I,
    PIN_P0_27   =  27 | IOCON_PINTYPE_I,
    PIN_P0_28   =  28 | IOCON_PINTYPE_I,
    PIN_P0_29   =  29 | IOCON_PINTYPE_A,
    PIN_P0_30   =  30 | IOCON_PINTYPE_A,
    PIN_P0_31   =  31 | IOCON_PINTYPE_A,
    PIN_P1_0    =  32 | IOCON_PINTYPE_A,
    PIN_P1_1    =  33 | IOCON_PINTYPE_A,
    PIN_P1_2    =  34 | IOCON_PINTYPE_A,
    PIN_P1_3    =  35 | IOCON_PINTYPE_A,
    PIN_P1_4    =  36 | IOCON_PINTYPE_A,
    PIN_P1_5    =  37 | IOCON_PINTYPE_A,
    PIN_P1_6    =  38 | IOCON_PINTYPE_A,
    PIN_P1_7    =  39 | IOCON_PINTYPE_A,
    PIN_P1_8    =  40 | IOCON_PINTYPE_A,
    PIN_P1_9    =  41 | IOCON_PINTYPE_D,
    PIN_P1_10   =  42 | IOCON_PINTYPE_D,
    PIN_P1_11   =  43 | IOCON_PINTYPE_D,
    PIN_P1_12   =  44 | IOCON_PINTYPE_D,
    PIN_P1_13   =  45 | IOCON_PINTYPE_D,
    PIN_P1_14   =  46 | IOCON_PINTYPE_D,
    PIN_P1_15   =  47 | IOCON_PINTYPE_D,
    PIN_P1_16   =  48 | IOCON_PINTYPE_D,
    PIN_P1_17   =  49 | IOCON_PINTYPE_D,
} IOCON_PinName;


typedef enum IOCON_Function {
    PIN_FUNCTION_0 = 0,         /**< Function selector 0 */
    PIN_FUNCTION_1 = 1,         /**< Function selector 1 */
    PIN_FUNCTION_2 = 2,         /**< Function selector 2 */
    PIN_FUNCTION_3 = 3,         /**< Function selector 3 */
    PIN_FUNCTION_4 = 4,         /**< Function selector 4 */
    PIN_FUNCTION_5 = 5,         /**< Function selector 5 */
    PIN_FUNCTION_6 = 6,         /**< Function selector 6 */
    PIN_FUNCTION_7 = 7,         /**< Function selector 7 */
} IOCON_Function;


typedef enum IOCON_Mode {
    PIN_PULL_NONE = 0,          /**< No pull-up or pull-down resistor */
    PIN_PULL_UP   = 2,          /**< Internal pull-up resistor */
    PIN_PULL_DOWN = 1,          /**< Internal pull-down resistor */
    PIN_PULL_REPEATER = 3,      /**< Keep last actively driven input level */
} IOCON_Mode;


typedef enum IOCON_OpenDrain {
    PIN_OPENDRAIN_OFF = 0,      /**< Open-drain disabled */
    PIN_OPENDRAIN_ON  = 1,      /**< Open-drain enabled */
} IOCON_OpenDrain;


typedef enum IOCON_SlewRate {
    PIN_SLEWRATE_NORMAL = 0,    /**< Standard mode */
    PIN_SLEWRATE_FAST = 1,      /**< Fast mode */
} IOCON_SlewRate;


typedef enum IOCON_Polarity {
    PIN_INPUT_NOT_INVERTED = 0, /**< Input not inverted */
    PIN_INPUT_INVERTED = 1,     /**< Input inverted */
} IOCON_Polarity;


typedef enum IOCON_Hysteresis {
    PIN_HYSTERESIS_OFF = 0,     /**< Hysteresis disabled */
    PIN_HYSTERESIS_ON  = 1,     /**< Hysteresis enabled */
} IOCON_Hysteresis;

typedef enum IOCON_AdMode {
    PIN_ADMODE_ANALOG = 0,      /**< Analog mode */
    PIN_ADMODE_DIGITAL = 1,     /**< Digital mode */
} IOCON_AdMode;

typedef enum IOCON_GlitchFilter {
    PIN_FILTER_OFF = 1,         /**< Glich filter disabled */
    PIN_FILTER_ON = 0,          /**< Glitch filter enabled */
} IOCON_GlitchFilter;

typedef enum IOCON_I2cMode {
    PIN_I2CMODE_GPIO_4MA = 0x01,    /**< GPIO mode, 4 mA */
    PIN_I2CMODE_GPIO_20MA = 0x11,   /**< GPIO mode, 20 mA */
    PIN_I2CMODE_NORMAL = 0x08,      /**< I2C mode, normal and fast */
    PIN_I2CMODE_FMPLUS = 0x38,      /**< I2C mode, Fast mode+ */
    PIN_I2CMODE_HS = 0x38,          /**< High Speed (slave)+ */
} IOCON_I2cMode;

typedef enum IOCON_DriveStrength {
    PIN_HIDRIVE_4MA = 0,        /**< Pin can drive 4 mA */
    PIN_HIDRIVE_20MA = 1,       /**< Pin can drive 20 mA */
} IOCON_DriveStrength;



typedef uint32_t IOCON_PinConfig;


typedef struct IOCON_PinConfigList {
    IOCON_PinName pin;
    IOCON_PinConfig config;
} IOCON_PinConfigList;


/** @} IOCON Types enums, macros */


/** \defgroup IOCON_Public_Functions IOCON API Functions
 *  @{
 */


/** Initialize the IOCON module.
 */
static void IOCON_open (void);



/** Configure a pin.
 *
 *  \param[in] pin Selects the pin to be configured
 *  \param[in] config Provides the configuration (as obtained by PINSEL_MakeConfig())
 */
static void IOCON_configurePin (IOCON_PinName pin, IOCON_PinConfig config);



/** Control a pin's pull-up and pull-down resistors.
 *
 *  \param[in] pin Selects the pin to be configured
 *  \param[in] mode Pull-up/pull-down selection
 */
static void IOCON_configureMode (IOCON_PinName pin, IOCON_Mode mode);



/** Check if an error occured during any of the pin configurations done so far.
 *
 *  \retval LPCLIB_SUCCESS No problems occurred.
 *  \retval LPCLIB_ILLEGAL_PARAMETER At least one mismatch between pin and
 *          makeConfigX function found
 */
static LPCLIB_Result IOCON_checkErrors (void);


/** @} IOCON API Functions */


extern LPCLIB_Result __IOCON_configError;


__FORCEINLINE(void IOCON_open (void))
{
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_IOCON);
    CLKPWR_deassertPeripheralReset(CLKPWR_RESET_IOCON);

    __IOCON_configError = LPCLIB_SUCCESS;
}

__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigD (
        IOCON_Function function,
        IOCON_Mode mode,
        IOCON_Polarity polarity,
        IOCON_SlewRate slew,
        IOCON_GlitchFilter filter,
        IOCON_OpenDrain openDrain))
{
    return (IOCON_PinConfig)(
        (function << 0)   |
        (mode << 3)       |
        (polarity << 6)   |
        (1u << 7)         |
        (filter << 8)     |
        (slew << 9)       |
        (openDrain << 10) |
        IOCON_PINTYPE_D
        );
}

__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigA (
        IOCON_Function function,
        IOCON_Mode mode,
        IOCON_Polarity polarity,
        IOCON_AdMode admode,
        IOCON_GlitchFilter filter,
        IOCON_OpenDrain openDrain))
{
    return (IOCON_PinConfig)(
        (function << 0)   |
        (mode << 3)       |
        (polarity << 6)   |
        (admode << 7)     |
        (filter << 8)     |
        (openDrain << 10) |
        IOCON_PINTYPE_A
        );
}

__FORCEINLINE(IOCON_PinConfig IOCON_makeConfigI (
        IOCON_Function func,
        IOCON_Polarity invert,
        IOCON_I2cMode hs,
        IOCON_GlitchFilter filter))
{
    return (IOCON_PinConfig)(
        (func << 0)       |
        (invert << 6)     |
        (hs << 5)         |
        (1 << 7)          |
        (filter << 8)     |
        IOCON_PINTYPE_I
        );
}



__FORCEINLINE(void IOCON_configurePin (IOCON_PinName pin, IOCON_PinConfig config))
{
    if ((uint32_t)(pin >> 28) != (config >> 28)) {    /* Type encoded into the upper 4 bits */
        __IOCON_configError = LPCLIB_ILLEGAL_PARAMETER;
        return;
    }

    ((volatile uint32_t *)LPC_IOCON)[pin & 0x0FFFFFFF] = (uint32_t)config & 0x0FFFFFFF;
}


__FORCEINLINE(void IOCON_configurePinDefault (IOCON_PinName pin, IOCON_Function function, IOCON_Mode mode))
{
    switch (pin & 0xF0000000) {
    case IOCON_PINTYPE_D:
        IOCON_configurePin(pin, IOCON_makeConfigD(function, mode,
                                                  PIN_INPUT_NOT_INVERTED,
                                                  PIN_SLEWRATE_NORMAL,
                                                  PIN_FILTER_OFF,
                                                  PIN_OPENDRAIN_OFF));
        break;
    case IOCON_PINTYPE_A:
        IOCON_configurePin(pin, IOCON_makeConfigA(function, mode,
                                                  PIN_INPUT_NOT_INVERTED,
                                                  PIN_ADMODE_DIGITAL,
                                                  PIN_FILTER_OFF,
                                                  PIN_OPENDRAIN_OFF));
        break;
    case IOCON_PINTYPE_I:
        IOCON_configurePin(pin, IOCON_makeConfigI(function,
                                                  PIN_INPUT_NOT_INVERTED,
                                                  PIN_I2CMODE_FMPLUS,
                                                  PIN_FILTER_OFF));
        break;
    default:
        __IOCON_configError = LPCLIB_ILLEGAL_PARAMETER;
        break;
    }
}



__FORCEINLINE(void IOCON_configureMode (IOCON_PinName pin, IOCON_Mode mode))
{
    if ((pin & 0xF0000000) == IOCON_PINTYPE_I) {
        __IOCON_configError = LPCLIB_ILLEGAL_PARAMETER;
        return;
    }

    ((volatile uint32_t *)LPC_IOCON)[pin & 0xFF] =
        (((volatile uint32_t *)LPC_IOCON)[pin & 0xFF] & ~0x00000018) | (mode << 3);
}



__FORCEINLINE(LPCLIB_Result IOCON_checkErrors (void))
{
    return __IOCON_configError;
}


/** @} IOCON API Functions */

/** @} IOCON */

#ifdef __cplusplus
}
#endif

#endif /* #ifndef __LPC54XXX_IOCON_H__ */

