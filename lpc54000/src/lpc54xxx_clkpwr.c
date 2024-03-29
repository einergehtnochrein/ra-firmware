/* Copyright (c) 2014-2017, DF9DQ
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
 *  \brief CLKPWR driver implementation.
 *
 *  This file contains the implementation of the CLKPWR block driver.
 */

/** \addtogroup CLKPWR
 *  @{
 */

#include "lpc54xxx_libconfig.h"


#include "lpclib_types.h"
#include "lpc54xxx_clkpwr.h"
#include "lpc54xxx_romhandler.h"


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
#define IRC_FREQUENCY                       (12000000ul)
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
#define FRO12_FREQUENCY                     (12000000ul)
#define FROHF48_FREQUENCY                   (48000000ul)
#define FROHF96_FREQUENCY                   (96000000ul)
#endif
#define RTC_FREQUENCY                       (32768ul)


/** Structure that reflects the PWRD API table in the ROM (see User Manual).
 */
typedef struct PWRD {
    uint32_t (*set_pll) (uint32_t multiplier, uint32_t input_freq);
    uint32_t (*set_voltage) (uint32_t mode, uint32_t desired_freq);
    void (*power_mode_configure) (uint32_t mode, uint32_t peripheral);
} PWRD;



LPCLIB_DefineRegBit(SYSCON_SYSPLLCLKSEL_SEL,        0,  2);
enum {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    PLLCLKSEL_SEL_IRC = 0,
    PLLCLKSEL_SEL_CLKIN = 1,
    PLLCLKSEL_SEL_RTC = 3,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    PLLCLKSEL_SEL_FRO12 = 0,
    PLLCLKSEL_SEL_CLKIN = 1,
    PLLCLKSEL_SEL_RTC = 3,
#endif
};

LPCLIB_DefineRegBit(SYSCON_MAINCLKSELA_SEL,         0,  2);
enum {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    MAINCLKSELA_SEL_IRC = 0,
    MAINCLKSELA_SEL_CLKIN = 1,
    MAINCLKSELA_SEL_WDT = 2,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    MAINCLKSELA_SEL_FRO12 = 0,
    MAINCLKSELA_SEL_CLKIN = 1,
    MAINCLKSELA_SEL_WDT = 2,
    MAINCLKSELA_SEL_FROHF = 3,
#endif
};
LPCLIB_DefineRegBit(SYSCON_MAINCLKSELB_SEL,         0,  2);
enum {
    MAINCLKSELB_SEL_MAINCLKSELA = 0,
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    MAINCLKSELB_SEL_PLLIN = 1,
#endif
    MAINCLKSELB_SEL_PLLOUT = 2,
    MAINCLKSELB_SEL_RTC = 3,
};


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
LPCLIB_DefineRegBit(SYSCON_SYSTCKCAL_CAL,           0, 24);
LPCLIB_DefineRegBit(SYSCON_SYSTCKCAL_SKEW,          24, 1);
LPCLIB_DefineRegBit(SYSCON_SYSTCKCAL_NOREF,         25, 1);

LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_IRC_OSC,   3,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_IRC,       4,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_FLASH,     5,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_BOD_RST,   7,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_BOD_INTR,  8,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_ADC0,      10, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM0A,    13, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM0N,    14, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM1,     15, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM2,     16, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_ROM,       17, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_VDDA,      19, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_WDT_OSC,   20, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SYS_PLL,   22, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_VREFP,     23, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_32K_OSC,   24, 1);

LPCLIB_DefineRegBit(SYSCON_AHBCLKDIV_DIV,           0,  8);
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
LPCLIB_DefineRegBit(SYSCON_SYSTCKCAL_CAL,           0, 24);
LPCLIB_DefineRegBit(SYSCON_SYSTCKCAL_SKEW,          24, 1);
LPCLIB_DefineRegBit(SYSCON_SYSTCKCAL_NOREF,         25, 1);

LPCLIB_DefineRegBit(SYSCON_PRESETCTRL0_FLASH_RST,   7,  1);

LPCLIB_DefineRegBit(SYSCON_AHBCLKCTRL0_ROM,         1,  1);
LPCLIB_DefineRegBit(SYSCON_AHBCLKCTRL0_SRAM1,       3,  1);
LPCLIB_DefineRegBit(SYSCON_AHBCLKCTRL0_SRAM2,       4,  1);
LPCLIB_DefineRegBit(SYSCON_AHBCLKCTRL0_FLASH,       7,  1);

LPCLIB_DefineRegBit(SYSCON_AHBCLKDIV_DIV,           0,  8);

LPCLIB_DefineRegBit(SYSCON_FLASHCFG_FETCHCFG,       0,  2);
LPCLIB_DefineRegBit(SYSCON_FLASHCFG_DATACFG,        2,  2);
LPCLIB_DefineRegBit(SYSCON_FLASHCFG_ACCEL,          4,  1);
LPCLIB_DefineRegBit(SYSCON_FLASHCFG_PREFEN,         5,  1);
LPCLIB_DefineRegBit(SYSCON_FLASHCFG_PREFOVR,        6,  1);
LPCLIB_DefineRegBit(SYSCON_FLASHCFG_FLASHTIM,       12, 4);

LPCLIB_DefineRegBit(SYSCON_FROCTRL_SEL,             14, 1);

LPCLIB_DefineRegBit(SYSCON_SYSPLLSTAT_LOCK,         0,  1);

LPCLIB_DefineRegBit(SYSCON_SYSPLLNDEC_NDEC,         0,  10);
LPCLIB_DefineRegBit(SYSCON_SYSPLLNDEC_NREQ,         10, 1);

LPCLIB_DefineRegBit(SYSCON_SYSPLLPDEC_PDEC,         0,  7);
LPCLIB_DefineRegBit(SYSCON_SYSPLLPDEC_PREQ,         7,  1);

LPCLIB_DefineRegBit(SYSCON_SYSPLLSSCTRL0_MDEC,      0,  17);
LPCLIB_DefineRegBit(SYSCON_SYSPLLSSCTRL0_MREQ,      17, 1);
LPCLIB_DefineRegBit(SYSCON_SYSPLLSSCTRL0_SEL_EXT,   18, 1);

LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_1,         1,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_3,         3,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_FRO,       4,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_FLASH,     5,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_TS,        6,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_BOD_RST,   7,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_BOD_INTR,  8,  1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_ADC0,      10, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM0,     13, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM1,     14, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAM2,     15, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SRAMX,     16, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_ROM,       17, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_18,        18, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_VDDA,      19, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_WDT_OSC,   20, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_USB_PHY,   21, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_SYS_PLL,   22, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_VREFP,     23, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_32K_OSC,   24, 1);
LPCLIB_DefineRegBit(SYSCON_PDRUNCFG_PDEN_26,        26, 1);
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
/* Flash wait states lookup. Index is ((system_clock_Hz - 1) / 12 MHz) */
static const uint8_t _clkpwrFlashWaitStates[9] = {0,1,2,2,3,3,4,5,5};
#endif


/** Watchdog oscillator frequency [Hz] (as accurate as we can get it...) */
uint32_t clkpwrWdoFrequency;


/** \addtogroup CLKPWR_Public_Functions
 *  @{
 */


/* Set a clock divider. */
//TODO
LPCLIB_Result CLKPWR_setDivider (CLKPWR_Divider divider, uint32_t value)
{
    __IO uint32_t * const pSyscon = (__IO uint32_t *)LPC_SYSCON_BASE;

    pSyscon[divider] = value;

    return LPCLIB_SUCCESS;
}



#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
static const CLKPWR_Clock _CLKPWR_mainClkselA[4] = {
        CLKPWR_CLOCK_IRC,
        CLKPWR_CLOCK_CLKIN,
        CLKPWR_CLOCK_WDT,
        _CLKPWR_CLOCK_NONE,
        };
static const CLKPWR_Clock _CLKPWR_mainClkselB[4] = {
        _CLKPWR_CLOCK_MAINCLKSELA,
        CLKPWR_CLOCK_SYSTEMPLLIN,
        CLKPWR_CLOCK_SYSTEMPLL,
        CLKPWR_CLOCK_RTC,
        };
static const CLKPWR_Clock _CLKPWR_asyncClkselA[4] = {
        CLKPWR_CLOCK_IRC,
        CLKPWR_CLOCK_WDT,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        };
static const CLKPWR_Clock _CLKPWR_asyncClkselB[4] = {
        CLKPWR_CLOCK_MAIN,
        CLKPWR_CLOCK_CLKIN,
        CLKPWR_CLOCK_SYSTEMPLL,
        _CLKPWR_CLOCK_ASYNCCLKSELA,
        };
static const CLKPWR_Clock _CLKPWR_syspllClksel[4] = {
        CLKPWR_CLOCK_IRC,
        CLKPWR_CLOCK_CLKIN,
        _CLKPWR_CLOCK_NONE,
        CLKPWR_CLOCK_RTC,
        };
#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
static const CLKPWR_Clock _CLKPWR_mainClkselA[4] = {
        CLKPWR_CLOCK_FRO12,
        CLKPWR_CLOCK_CLKIN,
        CLKPWR_CLOCK_WDT,
        CLKPWR_CLOCK_FROHF,
        };
static const CLKPWR_Clock _CLKPWR_mainClkselB[4] = {
        _CLKPWR_CLOCK_MAINCLKSELA,
        _CLKPWR_CLOCK_NONE,
        CLKPWR_CLOCK_SYSTEMPLL,
        CLKPWR_CLOCK_RTC,
        };
static const CLKPWR_Clock _CLKPWR_asyncClkselA[4] = {
        CLKPWR_CLOCK_MAIN,
        CLKPWR_CLOCK_FRO12,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        };
static const CLKPWR_Clock _CLKPWR_fClkSel[8] = {
        CLKPWR_CLOCK_FRO12,
        CLKPWR_CLOCK_FROHF,
        CLKPWR_CLOCK_SYSTEMPLL,
        CLKPWR_CLOCK_MCLK,
        CLKPWR_CLOCK_FRG,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        };
static const CLKPWR_Clock _CLKPWR_syspllClksel[8] = {
        CLKPWR_CLOCK_FRO12,
        CLKPWR_CLOCK_CLKIN,
        _CLKPWR_CLOCK_NONE,
        CLKPWR_CLOCK_RTC,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        _CLKPWR_CLOCK_NONE,
        };
#endif

/** Return the clock frequency (in Hz) of an internal bus.
 *
 *  \param[in] bus The bus to be queried
 */
uint32_t CLKPWR_getBusClock (CLKPWR_Clock clock)
{
//TODO Check if async clock is on!
    switch (clock) {
        /* Virtual clocks */
        case _CLKPWR_CLOCK_MAINCLKSELA:
            return CLKPWR_getBusClock(_CLKPWR_mainClkselA[LPC_SYSCON->MAINCLKSELA & 3]);
        case _CLKPWR_CLOCK_ASYNCCLKSELA:
            return CLKPWR_getBusClock(_CLKPWR_asyncClkselA[LPC_ASYNCSYSCON->ASYNCAPBCLKSELA & 3]);

        /* Base clocks */
        case CLKPWR_CLOCK_RTC:
            return RTC_FREQUENCY;
        case CLKPWR_CLOCK_CLKIN:
            //TODO
            return 0;
        case CLKPWR_CLOCK_SYSTEMPLLIN:
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
            return CLKPWR_getBusClock(_CLKPWR_syspllClksel[LPC_SYSCON->SYSPLLCLKSEL & 3]);
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
            return CLKPWR_getBusClock(_CLKPWR_syspllClksel[LPC_SYSCON->SYSPLLCLKSEL & 7]);
#endif
        case CLKPWR_CLOCK_SYSTEMPLL:
            //TODO
return 48000000;

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
        case CLKPWR_CLOCK_IRC:
            return IRC_FREQUENCY;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
        case CLKPWR_CLOCK_FRO12:
            return FRO12_FREQUENCY;
        case CLKPWR_CLOCK_FROHF:
            return LPC_SYSCON->FROCTRL & SYSCON_FROCTRL_SEL_Msk ? FROHF96_FREQUENCY : FROHF48_FREQUENCY;
#endif
            return 0;

        /* Branch clocks */
        case CLKPWR_CLOCK_MAIN:
            return CLKPWR_getBusClock(_CLKPWR_mainClkselB[LPC_SYSCON->MAINCLKSELB & 3]);
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
        case CLKPWR_CLOCK_CPU:
            return CLKPWR_getBusClock(CLKPWR_CLOCK_MAIN)
                    / ((LPC_SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_Msk) >> SYSCON_AHBCLKDIV_DIV_Pos);
        case CLKPWR_CLOCK_ASYNCAPB:
            return CLKPWR_getBusClock(_CLKPWR_asyncClkselB[LPC_ASYNCSYSCON->ASYNCAPBCLKSELB & 3]);
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
        case CLKPWR_CLOCK_CPU:
            return CLKPWR_getBusClock(CLKPWR_CLOCK_MAIN)
                    / (((LPC_SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_Msk) >> SYSCON_AHBCLKDIV_DIV_Pos) + 1);
        case CLKPWR_CLOCK_ASYNCAPB:
            return CLKPWR_getBusClock(_CLKPWR_CLOCK_ASYNCCLKSELA);
        case CLKPWR_CLOCK_FLEXCOMM0:
        case CLKPWR_CLOCK_FLEXCOMM1:
        case CLKPWR_CLOCK_FLEXCOMM2:
        case CLKPWR_CLOCK_FLEXCOMM3:
        case CLKPWR_CLOCK_FLEXCOMM4:
        case CLKPWR_CLOCK_FLEXCOMM5:
        case CLKPWR_CLOCK_FLEXCOMM6:
        case CLKPWR_CLOCK_FLEXCOMM7:
            return CLKPWR_getBusClock(_CLKPWR_fClkSel[LPC_SYSCON->FCLKSEL[clock - CLKPWR_CLOCK_FLEXCOMM0] & 7]);
#endif

        default:
            return 0;
    }
}



/** Attempts to set the CPU clock to the desired frequency.
 *
 *  Sets the CPU clock to the frequency which is closest to the requested
 *  value. Assumes that the PLL input clock has been selected before.
 *
 *  \param[in] targetCpuFrequency Desired CPU frequency (Hz)
 *  \return ...
 */
LPCLIB_Result CLKPWR_setCpuClock (uint32_t targetCpuFrequency, CLKPWR_Clock fundamentalClock)
{
    uint32_t inputFrequency;


    /* If core clock is yet unknown, set it to IRC frequency (reset state). */
    if (SystemCoreClock == 0) {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
        SystemCoreClock = FRO12_FREQUENCY;
#else
        SystemCoreClock = IRC_FREQUENCY;
#endif
    }

    /* Don't change anything if frequency doesn't change. */
    if (SystemCoreClock == targetCpuFrequency) {
        return LPCLIB_SUCCESS;
    }

    /* Select IRC clock before manipulating PLL */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    LPC_SYSCON->MAINCLKSELB = (MAINCLKSELB_SEL_PLLIN << SYSCON_MAINCLKSELB_SEL_Pos);
    LPC_SYSCON->MAINCLKSELA = (MAINCLKSELA_SEL_IRC << SYSCON_MAINCLKSELA_SEL_Pos);
    LPC_SYSCON->MAINCLKSELB = (MAINCLKSELB_SEL_MAINCLKSELA << SYSCON_MAINCLKSELB_SEL_Pos);
    LPC_SYSCON->AHBCLKDIV = 1;                          /* CPU clock = main clock */
    SystemCoreClock = IRC_FREQUENCY;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    LPC_SYSCON->MAINCLKSELA = (MAINCLKSELA_SEL_FRO12 << SYSCON_MAINCLKSELA_SEL_Pos);
    LPC_SYSCON->MAINCLKSELB = (MAINCLKSELB_SEL_MAINCLKSELA << SYSCON_MAINCLKSELB_SEL_Pos);
    LPC_SYSCON->AHBCLKDIV = 0;                          /* CPU clock = main clock */
    SystemCoreClock = FRO12_FREQUENCY;
#endif

    /* Disable PLL and set its input clock to the specified fundamental clock */
    bool syspllClkOk = true;
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    LPC_SYSCON->PDRUNCFGSET = SYSCON_PDRUNCFG_PDEN_SYS_PLL_Msk;
    switch (fundamentalClock) {
        case CLKPWR_CLOCK_IRC:
            LPC_SYSCON->SYSPLLCLKSEL = 0;
            break;
        case CLKPWR_CLOCK_CLKIN:
            LPC_SYSCON->SYSPLLCLKSEL = 1;
            break;
        case CLKPWR_CLOCK_RTC:
            LPC_SYSCON->SYSPLLCLKSEL = 3;
            break;
        default:
            LPC_SYSCON->SYSPLLCLKSEL = 2;
            syspllClkOk = false;
            break;
    }
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    LPC_SYSCON->PDRUNCFGSET0 = SYSCON_PDRUNCFG_PDEN_SYS_PLL_Msk;
    switch (fundamentalClock) {
        case CLKPWR_CLOCK_FRO12:
            LPC_SYSCON->SYSPLLCLKSEL = 0;
            break;
        case CLKPWR_CLOCK_CLKIN:
            LPC_SYSCON->SYSPLLCLKSEL = 1;
            break;
        case CLKPWR_CLOCK_RTC:
            LPC_SYSCON->SYSPLLCLKSEL = 3;
            break;
        default:
            LPC_SYSCON->SYSPLLCLKSEL = 7;
            syspllClkOk = false;
            break;
    }
#endif

    /* Get value of input frequency */
    inputFrequency = CLKPWR_getBusClock(fundamentalClock);

if((inputFrequency==targetCpuFrequency)||!syspllClkOk){
    SystemCoreClock=targetCpuFrequency;
    return LPCLIB_SUCCESS;
}

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    /* Call Power Profiles clocking routines */
    if (pRom->pPwrd->set_voltage(0, targetCpuFrequency) != ROM_ERR_SUCCESS) {
        return LPCLIB_ERROR;
    }
    if (pRom->pPwrd->set_pll(targetCpuFrequency / inputFrequency, inputFrequency) != ROM_ERR_SUCCESS) {
        return LPCLIB_ERROR;
    }

    LPC_SYSCON->MAINCLKSELB =                           /* Switch main clock to PLL */
        (MAINCLKSELB_SEL_PLLOUT << SYSCON_MAINCLKSELB_SEL_Pos);
    SystemCoreClock = CLKPWR_getBusClock(CLKPWR_CLOCK_MAIN) / LPC_SYSCON->AHBCLKDIV;
#endif

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    /* NOTE:
     * Increase VD1 from 0.9V to 1.0V when running faster than 12 MHz.
     * A few devices are failing @ 48 MHz when running with the default 0.9V.
     * There's a small increase in active power consumption.
     */
    if (targetCpuFrequency <= 12000000ul) {
        LPC_SYSCONEXTRA->VDCTRL[0] = 5;
    }
    else {
        LPC_SYSCONEXTRA->VDCTRL[0] = 7;
    }

if(targetCpuFrequency==48000000){
    // P=1 -> Fcco=96 MHz
    // N=2 -> Fref=6 MHz
    // Meff=2*8
    uint32_t x;
    int val;
    uint32_t N = 2;
    uint32_t Meff = 16;
    uint32_t M = Meff / 2;
    uint32_t P = 1;
(void)inputFrequency; //TODO

    /* NDEC, see UM 1.8, 6.5.49.6.1 */
    switch (N) {
        case 1:
            x = 0x302;
            break;
        case 2:
            x = 0x202;
            break;
        default:
            x = 0x80;
            for (val = N; val <= 256; val++) {
                x = (((x ^ (x >> 2) ^ (x >> 3) ^ (x >> 4)) & 1) << 7) | ((x >> 1) & 0x7F);
            }
            break;
    }
    LPC_SYSCON->SYSPLLNDEC = x | (1u << SYSCON_SYSPLLNDEC_NREQ_Pos);

    /* MDEC, see UM 1.8, 6.5.49.6.2 */
    switch (M) {
        case 1:
            x = 0x18003;
            break;
        case 2:
            x = 0x10003;
            break;
        default:
            x = 0x4000;
            for (val = M; val <= 32768; val++) {
                x = (((x ^ (x >> 1)) & 1) << 14) | ((x >> 1) & 0x3FFF);
            }
            break;
    }
    LPC_SYSCON->SYSPLLSSCTRL0 = 0
        | x
        | (1u << SYSCON_SYSPLLSSCTRL0_MREQ_Pos)
        | (1u << SYSCON_SYSPLLSSCTRL0_SEL_EXT_Pos)
        ;

    /* PDEC, see UM 1.8, 6.5.49.6.3 */
    switch (P) {
        case 1:
            x = 0x62;
            break;
        case 2:
            x = 0x42;
            break;
        default:
            x = 0x10;
            for (val = P; val <= 32; val++) {
                x = (((x ^ (x >> 2)) & 1) << 4) | ((x >> 1) & 0xF);
            }
            break;
    }
    LPC_SYSCON->SYSPLLPDEC = x | (1u << SYSCON_SYSPLLPDEC_PREQ_Pos);

    uint32_t selp, seli, selr;
    selp = 30;
    seli = 63;
selp = 5;
seli = 12;
    selr = 0;
    LPC_SYSCON->SYSPLLCTRL = 0
                | (selr << 0)
                | (seli << 4)
                | (selp << 10)
                | (1u << 18)
                ;

    LPC_SYSCON->PDRUNCFGCLR0 = SYSCON_PDRUNCFG_PDEN_SYS_PLL_Msk;
    while (!(LPC_SYSCON->SYSPLLSTAT & SYSCON_SYSPLLSTAT_LOCK_Msk))
        ;

    /* Set flash wait states for 48 MHz operation */
    LPC_SYSCON->FLASHCFG = (LPC_SYSCON->FLASHCFG & ~SYSCON_FLASHCFG_FLASHTIM_Msk)
        | (_clkpwrFlashWaitStates[(targetCpuFrequency - 1) / 12000000ul] << SYSCON_FLASHCFG_FLASHTIM_Pos);

    LPC_SYSCON->MAINCLKSELB =                           /* Switch main clock to PLL */
        (MAINCLKSELB_SEL_PLLOUT << SYSCON_MAINCLKSELB_SEL_Pos);

    uint32_t ahbDivider = 1 + ((LPC_SYSCON->AHBCLKDIV & SYSCON_AHBCLKDIV_DIV_Msk) >> SYSCON_AHBCLKDIV_DIV_Pos);
    SystemCoreClock = CLKPWR_getBusClock(CLKPWR_CLOCK_MAIN) / ahbDivider;
}
#endif

    /* Update SysTick calibration register */
    LPC_SYSCON->SYSTCKCAL = 0
            | (((SystemCoreClock / 100) << SYSCON_SYSTCKCAL_CAL_Pos) & SYSCON_SYSTCKCAL_CAL_Msk)
            | (0 << SYSCON_SYSTCKCAL_SKEW_Pos)
            | (0 << SYSCON_SYSTCKCAL_NOREF_Pos)
            ;

    return LPCLIB_SUCCESS;
}


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
/* Enter deep sleep mode. This function runs in SRAM0. */
void _CLKPWR_enterDeepSleep (uint32_t userPeripheralEnable);
__SECTION("LPCLIB_RAMCODE")
void _CLKPWR_enterDeepSleep (uint32_t userPeripheralEnable)
{
    uint32_t oldRUNCFG;
    uint32_t oldAHBClocks0 = LPC_SYSCON->AHBCLKCTRL0;

    /* Allow going deep */
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    oldRUNCFG = LPC_SYSCON->PDRUNCFG0;
    uint32_t newSLEEPCFG = oldRUNCFG;

    /* Enable unknown power switch 3 */
    newSLEEPCFG &= ~SYSCON_PDRUNCFG_PDEN_3_Msk;
    /* Try to disable all but some selected blocks.
     * User can override the disable requests.
     */
    newSLEEPCFG |= ~(0
        | SYSCON_PDRUNCFG_PDEN_1_Msk        /* DEEP_PD (unknown) */
        | SYSCON_PDRUNCFG_PDEN_3_Msk        /* ? */
        | SYSCON_PDRUNCFG_PDEN_SRAM0_Msk    /* SRAM0 */
        | SYSCON_PDRUNCFG_PDEN_26_Msk       /* VD3 (unknown) */
        );
    /* The following peripherals can be enabled by user request:
     * 4    FRO
     * 7    Brownout reset
     * 8    Brownout interrupt
     * 14   SRAM1
     * 15   SRAM2
     * 16   SRAMX
     * 20   Watchdog oscillator
     * 21   USB PHY
     * 24   RTC oscillator
     */
    newSLEEPCFG &= ~(userPeripheralEnable & (0
        | SYSCON_PDRUNCFG_PDEN_FRO_Msk
        | SYSCON_PDRUNCFG_PDEN_BOD_RST_Msk
        | SYSCON_PDRUNCFG_PDEN_BOD_INTR_Msk
        | SYSCON_PDRUNCFG_PDEN_SRAM1_Msk
        | SYSCON_PDRUNCFG_PDEN_SRAM2_Msk
        | SYSCON_PDRUNCFG_PDEN_SRAMX_Msk
        | SYSCON_PDRUNCFG_PDEN_WDT_OSC_Msk
        | SYSCON_PDRUNCFG_PDEN_USB_PHY_Msk
        | SYSCON_PDRUNCFG_PDEN_32K_OSC_Msk
        ));

    /* Apply */
    LPC_SYSCON->PDSLEEPCFG0 = newSLEEPCFG;

    /* Cut power to flash and keep it in reset */
    LPC_SYSCON->AHBCLKCTRLSET0 = SYSCON_AHBCLKCTRL0_FLASH_Msk;
    LPC_SYSCON->PDRUNCFGSET0 = SYSCON_PDRUNCFG_PDEN_FLASH_Msk | SYSCON_PDRUNCFG_PDEN_18_Msk;
    LPC_SYSCON->PRESETCTRLSET0 = SYSCON_PRESETCTRL0_FLASH_RST_Msk;

    __WFI();

    /* If flash was enabled before, reenable it now. */
    if (!(oldRUNCFG & SYSCON_PDRUNCFG_PDEN_FLASH_Msk)) {
        LPC_SYSCON->PDRUNCFGCLR0 = SYSCON_PDRUNCFG_PDEN_FLASH_Msk | SYSCON_PDRUNCFG_PDEN_18_Msk;

        /* Some delay */
        for (volatile int i = 0; i < 1000; i++)
            ;
    }

    LPC_SYSCON->PRESETCTRLCLR0 = SYSCON_PRESETCTRL0_FLASH_RST_Msk;
    LPC_SYSCON->PDRUNCFG0 = oldRUNCFG & ~SYSCON_PDRUNCFG_PDEN_3_Msk;

    /* Dummy read of flash sector 1 */
    volatile uint32_t dummy = ((volatile uint32_t *)0x1000)[0];
    (void)dummy;

    if (!(oldAHBClocks0 & SYSCON_AHBCLKCTRL0_FLASH_Msk)) {
        LPC_SYSCON->AHBCLKCTRLCLR0 = SYSCON_AHBCLKCTRL0_FLASH_Msk;
    }

    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
}
#endif



/* Enter a power-saving mode. */
void CLKPWR_enterPowerSaving (CLKPWR_PowerSavingMode mode)
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    /* SLEEPDEEP bit required for anything but SLEEP mode */
    if (mode & (1u << 16)) {
        SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
    }
    else {
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
    }

    /* Enter low-power mode. Wait for wake-up. */
    __WFI();

    /* Make sure any subsequent WFI will simply enter SLEEP */
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    uint32_t primask;

    switch (mode) {
        case CLKPWR_POWERSAVING_SLEEP:
            /* Don't go deep */
            SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

            /* Enter sleep. Wait for any interrupt to wake-up. */
            __WFI();
            break;

        case CLKPWR_POWERSAVING_DEEPSLEEP:
            /* No more interrupts from now on */
            primask = __get_PRIMASK();
            __disable_irq();

            _CLKPWR_enterDeepSleep(0
                | SYSCON_PDRUNCFG_PDEN_SRAM1_Msk
                | SYSCON_PDRUNCFG_PDEN_SRAM2_Msk
                | SYSCON_PDRUNCFG_PDEN_SRAMX_Msk
                );

            __set_PRIMASK(primask);
            break;

        case CLKPWR_POWERSAVING_DEEPPOWERDOWN:
            /* No more interrupts from now on */
            __disable_irq();

            /* Allow going deep */
            SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

            /* Make sure the RTC oscillator is running */
            LPC_SYSCON->PDRUNCFGCLR0 = SYSCON_PDRUNCFG_PDEN_32K_OSC_Msk;

            /* Enter deep power down. Wait RTC wakeup or reset. */
            __WFI();
            break;
    }
#endif
}


/** @} */

/** @} CLKPWR */
