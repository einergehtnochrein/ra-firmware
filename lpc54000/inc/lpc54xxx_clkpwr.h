/* Copyright (c) 2014-2016, DF9DQ
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
 *  \brief CLKPWR driver interface.
 *  This file defines all interface objects needed to use the CLKPWR driver.
 */


#ifndef __LPC54XXX_CLKPWR_H__
#define __LPC54XXX_CLKPWR_H__

/** \defgroup CLKPWR
 *  \ingroup API
 *  @{
 */

#include "lpc54xxx_libconfig.h"
#include "lpclib_types.h"


/** \defgroup CLKPWR_Public_Types CLKPWR Types, enums, macros
 *  @{
 */

#define CLKPWR_OSCILLATOR_IRC       1
#define CLKPWR_OSCILLATOR_SYSTEM    2


typedef enum CLKPWR_Clockswitch {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    CLKPWR_CLOCKSWITCH_ROM                  = 1,
    CLKPWR_CLOCKSWITCH_SRAM1                = 3,
    CLKPWR_CLOCKSWITCH_SRAM2                = 4,
    CLKPWR_CLOCKSWITCH_FLASH                = 7,
    CLKPWR_CLOCKSWITCH_FMC                  = 8,
    CLKPWR_CLOCKSWITCH_INPUTMUX             = 11,
    CLKPWR_CLOCKSWITCH_IOCON                = 13,
    CLKPWR_CLOCKSWITCH_GPIO0                = 14,
    CLKPWR_CLOCKSWITCH_GPIO1                = 15,
    CLKPWR_CLOCKSWITCH_PINT                 = 18,
    CLKPWR_CLOCKSWITCH_GINT                 = 19,
    CLKPWR_CLOCKSWITCH_DMA                  = 20,
    CLKPWR_CLOCKSWITCH_CRC                  = 21,
    CLKPWR_CLOCKSWITCH_WWDT                 = 22,
    CLKPWR_CLOCKSWITCH_RTC                  = 23,
    CLKPWR_CLOCKSWITCH_MAILBOX              = 26,
    CLKPWR_CLOCKSWITCH_ADC0                 = 27,
    CLKPWR_CLOCKSWITCH_MRT                  = 32 + 0,
    CLKPWR_CLOCKSWITCH_RIT                  = 32 + 1,
    CLKPWR_CLOCKSWITCH_SCT0                 = 32 + 2,
    CLKPWR_CLOCKSWITCH_FIFO                 = 32 + 9,
    CLKPWR_CLOCKSWITCH_UTICK                = 32 + 10,
    CLKPWR_CLOCKSWITCH_TIMER2               = 32 + 22,
    CLKPWR_CLOCKSWITCH_TIMER3               = 32 + 26,
    CLKPWR_CLOCKSWITCH_TIMER4               = 32 + 27,
    CLKPWR_CLOCKSWITCH_EZH                  = 32 + 31,
    CLKPWR_CLOCKSWITCH_UART0                = 64 + 1,
    CLKPWR_CLOCKSWITCH_UART1                = 64 + 2,
    CLKPWR_CLOCKSWITCH_UART2                = 64 + 3,
    CLKPWR_CLOCKSWITCH_UART3                = 64 + 4,
    CLKPWR_CLOCKSWITCH_I2C0                 = 64 + 5,
    CLKPWR_CLOCKSWITCH_I2C1                 = 64 + 6,
    CLKPWR_CLOCKSWITCH_I2C2                 = 64 + 7,
    CLKPWR_CLOCKSWITCH_SPI0                 = 64 + 9,
    CLKPWR_CLOCKSWITCH_SPI1                 = 64 + 10,
    CLKPWR_CLOCKSWITCH_TIMER0               = 64 + 13,
    CLKPWR_CLOCKSWITCH_TIMER1               = 64 + 14,
    CLKPWR_CLOCKSWITCH_FRG0                 = 64 + 15,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    CLKPWR_CLOCKSWITCH_ROM                  = 1,
    CLKPWR_CLOCKSWITCH_SRAM1                = 3,
    CLKPWR_CLOCKSWITCH_SRAM2                = 4,
    CLKPWR_CLOCKSWITCH_FLASH                = 7,
    CLKPWR_CLOCKSWITCH_FMC                  = 8,
    CLKPWR_CLOCKSWITCH_INPUTMUX             = 11,
    CLKPWR_CLOCKSWITCH_IOCON                = 13,
    CLKPWR_CLOCKSWITCH_GPIO0                = 14,
    CLKPWR_CLOCKSWITCH_GPIO1                = 15,
    CLKPWR_CLOCKSWITCH_PINT                 = 18,
    CLKPWR_CLOCKSWITCH_GINT                 = 19,
    CLKPWR_CLOCKSWITCH_DMA                  = 20,
    CLKPWR_CLOCKSWITCH_CRC                  = 21,
    CLKPWR_CLOCKSWITCH_WWDT                 = 22,
    CLKPWR_CLOCKSWITCH_RTC                  = 23,
    CLKPWR_CLOCKSWITCH_MAILBOX              = 26,
    CLKPWR_CLOCKSWITCH_ADC0                 = 27,
    CLKPWR_CLOCKSWITCH_MRT                  = 32 + 0,
    CLKPWR_CLOCKSWITCH_SCT0                 = 32 + 2,
    CLKPWR_CLOCKSWITCH_UTICK                = 32 + 10,
    CLKPWR_CLOCKSWITCH_FLEXCOMM0            = 32 + 11,
    CLKPWR_CLOCKSWITCH_FLEXCOMM1            = 32 + 12,
    CLKPWR_CLOCKSWITCH_FLEXCOMM2            = 32 + 13,
    CLKPWR_CLOCKSWITCH_FLEXCOMM3            = 32 + 14,
    CLKPWR_CLOCKSWITCH_FLEXCOMM4            = 32 + 15,
    CLKPWR_CLOCKSWITCH_FLEXCOMM5            = 32 + 16,
    CLKPWR_CLOCKSWITCH_FLEXCOMM6            = 32 + 17,
    CLKPWR_CLOCKSWITCH_FLEXCOMM7            = 32 + 18,
    CLKPWR_CLOCKSWITCH_UART0                = CLKPWR_CLOCKSWITCH_FLEXCOMM0,
    CLKPWR_CLOCKSWITCH_UART1                = CLKPWR_CLOCKSWITCH_FLEXCOMM1,
    CLKPWR_CLOCKSWITCH_UART2                = CLKPWR_CLOCKSWITCH_FLEXCOMM2,
    CLKPWR_CLOCKSWITCH_UART3                = CLKPWR_CLOCKSWITCH_FLEXCOMM3,
    CLKPWR_CLOCKSWITCH_UART4                = CLKPWR_CLOCKSWITCH_FLEXCOMM4,
    CLKPWR_CLOCKSWITCH_UART5                = CLKPWR_CLOCKSWITCH_FLEXCOMM5,
    CLKPWR_CLOCKSWITCH_UART6                = CLKPWR_CLOCKSWITCH_FLEXCOMM6,
    CLKPWR_CLOCKSWITCH_UART7                = CLKPWR_CLOCKSWITCH_FLEXCOMM7,
    CLKPWR_CLOCKSWITCH_I2C0                 = CLKPWR_CLOCKSWITCH_FLEXCOMM0,
    CLKPWR_CLOCKSWITCH_I2C1                 = CLKPWR_CLOCKSWITCH_FLEXCOMM1,
    CLKPWR_CLOCKSWITCH_I2C2                 = CLKPWR_CLOCKSWITCH_FLEXCOMM2,
    CLKPWR_CLOCKSWITCH_I2C3                 = CLKPWR_CLOCKSWITCH_FLEXCOMM3,
    CLKPWR_CLOCKSWITCH_I2C4                 = CLKPWR_CLOCKSWITCH_FLEXCOMM4,
    CLKPWR_CLOCKSWITCH_I2C5                 = CLKPWR_CLOCKSWITCH_FLEXCOMM5,
    CLKPWR_CLOCKSWITCH_I2C6                 = CLKPWR_CLOCKSWITCH_FLEXCOMM6,
    CLKPWR_CLOCKSWITCH_I2C7                 = CLKPWR_CLOCKSWITCH_FLEXCOMM7,
    CLKPWR_CLOCKSWITCH_SPI0                 = CLKPWR_CLOCKSWITCH_FLEXCOMM0,
    CLKPWR_CLOCKSWITCH_SPI1                 = CLKPWR_CLOCKSWITCH_FLEXCOMM1,
    CLKPWR_CLOCKSWITCH_SPI2                 = CLKPWR_CLOCKSWITCH_FLEXCOMM2,
    CLKPWR_CLOCKSWITCH_SPI3                 = CLKPWR_CLOCKSWITCH_FLEXCOMM3,
    CLKPWR_CLOCKSWITCH_SPI4                 = CLKPWR_CLOCKSWITCH_FLEXCOMM4,
    CLKPWR_CLOCKSWITCH_SPI5                 = CLKPWR_CLOCKSWITCH_FLEXCOMM5,
    CLKPWR_CLOCKSWITCH_SPI6                 = CLKPWR_CLOCKSWITCH_FLEXCOMM6,
    CLKPWR_CLOCKSWITCH_SPI7                 = CLKPWR_CLOCKSWITCH_FLEXCOMM7,
    CLKPWR_CLOCKSWITCH_DMIC                 = 32 + 19,
    CLKPWR_CLOCKSWITCH_TIMER2               = 32 + 22,
    CLKPWR_CLOCKSWITCH_USB                  = 32 + 25,
    CLKPWR_CLOCKSWITCH_TIMER0               = 32 + 26,
    CLKPWR_CLOCKSWITCH_TIMER1               = 32 + 27,
    CLKPWR_CLOCKSWITCH_EZH                  = 32 + 31,
    CLKPWR_CLOCKSWITCH_TIMER3               = 64 + 13,
    CLKPWR_CLOCKSWITCH_TIMER4               = 64 + 14,
#endif
} CLKPWR_Clockswitch;

typedef enum CLKPWR_Divider {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    CLKPWR_DIVIDER_AHB                      = 64,
    CLKPWR_DIVIDER_ADC                      = 66,
    CLKPWR_DIVIDER_CLKOUT                   = 67,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    CLKPWR_DIVIDER_AHB                      = 224,
    CLKPWR_DIVIDER_CLKOUT                   = 225,
    CLKPWR_DIVIDER_ADC                      = 229,
    CLKPWR_DIVIDER_USB                      = 230,
    CLKPWR_DIVIDER_DMIC                     = 234,
    CLKPWR_DIVIDER_I2SMCLK                  = 235,
#endif
} CLKPWR_Divider;


typedef enum CLKPWR_Clock {
    _CLKPWR_CLOCK_NONE = 0,
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    _CLKPWR_CLOCK_MAINCLKSELA,
    _CLKPWR_CLOCK_ASYNCCLKSELA,

    CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_IRC,
    CLKPWR_CLOCK_WDT,
    CLKPWR_CLOCK_MAIN,
    CLKPWR_CLOCK_CLKIN,
    CLKPWR_CLOCK_SYSTEMPLLIN,
    CLKPWR_CLOCK_SYSTEMPLL,
    CLKPWR_CLOCK_ADC,
    CLKPWR_CLOCK_RTC,
    CLKPWR_CLOCK_CPU,

    CLKPWR_CLOCK_TIMER2 = CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_TIMER3 = CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_TIMER4 = CLKPWR_CLOCK_CPU,

    CLKPWR_CLOCK_UART0 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_UART1 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_UART2 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_UART3 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_I2C0 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_I2C1 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_I2C2 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_SPI0 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_SPI1 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_TIMER0 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_TIMER1 = CLKPWR_CLOCK_ASYNCAPB,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    _CLKPWR_CLOCK_MAINCLKSELA,
    _CLKPWR_CLOCK_ASYNCCLKSELA,

    CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_FRO12,
    CLKPWR_CLOCK_FROHF,
    CLKPWR_CLOCK_WDT,
    CLKPWR_CLOCK_MAIN,
    CLKPWR_CLOCK_CLKIN,
    CLKPWR_CLOCK_SYSTEMPLLIN,
    CLKPWR_CLOCK_SYSTEMPLL,
    CLKPWR_CLOCK_ADC,
    CLKPWR_CLOCK_FRG,
    CLKPWR_CLOCK_MCLK,
    CLKPWR_CLOCK_DMIC,
    CLKPWR_CLOCK_RTC,
    CLKPWR_CLOCK_CPU,

    CLKPWR_CLOCK_TIMER0 = CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_TIMER1 = CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_TIMER2 = CLKPWR_CLOCK_CPU,
    CLKPWR_CLOCK_FLEXCOMM0,
    CLKPWR_CLOCK_FLEXCOMM1,
    CLKPWR_CLOCK_FLEXCOMM2,
    CLKPWR_CLOCK_FLEXCOMM3,
    CLKPWR_CLOCK_FLEXCOMM4,
    CLKPWR_CLOCK_FLEXCOMM5,
    CLKPWR_CLOCK_FLEXCOMM6,
    CLKPWR_CLOCK_FLEXCOMM7,
    CLKPWR_CLOCK_UART0 = CLKPWR_CLOCK_FLEXCOMM0,
    CLKPWR_CLOCK_UART1 = CLKPWR_CLOCK_FLEXCOMM1,
    CLKPWR_CLOCK_UART2 = CLKPWR_CLOCK_FLEXCOMM2,
    CLKPWR_CLOCK_UART3 = CLKPWR_CLOCK_FLEXCOMM3,
    CLKPWR_CLOCK_UART4 = CLKPWR_CLOCK_FLEXCOMM4,
    CLKPWR_CLOCK_UART5 = CLKPWR_CLOCK_FLEXCOMM5,
    CLKPWR_CLOCK_UART6 = CLKPWR_CLOCK_FLEXCOMM6,
    CLKPWR_CLOCK_UART7 = CLKPWR_CLOCK_FLEXCOMM7,
    CLKPWR_CLOCK_SPI0 = CLKPWR_CLOCK_FLEXCOMM0,
    CLKPWR_CLOCK_SPI1 = CLKPWR_CLOCK_FLEXCOMM1,
    CLKPWR_CLOCK_SPI2 = CLKPWR_CLOCK_FLEXCOMM2,
    CLKPWR_CLOCK_SPI3 = CLKPWR_CLOCK_FLEXCOMM3,
    CLKPWR_CLOCK_SPI4 = CLKPWR_CLOCK_FLEXCOMM4,
    CLKPWR_CLOCK_SPI5 = CLKPWR_CLOCK_FLEXCOMM5,
    CLKPWR_CLOCK_SPI6 = CLKPWR_CLOCK_FLEXCOMM6,
    CLKPWR_CLOCK_SPI7 = CLKPWR_CLOCK_FLEXCOMM7,
    CLKPWR_CLOCK_I2C0 = CLKPWR_CLOCK_FLEXCOMM0,
    CLKPWR_CLOCK_I2C1 = CLKPWR_CLOCK_FLEXCOMM1,
    CLKPWR_CLOCK_I2C2 = CLKPWR_CLOCK_FLEXCOMM2,
    CLKPWR_CLOCK_I2C3 = CLKPWR_CLOCK_FLEXCOMM3,
    CLKPWR_CLOCK_I2C4 = CLKPWR_CLOCK_FLEXCOMM4,
    CLKPWR_CLOCK_I2C5 = CLKPWR_CLOCK_FLEXCOMM5,
    CLKPWR_CLOCK_I2C6 = CLKPWR_CLOCK_FLEXCOMM6,
    CLKPWR_CLOCK_I2C7 = CLKPWR_CLOCK_FLEXCOMM7,

    CLKPWR_CLOCK_TIMER3 = CLKPWR_CLOCK_ASYNCAPB,
    CLKPWR_CLOCK_TIMER4 = CLKPWR_CLOCK_ASYNCAPB,
#endif
} CLKPWR_Clock;

typedef enum CLKPWR_Reset {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    CLKPWR_RESET_FLASH = 7,
    CLKPWR_RESET_FMC = 8,
    CLKPWR_RESET_MUX = 11,
    CLKPWR_RESET_IOCON = 13,
    CLKPWR_RESET_GPIO0 = 14,
    CLKPWR_RESET_GPIO1 = 15,
    CLKPWR_RESET_PINT = 18,
    CLKPWR_RESET_GINT = 19,
    CLKPWR_RESET_DMA = 20,
    CLKPWR_RESET_CRC = 21,
    CLKPWR_RESET_WWDT = 22,
    CLKPWR_RESET_ADC0 = 27,
    CLKPWR_RESET_MRT = 32 + 0,
    CLKPWR_RESET_RIT = 32 + 1,
    CLKPWR_RESET_SCT0 = 32 + 2,
    CLKPWR_RESET_FIFO = 32 + 9,
    CLKPWR_RESET_UTICK = 32 + 10,
    CLKPWR_RESET_CT32B2 = 32 + 22,
    CLKPWR_RESET_CT32B3 = 32 + 26,
    CLKPWR_RESET_CT32B4 = 32 + 27,
    CLKPWR_RESET_EZH = 32 + 31,
    CLKPWR_RESET_USART0 = 64 + 1,
    CLKPWR_RESET_USART1 = 64 + 2,
    CLKPWR_RESET_USART2 = 64 + 3,
    CLKPWR_RESET_USART3 = 64 + 4,
    CLKPWR_RESET_I2C0 = 64 + 5,
    CLKPWR_RESET_I2C1 = 64 + 6,
    CLKPWR_RESET_I2C2 = 64 + 7,
    CLKPWR_RESET_SPI0 = 64 + 9,
    CLKPWR_RESET_SPI1 = 64 + 10,
    CLKPWR_RESET_CT32B0 = 64 + 13,
    CLKPWR_RESET_CT32B1 = 64 + 14,
    CLKPWR_RESET_FRG0 = 64 + 15,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    CLKPWR_RESET_FLASH = 7,
    CLKPWR_RESET_FMC = 8,
    CLKPWR_RESET_MUX = 11,
    CLKPWR_RESET_IOCON = 13,
    CLKPWR_RESET_GPIO0 = 14,
    CLKPWR_RESET_GPIO1 = 15,
    CLKPWR_RESET_PINT = 18,
    CLKPWR_RESET_GINT = 19,
    CLKPWR_RESET_DMA = 20,
    CLKPWR_RESET_CRC = 21,
    CLKPWR_RESET_WWDT = 22,
    CLKPWR_RESET_ADC0 = 27,
    CLKPWR_RESET_MRT = 32 + 0,
    CLKPWR_RESET_SCT0 = 32 + 2,
    CLKPWR_RESET_UTICK = 32 + 10,
    CLKPWR_RESET_FC0 = 32 + 11,
    CLKPWR_RESET_FC1 = 32 + 12,
    CLKPWR_RESET_FC2 = 32 + 13,
    CLKPWR_RESET_FC3 = 32 + 14,
    CLKPWR_RESET_FC4 = 32 + 15,
    CLKPWR_RESET_FC5 = 32 + 16,
    CLKPWR_RESET_FC6 = 32 + 17,
    CLKPWR_RESET_FC7 = 32 + 18,
    CLKPWR_RESET_DMIC = 32 + 19,
    CLKPWR_RESET_CTIMER2 = 32 + 22,
    CLKPWR_RESET_USB = 32 + 25,
    CLKPWR_RESET_CTIMER0 = 32 + 26,
    CLKPWR_RESET_CTIMER1 = 32 + 27,
    CLKPWR_RESET_EZH = 32 + 31,
    CLKPWR_RESET_CTIMER3 = 64 + 13,
    CLKPWR_RESET_CTIMER4 = 64 + 14,
#endif
} CLKPWR_Reset;

typedef enum CLKPWR_UnitPower {
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    CLKPWR_UNITPOWER___DUMMY__,
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    CLKPWR_UNIT_FRO = 4,
    CLKPWR_UNIT_TS = 6,
    CLKPWR_UNIT_BODRST = 7,
    CLKPWR_UNIT_BODINTR = 8,
    CLKPWR_UNIT_ADC0 = 10,
    CLKPWR_UNIT_SRAM0 = 13,
    CLKPWR_UNIT_SRAM1 = 14,
    CLKPWR_UNIT_SRAM2 = 15,
    CLKPWR_UNIT_SRAMX = 16,
    CLKPWR_UNIT_ROM = 17,
    CLKPWR_UNIT_VDDA = 19,
    CLKPWR_UNIT_WDTOSC = 20,
    CLKPWR_UNIT_USBPAD = 21,
    CLKPWR_UNIT_SYSPLL = 22,
    CLKPWR_UNIT_VREFP = 23,
#endif
} CLKPWR_UnitPower;


typedef enum CLKPWR_PowerSavingMode {
    CLKPWR_POWERSAVING_SLEEP            = (0u << 16) | (1u <<  8) | 0,
    CLKPWR_POWERSAVING_DEEPSLEEP        = (1u << 16) | (1u <<  9) | 0,
    CLKPWR_POWERSAVING_DEEPPOWERDOWN    = (1u << 16) | (1u << 11) | 3,
} CLKPWR_PowerSavingMode;



/** @} CLKPWR Types, enums, macros */


/** \defgroup CLKPWR_Public_Functions CLKPWR API Functions
 *  @{
 */


/** Return the clock frequency of an internal bus.
 *
 *  \param[in] bus The bus to be queried
 *  \return Bus clock in Hz
 */
uint32_t CLKPWR_getBusClock (CLKPWR_Clock clock);


/** Enable a clock signal.
 *
 *  \param[in] clock Clock switch selector
 */
static void CLKPWR_enableClock (CLKPWR_Clockswitch clock);


/** Disable a clock signal.
 *
 *  \param[in] clock Clock switch selector
 */
static void CLKPWR_disableClock (CLKPWR_Clockswitch clock);


/** Power up a functional unit.
 *
 *  \param[in] unit Unit selector
 */
static void CLKPWR_unitPowerUp (CLKPWR_UnitPower unit);


/** Power down a functional unit.
 *
 *  \param[in] unit Unit selector
 */
static void CLKPWR_unitPowerDown (CLKPWR_UnitPower unit);


/** Assert the reset signal of the selected peripheral block.
 *
 *  \param[in] peripheral Peripheral block selector
 */
static void CLKPWR_assertPeripheralReset (CLKPWR_Reset peripheral);


/** Deassert the reset signal of the selected peripheral block.
 *
 *  \param[in] peripheral Peripheral block selector
 */
static void CLKPWR_deassertPeripheralReset (CLKPWR_Reset peripheral);


/** Reset all peripherals.
 *
 *  Apply a reset to all peripherals. This can be useful to do at system startup in a
 *  debug session. In this case the peripherals may be in an undefined state, already
 *  fully or partially initialized from a previous run of the code.
 */
void CLKPWR_resetAllPeripherals (void);


/** Set a clock divider.
 *
 *  \param[in] divider Clock divider selector
 *  \param[in] value Division ratio
 *  \retval LPCLIB_SUCCESS Divider was set as requested.
 *  \retval LPCLIB_ILLEGAL_PARAMETER Divider cannot be set
 *          LPC178x: Use 'CLKPWR_DIVIDER_PCLK' to set the global
 *                   peripheral clock common to all peripherals.)
 *                   Use 'CLKPWR_DIVIDER_EMC' to set the CCLK vs EMC clock ratio.
 *          LPC17xx: You cannot use CLKPWR_DIVIDER_PCLK or CLKPWR_DIVIDER_EMC here.
 *                   Use individual peripheral clock selectors.
 */
LPCLIB_Result CLKPWR_setDivider (CLKPWR_Divider divider, uint32_t value);


/** Attempts to set the CPU clock to the desired frequency.
 *
 *  Sets the CPU clock to the frequency which is closest to the requested
 *  value. Assumes that the PLL input clock has been selected before.
 *
 *  \param[in] targetCpuFrequency Desired CPU frequency (Hz)
 *  \return ...
 */
LPCLIB_Result CLKPWR_setCpuClock (uint32_t targetCpuFrequency, CLKPWR_Clock fundamentalClock);


/** Sets up the USB clock using the USB PLL.
 *
 *  Only works if the crystal frequency is an integer fraction of 48 MHz.
 *
 *  \param[in] targetCpuFrequency Desired CPU frequency (kHz)
 *  \param[out] newCpuFrequency CPU frequency (kHz) after the call
 *  \return ...
 */
LPCLIB_Result CLKPWR_setUsbClock (void);



/** Enter a power-saving mode.
 *
 *  Enters the selected power-saving mode, and does not return before a qualified
 *  wake-up event occurs.
 *  Note 1: You must prepare a suitable wake-up source before entering this function.
 *          Failure to do so may cause the system to stay in power-saving mode forever.
 *  Note 2: This function uses the WFI instruction, not WFE.
 *
 *  \param[in] mode
 */
void CLKPWR_enterPowerSaving (CLKPWR_PowerSavingMode mode);



__FORCEINLINE(void CLKPWR_unitPowerUp (CLKPWR_UnitPower unit))
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    (void)unit;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    LPC_SYSCON->PDRUNCFGCLR0 = (1u << unit);
#endif
}

__FORCEINLINE(void CLKPWR_unitPowerDown (CLKPWR_UnitPower unit))
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    (void)unit;
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    LPC_SYSCON->PDRUNCFGSET0 = (1u << unit);
#endif
}

__FORCEINLINE(void CLKPWR_assertPeripheralReset (CLKPWR_Reset peripheral))
{
    if ((int)peripheral >= 64) {
        LPC_ASYNCSYSCON->ASYNCPRESETCTRLSET = (1u << ((int)peripheral % 32));
    }
    else {
        LPC_SYSCON->PRESETCTRLSET[(int)peripheral / 32] = (1u << ((int)peripheral % 32));
    }
}

__FORCEINLINE(void CLKPWR_deassertPeripheralReset (const CLKPWR_Reset peripheral))
{
    if ((int)peripheral >= 64) {
        LPC_ASYNCSYSCON->ASYNCPRESETCTRLCLR = (1u << ((int)peripheral % 32));
    }
    else {
        LPC_SYSCON->PRESETCTRLCLR[(int)peripheral / 32] = (1u << ((int)peripheral % 32));
    }
}



__FORCEINLINE(void CLKPWR_enableClock (CLKPWR_Clockswitch clock))
{
    if ((int)clock >= 64) {
        LPC_ASYNCSYSCON->ASYNCAPBCLKCTRLSET = (1u << ((int)clock % 32));
    }
    else {
        LPC_SYSCON->AHBCLKCTRLSET[(int)clock / 32] = (1u << ((int)clock % 32));
    }
}



__FORCEINLINE(void CLKPWR_disableClock (CLKPWR_Clockswitch clock))
{
    if ((int)clock >= 64) {
        LPC_ASYNCSYSCON->ASYNCAPBCLKCTRLCLR = (1u << ((int)clock % 32));
    }
    else {
        LPC_SYSCON->AHBCLKCTRLCLR[(int)clock / 32] = (1u << ((int)clock % 32));
    }
}


/** @} CLKPWR API Functions */

/** @} CLKPWR */

#endif /* #ifndef __LPC54XXX_CLKPWR_H__ */

