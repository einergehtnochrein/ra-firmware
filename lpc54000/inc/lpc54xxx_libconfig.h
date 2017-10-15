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
 *  LPC54xxx Library Configuration.
 *  This file defines all compile time options that enable (disable) and/or
 *  parameterize features of the peripheral driver library.
 */


/** \defgroup LPC_LibConfig Library Configuration
 *  \ingroup LPC54000_PDL
 *  \{
 */


#ifndef LPCLIB_CONFIG_H
#define LPCLIB_CONFIG_H


/* Application config options */
#include "lpclib_config.h"

/* OS header included here, because it's needed by most drivers */
#include "cmsis_os.h"


/** Device family.
 *  LPCLIB_FAMILY_5410X:
 *  LPCLIB_FAMILY_5411X:
 */
#define LPCLIB_FAMILY_LPC5410X                          0x54100001
#define LPCLIB_FAMILY_LPC5411X                          0x54110002
#ifndef LPCLIB_FAMILY
#define LPCLIB_FAMILY                                   LPCLIB_FAMILY_LPC5410X
#endif


/** Support GPIO interrupts.
 */
#ifndef LPCLIB_GPIO_INTERRUPTS
#define LPCLIB_GPIO_INTERRUPTS                          0
#endif



/** Crystal frequency (in Hz).
 *  Set to 0 if no crystal is used. In this case no attempt
 *  is made to start the main oscillator.
 */
#ifndef LPCLIB_CLKPWR_XTAL_FREQUENCY
#define LPCLIB_CLKPWR_XTAL_FREQUENCY                    12000000ul
#endif


/** PLL input clock source.
 *  This must be one of CLKPWR_OSCILLATOR_IRC or CLKPWR_OSCILLATOR_SYSTEM.
 *  (See enum type \ref CLKPWR_Oscillator).
 */
#ifndef LPCLIB_CLKPWR_PLL_INPUT_CLOCK
#define LPCLIB_CLKPWR_PLL_INPUT_CLOCK                   CLKPWR_OSCILLATOR_SYSTEM
#endif




/** Include TIMER driver. */
#ifndef LPCLIB_TIMER
#define LPCLIB_TIMER                                    1
#endif

/** Simple timer objects.
 *  If enabled, one of the timers must be assigned to it
 *  by the \ref LPCLIB_TIMER_SIMPLE_TIMER macro.
 */
#define LPCLIB_TIMER_USE_SIMPLE_TIMERS                  0

/** Select the hardware timer to be used as the simple timer base.
 *  0=TMR16_0, 1=TMR16_1, 2=TMR32_0, 3=TMR32_1
 */
#define LPCLIB_TIMER_SIMPLE_TIMER                       3

/** Number of objects supported by the simple timer.
 */
#define LPCLIB_TIMER_NUM_SIMPLE_TIMERS                  10



/** Include UART driver. */
#ifndef LPCLIB_UART
#define LPCLIB_UART                                     1
#endif

/** Support modem control signals. */
#ifndef LPCLIB_UART_MODEM_CONTROL
#define LPCLIB_UART_MODEM_CONTROL                       0
#endif

/** Support RS485 mode. */
#ifndef LPCLIB_UART_RS485
#define LPCLIB_UART_RS485                               0
#endif

/** Support for auto-bauding. */
#ifndef LPCLIB_UART_AUTOBAUDING
#define LPCLIB_UART_AUTOBAUDING                         0
#endif

/** Support handling of overflow, framing and parity errors.
 *  These error conditions are ignored if this option is disabled.
 */
#define LPCLIB_UART_ERROR_HANDLING                      0

/** Size of RX buffer(s) (in bytes) */
#ifndef LPCLIB_UART_RX_BUFFER_SIZE
#define LPCLIB_UART_RX_BUFFER_SIZE                      10
#endif

/** Size of TX buffer(s) (in bytes) */
#ifndef LPCLIB_UART_TX_BUFFER_SIZE
#define LPCLIB_UART_TX_BUFFER_SIZE                      10
#endif

/** Blocking UART_Write() in case buffer is full */
#define LPCLIB_UART_WRITE_BLOCKING                      1



/** Support master mode */
#ifndef LPCLIB_I2C_MASTER
#define LPCLIB_I2C_MASTER                               1
#endif

/** Support I2C slave mode */
#ifndef LPCLIB_I2C_SLAVE
#define LPCLIB_I2C_SLAVE                                0
#endif

/** Number of supported slave addresses (set to either 1 or 4) */
#ifndef LPCLIB_I2C_NUM_SLAVE_ADDRESSES
#define LPCLIB_I2C_NUM_SLAVE_ADDRESSES                  4
#endif

/** Default slave address (2*0x01...2*0x78, 0x00=off) */
#define LPCLIB_I2C_SLAVE_DEFAULT_ADDRESS                (0x00)
/** Mask for default slave address (0xFE=all address bits relevant) */
#define LPCLIB_I2C_SLAVE_DEFAULT_MASK                   (0x00)

/** Support I2C bit-banging via GPIO's */
#ifndef LPCLIB_I2CEMU
#define LPCLIB_I2CEMU                                   0
#endif



/** Include ADC driver */
#ifndef LPCLIB_ADC
#define LPCLIB_ADC                                      0
#endif



/** Include DMA driver */
#ifndef LPCLIB_DMA
#define LPCLIB_DMA                                      0
#endif



/*******************  Automatic settings  *****************/

/* Include the correct CMSIS-compatible device header.
 */
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
#include "LPC5410x.h"
#include "system_LPC5410x.h"

void HardFault_Handler (void);
void MPU_Handler (void);
void BusFault_Handler (void);
void UsageFault_Handler (void);
void SVC_Handler (void);
void PendSV_Handler (void);
void SysTick_Handler (void);

void WDT_IRQHandler (void);
void BOD_IRQHandler (void);
void DMA_IRQHandler (void);
void GINT0_IRQHandler (void);
void PIN_INT0_IRQHandler (void);
void PIN_INT1_IRQHandler (void);
void PIN_INT2_IRQHandler (void);
void PIN_INT3_IRQHandler (void);
void UTICK_IRQHandler (void);
void MRT_IRQHandler (void);
void TIMER0_IRQHandler (void);
void TIMER1_IRQHandler (void);
void TIMER2_IRQHandler (void);
void TIMER3_IRQHandler (void);
void TIMER4_IRQHandler (void);
void SCT0_IRQHandler (void);
void UART0_IRQHandler (void);
void UART1_IRQHandler (void);
void UART2_IRQHandler (void);
void UART3_IRQHandler (void);
void PWM1_IRQHandler (void);
void I2C0_IRQHandler (void);
void I2C1_IRQHandler (void);
void I2C2_IRQHandler (void);
void SPI0_IRQHandler (void);
void SPI1_IRQHandler (void);
void ADC0_SEQA_IRQHandler (void);
void ADC0_SEQB_IRQHandler (void);
void ADC0_THCMP_IRQHandler (void);
void RTC_IRQHandler (void);
void EZH_IRQHandler (void);
void MAILBOX_IRQHandler (void);

void GINT1_IRQHandler (void);
void PIN_INT4_IRQHandler (void);
void PIN_INT5_IRQHandler (void);
void PIN_INT6_IRQHandler (void);
void PIN_INT7_IRQHandler (void);
void RIT_IRQHandler (void);

#endif


#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
#include "LPC5411x.h"
#include "system_LPC5411x.h"

void HardFault_Handler (void);
void MPU_Handler (void);
void BusFault_Handler (void);
void UsageFault_Handler (void);
void SVC_Handler (void);
void PendSV_Handler (void);
void SysTick_Handler (void);

void WDT_BOD_IRQHandler (void);
void DMA_IRQHandler (void);
void GINT0_IRQHandler (void);
void GINT1_IRQHandler (void);
void PIN_INT0_IRQHandler (void);
void PIN_INT1_IRQHandler (void);
void PIN_INT2_IRQHandler (void);
void PIN_INT3_IRQHandler (void);
void UTICK_IRQHandler (void);
void MRT_IRQHandler (void);
void TIMER0_IRQHandler (void);
void TIMER1_IRQHandler (void);
void SCT0_IRQHandler (void);
void TIMER3_IRQHandler (void);
void FLEXCOMM0_IRQHandler (void);
void FLEXCOMM1_IRQHandler (void);
void FLEXCOMM2_IRQHandler (void);
void FLEXCOMM3_IRQHandler (void);
void FLEXCOMM4_IRQHandler (void);
void FLEXCOMM5_IRQHandler (void);
void FLEXCOMM6_IRQHandler (void);
void FLEXCOMM7_IRQHandler (void);
void ADC0_SEQA_IRQHandler (void);
void ADC0_SEQB_IRQHandler (void);
void ADC0_THCMP_IRQHandler (void);
void DMIC_IRQHandler (void);
void HWVAD_IRQHandler (void);
void USB_NEEDCLK_IRQHandler (void);
void USB_IRQHandler (void);
void RTC_IRQHandler (void);
void EZH_IRQHandler (void);
void MAILBOX_IRQHandler (void);
void PIN_INT4_IRQHandler (void);
void PIN_INT5_IRQHandler (void);
void PIN_INT6_IRQHandler (void);
void PIN_INT7_IRQHandler (void);
void TIMER2_IRQHandler (void);
void TIMER4_IRQHandler (void);
void SPIFI_IRQHandler (void);

#endif


/*******************  Check the config parameters  *****************/


#endif /* LPCLIB_CONFIG_H */

/** @} */

