/* Copyright (c) 2014, DF9DQ
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

#include "lpc54xxx_libconfig.h"


/** \addtogroup GPIO
 *  @{
 */

#include "lpc54xxx_clkpwr.h"
#include "lpc54xxx_gpio.h"



/** Field definition for hardware registers. */
LPCLIB_DefineRegBit(GINTx_CTRL_INT,             0,  1);
LPCLIB_DefineRegBit(GINTx_CTRL_COMB,            1,  1);
LPCLIB_DefineRegBit(GINTx_CTRL_TRIG,            2,  1);


/** GPIO module context. */
static struct GPIO_Context {
    osMutexId accessMutex;
#if LPCLIB_GPIO_INTERRUPTS
    GPIO_InterruptMode pinInterruptMode[__NUM_GPIO_PIN_INTERRUPT_LINES__];
    LPCLIB_Callback pinCallbacks[__NUM_GPIO_PIN_INTERRUPT_LINES__];
    LPCLIB_Callback groupCallbacks[__NUM_GPIO_GROUP_INTERRUPT_LINES__];
#endif
} gpioContext;

osMutexDef(gpioAccessMutexDef);



void GPIO_open (void)
{
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_GPIO0);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_GPIO1);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_PINT);
    gpioContext.accessMutex = osMutexCreate(osMutex(gpioAccessMutexDef));
}


void GPIO_setDir32 (GPIO_Port32 port, uint32_t value, uint32_t mask)
{
    if (osMutexWait(gpioContext.accessMutex, osWaitForever) == osOK) {
        LPC_GPIO->DIR[port] = (LPC_GPIO->DIR[port] & ~(~value & mask)) | (value & mask);

        osMutexRelease(gpioContext.accessMutex);
    }
}

void GPIO_setDirBit (GPIO_Pin pin, LPCLIB_Switch outputEnable)
{
    if (osMutexWait(gpioContext.accessMutex, osWaitForever) == osOK) {
        LPC_GPIO->DIR[pin / 32] = (LPC_GPIO->DIR[pin / 32] & ~(1u << (pin & 0x1F))) | (outputEnable << (pin & 0x1F));

        osMutexRelease(gpioContext.accessMutex);
    }
}


/* Set GPIO options. */
LPCLIB_Result GPIO_ioctl (const GPIO_Config *pConfig)
{
#if LPCLIB_GPIO_INTERRUPTS
    int port;
    int intNum;
    uint32_t mask;
    LPC_GPIO_GROUP_INT_Type *pGroupInt;
#endif
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;


    switch (pConfig->opcode) {
#if LPCLIB_GPIO_INTERRUPTS
    case GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT:
        intNum = pConfig->pinInterrupt.interruptLine;
        if (intNum >= __NUM_GPIO_PIN_INTERRUPT_LINES__) {
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        gpioContext.pinInterruptMode[intNum] = pConfig->pinInterrupt.mode;

        mask = 1u << intNum;

        LPC_PINT->CIENR = mask;                         /* Always mask interrupts during configuration */
        LPC_PINT->CIENF = mask;

        /* Connect selected GPIO to the indiviual interrupt line */
        CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_INPUTMUX);
        LPC_INPUTMUX->PINTSEL[intNum] =
                (LPC_INPUTMUX->PINTSEL[intNum] & ~0x7F)
            |   ((pConfig->pinInterrupt.pin / 32) << 5)
            |   ((pConfig->pinInterrupt.pin % 32) << 0);
        CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_INPUTMUX);

        LPC_PINT->RISE = mask;
        LPC_PINT->FALL = mask;
//            LPC_GPIO_PIN_INT->IST = mask;

        if (pConfig->pinInterrupt.enable) {
            switch (pConfig->pinInterrupt.mode) {
            case GPIO_INT_FALLING_EDGE:
                LPC_PINT->ISEL &= ~mask;                /* Edge sensitive */
                LPC_PINT->SIENF = mask;                 /* Falling edge */
                break;
            case GPIO_INT_RISING_EDGE:
                LPC_PINT->ISEL &= ~mask;                /* Edge sensitive */
                LPC_PINT->SIENR = mask;                 /* Rising edge */
                break;
            case GPIO_INT_BOTH_EDGES:
                LPC_PINT->ISEL &= ~mask;                /* Edge sensitive */
                LPC_PINT->SIENR = mask;                 /* Rising edge */
                LPC_PINT->SIENF = mask;                 /* Falling edge */
                break;
            case GPIO_INT_LOW_LEVEL:
            case GPIO_INT_LOW_LEVEL_ONCE:
                LPC_PINT->ISEL |= mask;                 /* Level sensitive */
                LPC_PINT->CIENF = mask;                 /* Low level */
                LPC_PINT->SIENR = mask;                 /* Enable(!) */
                break;
            case GPIO_INT_HIGH_LEVEL:
            case GPIO_INT_HIGH_LEVEL_ONCE:
                LPC_PINT->ISEL |= mask;                 /* Level sensitive */
                LPC_PINT->SIENF = mask;                 /* High level */
                LPC_PINT->SIENR = mask;                 /* Enable(!) */
                break;
            }

            gpioContext.pinCallbacks[intNum] = pConfig->pinInterrupt.callback;
        }
        break;

    case GPIO_OPCODE_CONFIGURE_GROUP_INTERRUPT:
        intNum = pConfig->groupInterrupt.interruptLine;
        if (intNum < __NUM_GPIO_GROUP_INTERRUPT_LINES__) {
            CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_GINT);
            port = pConfig->groupInterrupt.pin / 32;            /* Port number */
            mask = 1u << (pConfig->groupInterrupt.pin % 32);    /* Bit mask */
            pGroupInt = pConfig->groupInterrupt.interruptLine == GPIO_GROUP_INT_0 ? LPC_GINT0 : LPC_GINT1;

            pGroupInt->PORT_ENA[port] &= ~mask;             /* Always mask interrupts during configuration */
            pGroupInt->CTRL |= GINTx_CTRL_TRIG_Msk;         /* Level-triggered */

            if (pConfig->groupInterrupt.enable) {
                switch (pConfig->groupInterrupt.mode) {
                case GPIO_INT_FALLING_EDGE:
                    if (GPIO_readBit(pConfig->groupInterrupt.pin)) {
                        pGroupInt->PORT_POL[port] &= ~mask;
                    }
                    else {
                        pGroupInt->PORT_POL[port] |= mask;
                    }
                    result = LPCLIB_SUCCESS;
                    break;
                case GPIO_INT_RISING_EDGE:
                    if (GPIO_readBit(pConfig->groupInterrupt.pin)) {
                        pGroupInt->PORT_POL[port] |= mask;
                    }
                    else {
                        pGroupInt->PORT_POL[port] &= ~mask;
                    }
                    result = LPCLIB_SUCCESS;
                    break;
                case GPIO_INT_LOW_LEVEL:
                    pGroupInt->PORT_POL[port] &= ~mask;
                    result = LPCLIB_SUCCESS;
                    break;
                case GPIO_INT_HIGH_LEVEL:
                    pGroupInt->PORT_POL[port] |= mask;
                    result = LPCLIB_SUCCESS;
                    break;
                default:
                    break;
                }

                gpioContext.groupCallbacks[intNum] = pConfig->groupInterrupt.callback;

                pGroupInt->PORT_ENA[port] |= mask;          /* Add the pin to the selected group. */
            }

            //TODO disable

            if ((pGroupInt->PORT_ENA[0] | pGroupInt->PORT_ENA[1]) == 0) {
                CLKPWR_disableClock(CLKPWR_CLOCKSWITCH_GINT);
            }
        }
        break;
#endif

    default:
        break;
    }

    return result;
}



#if LPCLIB_GPIO_INTERRUPTS

static void PIN_INT_commonIRQHandler (int channel)
{
    LPCLIB_Event event;
    GPIO_InterruptMode mode;

    event.id = LPCLIB_EVENTID_GPIO;
    event.opcode = GPIO_EVENT_PIN_INTERRUPT;
    event.block = 0;
    event.channel = channel;
    event.parameter =                                   /* Return active level for a level-triggered interrupt */
        (void *)((LPC_PINT->IENF & (1 << channel)) ? GPIO_INT_HIGH_LEVEL : GPIO_INT_LOW_LEVEL);

    if (gpioContext.pinCallbacks[channel]) {
        gpioContext.pinCallbacks[channel](event);
    }

    /* Acknowledge interrupt. */
    if (LPC_PINT->ISEL & (1u << channel)) {
        /* Level interrupt */

        /* If the user selected a one-shot level interrupt, we must disable it now.
         * The application will re-enable it later after it has acknowleged the requesting source.
         * This is used if the request cannot be acknowleged within the callback.
         */
        mode = gpioContext.pinInterruptMode[channel];
        switch (mode) {
        case GPIO_INT_LOW_LEVEL_ONCE:
        case GPIO_INT_HIGH_LEVEL_ONCE:
            LPC_PINT->CIENR = (1u << channel);          /* Disable further level interrupts */
            LPC_PINT->IST = (1u << channel);            /* Clear request */
            gpioContext.pinCallbacks[channel] = NULL;   /* Forget callback */
            break;
        case GPIO_INT_FALLING_EDGE:
            LPC_PINT->FALL = (1u << channel);           /* Acknowledge falling edge */
            LPC_PINT->IST = (1u << channel);            /* Clear request */
            break;
        case GPIO_INT_RISING_EDGE:
            LPC_PINT->RISE = (1u << channel);           /* Acknowledge rising edge */
            LPC_PINT->IST = (1u << channel);            /* Clear request */
            break;
        case GPIO_INT_LOW_LEVEL:
        case GPIO_INT_HIGH_LEVEL:
            /* Nothing to do! Callback must have cleared the request, or... */
            if (!gpioContext.pinCallbacks[channel]) {
                /* Well, there is no callback here */
                LPC_PINT->CIENR = (1u << channel);      /* Disable further level interrupts */
            }
            break;
        case GPIO_INT_BOTH_EDGES:
            //TODO
            break;
        }
    }
    else {
        /* Edge-sensitive interrupt. Just ack to reset the request. */
        LPC_PINT->IST = (1u << channel);
    }
}


void PIN_INT0_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(0);
}

void PIN_INT1_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(1);
}

void PIN_INT2_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(2);
}

void PIN_INT3_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(3);
}

void PIN_INT4_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(4);
}

void PIN_INT5_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(5);
}

void PIN_INT6_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(6);
}

void PIN_INT7_IRQHandler (void)
{
    PIN_INT_commonIRQHandler(7);
}

void GINT0_IRQHandler (void)
{
    LPCLIB_Event event;

//TODO this is a hard-coded test...
    if (!(LPC_GINT0->PORT_POL[0] & 0x10)) {
        event.id = LPCLIB_EVENTID_GPIO;
        event.opcode = GPIO_EVENT_GROUP_INTERRUPT;
        event.block = 0;
        event.channel = 4;
        event.parameter = 0;  //TODO

        if (gpioContext.groupCallbacks[0]) {
            gpioContext.groupCallbacks[0](event);
        }
    }

    LPC_GINT0->PORT_POL[0] ^= 0x10;
 //   LPC_GINT0->CTRL |= GINTx_CTRL_INT_Msk;      /* Clear interrupt. */ //TODO bit-banding on M4
        /* NOTE: Not necessary for level-triggered interrupt, as switching polarity to
         * an inactive input level also clears the interrupt! If not, we want a new interrupt anyway.
         * TODO: For edge-triggered interrupt double-check sequence of operation...
         */
}

void GINT1_IRQHandler (void)
{
    LPC_GINT1->CTRL |= GINTx_CTRL_INT_Msk;
}
#endif

/** @} */

