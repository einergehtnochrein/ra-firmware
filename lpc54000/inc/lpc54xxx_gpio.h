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
 *  \brief GPIO driver interface.
 *  This file defines all interface objects needed to use the GPIO driver.
 */

#ifndef __LPC54XXX_GPIO_H__
#define __LPC54XXX_GPIO_H__

/** \defgroup GPIO
 *  \ingroup API
 *  @{
 */


#include "lpc54xxx_libconfig.h"
#include "lpclib_types.h"


/** \defgroup GPIO_Public_Types GPIO Types, enums, macros
 *  @{
 */

typedef enum GPIO_Port32 {
    GPIO_PORT0 = 0,                     /**< GPIO port 0, 32-bit access */
    GPIO_PORT1 = 1,                     /**< GPIO port 1, 32-bit access */
    __NUM_GPIO__,
} GPIO_Port32;


typedef enum GPIO_Port16 {
    GPIO_PORT0_L = (0 << 4) | (0 << 0), /**< GPIO port 0, 16-bit access, lower half-word */
    GPIO_PORT0_H = (1 << 4) | (0 << 0), /**< GPIO port 0, 16-bit access, upper half-word */
    GPIO_PORT1_L = (0 << 4) | (1 << 0), /**< GPIO port 1, 16-bit access, lower half-word */
    GPIO_PORT1_H = (1 << 4) | (1 << 0), /**< GPIO port 1, 16-bit access, upper half-word */
} GPIO_Port16;


typedef enum GPIO_Port8 {
    GPIO_PORT0_0 = (0 << 4) | (0 << 0), /**< GPIO port 0, 8-bit access, byte 0 */
    GPIO_PORT0_1 = (1 << 4) | (0 << 0), /**< GPIO port 0, 8-bit access, byte 1 */
    GPIO_PORT0_2 = (2 << 4) | (0 << 0), /**< GPIO port 0, 8-bit access, byte 2 */
    GPIO_PORT0_3 = (3 << 4) | (0 << 0), /**< GPIO port 0, 8-bit access, byte 3 */
    GPIO_PORT1_0 = (0 << 4) | (1 << 0), /**< GPIO port 1, 8-bit access, byte 0 */
    GPIO_PORT1_1 = (1 << 4) | (1 << 0), /**< GPIO port 1, 8-bit access, byte 1 */
    GPIO_PORT1_2 = (2 << 4) | (1 << 0), /**< GPIO port 1, 8-bit access, byte 2 */
} GPIO_Port8;


typedef enum GPIO_Pin {
    GPIO_0_0  = 0*32 + 0,                   /**< GPIO 0.0 */
    GPIO_0_1  = 0*32 + 1,                   /**< GPIO 0.1 */
    GPIO_0_2  = 0*32 + 2,                   /**< GPIO 0.2 */
    GPIO_0_3  = 0*32 + 3,                   /**< GPIO 0.3 */
    GPIO_0_4  = 0*32 + 4,                   /**< GPIO 0.4 */
    GPIO_0_5  = 0*32 + 5,                   /**< GPIO 0.5 */
    GPIO_0_6  = 0*32 + 6,                   /**< GPIO 0.6 */
    GPIO_0_7  = 0*32 + 7,                   /**< GPIO 0.7 */
    GPIO_0_8  = 0*32 + 8,                   /**< GPIO 0.8 */
    GPIO_0_9  = 0*32 + 9,                   /**< GPIO 0.9 */
    GPIO_0_10 = 0*32 + 10,                  /**< GPIO 0.10 */
    GPIO_0_11 = 0*32 + 11,                  /**< GPIO 0.11 */
    GPIO_0_12 = 0*32 + 12,                  /**< GPIO 0.12 */
    GPIO_0_13 = 0*32 + 13,                  /**< GPIO 0.13 */
    GPIO_0_14 = 0*32 + 14,                  /**< GPIO 0.14 */
    GPIO_0_15 = 0*32 + 15,                  /**< GPIO 0.15 */
    GPIO_0_16 = 0*32 + 16,                  /**< GPIO 0.16 */
    GPIO_0_17 = 0*32 + 17,                  /**< GPIO 0.17 */
    GPIO_0_18 = 0*32 + 18,                  /**< GPIO 0.18 */
    GPIO_0_19 = 0*32 + 19,                  /**< GPIO 0.19 */
    GPIO_0_20 = 0*32 + 20,                  /**< GPIO 0.20 */
    GPIO_0_21 = 0*32 + 21,                  /**< GPIO 0.21 */
    GPIO_0_22 = 0*32 + 22,                  /**< GPIO 0.22 */
    GPIO_0_23 = 0*32 + 23,                  /**< GPIO 0.23 */
    GPIO_0_24 = 0*32 + 24,                  /**< GPIO 0.24 */
    GPIO_0_25 = 0*32 + 25,                  /**< GPIO 0.25 */
    GPIO_0_26 = 0*32 + 26,                  /**< GPIO 0.26 */
    GPIO_0_27 = 0*32 + 27,                  /**< GPIO 0.27 */
    GPIO_0_28 = 0*32 + 28,                  /**< GPIO 0.28 */
    GPIO_0_29 = 0*32 + 29,                  /**< GPIO 0.29 */
    GPIO_0_30 = 0*32 + 30,                  /**< GPIO 0.30 */
    GPIO_0_31 = 0*32 + 31,                  /**< GPIO 0.31 */
    GPIO_1_0  = 1*32 + 0,                   /**< GPIO 1.0 */
    GPIO_1_1  = 1*32 + 1,                   /**< GPIO 1.1 */
    GPIO_1_2  = 1*32 + 2,                   /**< GPIO 1.2 */
    GPIO_1_3  = 1*32 + 3,                   /**< GPIO 1.3 */
    GPIO_1_4  = 1*32 + 4,                   /**< GPIO 1.4 */
    GPIO_1_5  = 1*32 + 5,                   /**< GPIO 1.5 */
    GPIO_1_6  = 1*32 + 6,                   /**< GPIO 1.6 */
    GPIO_1_7  = 1*32 + 7,                   /**< GPIO 1.7 */
    GPIO_1_8  = 1*32 + 8,                   /**< GPIO 1.8 */
    GPIO_1_9  = 1*32 + 9,                   /**< GPIO 1.9 */
    GPIO_1_10 = 1*32 + 10,                  /**< GPIO 1.10 */
    GPIO_1_11 = 1*32 + 11,                  /**< GPIO 1.11 */
    GPIO_1_12 = 1*32 + 12,                  /**< GPIO 1.12 */
    GPIO_1_13 = 1*32 + 13,                  /**< GPIO 1.13 */
    GPIO_1_14 = 1*32 + 14,                  /**< GPIO 1.14 */
    GPIO_1_15 = 1*32 + 15,                  /**< GPIO 1.15 */
    GPIO_1_16 = 1*32 + 16,                  /**< GPIO 1.16 */
    GPIO_1_17 = 1*32 + 17,                  /**< GPIO 1.17 */
} GPIO_Pin;


/** Opcodes to specify the configuration command in a call to \ref GPIO_ioctl. */
typedef enum GPIO_Opcode {
    GPIO_OPCODE_INVALID = 0,                /**< (List terminator) */
#if LPCLIB_GPIO_INTERRUPTS
    GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,    /**< Config action: Pin interrupt configuration */
    GPIO_OPCODE_CONFIGURE_GROUP_INTERRUPT,  /**< Config action: Group interrupt configuration */
#endif
} GPIO_Opcode;


#if LPCLIB_GPIO_INTERRUPTS
/** GPIO interrupt mode (edge vs. level) */
typedef enum GPIO_InterruptMode {
    GPIO_INT_FALLING_EDGE = 0,              /**< GPIO interrupt on falling edge */
    GPIO_INT_RISING_EDGE,                   /**< GPIO interrupt on rising edge */
    GPIO_INT_BOTH_EDGES,                    /**< GPIO interrupt on both edges */
    GPIO_INT_LOW_LEVEL,                     /**< GPIO interrupt on low level */
    GPIO_INT_LOW_LEVEL_ONCE,                /**< GPIO interrupt on low level, no auto-rearm */
    GPIO_INT_HIGH_LEVEL,                    /**< GPIO interrupt on high level */
    GPIO_INT_HIGH_LEVEL_ONCE,               /**< GPIO interrupt on high level, no auto-rearm */
} GPIO_InterruptMode;


/** GPIO pin interrupt line (each has its own vector). */
typedef enum GPIO_PinInterruptLine {
    GPIO_PIN_INT_0 = 0,
    GPIO_PIN_INT_1,
    GPIO_PIN_INT_2,
    GPIO_PIN_INT_3,
    GPIO_PIN_INT_4,
    GPIO_PIN_INT_5,
    GPIO_PIN_INT_6,
    GPIO_PIN_INT_7,
    __NUM_GPIO_PIN_INTERRUPT_LINES__
} GPIO_PinInterruptLine;


/** GPIO group interrupt line (each has its own vector). */
typedef enum GPIO_GroupInterruptLine {
    GPIO_GROUP_INT_0 = 0,
    GPIO_GROUP_INT_1,
    __NUM_GPIO_GROUP_INTERRUPT_LINES__
} GPIO_GroupInterruptLine;


/** Configuration parameters for GPIO pin interrupt. */
struct GPIO_ConfigPinInterrupt {
    GPIO_Pin pin;                           /**< The pin to be configured */
    LPCLIB_Switch enable;                   /**< Flag: Enable interrupt */
    GPIO_InterruptMode mode;                /**< Level (vs edge) sensitive */
    GPIO_PinInterruptLine interruptLine;    /**< Selects one of eight individual interrupts */
    LPCLIB_Callback callback;               /**< Callback handler (or NULL) */
};


/** Configuration parameters for GPIO group interrupt. */
struct GPIO_ConfigGroupInterrupt {
    GPIO_Pin pin;                           /**< The pin to be configured */
    LPCLIB_Switch enable;                   /**< Flag: Enable interrupt */
    GPIO_InterruptMode mode;                /**< Level (vs edge) sensitive */
    GPIO_GroupInterruptLine interruptLine;  /**< Selects one of two group interrupts */
    LPCLIB_Callback callback;               /**< Callback handler (or NULL) */
};
#endif


/** Descriptor to specify the configuration in a call to \ref I2C_Ioctl. */
typedef struct GPIO_Config {
    GPIO_Opcode opcode;                     /**< Config action opcode */

    union {
    #if LPCLIB_GPIO_INTERRUPTS
        struct GPIO_ConfigPinInterrupt pinInterrupt;        /**< Pin interrupt */
        struct GPIO_ConfigGroupInterrupt groupInterrupt;    /**< Group interrupt */
    #endif
        uint8_t __dummy__;
    };
} GPIO_Config;


/** Config list terminator. */
#define GPIO_CONFIG_END \
    {.opcode = GPIO_OPCODE_INVALID}


typedef enum GPIO_CallbackEvent {
    GPIO_EVENT_PIN_INTERRUPT,               /**< Interrupt from individual pin */
    GPIO_EVENT_GROUP_INTERRUPT,             /**< Interrupt from pin group */
} GPIO_CallbackEvent;


/** @} GPIO Types, enums, macros */

/** \defgroup GPIO_Public_Functions GPIO API Functions
 *  @{
 */


/** Prepare the use of the GPIO block.
 */
void GPIO_open (void);


/** Configure the GPIO block.
 *
 *  Pass a configuration command to the GPIO block.
 *
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
LPCLIB_Result GPIO_ioctl (const GPIO_Config *pConfig);


/** Set GPIO pin direction for whole 32-bit port.
 *
 *  \param[in] port Port number
 *  \param[in] value 32-bit value where each bit corresponds to the pin in the
 *                   same bit position of the port. (0=input, 1=output).
 *  \param[in] mask 32-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_setDir32 (GPIO_Port32 port, uint32_t value, uint32_t mask);

/** Set GPIO pin direction for whole 32-bit port.
 *
 *  \param[in] port Pin selector
 *  \param[in] outputEnable Boolean value to enable the output.
 */
void GPIO_setDirBit (GPIO_Pin pin, LPCLIB_Switch outputEnable);

/** Write value to a 32-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value to be written to port pins.
 */
static void GPIO_write32 (GPIO_Port32 port, uint32_t value);

/** Write value to a 16-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value to be written to port pins.
 */
void GPIO_write16 (GPIO_Port16 port, uint16_t value);

/** Write value to an 8-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value to be written to port pins.
 */
void GPIO_write8  (GPIO_Port8  port, uint8_t  value);

/** Write value (0/1) to a GPIO port pin.
 *
 *  \param[in] port Pin selector

 *  \param[in] value Binary value to be written to port pin.
 */
static void GPIO_writeBit(GPIO_Pin pin, uint8_t value);

/** Write value to a 32-bit GPIO port through a mask.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value to be written to port pins.
 *  \param[in] mask 32-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_write32WithMask (GPIO_Port32 port, uint32_t value, uint32_t mask);

/** Write value to a 16-bit GPIO port through a mask.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value to be written to port pins.
 *  \param[in] mask 16-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_write16WithMask (GPIO_Port16 port, uint16_t value, uint16_t mask);

/** Write value to an 8-bit GPIO port through a mask.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value to be written to port pins.
 *  \param[in] mask 8-bit value with a mask for parameter \a value. Only those
 *                  bits of \a value that have a 1 in the corresponding position
 *                  of \a mask take effect.
 */
void GPIO_write8WithMask (GPIO_Port8 port, uint8_t value, uint8_t mask);

/** Return current state of the pins of a 32-bit GPIO port.
 *
 *  \param port Port selector
 *  \return 32-bit value with state of all port pins.
 */
static uint32_t GPIO_read32 (GPIO_Port32 port);

/** Return current state of the pins of a 16-bit GPIO port.
 *
 *  \param port Port selector
 *  \return 16-bit value with state of all port pins.
 */
static uint16_t GPIO_read16 (GPIO_Port16 port);

/** Return current state of the pins of an 8-bit GPIO port.
 *
 *  \param port Port selector
 *  \return 8-bit value with state of all port pins.
 */
static uint8_t GPIO_read8 (GPIO_Port8 port);

/** Return current state of a single GPIO port pin.
 *
 *  \param port Pin selector
 *  \return (32-bit) value with state (0/1) of the port pin.
 */
static uint32_t GPIO_readBit (GPIO_Pin pin);

/** Set pins of a 32-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value where a 1 in bit position n sets pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_set32 (GPIO_Port32 port, uint32_t value);

/** Set pins of a 16-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value where a 1 in bit position n sets pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_set16 (GPIO_Port16 port, uint16_t value);

/** Set pins of an 8-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value where a 1 in bit position n sets pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_set8 (GPIO_Port8 port, uint8_t value);

/** Clear pins of a 32-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 32-bit value where a 1 in bit position n clears pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_clr32 (GPIO_Port32 port, uint32_t value);

/** Clear pins of a 16-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 16-bit value where a 1 in bit position n clears pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_clr16 (GPIO_Port16 port, uint16_t value);

/** Clear pins of an 8-bit GPIO port.
 *
 *  \param[in] port Port selector
 *  \param[in] value 8-bit value where a 1 in bit position n clears pin n
 *                   of the port. A zero has no effect.
 */
static void GPIO_clr8 (GPIO_Port8 port, uint8_t value);


/** Get port number from GPIO pin type.
 */
static int GPIO_getPortNumberFromPin (GPIO_Pin pin);

/** Get bit number from GPIO pin type.
 */
static int GPIO_getBitNumberFromPin (GPIO_Pin pin);


__FORCEINLINE(void GPIO_write32(GPIO_Port32 port, uint32_t value))
{
    LPC_GPIO->PIN[port] = value;
}

__FORCEINLINE(void GPIO_writeBit(GPIO_Pin pin, uint8_t value))
{
    LPC_GPIO->B[pin] = value;
}



__FORCEINLINE(uint32_t GPIO_read32(GPIO_Port32 port))
{
    return LPC_GPIO->PIN[port];
}

__FORCEINLINE(uint16_t GPIO_read16(GPIO_Port16 port))
{
    if ((port >> 4) == 0) {
        return LPC_GPIO->PIN[port] & 0x0000FFFF;
    }
    else if ((port >> 4) == 1) {
        return LPC_GPIO->PIN[port] >> 16;
    }

    return 0;
}

__FORCEINLINE(uint8_t GPIO_read8(GPIO_Port8 port))
{
    if ((port >> 4) == 0) {
        return (LPC_GPIO->PIN[port] >> 0) & 0xFF;
    }
    else if ((port >> 4) == 1) {
        return (LPC_GPIO->PIN[port] >> 8) & 0xFF;
    }

    return 0;
}

__FORCEINLINE(uint32_t GPIO_readBit(GPIO_Pin pin))
{
    return LPC_GPIO->B[pin];
}



__FORCEINLINE(void GPIO_set32 (GPIO_Port32 port, uint32_t value))
{
    LPC_GPIO->SET[port] = value;
}

__FORCEINLINE(void GPIO_set16 (GPIO_Port16 port, uint16_t value))
{
    if ((port >> 4) == 0) {
        LPC_GPIO->SET[port & 0x0F] = value;
    }
    else if ((port >> 4) == 1) {
        LPC_GPIO->SET[port & 0x0F] = value << 16;
    }
}

__FORCEINLINE(void GPIO_set8 (GPIO_Port8 port, uint8_t value))
{
    if ((port >> 4) == 0) {
        LPC_GPIO->SET[port & 0x0F] = value;
    }
    else if ((port >> 4) == 1) {
        LPC_GPIO->SET[port & 0x0F] = value << 8;
    }
}

__FORCEINLINE(void GPIO_clr32 (GPIO_Port32 port, uint32_t value))
{
    LPC_GPIO->CLR[port] = value;
}

__FORCEINLINE(void GPIO_clr16 (GPIO_Port16 port, uint16_t value))
{
    if ((port >> 4) == 0) {
        LPC_GPIO->CLR[port & 0x0F] = value;
    }
    else if ((port >> 4) == 1) {
        LPC_GPIO->CLR[port & 0x0F] = value << 16;
    }
}

__FORCEINLINE(void GPIO_clr8 (GPIO_Port8 port, uint8_t value))
{
    if ((port >> 4) == 0) {
        LPC_GPIO->CLR[port & 0x0F] = value;
    }
    else if ((port >> 4) == 1) {
        LPC_GPIO->CLR[port & 0x0F] = value << 8;
    }
}

__FORCEINLINE(int GPIO_getPortNumberFromPin (GPIO_Pin pin))
{
    return pin / 32;
}

__FORCEINLINE(int GPIO_getBitNumberFromPin (GPIO_Pin pin))
{
    return pin % 32;
}

/** @} GPIO API Functions */

/** @} GPIO */

#endif /* #ifndef __LPC54XXX_GPIO_H__ */

