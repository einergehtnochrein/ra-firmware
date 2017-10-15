/* Copyright (c) 2012, NXP Semiconductors
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


/** I2C emulation with GPIO. */

#include "i2c-bitbang.h"
#include "lpc54xxx_gpio.h"

#if LPCLIB_I2CEMU

/** Declare I2C_Context (different from I2C hardware driver!).
 *
 *  The I2C driver makes sure that these are not mixed up.
 */
struct I2C_Context {
    /* NOTE:
     * The first element MUST be "I2C_Name bus".
     * This structure definition can be done in multiple C source files
     * (I2C GPIO bit-banging), and "bus" must exist in all of these
     * possible implementations at offset 0.
     */
    I2C_Name bus;                           /**< Bus identifier */

    GPIO_Pin pinSDA;                        /**< GPIO port bit for use as SDA */
    GPIO_Pin pinSCL;                        /**< GPIO port bit for use as SCL */
    uint32_t rate;
    volatile I2C_Job *job;
    volatile const I2C_JobPhase *phase;
} i2cemu[1];


typedef enum {
    I2CEMU_BIT0 = 1,
    I2CEMU_BIT1 = 0,
} I2CEMU_Level;

static void I2CEMU_SDA_WRITE(I2C_Handle handle, I2CEMU_Level val)
{
    GPIO_writeBit(handle->pinSDA, 0);
    GPIO_setDirBit(handle->pinSDA,
                   (val == I2CEMU_BIT0) ? ENABLE : DISABLE);
}

static uint32_t I2CEMU_SDA_READ(I2C_Handle handle)
{
    return GPIO_readBit(handle->pinSDA);
}

static void I2CEMU_SCL_WRITE(I2C_Handle handle, I2CEMU_Level val)
{
    GPIO_writeBit(handle->pinSCL, 0);
    GPIO_setDirBit(handle->pinSCL,
                   (val == I2CEMU_BIT0) ? ENABLE : DISABLE);
}

static uint32_t I2CEMU_SCL_READ(I2C_Handle handle)
{
    return GPIO_readBit(handle->pinSCL);
}

/*---- END OF PIN CONFIGURATION ----------------------------------------*/


/** Constant that determines the I2C bit rate. (Higher value --> slower rate). */
#define I2CEMU_BIT_DELAY        (1)

/** Constant that determines an I2C timeout (Higher value --> longer timeout). */
#define I2CEMU_TIMEOUT          (10)



/** Software delay that determines the I2C bit rate.
 * @param multi Integer multiple of the delay quantum.
 */
static void I2CEMU_delay (uint8_t multi)
{
    volatile uint32_t i;


    while (multi) {
        /* A bit of delay... */
        for (i = 0; i < I2CEMU_BIT_DELAY; i++)
        {
        }

        multi--;
    }
}

/** Send a start condition. */
static void I2CEMU_start (I2C_Handle handle)
{
    I2CEMU_SCL_WRITE(handle, I2CEMU_BIT1);
    I2CEMU_delay(1);
    I2CEMU_SDA_WRITE(handle, I2CEMU_BIT0);
    I2CEMU_delay(1);
}


/** Send a start condition. */
static void I2CEMU_stop (I2C_Handle handle)
{
    I2CEMU_SDA_WRITE(handle, I2CEMU_BIT0);
    I2CEMU_delay(1);
    I2CEMU_SCL_WRITE(handle, I2CEMU_BIT1);
    I2CEMU_delay(1);
    I2CEMU_SDA_WRITE(handle, I2CEMU_BIT1);
    I2CEMU_delay(1);
}

/** Send one byte, and return the answer (ACK/NAK).
 *  @param data Byte to be sent
 *  @return true=ACK, false=NAK
 */
static bool I2CEMU_sendByte (I2C_Handle handle, uint8_t data)
{
    bool result = false;
    uint32_t i;
    uint32_t timeout;


    /* Send eight bits plus the clock to request the acknowledge. */
    for (i = 0; i < 9; i++) {
        I2CEMU_SCL_WRITE(handle, I2CEMU_BIT0);
        I2CEMU_delay(1);
        I2CEMU_SDA_WRITE(handle, (data & 0x80) ? I2CEMU_BIT1 : I2CEMU_BIT0);
        data = (data << 1) | 1; /* (Make sure the ninth bit is a 1!) */
        I2CEMU_delay(1);

        I2CEMU_SCL_WRITE(handle, I2CEMU_BIT1);
        I2CEMU_delay(2);
    }

    /* Read acknowledge bit */
    timeout = I2CEMU_TIMEOUT;
    do {
        /* Possible clock stretching ended? */
        if (I2CEMU_SCL_READ(handle) == 1) {
            /* ACK status is valid now */
            if (I2CEMU_SDA_READ(handle) == 0) {
                result = true;
                break;
            }
        }

        I2CEMU_delay(1);

        if (timeout) {
            timeout--;
        }
    } while (timeout);

    /* SCL=0 ends transfer of this byte. */
    I2CEMU_SCL_WRITE(handle, I2CEMU_BIT0);
    I2CEMU_delay(1);

    return result;
}


/** Receive one byte, and send ACK.
 *  @return Received byte
 */
static uint8_t I2CEMU_receiveByte (I2C_Handle handle, bool send_ack)
{
    uint8_t data = 0;
    uint32_t i;
    uint32_t timeout;

    /* Receive eight bits. */
    for (i = 0; i < 8; i++)
    {
        I2CEMU_SCL_WRITE(handle, I2CEMU_BIT0);
        I2CEMU_delay(1);
        I2CEMU_SCL_WRITE(handle, I2CEMU_BIT1);
        timeout = I2CEMU_TIMEOUT;
        do {
            /* Possible clock stretching ended? */
            if (I2CEMU_SCL_READ(handle) == 1)
            {
                /* ACK status is valid now */
                data = (data << 1) | I2CEMU_SDA_READ(handle);
                break;
            }
            else
            {
                I2CEMU_delay(1);
                if (timeout)
                {
                    timeout--;
                }
            }
        } while (timeout);
    }

    /* Send acknowledge */
    I2CEMU_SCL_WRITE(handle, I2CEMU_BIT0);
    I2CEMU_delay(1);
    I2CEMU_SDA_WRITE(handle, send_ack ? I2CEMU_BIT0 : I2CEMU_BIT1);
    I2CEMU_delay(1);
    I2CEMU_SCL_WRITE(handle, I2CEMU_BIT1);
    I2CEMU_delay(2);
    I2CEMU_SCL_WRITE(handle, I2CEMU_BIT0);
    I2CEMU_SDA_WRITE(handle, I2CEMU_BIT1);
    I2CEMU_delay(1);

    return data;
}


/* Open an I2C emulation. */
LPCLIB_Result I2CEMU_open (I2C_Name bus, I2C_Handle *pHandle)
{
    if (bus == I2CEMU0) {
        i2cemu[0].bus = bus;
        *pHandle = &i2cemu[0];

        return LPCLIB_SUCCESS;
    }

    return LPCLIB_BUSY;
}



/* Close an I2C bus. */
void I2CEMU_close (I2C_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    I2CEMU_SDA_WRITE(*pHandle, I2CEMU_BIT1);
    I2CEMU_SCL_WRITE(*pHandle, I2CEMU_BIT1);

    *pHandle = LPCLIB_INVALID_HANDLE;
}



/* Configure the I2C emulator. */
void I2CEMU_ioctl (I2C_Handle handle, const I2C_Config *pConfig)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return;
    }

    if (handle->bus != I2CEMU0) {
        return;
    }

    while (pConfig->opcode != I2C_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case I2C_OPCODE_SET_CALLBACK:
            /* TODO */
            break;

        case I2C_OPCODE_EMU_DEFINE_PINS:
            i2cemu[0].pinSCL = pConfig->pins.pinSCL;
            i2cemu[0].pinSDA = pConfig->pins.pinSDA;

            I2CEMU_SDA_WRITE(handle, I2CEMU_BIT1);
            I2CEMU_SCL_WRITE(handle, I2CEMU_BIT1);
            I2CEMU_start(handle);
            I2CEMU_stop(handle);
            break;

        case I2C_OPCODE_SET_BITRATE:
            /* Opcodes not needed in bit-banging driver. Listed here to suppress GCC compiler warning. */
            break;

#if LPCLIB_I2C_SLAVE
        case I2C_OPCODE_SET_SLAVE_ADDRESS:
            /* TODO */
            break;
#endif

        case I2C_OPCODE_INVALID:
            /* Dummy to suppress compiler warning */
            break;
        }

        ++pConfig;
    }
}



LPCLIB_Result I2CEMU_submitJob (I2C_Handle handle, I2C_Job *pJob)
{
    uint8_t data;
    int transferred;
    LPCLIB_Result result = LPCLIB_SUCCESS;
    _Bool ack;
    int sendAddress = 1;

    handle->job = pJob; //needed?

    handle->phase = handle->job->firstPhase;
    while (handle->phase) {
        ack = 1;
        if (sendAddress) {
            I2CEMU_start(handle);

            data = handle->job->slaveAddress << 1;              /* SLA+W */
            if (handle->phase->option == I2C_PHASE_RECEIVE) {
                data |= 1;                                      /* SLA+R */
            }

            ack = I2CEMU_sendByte(handle, data);                /* Send SLA+R/SLA+W */
            sendAddress = 0;
        }

        if (!ack) {
            result = LPCLIB_NO_RESPONSE;
            break;
        }

        transferred = 0;                            /* Reset counter */
        while (transferred < handle->phase->length) {
            if (handle->phase->option == I2C_PHASE_SEND) {
                ack = I2CEMU_sendByte(handle, handle->phase->txstart[transferred]);
            }
            if (handle->phase->option == I2C_PHASE_RECEIVE) {
                data = I2CEMU_receiveByte(handle,
                                          ((transferred + 1) < handle->phase->length) ? true : false);
                handle->phase->rxstart[transferred] = data;
                ack = 1;
            }

            ++transferred;

            if (!ack && (transferred < handle->phase->length)) {
                /* NAK before last byte --> error */
                result = LPCLIB_ERROR;
                break;
            }
        }

        if (!ack) {
            break;
        }

        if (handle->phase->next) {
            if (handle->phase->option != handle->phase->next->option) {
                sendAddress = 1;
            }
        }
        handle->phase = handle->phase->next;
    }

    I2CEMU_stop(handle);

    return result;
}

#endif  /* #ifdef LPCLIB_I2CEMU */

