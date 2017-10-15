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

/** \file
 *  \brief I2C emulation.
 *
 *  \author NXP Semiconductors
 */

#ifndef __LPC54XXX_I2CEMU_H__
#define __LPC54XXX_I2CEMU_H__

#include "lpc54xxx_libconfig.h"

#if LPCLIB_I2CEMU

#include "lpclib_types.h"
#include "lpc54xxx_i2c.h"


/** Open an I2C emulation.
 *
 *  \param[in] bus Indicator that selects an I2C interface block
 *  \param[out] pHandle Handle to be used in future API calls to the I2C module.
 *  \retval LPCLIB_SUCCESS Success. \ref handle contains a valid handle.
 *  \retval LPCLIB_BUSY Failure (interface already open). \ref pHandle does not
 *  contain a valid handle in this case.
 */
LPCLIB_Result I2CEMU_open (I2C_Name bus, I2C_Handle *pHandle);


/** Close an I2C bus.
 *
 *  \param[in] pHandle I2C bus handle.
 */
void I2CEMU_close (I2C_Handle *pHandle);


/** Configure the I2C emulator.
 *
 *  \param[in] handle I2C bus handle.
 *  \param[in] pConfig Pointer to a configuration descriptor of type
 *             \ref I2C_Config.
 */
void I2CEMU_ioctl (I2C_Handle handle, const I2C_Config *pConfig);


/** Submit a new I2C job to the driver.
 *
 *  \param[in] handle I2C bus handle.
 *  \param[in] pJob Pointer to a job descriptor that describes the details
 *      of the transaction.
 *
 *  \retval LPCLIB_SUCCESS Success. Job will be executed, and the callback function
 *      called once the transaction ends.
 *  \retval LPCLIB_BUSY Failure. Job NOT submitted.
 */
LPCLIB_Result I2CEMU_submitJob (I2C_Handle handle, I2C_Job *pJob);


#endif /* #if LPCLIB_I2CEMU */

#endif /* #ifndef __LPC54XXX_I2CEMU_H__ */
