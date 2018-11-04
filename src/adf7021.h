
#ifndef __ADF7021_H
#define __ADF7021_H

#include "lpclib.h"


/** \defgroup ADF7021_Public_Types ADF7021 Types, enums, macros
 *  @{
 */


/** Register definitions */
typedef enum ADF7021_Register {
    ADF7021_REGISTER_0 = 0,
    ADF7021_REGISTER_1 = 1,
    ADF7021_REGISTER_2 = 2,
    ADF7021_REGISTER_3 = 3,
    ADF7021_REGISTER_4 = 4,
    ADF7021_REGISTER_5 = 5,
    ADF7021_REGISTER_6 = 6,
    ADF7021_REGISTER_7 = 7,
    ADF7021_REGISTER_8 = 8,
    ADF7021_REGISTER_9 = 9,
    ADF7021_REGISTER_10 = 10,
    ADF7021_REGISTER_11 = 11,
    ADF7021_REGISTER_12 = 12,
    ADF7021_REGISTER_13 = 13,
    ADF7021_REGISTER_14 = 14,
    ADF7021_REGISTER_15 = 15,

    ADF7021_REGISTER_N = ADF7021_REGISTER_0,
    ADF7021_REGISTER_VCO_OSCILLATOR = ADF7021_REGISTER_1,
    ADF7021_REGISTER_TRANSMIT_MODULATION = ADF7021_REGISTER_2,
    ADF7021_REGISTER_TRANSMIT_RECEIVE_CLOCK = ADF7021_REGISTER_3,
    ADF7021_REGISTER_DEMODULATOR_SETUP = ADF7021_REGISTER_4,
    ADF7021_REGISTER_IF_FILTER_SETUP = ADF7021_REGISTER_5,
    ADF7021_REGISTER_READBACK_SETUP = ADF7021_REGISTER_7,
} ADF7021_Register;


/** Data interface modes */
typedef enum ADF7021_InterfaceMode {
    /* [20:17] R14.TEST_DAC_GAIN
     * [16]    R14.TEST_DAC_EN
     * [15:13] R15.CLK_MUX
     * [9:7]   R15.SIGMA-DELTA_TEST_MODES
     * [6:4]   R15.TX_TEST_MODES
     * [3:0]   R15.RX_TEST_MODES
     */
    ADF7021_INTERFACEMODE_FSK = 0
            | (0 << 16)                     /* DAC off */
            | (7 << 13)                     /* CLKOUT = TxRxCLK */ 
            | (0 << 0)                      /* RX normal */
            ,
    ADF7021_INTERFACEMODE_AFSK_GAIN4 = 0
            | (4 << 17)                     /* DAC gain */
            | (1 << 16)                     /* DAC on */
            | (2 << 13)                     /* CLKOUT = CDR CLK */
            | (9 << 0)                      /* RX R14 modes */
            ,
    ADF7021_INTERFACEMODE_AFSK_GAIN6 = 0
            | (6 << 17)                     /* DAC gain */
            | (1 << 16)                     /* DAC on */
            | (2 << 13)                     /* CLKOUT = CDR CLK */
            | (9 << 0)                      /* RX R14 modes */
            ,
} ADF7021_InterfaceMode;


/** Type of demodulator */
typedef enum ADF7021_DemodulatorType {
    ADF7021_DEMODULATORTYPE_LINEAR = 0,
    ADF7021_DEMODULATORTYPE_2FSK_CORR = 1,
    ADF7021_DEMODULATORTYPE_3FSK = 2,
    ADF7021_DEMODULATORTYPE_4FSK = 3,
} ADF7021_DemodulatorType;


/** IF bandwidth */
typedef enum ADF7021_Bandwidth {
    ADF7021_BANDWIDTH_9k5 = 0,
    ADF7021_BANDWIDTH_13k5 = 1,
    ADF7021_BANDWIDTH_18k5 = 2,
} ADF7021_Bandwidth;


/** AGC Mode */
typedef enum ADF7021_AGCMode {
    ADF7021_AGCMODE_AUTO = 0,
    ADF7021_AGCMODE_MANUAL = 1,
    ADF7021_AGCMODE_FREEZE = 2,
} ADF7021_AGCMode;


/** Opcodes to specify the configuration command in a call to \ref ADF7021_ioctl. */
typedef enum ADF7021_Opcode {
    ADF7021_OPCODE_INVALID = 0,             /**< List terminator */
    ADF7021_OPCODE_POWER_ON,                /**< Power on (CE = 1) + basic initialization */
    ADF7021_OPCODE_SET_REFERENCE,           /**< Set the reference frequency */
    ADF7021_OPCODE_SET_INTERFACE_MODE,      /**< Define the type of data interface */
    ADF7021_OPCODE_SET_BANDWIDTH,           /**< Set the RX IF bandwidth */
    ADF7021_OPCODE_SET_AFC,
    ADF7021_OPCODE_SET_DEMODULATOR,
    ADF7021_OPCODE_SET_DEMODULATOR_PARAMS,
    ADF7021_OPCODE_SET_AGC,                 /** Set AGC mode and parameters */
    ADF7021_OPCODE_SET_AGC_CLOCK,           /** Set AGC clock frequency (default: 8 kHz) */
    ADF7021_OPCODE_CONFIGURE,               /** Apply all configuration settings */
} ADF7021_Opcode;

struct ADF7021_ConfigAFC {
    LPCLIB_Switch enable;                   /**< Flag: Enable AFC */
    uint8_t KI;
    uint8_t KP;
    uint8_t maxRange;
};

struct ADF7021_ConfigDemodulatorParams {
    uint16_t deviation;
    uint16_t postDemodBandwidth;
};

struct ADF7021_ConfigAGC {
    ADF7021_AGCMode mode;
    uint8_t lnaGain;
    uint8_t filterGain;
};

/** Descriptor to specify the configuration in a call to \ref ADF7021_ioctl. */
typedef struct ADF7021_Config {
    ADF7021_Opcode opcode;                  /**< Config action opcode */
    union {
        float referenceFrequency;
        ADF7021_InterfaceMode interfaceMode;
        ADF7021_Bandwidth bandwidth;
        struct ADF7021_ConfigAFC afc;
        ADF7021_DemodulatorType demodType;
        struct ADF7021_ConfigDemodulatorParams demodParams;
        struct ADF7021_ConfigAGC agc;
        float agcClockFrequency;
    };
} ADF7021_Config;

/** Config list terminator. */
#define ADF7021_CONFIG_END \
    {.opcode = ADF7021_OPCODE_INVALID}


/** Handle for an ADF7021 device. */
typedef struct ADF7021_Context *ADF7021_Handle;



/** @} ADF7021 Types, enums, macros */


/** \defgroup ADF7021_Public_Functions ADF7021 API Functions
 *  @{
 */


/** Open the device.
 *
 *  Enable access to an ADF7021 rado.
 *
 *  \param[in] bus Handle of SSP bus to which the device is connected.
 *  \param[in] pDeviceSelect Method to handle SSEL demuxing
 *  \param[out] pHandle Device handle
 *  \retval LPCLIB_SUCCESS Success. \ref pHandle contains a valid handle.
 */
LPCLIB_Result ADF7021_open (LPC_SPI_Type *spi, int ssel, GPIO_Pin muxoutPin, ADF7021_Handle *pHandle);


/** Close device.
 *
 *  \param pHandle Device handle
 */
LPCLIB_Result ADF7021_close (ADF7021_Handle *pHandle);


/** Configure the device.
 *
 *  Pass a configuration command to the ADF7021.
 *
 *  \param[in] handle Device handle.
 *  \param[in] pConfig Pointer to a configuration descriptor
 */
LPCLIB_Result ADF7021_ioctl (ADF7021_Handle handle, const ADF7021_Config *pConfig);


/** Write a command to a register
 *
 *  \param[in] handle Device handle.
 *  \param[in] reg Register name
 *  \param[in] command Register content, right aligned
 */
LPCLIB_Result ADF7021_write (ADF7021_Handle handle,
                             ADF7021_Register reg,
                             uint32_t command);


/** Set PLL frequency
 *
 *  \param[in] handle Device handle.
 *  \param[in] frequency PLL frequency in Hz
 */
LPCLIB_Result ADF7021_setPLL (ADF7021_Handle handle, float frequency);


/** Read RSSI
 *
 *  \param[in] handle Device handle.
 *  \param[in] rssi Pointer to RSSI result. Format: absolute dBm (1/10 dB resolution)
 */
LPCLIB_Result ADF7021_readRSSI (ADF7021_Handle handle, float *rssi_dBm);


/** Read frequency offset
 *
 *  \param[in] handle Device handle.
 *  \param[in] offset Pointer to offset
 */
LPCLIB_Result ADF7021_readOffset (ADF7021_Handle handle, int32_t *offset);



/** Perform IF filter calibration
 */
LPCLIB_Result ADF7021_calibrateIF (ADF7021_Handle handle, int mode);

LPCLIB_Result ADF7021_setDemodClockDivider (ADF7021_Handle handle, int divider);

LPCLIB_Result ADF7021_setBitRate (ADF7021_Handle handle, int bitRate);

LPCLIB_Result ADF7021_getDemodClock (ADF7021_Handle handle, float *demodClock);


void ADF7021_handleSpiEvent (void);


/** @} ADF7021 API Functions */

#endif
