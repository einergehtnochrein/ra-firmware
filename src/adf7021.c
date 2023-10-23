
#include <math.h>
#include <string.h>

#include "lpclib.h"

#include "adf7021.h"

/* Readbacks */
typedef enum ADF7021_Readback {
    ADF7021_READBACK_AFC = 0,
    ADF7021_READBACK_RSSI = 4,
    ADF7021_READBACK_BATTERY = 5,
    ADF7021_READBACK_TEMPERATURE = 6,
    ADF7021_READBACK_EXTPIN = 7,
    ADF7021_READBACK_FILTERCAL = 8,
    ADF7021_READBACK_SILICONREV = 12,
} ADF7021_Readback;


/* Readbacks */
typedef enum ADF7021_Muxout {
    ADF7021_MUXOUT_INVALID = -1,

    ADF7021_MUXOUT_REGULATOR_READY = 0,
    ADF7021_MUXOUT_FILTER_CAL_COMPLETE = 1,
    ADF7021_MUXOUT_DIGITAL_LOCK_DETECT = 2,
    ADF7021_MUXOUT_RSSI_READY = 3,
    ADF7021_MUXOUT_TX_RX = 4,
    ADF7021_MUXOUT_LOGIC_ZERO = 5,
    ADF7021_MUXOUT_TRISTATE = 6,
    ADF7021_MUXOUT_LOGIC_ONE = 7,
} ADF7021_Muxout;


/** Local device context. */
typedef struct ADF7021_Context {
    LPC_SPI_Type *spi;
    int ssel;

    GPIO_Pin muxoutPin;                 /* Pin used for reading MUXOUT */
    ADF7021_Muxout muxout;              /* Currently selected MUXOUT function */
    volatile bool muxoutEvent;          /* Indicator of MUXOUT event (signal edge) */

    ADF7021_InterfaceMode interfaceMode;
    ADF7021_Bandwidth bandwidth;
    int demodClockDivider;
    struct ADF7021_ConfigDemodulatorParams demodParams;
    ADF7021_DemodulatorType demodType;
    float referenceFrequency;
    float frequency;
    int bitRate;
    uint32_t ifBandwidthSelect;
    float agcClockFrequency;            /* Default is 8 kHz */

    uint8_t currentFG;                  /* Last known state of filter gain (Reg9 FG1,FG2) */
    uint8_t currentLG;                  /* Last known state of LNA gain (Reg9 LG1,LG2) */

    MRT_Handle mrt;
    volatile bool timeout;
    uint8_t mrtChannel;
    _Bool wideband;                     /* ADF7021 detected (wider IF than ADF7021-N) */
} ADF7021_Context;


static ADF7021_Context _adf7021Context;



/* Write a command or data string */
LPCLIB_Result ADF7021_write (ADF7021_Handle handle,
                             ADF7021_Register reg,
                             uint32_t command)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    volatile uint32_t *txdatReg = &LPC_FIFO->TXDATCTLSPI1;  //TODO...
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    volatile uint32_t *txdatReg = &handle->spi->FIFOWR;
#endif

    *txdatReg = 0
            | (command >> 16)               /* command[31:16] */
            | (((1u << handle->ssel) ^ 0x0F) << 16) /* Activate SLE (=0) */
            | (1u << 22)                    /* Ignore RX */
            | ((16 - 1) << 24)              /* Send 16 bits */
            ;
    *txdatReg = 0
            | (((command & ~0x0F) | reg) & 0xFFFF)  /* command[15:4] + register number */
            | (((1u << handle->ssel) ^ 0x0F) << 16) /* Activate SLE (=0) */
            | (1u << 20)                    /* End of transfer */
            | (1u << 21)                    /* End of frame */
            | (1u << 22)                    /* Ignore RX */
            | ((16 - 1) << 24)              /* Send 16 bits */
            ;
    while (!(handle->spi->STAT & (1u << 8)))        /* Wait until master becomes idle */ //TODO
        ;

    return LPCLIB_SUCCESS;
}



/* Raw 16-bit read from ADF7021 */
static LPCLIB_Result _ADF7021_read (ADF7021_Handle handle, ADF7021_Readback readback, uint32_t *data)
{
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    (void) handle;
    volatile uint32_t *txdatReg = &LPC_FIFO->TXDATCTLSPI1;  //TODO...
    volatile uint16_t *rxdatReg = (volatile uint16_t *)&LPC_FIFO->RXDATSPI1;     //TODO...
    volatile uint32_t *statReg = &LPC_FIFO->STATSPI1;       //TODO...
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    volatile uint32_t *txdatReg = &handle->spi->FIFOWR;
#endif

    /* Configure readback (register 7) */
    *txdatReg = 0
            | (((1u << 8) | (readback << 4) | 7) & 0xFFFF)  /* Enable readback */
            | (((1u << handle->ssel) ^ 0x0F) << 16) /* Activate SLE (=0) */
            | (1u << 20)                    /* End of transfer */
            | (1u << 21)                    /* End of frame */
            | (1u << 22)                    /* Ignore RX */
            | ((9 - 1) << 24)               /* Send 9 bits (Reg 7 is a short register!) */
            ;
    while (!(handle->spi->STAT & (1u << 8)))        /* Wait until master becomes idle */ //TODO
        ;

    *txdatReg = 0
            | (0 << 0)                      /* MOSI=0 */
            | (0x0F << 16)                  /* All SSEL inactive (SLE=1) */
            | ((9 - 1) << 24)               /* Receive 9 bits */
            ;
    *txdatReg = 0
            | (0 << 0)                      /* MOSI=0 */
            | (0x0F << 16)                  /* All SSEL inactive (SLE=1) */
            | (1u << 20)                    /* End of transfer */
            | (1u << 21)                    /* End of frame */
            | ((9 - 1) << 24)               /* Receive 9 bits */
            ;

    uint32_t rxdata;
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
    while (((*statReg >> 16) & 0xFF) < 2)           /* Wait for 2 frames */
        ;
    rxdata = (*rxdatReg & 0x7F) << 9;
    rxdata |= (*rxdatReg & 0x1FF);
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
    while (!(handle->spi->FIFOSTAT & (1u << 6)))    /* Wait for first RX frame */
        ;
    rxdata = (handle->spi->FIFORD & 0x7F) << 9;     /* Only last seven bits of first frame are valid */
    while (!(handle->spi->FIFOSTAT & (1u << 6)))    /* Wait for second RX frame */
        ;
    rxdata |= (handle->spi->FIFORD & 0x1FF);
#endif

    *data = rxdata;

    return LPCLIB_SUCCESS;
}


/* Update register 0 (MUXOUT and PLL divider settings) */
static void _ADF7021_updateRegister0 (ADF7021_Handle handle)
{
    uint32_t regval = 0
            | (handle->muxout << 29)
            | (1u << 28)                    /* UART/SPI mode */
            | (1u << 27)                    /* RX */
            | ((lrintf((handle->frequency * 32768.0f) / handle->referenceFrequency) & 0x007FFFFF) << 4)
            ;
    ADF7021_write(handle, ADF7021_REGISTER_N, regval);
}


/* Set MUXOUT function */
static void _ADF7021_setMuxout (ADF7021_Handle handle, ADF7021_Muxout muxout, bool forceUpdate)
{
    /* Check if MUXOUT needs to be modified */
    if ((handle->muxout != muxout) || forceUpdate) {
        handle->muxout = muxout;

        /* MUXOUT is controlled by register 0 (= REGISTER_N) */
        _ADF7021_updateRegister0(handle);
    }
}


/* Set up clocks */
static void _ADF7021_setClocks (ADF7021_Handle handle)
{
    uint32_t bbos_clk_divide = lrintf(log2f(handle->referenceFrequency / 1.5e6f) - 2.0f);
    uint32_t seq_clk_divide = lrintf(handle->referenceFrequency / 100e3f);
    uint32_t agc_clk_divide = lrintf(handle->referenceFrequency / (seq_clk_divide * handle->agcClockFrequency));
    uint32_t cdr_clk_divide = 1;
    if ((handle->demodClockDivider > 0) && (handle->bitRate > 0) && !(handle->interfaceMode & (1u << 16))) {
        cdr_clk_divide = lrintf(handle->referenceFrequency / (handle->demodClockDivider * 32 * handle->bitRate));
    }
    uint32_t regval = 0
            | (agc_clk_divide << 26)
            | (seq_clk_divide << 18)
            | (cdr_clk_divide << 10)
            | (handle->demodClockDivider << 6)
            | (bbos_clk_divide << 4)
            ;
    ADF7021_write(handle, ADF7021_REGISTER_TRANSMIT_RECEIVE_CLOCK, regval);
}


/* Configure interface (test DAC on/off) */
static void _ADF7021_configureInterface (ADF7021_Handle handle)
{
    uint32_t regval;

    regval = 0
        | ((handle->interfaceMode << 4) & (0xF << 21))
        | (lrintf((4.0f * 65536.0f * handle->demodClockDivider * 100e3f) / handle->referenceFrequency) << 5)
        | ((handle->interfaceMode >> 12) & (1u << 4))
        ;
    ADF7021_write(handle, ADF7021_REGISTER_14, regval);
    regval = 0
        | ((handle->interfaceMode & 0xE3FF) << 4)
        ;
    ADF7021_write(handle, ADF7021_REGISTER_15, regval);
}


/* Configure the demodulator */
static void _ADF7021_configureDemodulator (ADF7021_Handle handle)
{
    uint32_t regval;
    uint32_t K;
    float demodClock;
    const uint8_t R4_987[4] = {0,5,4,1};
    const uint8_t bwNarrow[4] = {0,1,2,2};
    const uint8_t bwWide[4] = {0,0,1,2};

    
    demodClock = handle->referenceFrequency;
    if (handle->demodClockDivider > 0) {
        demodClock /= handle->demodClockDivider;
    }

    if (handle->demodParams.deviation == 0) {
        K = 42;     /* Not a joke! deviation = 2400 Hz (2FSK) */
    }
    else {
        K = lrintf(100e3f / handle->demodParams.deviation);
    }

    /* Bandwidth code for demodulator register */
    handle->ifBandwidthSelect = handle->wideband ? bwWide[handle->bandwidth] : bwNarrow[handle->bandwidth];
    regval = 0
        | (handle->ifBandwidthSelect << 30)
        | (lrintf(ceil((6433.98176f * handle->demodParams.postDemodBandwidth) / demodClock)) << 20)
        | (lrintf((demodClock * K) / 400e3f) << 10)
        | (R4_987[K % 4] << 7)
        | (handle->demodType << 4)
        ;
    ADF7021_write(handle, ADF7021_REGISTER_4, regval);
}


static void _ADF7021_configureAll (ADF7021_Handle handle)
{
    _ADF7021_configureInterface(handle);
    _ADF7021_setClocks(handle);
    _ADF7021_configureDemodulator(handle);
}


void ADF7021_handleSpiEvent (void)
{
}


/* Open access to device */
LPCLIB_Result ADF7021_open (
        LPC_SPI_Type *spi,
        int ssel,
        GPIO_Pin muxoutPin,
        MRT_Handle mrt,
        ADF7021_Handle *pHandle)
{
    ADF7021_Context *handle = &_adf7021Context;

    handle->spi = spi;
    handle->ssel = ssel;
    handle->muxoutPin = muxoutPin;
    handle->muxout = ADF7021_MUXOUT_REGULATOR_READY;
    handle->demodClockDivider = 4;
    handle->agcClockFrequency = 8e3f;
    handle->mrt = mrt;

    spi->CFG = 0
            | (1u << 2)                     /* Master */
            | (0u << 4)                     /* CPHA=0 */
            | (0u << 5)                     /* CPOL=0 */
            ;
    spi->DIV = 2;
    spi->DLY = 0
            | (1 << 0)                      /* PRE_DELAY */
            | (1 << 4)                      /* POST_DELAY */
            | (1 << 8)                      /* FRAME_DELAY */
            | (1 << 12)                     /* TRANSFER_DELAY */
            ;
    spi->CFG |= (1u << 0);                  /* Enable */

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Close device */
LPCLIB_Result ADF7021_close (ADF7021_Handle *pHandle)
{
    if (*pHandle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    (*pHandle)->spi = NULL;

    *pHandle = LPCLIB_INVALID_HANDLE;

    return LPCLIB_SUCCESS;
}



/* Configure the device */
LPCLIB_Result ADF7021_ioctl (ADF7021_Handle handle, const ADF7021_Config *pConfig)
{
    uint32_t regval;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    while (pConfig->opcode != ADF7021_OPCODE_INVALID) {
        switch (pConfig->opcode) {
        case ADF7021_OPCODE_POWER_ON:
            //TODO activate CE

            /* Detect chip type */
            handle->wideband = false;
            if (_ADF7021_read(handle, ADF7021_READBACK_SILICONREV, &regval) == LPCLIB_SUCCESS) {
                uint32_t productCode = (regval >> 4) & 0xFFF;
                switch (productCode) {
                    case 0x210:     /* ADF7021 */
                        handle->wideband = true;
                        break;
                    case 0x211:     /* ADF7021-N */
                        handle->wideband = false;
                        break;
                }
            }

            /* Force sending R0 (configure UART/SPI mode) */
            _ADF7021_setMuxout(handle, ADF7021_MUXOUT_LOGIC_ZERO, true);

            ADF7021_write(handle, ADF7021_REGISTER_1, 0
                            | (1u << 25)        /* External L */
                            | (8u << 19)        /* VCO bias = 2.00 mA */
                            | (1u << 17)        /* VCO on */
                            | (1u << 12)        /* XOSC on */
                            | (0u << 7)         /* CLKOUT off */
                            | (1u << 4)         /* R = 1 */
                            );
            break;

        case ADF7021_OPCODE_SET_REFERENCE:
            handle->referenceFrequency = pConfig->referenceFrequency;
            break;

        case ADF7021_OPCODE_SET_INTERFACE_MODE:
            handle->interfaceMode = pConfig->interfaceMode;
            break;

        case ADF7021_OPCODE_SET_BANDWIDTH:
            handle->bandwidth = pConfig->bandwidth;
            break;

        case ADF7021_OPCODE_SET_AFC:
            regval = 0;
            if (pConfig->afc.enable) {
                regval = 0
                    | (1u << 4)
                    | (lrintf(8.388608e9f / handle->referenceFrequency) << 5)
                    | (pConfig->afc.KI << 17)
                    | (pConfig->afc.KP << 21)
                    | (pConfig->afc.maxRange << 24)
                    ;
            }
            ADF7021_write(handle, ADF7021_REGISTER_10, regval);
            break;

        case ADF7021_OPCODE_SET_DEMODULATOR:
            handle->demodType = pConfig->demodType;
            break;

        case ADF7021_OPCODE_SET_DEMODULATOR_PARAMS:
            handle->demodParams = pConfig->demodParams;
            break;

        case ADF7021_OPCODE_SET_AGC:
            regval = 0
                | (30u << 4)
                | (70u << 11)
                | (pConfig->agc.mode << 18)
                | (pConfig->agc.lnaGain << 20)
                | (pConfig->agc.filterGain << 22)
                | (0 << 24)
                | (0 << 25)
                | (0 << 26)
                | (0 << 28)
                ;
            ADF7021_write(handle, ADF7021_REGISTER_9, regval);
            break;

        case ADF7021_OPCODE_SET_AGC_CLOCK:
            handle->agcClockFrequency = pConfig->agcClockFrequency;
            break;

        case ADF7021_OPCODE_CONFIGURE:
            _ADF7021_configureAll(handle);
            break;

        case ADF7021_OPCODE_SET_SYNCWORD:
            regval = 0
                | (3 << 4)                              /* 24 bits */
                | ((pConfig->sync.maxErrors % 4) << 6)  /* Max number of bit mismatches (0...3) */
                | (pConfig->sync.pattern << 8)          /* Sync pattern (24 bits) */
                ;
            ADF7021_write(handle, ADF7021_REGISTER_11, regval);
            regval = 0x100;
            if (pConfig->sync.enable) {
                regval = 0
                    | ((pConfig->sync.packetLength ? 2 : 1) << 4)   /* Threshold lock duration */
                    | (2 << 6)
                    | (pConfig->sync.packetLength << 8)             /* Payload length in bytes (1...255) */
                    ;
            }
            ADF7021_write(handle, ADF7021_REGISTER_12, regval);
            break;

        case ADF7021_OPCODE_INVALID:
            /* Nothing to do */
            break;
        }

        ++pConfig;                                      /* More config's follow */
    }

    return LPCLIB_SUCCESS;
}


static LPCLIB_Result _ADF7021_accessTimeoutCallback (LPCLIB_Event event)
{
    (void)event;
    ADF7021_Handle handle = &_adf7021Context;

    handle->timeout = true;

    return LPCLIB_SUCCESS;
}


/* Set PLL frequency */
LPCLIB_Result ADF7021_setPLL (ADF7021_Handle handle, float frequency)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LPCLIB_Result result = LPCLIB_SUCCESS;

    handle->timeout = false;
    MRT_oneshot_millisecs(handle->mrt, 50, &handle->mrtChannel, _ADF7021_accessTimeoutCallback);

    handle->frequency = frequency;
    /* NOTE: Selecting the MUXOUT function requires writing to register 0 of the ADF7021.
     * This register also contains the PLL feedback divider, and therefore changing MUXOUT
     * also updates the PLL setting!
     */
    _ADF7021_setMuxout(handle, ADF7021_MUXOUT_DIGITAL_LOCK_DETECT, true);
    while ((GPIO_readBit(handle->muxoutPin) == 0) && !handle->timeout)
        ;
    if (handle->timeout) {
        result = LPCLIB_TIMEOUT;
    }

    MRT_stopChannel(handle->mrt, handle->mrtChannel);

    return result;
}


/* Set demod clock divider */
LPCLIB_Result ADF7021_setDemodClockDivider (ADF7021_Handle handle, int divider)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->demodClockDivider = divider;

    return LPCLIB_SUCCESS;
}


/* Specify bit rate */
LPCLIB_Result ADF7021_setBitRate (ADF7021_Handle handle, int bitRate)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->bitRate = bitRate;
    _ADF7021_setClocks(handle);

    return LPCLIB_SUCCESS;
}


LPCLIB_Result ADF7021_getDemodClock (ADF7021_Handle handle, float *demodClock)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (demodClock == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    float f = handle->referenceFrequency;
    if (handle->demodClockDivider > 0) {
        f /= handle->demodClockDivider;
    }

    *demodClock = f;

    return LPCLIB_SUCCESS;
}



static const int _ADF7021_rssiGainCorrection[16] = {
    86,86,86,86,
    58,38,24,24,
    0, 0, 0, 0,
    0, 0, 0, 0,
};


/* Read RSSI information */
LPCLIB_Result ADF7021_readRSSI (ADF7021_Handle handle, float *rssi_dBm)
{
    uint32_t rawdata = 0;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (rssi_dBm == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    *rssi_dBm = -174.0f;

    handle->timeout = false;
    MRT_oneshot_millisecs(handle->mrt, 2, &handle->mrtChannel, _ADF7021_accessTimeoutCallback);

    _ADF7021_setMuxout(handle, ADF7021_MUXOUT_RSSI_READY, false);
    while ((GPIO_readBit(handle->muxoutPin) == 0) && !handle->timeout)
        ;

    MRT_stopChannel(handle->mrt, handle->mrtChannel);

    if (_ADF7021_read(handle, ADF7021_READBACK_RSSI, &rawdata) == LPCLIB_SUCCESS) {
        *rssi_dBm = -130.0f + (((int)rawdata & 0x7F) + _ADF7021_rssiGainCorrection[(rawdata >> 7) & 0x0F]) * 0.5f;

        /* Remember filter gain and LNA gain */
        handle->currentFG = (rawdata >> 7) & 0x03;
        handle->currentLG = (rawdata >> 9) & 0x03;

        if (!handle->timeout) {
            return LPCLIB_SUCCESS;
        }
    }

    return LPCLIB_ERROR;
}



/* Read frequency offset */
LPCLIB_Result ADF7021_readOffset (ADF7021_Handle handle, int32_t *offset)
{
    uint32_t rawdata = 0;

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (offset == NULL) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }
    *offset = 0;

    if (_ADF7021_read(handle, ADF7021_READBACK_AFC, &rawdata) == LPCLIB_SUCCESS) {
        //TODO must determine DEMOD_CLOCK
//            *offset = ((int64_t)rawdata * 203125ll) / 16384ll - 100000;
        *offset = rawdata;

        return LPCLIB_SUCCESS;
    }

    return LPCLIB_ERROR;
}


static LPCLIB_Result _ADF7021_handleEvent (LPCLIB_Event event)
{
    ADF7021_Context *handle = &_adf7021Context;
    (void) event;

    handle->muxoutEvent = true;

    return LPCLIB_SUCCESS;
}


static GPIO_Config _muxoutEdgeEnable[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = 0,
            .enable = LPCLIB_YES,
            .mode = GPIO_INT_RISING_EDGE,
            .interruptLine = GPIO_PIN_INT_2,
            .callback = _ADF7021_handleEvent, }}},

    GPIO_CONFIG_END
};

static GPIO_Config _muxoutEdgeDisable[] = {
    {.opcode = GPIO_OPCODE_CONFIGURE_PIN_INTERRUPT,
        {.pinInterrupt = {
            .pin = 0,
            .enable = LPCLIB_NO,
            .interruptLine = GPIO_PIN_INT_2, }}},

    GPIO_CONFIG_END
};


/* Perform IF filter calibration */
LPCLIB_Result ADF7021_calibrateIF (ADF7021_Handle handle, int mode)
{
    uint32_t regval;
    const uint32_t calibrationFrequencies[2][4][2] = {
        {
            {78100, 116300},
            {79400, 116300},
            {78100, 119000},
            {78100, 119000},
        },
        {
            {65800, 131500},
            {65800, 131500},
            {65800, 131500},
            {65800, 131500},
        },
    };

    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    _ADF7021_setMuxout(handle, ADF7021_MUXOUT_FILTER_CAL_COMPLETE, false);
    _muxoutEdgeEnable[0].pinInterrupt.pin = handle->muxoutPin;
    GPIO_ioctl(_muxoutEdgeEnable);

    /* Select fine or coarse calibration */
    int wide = handle->wideband ? 1 : 0;
    regval = 0
            | ((mode ? 1 : 0) << 4)         /* Enable fine calibration */
            | (lrintf(handle->referenceFrequency / (2 * calibrationFrequencies[wide][handle->ifBandwidthSelect][0])) << 5)
            | (lrintf(handle->referenceFrequency / (2 * calibrationFrequencies[wide][handle->ifBandwidthSelect][1])) << 13)
            | (80u << 21)
            ;
    ADF7021_write(handle, ADF7021_REGISTER_6, regval);

    /* Start calibration */
    handle->timeout = false;
    MRT_oneshot_millisecs(handle->mrt, 50, &handle->mrtChannel, _ADF7021_accessTimeoutCallback);
    handle->muxoutEvent = false;
    regval = 0
            | (1u << 4)                     /* Do calibration */
            | (lrintf(handle->referenceFrequency / 50e3f) << 5)   /* --> 50 kHz */
            //TODO configure image rejection
            ;
    ADF7021_write(handle, ADF7021_REGISTER_5, regval);

    /* Wait for end of calibration */
    while (!handle->muxoutEvent && !handle->timeout)
        ;

    MRT_stopChannel(handle->mrt, handle->mrtChannel);

    _muxoutEdgeDisable[0].pinInterrupt.pin = handle->muxoutPin;
    GPIO_ioctl(_muxoutEdgeDisable);

    regval &= ~(1u << 4);
    ADF7021_write(handle, ADF7021_REGISTER_5, regval);

    return LPCLIB_SUCCESS;
}


/* Check for chip version */
LPCLIB_Result ADF7021_getWideband (ADF7021_Handle handle, _Bool *isWideband)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    if (isWideband) {
        *isWideband = handle->wideband;
    }

    return LPCLIB_SUCCESS;
}
