
#ifndef __LPC541xx_H__
#define __LPC541xx_H__

// ------------------------------------------------------------------------------------------------
// -----                              Interrupt Numbers                                       -----
// ------------------------------------------------------------------------------------------------

typedef enum IRQn {
    /******  Cortex-M4 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn         = -14,
    MemoryManagement_IRQn       = -12,
    BusFault_IRQn               = -11,
    UsageFault_IRQn             = -10,
    SVCall_IRQn                 = -5,
    DebugMonitor_IRQn           = -4,
    PendSV_IRQn                 = -2,
    SysTick_IRQn                = -1,

    /******  LPC5411x Specific Interrupt Numbers *******************************************************/
    WDT_BOD_IRQn                = 0,
    DMA_IRQn                    = 1,
    GINT0_IRQn                  = 2,
    GINT1_IRQn                  = 3,
    PIN_INT0_IRQn               = 4,
    PIN_INT1_IRQn               = 5,
    PIN_INT2_IRQn               = 6,
    PIN_INT3_IRQn               = 7,
    UTICK_IRQn                  = 8,
    MRT_IRQn                    = 9,
    TIMER0_IRQn                 = 10,
    TIMER1_IRQn                 = 11,
    SCT0_IRQn                   = 12,
    TIMER3_IRQn                 = 13,
    FLEXCOMM0_IRQn              = 14,
    FLEXCOMM1_IRQn              = 15,
    FLEXCOMM2_IRQn              = 16,
    FLEXCOMM3_IRQn              = 17,
    FLEXCOMM4_IRQn              = 18,
    FLEXCOMM5_IRQn              = 19,
    FLEXCOMM6_IRQn              = 20,
    FLEXCOMM7_IRQn              = 21,
        I2C0_IRQn                   = FLEXCOMM0_IRQn,
        I2C1_IRQn                   = FLEXCOMM1_IRQn,
        I2C2_IRQn                   = FLEXCOMM2_IRQn,
        I2C3_IRQn                   = FLEXCOMM3_IRQn,
        I2C4_IRQn                   = FLEXCOMM4_IRQn,
        I2C5_IRQn                   = FLEXCOMM5_IRQn,
        I2C6_IRQn                   = FLEXCOMM6_IRQn,
        I2C7_IRQn                   = FLEXCOMM7_IRQn,
        SPI0_IRQn                   = FLEXCOMM0_IRQn,
        SPI1_IRQn                   = FLEXCOMM1_IRQn,
        SPI2_IRQn                   = FLEXCOMM2_IRQn,
        SPI3_IRQn                   = FLEXCOMM3_IRQn,
        SPI4_IRQn                   = FLEXCOMM4_IRQn,
        SPI5_IRQn                   = FLEXCOMM5_IRQn,
        SPI6_IRQn                   = FLEXCOMM6_IRQn,
        SPI7_IRQn                   = FLEXCOMM7_IRQn,
        UART0_IRQn                  = FLEXCOMM0_IRQn,
        UART1_IRQn                  = FLEXCOMM1_IRQn,
        UART2_IRQn                  = FLEXCOMM2_IRQn,
        UART3_IRQn                  = FLEXCOMM3_IRQn,
        UART4_IRQn                  = FLEXCOMM4_IRQn,
        UART5_IRQn                  = FLEXCOMM5_IRQn,
        UART6_IRQn                  = FLEXCOMM6_IRQn,
        UART7_IRQn                  = FLEXCOMM7_IRQn,
    ADC0_SEQA_IRQn              = 22,
    ADC0_SEQB_IRQn              = 23,
    ADC0_THCMP_IRQn             = 24,
    DMIC_IRQn                   = 25,
    HWVAD_IRQn                  = 26,
    USB_NEEDCLK_IRQn            = 27,
    USB_IRQn                    = 28,
    RTC_IRQn                    = 29,
    EZH_IRQn                    = 30,
    MAILBOX_IRQn                = 31,

    PIN_INT4_IRQn               = 32,
    PIN_INT5_IRQn               = 33,
    PIN_INT6_IRQn               = 34,
    PIN_INT7_IRQn               = 35,
    TIMER2_IRQn                 = 36,
    TIMER4_IRQn                 = 37,

    IRQ38_IRQn                  = 38,
    IRQ39_IRQn                  = 39,
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M3 Processor and Core Peripherals */
#define __MPU_PRESENT             1         /*!< MPU present or not                               */
#define __FPU_PRESENT             1
#define __NVIC_PRIO_BITS          3         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */


#include "core_cm4.h"                       /* Cortex-M4 processor and core peripherals           */
//#include "system_LPC54xxx.h"                   /* System Header                                      */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


// ------------------------------------------------------------------------------------------------
// -----                                          ADC                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t CTRL;
    __I  uint32_t RESERVED004[(0x008-0x004)/4];
    __IO uint32_t SEQA_CTRL;
    __IO uint32_t SEQB_CTRL;
    __IO uint32_t SEQA_GDAT;
    __IO uint32_t SEQB_GDAT;
    __I  uint32_t RESERVED018[(0x020-0x018)/4];
    union {
        __I  uint32_t DAT[12];
        struct {
            __I  uint32_t DAT0;
            __I  uint32_t DAT1;
            __I  uint32_t DAT2;
            __I  uint32_t DAT3;
            __I  uint32_t DAT4;
            __I  uint32_t DAT5;
            __I  uint32_t DAT6;
            __I  uint32_t DAT7;
            __I  uint32_t DAT8;
            __I  uint32_t DAT9;
            __I  uint32_t DAT10;
            __I  uint32_t DAT11;
        };
    };
    __IO uint32_t THR0_LOW;
    __IO uint32_t THR1_LOW;
    __IO uint32_t THR0_HIGH;
    __IO uint32_t THR1_HIGH;
    __IO uint32_t CHAN_THRSEL;
    __IO uint32_t INTEN;
    __IO uint32_t FLAGS;
    __IO uint32_t STARTUP;
    __IO uint32_t CALIB;
} LPC_ADC_Type;


// ------------------------------------------------------------------------------------------------
// -----                                         CRC                                          -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t MODE;
    __IO uint32_t SEED;
    union {
        __I  uint32_t SUM;
        __O  uint32_t WR_DATA;
        __O  uint32_t WR_DATA32;
        __O  uint16_t WR_DATA16;
        __O  uint8_t  WR_DATA8;
    };
} LPC_CRC_Type;



// ------------------------------------------------------------------------------------------------
// -----                                        DMIC                                          -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t OSR0;
    __IO uint32_t DIVHFCLK0;
    __IO uint32_t PREAC2FSCOEF0;
    __IO uint32_t PREAC4FSCOEF0;
    __IO uint32_t GAINSHIFT0;
    __I  uint32_t RESERVED014[(0x080-0x014)/4];
    __IO uint32_t FIFOCTRL0;
    __IO uint32_t FIFOSTAT0;
    __IO uint32_t FIFODATA0;
    __IO uint32_t PDMSRCCFG0;
    __IO uint32_t DCCTRL0;
    __I  uint32_t RESERVED094[(0x100-0x094)/4];

    __IO uint32_t OSR1;
    __IO uint32_t DIVHFCLK1;
    __IO uint32_t PREAC2FSCOEF1;
    __IO uint32_t PREAC4FSCOEF1;
    __IO uint32_t GAINSHIFT1;
    __I  uint32_t RESERVED114[(0x180-0x114)/4];
    __IO uint32_t FIFOCTRL1;
    __IO uint32_t FIFOSTAT1;
    __IO uint32_t FIFODATA1;
    __IO uint32_t PDMSRCCFG1;
    __IO uint32_t DCCTRL1;
    __I  uint32_t RESERVED194[(0xF00-0x194)/4];

    __IO uint32_t CHANEN;
    __I  uint32_t RESERVEDF04[(0xF0C-0xF04)/4];
    __IO uint32_t IOCFG;
    __IO uint32_t USE2FS;
    __I  uint32_t RESERVEDF14[(0xF80-0xF14)/4];

    __IO uint32_t HWVADGAIN;
    __IO uint32_t HWVADHPFS;
    __IO uint32_t HWVADST10;
    __IO uint32_t HWVADRSTT;
    __IO uint32_t HWVADTHGN;
    __IO uint32_t HWVADTHGS;
    __I  uint32_t HWVADLOWZ;
} LPC_DMIC_Type;


// ------------------------------------------------------------------------------------------------
// -----                                       FLEXCOMM                                       -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __I  uint32_t RESERVED000[(0xFF8-0x000)/4];
    __IO uint32_t PSELID;
    __I  uint32_t PID;
} LPC_FLEXCOMM_Type;



// ------------------------------------------------------------------------------------------------
// -----                                         GPIO                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    union {
        __IO uint8_t B[50];
        struct {
            __IO uint8_t B0;
            __IO uint8_t B1;
            __IO uint8_t B2;
            __IO uint8_t B3;
            __IO uint8_t B4;
            __IO uint8_t B5;
            __IO uint8_t B6;
            __IO uint8_t B7;
            __IO uint8_t B8;
            __IO uint8_t B9;
            __IO uint8_t B10;
            __IO uint8_t B11;
            __IO uint8_t B12;
            __IO uint8_t B13;
            __IO uint8_t B14;
            __IO uint8_t B15;
            __IO uint8_t B16;
            __IO uint8_t B17;
            __IO uint8_t B18;
            __IO uint8_t B19;
            __IO uint8_t B20;
            __IO uint8_t B21;
            __IO uint8_t B22;
            __IO uint8_t B23;
            __IO uint8_t B24;
            __IO uint8_t B25;
            __IO uint8_t B26;
            __IO uint8_t B27;
            __IO uint8_t B28;
            __IO uint8_t B29;
            __IO uint8_t B30;
            __IO uint8_t B31;
            __IO uint8_t B32;
            __IO uint8_t B33;
            __IO uint8_t B34;
            __IO uint8_t B35;
            __IO uint8_t B36;
            __IO uint8_t B37;
            __IO uint8_t B38;
            __IO uint8_t B39;
            __IO uint8_t B40;
            __IO uint8_t B41;
            __IO uint8_t B42;
            __IO uint8_t B43;
            __IO uint8_t B44;
            __IO uint8_t B45;
            __IO uint8_t B46;
            __IO uint8_t B47;
            __IO uint8_t B48;
            __IO uint8_t B49;
        };
    };
         uint8_t _RESERVED0_[4096-50];
    __IO uint32_t W[50];
         uint32_t _RESERVED1_[1024-50];
    union {
        __IO uint32_t DIR[2];
        struct {
            __IO uint32_t DIR0;
            __IO uint32_t DIR1;
        };
    };
         uint32_t _RESERVED2_[32-2];
    union {
        __IO uint32_t MASK[2];
        struct {
            __IO uint32_t MASK0;
            __IO uint32_t MASK1;
        };
    };
         uint32_t RESERVED3[32-2];
    union {
        __IO uint32_t PIN[2];
        struct {
            __IO uint32_t PIN0;
            __IO uint32_t PIN1;
        };
    };
         uint32_t RESERVED4[32-2];
    union {
        __IO uint32_t MPIN[2];
        struct {
            __IO uint32_t MPIN0;
            __IO uint32_t MPIN1;
        };
    };
         uint32_t RESERVED5[32-2];
    union {
        __IO uint32_t SET[2];
        struct {
            __IO uint32_t SET0;
            __IO uint32_t SET1;
        };
    };
         uint32_t RESERVED6[32-2];
    union {
        __O  uint32_t CLR[2];
        struct {
            __IO uint32_t CLR0;
            __IO uint32_t CLR1;
        };
    };
    __I  uint32_t RESERVED2288[(0x2300-0x2288)/4];
    union {
        __O  uint32_t NOT[2];
        struct {
            __O  uint32_t NOT0;
            __O  uint32_t NOT1;
        };
    };
    __I  uint32_t RESERVED2308[(0x2380-0x2308)/4];
    union {
        __O  uint32_t DIRSET[2];
        struct {
            __O  uint32_t DIRSET0;
            __O  uint32_t DIRSET1;
        };
    };
    __I  uint32_t RESERVED2388[(0x2400-0x2388)/4];
    union {
        __O  uint32_t DIRCLR[2];
        struct {
            __O  uint32_t DIRCLR0;
            __O  uint32_t DIRCLR1;
        };
    };
    __I  uint32_t RESERVED2408[(0x2480-0x2408)/4];
    union {
        __O  uint32_t DIRNOT[2];
        struct {
            __O  uint32_t DIRNOT0;
            __O  uint32_t DIRNOT1;
        };
    };
} LPC_GPIO_Type;


// ------------------------------------------------------------------------------------------------
// -----                                    GPIO_GROUP_INT0/1                                   -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t CTRL;
    __I  uint32_t RESERVED004[(0x020-0x004)/4];
    union {
        __IO uint32_t PORT_POL[2];
        struct {
            __IO uint32_t PORT_POL0;
            __IO uint32_t PORT_POL1;
        };
    };
    __I  uint32_t RESERVED02C[(0x040-0x028)/4];
    union {
        __IO uint32_t PORT_ENA[2];
        struct {
            __IO uint32_t PORT_ENA0;
            __IO uint32_t PORT_ENA1;
        };
    };
} LPC_GPIO_GROUP_INT_Type;


// ------------------------------------------------------------------------------------------------
// -----                                     GPIO_PIN_INT                                     -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t ISEL;
    __IO uint32_t IENR;
    __IO uint32_t SIENR;
    __IO uint32_t CIENR;
    __IO uint32_t IENF;
    __IO uint32_t SIENF;
    __IO uint32_t CIENF;
    __IO uint32_t RISE;
    __IO uint32_t FALL;
    __IO uint32_t IST;
    __IO uint32_t PMCTRL;
    __IO uint32_t PMSRC;
    __IO uint32_t PMCFG;
} LPC_GPIO_PIN_INT_Type;


// ------------------------------------------------------------------------------------------------
// -----                                          I2C                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t CFG;
    __IO uint32_t STAT;
    __IO uint32_t INTENSET;
    __O  uint32_t INTENCLR;
    __IO uint32_t TIMEOUT;
    __IO uint32_t CLKDIV;
    __I  uint32_t INTSTAT;
         uint32_t _RESERVED1_;
    __IO uint32_t MSTCTL;
    __IO uint32_t MSTTIME;
    __IO uint32_t MSTDAT;
         uint32_t _RESERVED2_[5];
    __IO uint32_t SLVCTL;
    __IO uint32_t SLVDAT;
    union {
        struct {
            __IO uint32_t SLVADR0;
            __IO uint32_t SLVADR1;
            __IO uint32_t SLVADR2;
            __IO uint32_t SLVADR3;
        };
        __IO uint32_t SLVADR[4];
    };
    __IO uint32_t SLVQUAL0;
         uint32_t _RESERVED3_[9];
    __IO uint32_t MONRXDAT;
} LPC_I2C_Type;



// ------------------------------------------------------------------------------------------------
// -----                                       INPUTMUX                                       -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __I  uint32_t RESERVED000[(0x0A0-0x000)/4];

    union {     /* 0x0A0 */
        struct {
            __IO uint32_t EZH_SLICE_INMUX0;
            __IO uint32_t EZH_SLICE_INMUX1;
            __IO uint32_t EZH_SLICE_INMUX2;
            __IO uint32_t EZH_SLICE_INMUX3;
            __IO uint32_t EZH_SLICE_INMUX4;
            __IO uint32_t EZH_SLICE_INMUX5;
            __IO uint32_t EZH_SLICE_INMUX6;
            __IO uint32_t EZH_SLICE_INMUX7;
        };
        __IO uint32_t EZH_SLICE_INMUX[8];
    };

    union {     /* 0x0C0 */
        struct {
            __IO uint32_t PINTSEL0;
            __IO uint32_t PINTSEL1;
            __IO uint32_t PINTSEL2;
            __IO uint32_t PINTSEL3;
            __IO uint32_t PINTSEL4;
            __IO uint32_t PINTSEL5;
            __IO uint32_t PINTSEL6;
            __IO uint32_t PINTSEL7;
        };
        __IO uint32_t PINTSEL[8];
    };

    union {     /* 0x0E0 */
        struct {
            __IO uint32_t DMA_ITRIG_INMUX0;
            __IO uint32_t DMA_ITRIG_INMUX1;
            __IO uint32_t DMA_ITRIG_INMUX2;
            __IO uint32_t DMA_ITRIG_INMUX3;
            __IO uint32_t DMA_ITRIG_INMUX4;
            __IO uint32_t DMA_ITRIG_INMUX5;
            __IO uint32_t DMA_ITRIG_INMUX6;
            __IO uint32_t DMA_ITRIG_INMUX7;
            __IO uint32_t DMA_ITRIG_INMUX8;
            __IO uint32_t DMA_ITRIG_INMUX9;
            __IO uint32_t DMA_ITRIG_INMUX10;
            __IO uint32_t DMA_ITRIG_INMUX11;
            __IO uint32_t DMA_ITRIG_INMUX12;
            __IO uint32_t DMA_ITRIG_INMUX13;
            __IO uint32_t DMA_ITRIG_INMUX14;
            __IO uint32_t DMA_ITRIG_INMUX15;
            __IO uint32_t DMA_ITRIG_INMUX16;
            __IO uint32_t DMA_ITRIG_INMUX17;
            __IO uint32_t DMA_ITRIG_INMUX18;
            __IO uint32_t DMA_ITRIG_INMUX19;
            __IO uint32_t DMA_ITRIG_INMUX20;
            __IO uint32_t DMA_ITRIG_INMUX21;
        };
        __IO uint32_t DMA_ITRIG_INMUX[22];
    };

    __I  uint32_t RESERVED138[(0x160-0x138)/4];

    union {     /* 0x160 */
        struct {
            __IO uint32_t DMA_OTRIG_INMUX0;
            __IO uint32_t DMA_OTRIG_INMUX1;
            __IO uint32_t DMA_OTRIG_INMUX2;
            __IO uint32_t DMA_OTRIG_INMUX3;
        };
        __IO uint32_t DMA_OTRIG_INMUX[4];
    };

    __I  uint32_t RESERVED170[(0x180-0x170)/4];

    __IO uint32_t FREQMEAS_REF;
    __IO uint32_t FREQMEAS_TARGET;
} LPC_INPUTMUX_t;


// ------------------------------------------------------------------------------------------------
// -----                                       IOCONFIG                                       -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t PIO0_0;
    __IO uint32_t PIO0_1;
    __IO uint32_t PIO0_2;
    __IO uint32_t PIO0_3;
    __IO uint32_t PIO0_4;
    __IO uint32_t PIO0_5;
    __IO uint32_t PIO0_6;
    __IO uint32_t PIO0_7;
    __IO uint32_t PIO0_8;
    __IO uint32_t PIO0_9;
    __IO uint32_t PIO0_10;
    __IO uint32_t PIO0_11;
    __IO uint32_t PIO0_12;
    __IO uint32_t PIO0_13;
    __IO uint32_t PIO0_14;
    __IO uint32_t PIO0_15;
    __IO uint32_t PIO0_16;
    __IO uint32_t PIO0_17;
    __IO uint32_t PIO0_18;
    __IO uint32_t PIO0_19;
    __IO uint32_t PIO0_20;
    __IO uint32_t PIO0_21;
    __IO uint32_t PIO0_22;
    __IO uint32_t PIO0_23;
    __IO uint32_t PIO0_24;
    __IO uint32_t PIO0_25;
    __IO uint32_t PIO0_26;
    __IO uint32_t PIO0_27;
    __IO uint32_t PIO0_28;
    __IO uint32_t PIO0_29;
    __IO uint32_t PIO0_30;
    __IO uint32_t PIO0_31;
    __IO uint32_t PIO1_0;
    __IO uint32_t PIO1_1;
    __IO uint32_t PIO1_2;
    __IO uint32_t PIO1_3;
    __IO uint32_t PIO1_4;
    __IO uint32_t PIO1_5;
    __IO uint32_t PIO1_6;
    __IO uint32_t PIO1_7;
    __IO uint32_t PIO1_8;
    __IO uint32_t PIO1_9;
    __IO uint32_t PIO1_10;
    __IO uint32_t PIO1_11;
    __IO uint32_t PIO1_12;
    __IO uint32_t PIO1_13;
    __IO uint32_t PIO1_14;
    __IO uint32_t PIO1_15;
    __IO uint32_t PIO1_16;
    __IO uint32_t PIO1_17;
} LPC_IOCON_Type;


// ------------------------------------------------------------------------------------------------
// -----                                     Mailbox                                          -----
// ------------------------------------------------------------------------------------------------

typedef struct
{
    __IO uint32_t IRQ0;
    __O  uint32_t IRQ0SET;
    __O  uint32_t IRQ0CLR;
    __I  uint32_t RESERVED00C[(0x010-0x00C)/4];
    __IO uint32_t IRQ1;
    __O  uint32_t IRQ1SET;
    __O  uint32_t IRQ1CLR;
    __I  uint32_t RESERVED01C[(0x0F8-0x01C)/4];
    __IO uint32_t MUTEX;
} LPC_MAILBOX_Type;


// ------------------------------------------------------------------------------------------------
// -----                                         MRT                                          -----
// ------------------------------------------------------------------------------------------------

typedef struct {

    union {     /* 0x000 */
        struct {
            __IO uint32_t INTVAL0;
            __I  uint32_t TIMER0;
            __IO uint32_t CTRL0;
            __IO uint32_t STAT0;

            __IO uint32_t INTVAL1;
            __I  uint32_t TIMER1;
            __IO uint32_t CTRL1;
            __IO uint32_t STAT1;

            __IO uint32_t INTVAL2;
            __I  uint32_t TIMER2;
            __IO uint32_t CTRL2;
            __IO uint32_t STAT2;

            __IO uint32_t INTVAL3;
            __I  uint32_t TIMER3;
            __IO uint32_t CTRL3;
            __IO uint32_t STAT3;
        };

        struct {
            __IO uint32_t INTVAL;
            __I  uint32_t TIMER;
            __IO uint32_t CTRL;
            __IO uint32_t STAT;
        } channel[4];
    };

    __I  uint32_t RESERVED040[(0x0F0-0x040)/4];

    __IO uint32_t MODCFG;
    __I  uint32_t IDLE_CH;
    __IO uint32_t IRQ_FLAG;
} LPC_MRT_t;


// ------------------------------------------------------------------------------------------------
// -----                                          RTC                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t CTRL;
    __IO uint32_t MATCH;
    __IO uint32_t COUNT;
    __IO uint32_t WAKE;
} LPC_RTC_Type;


// ------------------------------------------------------------------------------------------------
// -----                                          SCT                                         -----
// ------------------------------------------------------------------------------------------------

#define CONFIG_SCT_nEV      13          /* Number of events */
#define CONFIG_SCT_nRG      13          /* Number of match/compare registers */
#define CONFIG_SCT_nOU      8           /* Number of outputs */

typedef struct {
    __IO  uint32_t CONFIG;              /* 0x000 Configuration Register */
    union {
        __IO uint32_t CTRL;             /* 0x004 Control Register */
        __IO uint32_t CTRL_U;           /* 0x004 unified control Register */
        struct {
            __IO uint16_t CTRL_L;       /* 0x004 low control register */
            __IO uint16_t CTRL_H;       /* 0x006 high control register */
        };
    };
    __IO uint16_t LIMIT_L;              /* 0x008 limit register for counter L */
    __IO uint16_t LIMIT_H;              /* 0x00A limit register for counter H */
    __IO uint16_t HALT_L;               /* 0x00C halt register for counter L */
    __IO uint16_t HALT_H;               /* 0x00E halt register for counter H */
    __IO uint16_t STOP_L;               /* 0x010 stop register for counter L */
    __IO uint16_t STOP_H;               /* 0x012 stop register for counter H */
    __IO uint16_t START_L;              /* 0x014 start register for counter L */
    __IO uint16_t START_H;              /* 0x016 start register for counter H */
         uint32_t RESERVED1[10];        /* 0x018-0x03C reserved */
    union {
        __IO uint32_t COUNT;            /* 0x040 counter register */
        __IO uint32_t COUNT_U;          /* 0x040 unified counter register */
        struct {
            __IO uint16_t COUNT_L;      /* 0x040 counter register for counter L */
            __IO uint16_t COUNT_H;      /* 0x042 counter register for counter H */
        };
    };
    __IO uint16_t STATE_L;              /* 0x044 state register for counter L */
    __IO uint16_t STATE_H;              /* 0x046 state register for counter H */
    __I  uint32_t INPUT;                /* 0x048 input register */
    __IO uint16_t REGMODE_L;            /* 0x04C match - capture registers mode register L */
    __IO uint16_t REGMODE_H;            /* 0x04E match - capture registers mode register H */
    __IO uint32_t OUTPUT;               /* 0x050 output register */
    __IO uint32_t OUTPUTDIRCTRL;        /* 0x054 Output counter direction Control Register */
    __IO uint32_t RES;                  /* 0x058 conflict resolution register */
    __IO uint32_t DMA0REQUEST;          /* 0x05C DMA0 Request Register */
    __IO uint32_t DMA1REQUEST;          /* 0x060 DMA1 Request Register */
         uint32_t RESERVED2[35];        /* 0x064-0x0EC reserved */
    __IO uint32_t EVEN;                 /* 0x0F0 event enable register */
    __IO uint32_t EVFLAG;               /* 0x0F4 event flag register */
    __IO uint32_t CONEN;                /* 0x0F8 conflict enable register */
    __IO uint32_t CONFLAG;              /* 0x0FC conflict flag register */

    union {
        __IO union {                    /* 0x100-... Match / Capture value */
            uint32_t U;                 /*       SCTMATCH[i].U  Unified 32-bit register */
            struct {
                uint16_t L;             /*       SCTMATCH[i].L  Access to L value */
                uint16_t H;             /*       SCTMATCH[i].H  Access to H value */
            };
        } MATCH[CONFIG_SCT_nRG];
        __I union {
            uint32_t U;                 /*       SCTCAP[i].U  Unified 32-bit register */
            struct {
                uint16_t L;             /*       SCTCAP[i].L  Access to H value */
                uint16_t H;             /*       SCTCAP[i].H  Access to H value */
            };
        } CAP[CONFIG_SCT_nRG];
    };

         uint32_t RESERVED3[32-CONFIG_SCT_nRG];   /* ...-0x17C reserved */

    union {
        __IO uint16_t MATCH_L[CONFIG_SCT_nRG];    /* 0x180-... Match Value L counter */
        __I  uint16_t CAP_L[CONFIG_SCT_nRG];      /* 0x180-... Capture Value L counter */
    };
         uint16_t RESERVED4[32-CONFIG_SCT_nRG];   /* ...-0x1BE reserved */
    union {
        __IO uint16_t MATCH_H[CONFIG_SCT_nRG];    /* 0x1C0-... Match Value H counter */
        __I  uint16_t CAP_H[CONFIG_SCT_nRG];      /* 0x1C0-... Capture Value H counter */
    };
         uint16_t RESERVED5[32-CONFIG_SCT_nRG];   /* ...-0x1FE reserved */

    union {
        __IO union {                    /* 0x200-... Match Reload / Capture Control value */
            uint32_t U;                 /*       SCTMATCHREL[i].U  Unified 32-bit register */
            struct {
                uint16_t L;             /*       SCTMATCHREL[i].L  Access to L value */
                uint16_t H;             /*       SCTMATCHREL[i].H  Access to H value */
            };
        } MATCHREL[CONFIG_SCT_nRG];
        __IO union {
            uint32_t U;                 /*       SCTCAPCTRL[i].U  Unified 32-bit register */
            struct {
                uint16_t L;             /*       SCTCAPCTRL[i].L  Access to H value */
                uint16_t H;             /*       SCTCAPCTRL[i].H  Access to H value */
            };
        } CAPCTRL[CONFIG_SCT_nRG];
    };

         uint32_t RESERVED6[32-CONFIG_SCT_nRG];       /* ...-0x27C reserved */

    union {
        __IO uint16_t MATCHREL_L[CONFIG_SCT_nRG];     /* 0x280-... Match Reload value L counter */
        __IO uint16_t CAPCTRL_L[CONFIG_SCT_nRG];      /* 0x280-... Capture Control value L counter */
    };
         uint16_t RESERVED7[32-CONFIG_SCT_nRG];       /* ...-0x2BE reserved */
    union {
        __IO uint16_t MATCHREL_H[CONFIG_SCT_nRG];     /* 0x2C0-... Match Reload value H counter */
        __IO uint16_t CAPCTRL_H[CONFIG_SCT_nRG];      /* 0x2C0-... Capture Control value H counter */
    };
         uint16_t RESERVED8[32-CONFIG_SCT_nRG];       /* ...-0x2FE reserved */

    __IO struct {                       /* 0x300-0x3FC  SCTEVENT[i].STATE / SCTEVENT[i].CTRL*/
        uint32_t STATE;                 /* Event State Register */
        uint32_t CTRL;                  /* Event Control Register */
    } EVENT[CONFIG_SCT_nEV];

         uint32_t RESERVED9[128-2*CONFIG_SCT_nEV];    /* ...-0x4FC reserved */

    __IO struct {                       /* 0x500-0x57C  SCTOUT[i].SET / SCTOUT[i].CLR */
        uint32_t SET;                   /* Output n Set Register */
        uint32_t CLR;                   /* Output n Clear Register */
    } OUT[CONFIG_SCT_nOU];

         uint32_t RESERVED10[191-2*CONFIG_SCT_nOU];   /* ...-0x7F8 reserved */

    __I  uint32_t MODULECONTENT;        /* 0x7FC Module Content */

} LPC_SCT0_Type;


// ------------------------------------------------------------------------------------------------
// -----                                        SPI                                           -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __I  uint32_t RESERVED000[(0x400-0x000)/4];
    __IO uint32_t CFG;
    __IO uint32_t DLY;
    __IO uint32_t STAT;
    __IO uint32_t INTENSET;
    __O  uint32_t INTENCLR;
    __I  uint32_t RESERVED414[(0x424-0x414)/4];
    __IO uint32_t DIV;
    __I  uint32_t INTSTAT;
    __I  uint32_t RESERVED42C[(0xE00-0x42C)/4];
    __IO uint32_t FIFOCFG;
    __IO uint32_t FIFOSTAT;
    __IO uint32_t FIFOTRIG;
    __I  uint32_t RESERVEDE0C[(0xE10-0xE0C)/4];
    __IO uint32_t FIFOINTENSET;
    __IO uint32_t FIFOINTENCLR;
    __I  uint32_t FIFOINTSTAT;
    __I  uint32_t RESERVEDE1C[(0xE20-0xE1C)/4];
    union {
        __O  uint32_t FIFOWR;
        struct {
            __O  uint16_t FIFOWR_TXDATA16;
            __O  uint16_t FIFOWR_CTRL16;
        };
    };
    __I  uint32_t RESERVEDE24[(0xE30-0xE24)/4];
    __I  uint32_t FIFORD;
    __I  uint32_t RESERVEDE34[(0xE40-0xE34)/4];
    __I  uint32_t FIFORDNOPOP;
} LPC_SPI_Type;


// ------------------------------------------------------------------------------------------------
// -----                                 SYSCONEXTRA                                          -----
// ------------------------------------------------------------------------------------------------

typedef struct
{
    __IO uint32_t VDCTRL[5];
    __I  uint32_t RESERVED014[(0x044-0x014)/4];
    __IO uint32_t BODCTRL;
} LPC_SYSCONEXTRA_Type;


// ------------------------------------------------------------------------------------------------
// -----                                        Timer                                       -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t IR;
    __IO uint32_t TCR;
    __IO uint32_t TC;
    __IO uint32_t PR;
    __IO uint32_t PC;
    __IO uint32_t MCR;
    union {
        __IO uint32_t MR[4];
        struct{
            __IO uint32_t MR0;
            __IO uint32_t MR1;
            __IO uint32_t MR2;
            __IO uint32_t MR3;
        };
    };
    __IO uint32_t CCR;
    union{
        __I  uint32_t CR[4];
        struct{
            __I  uint32_t CR0;
            __I  uint32_t CR1;
            __I  uint32_t CR2;
            __I  uint32_t CR3;
        };
    };
    __IO uint32_t EMR;
    __I  uint32_t RESERVED040[(0x070-0x040)/4];
    __IO uint32_t CTCR;
    __IO uint32_t PWMC;
} LPC_TIMER_Type;


// ------------------------------------------------------------------------------------------------
// -----                                         UART                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t CFG;
    __IO uint32_t CTL;
    __IO uint32_t STAT;
    __IO uint32_t INTENSET;
    __O  uint32_t INTENCLR;
    __I  uint32_t RXDAT;
    __I  uint32_t RXDATSTAT;
    __IO uint32_t TXDAT;
    __IO uint32_t BRG;
    __IO uint32_t INTSTAT;
    __IO uint32_t OSR;
    __IO uint32_t ADDR;
    __I  uint32_t RESERVED030[(0xE00-0x030)/4];
    __IO uint32_t FIFOCFG;
    __IO uint32_t FIFOSTAT;
    __IO uint32_t FIFOTRIG;
    __I  uint32_t RESERVEDE0C[(0xE10-0xE0C)/4];
    __IO uint32_t FIFOINTENSET;
    __IO uint32_t FIFOINTENCLR;
    __I  uint32_t FIFOINTSTAT;
    __I  uint32_t RESERVEDE1C[(0xE20-0xE1C)/4];
    __O  uint32_t FIFOWR;
    __I  uint32_t RESERVEDE24[(0xE30-0xE24)/4];
    __I  uint32_t FIFORD;
    __I  uint32_t RESERVEDE34[(0xE40-0xE34)/4];
    __I  uint32_t FIFORDNOPOP;
} LPC_UART_Type;


// ------------------------------------------------------------------------------------------------
// -----                                         WWDT                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t MOD;
    __IO uint32_t TC;
    __IO uint32_t FEED;
    __I  uint32_t TV;
    __IO uint32_t CLKSEL;
    __IO uint32_t WARNINT;
    __IO uint32_t WINDOW;
} LPC_WWDT_Type;





typedef struct
{
    __IO uint32_t ASYNCPRESETCTRL;
    __O  uint32_t ASYNCPRESETCTRLSET;
    __O  uint32_t ASYNCPRESETCTRLCLR;
    __I  uint32_t RESERVED00C[(0x010-0x00C)/4];
    __IO uint32_t ASYNCAPBCLKCTRL;
    __O  uint32_t ASYNCAPBCLKCTRLSET;
    __O  uint32_t ASYNCAPBCLKCTRLCLR;
    __I  uint32_t RESERVED01C[(0x020-0x01C)/4];
    __IO uint32_t ASYNCAPBCLKSELA;
} LPC_ASYNCSYSCON_Type;



typedef struct
{
    __IO uint32_t SYSMEMREMAP;
    __I  uint32_t RESERVED000[(0x010-0x004)/4];
    __IO uint32_t AHBMATPRIO;
    __I  uint32_t RESERVED014[(0x040-0x014)/4];
    __IO uint32_t SYSTCKCAL;
    __I  uint32_t RESERVED044[(0x048-0x044)/4];
    __IO uint32_t NMISRC;
    __IO uint32_t ASYNCAPBCTRL;
    __I  uint32_t RESERVED050[(0x0C0-0x050)/4];
    union {
        __I  uint32_t PIOPORCAP[2];
        struct {
            __I  uint32_t PIOPORCAP0;
            __I  uint32_t PIOPORCAP1;
        };
    };
    __I  uint32_t RESERVED0C8[(0x0D0-0x0C8)/4];
    union {
        __I  uint32_t PIORESCAP[2];
        struct {
            __I  uint32_t PIORESCAP0;
            __I  uint32_t PIORESCAP1;
        };
    };
    __I  uint32_t RESERVED0D8[(0x100-0x0D8)/4];
    union {
        __IO uint32_t PRESETCTRL[2];
        struct {
            __IO uint32_t PRESETCTRL0;
            __IO uint32_t PRESETCTRL1;
        };
    };
    __I  uint32_t RESERVED108[(0x120-0x108)/4];
    union {
        __O  uint32_t PRESETCTRLSET[2];
        struct {
            __O  uint32_t PRESETCTRLSET0;
            __O  uint32_t PRESETCTRLSET1;
        };
    };
    __I  uint32_t RESERVED128[(0x140-0x128)/4];
    union {
        __O  uint32_t PRESETCTRLCLR[2];
        struct {
            __O  uint32_t PRESETCTRLCLR0;
            __O  uint32_t PRESETCTRLCLR1;
        };
    };
    __I  uint32_t RESERVED148[(0x1F0-0x148)/4];
    __IO uint32_t SYSRSTSTAT;
    __I  uint32_t RESERVED1F4[(0x200-0x1F4)/4];
    union {
        __IO uint32_t AHBCLKCTRL[2];
        struct {
            __IO uint32_t AHBCLKCTRL0;
            __IO uint32_t AHBCLKCTRL1;
        };
    };
    __I  uint32_t RESERVED208[(0x220-0x208)/4];
    union {
        __O  uint32_t AHBCLKCTRLSET[2];
        struct {
            __O  uint32_t AHBCLKCTRLSET0;
            __O  uint32_t AHBCLKCTRLSET1;
        };
    };
    __I  uint32_t RESERVED228[(0x240-0x228)/4];
    union {
        __O  uint32_t AHBCLKCTRLCLR[2];
        struct {
            __O  uint32_t AHBCLKCTRLCLR0;
            __O  uint32_t AHBCLKCTRLCLR1;
        };
    };
    __I  uint32_t RESERVED248[(0x280-0x248)/4];
    __IO uint32_t MAINCLKSELA;
    __IO uint32_t MAINCLKSELB;
    __IO uint32_t CLKOUTSELA;
    __I  uint32_t RESERVED28C[(0x290-0x28C)/4];
    __IO uint32_t SYSPLLCLKSEL;
    __I  uint32_t RESERVED294[(0x2A4-0x294)/4];
    __IO uint32_t ADCCLKSEL;
    __IO uint32_t USBCLKSEL;
    __I  uint32_t RESERVED2AC[(0x2B0-0x2AC)/4];
    union {
        __O  uint32_t FCLKSEL[8];
        struct {
            __O  uint32_t FCLKSEL0;
            __O  uint32_t FCLKSEL1;
            __O  uint32_t FCLKSEL2;
            __O  uint32_t FCLKSEL3;
            __O  uint32_t FCLKSEL4;
            __O  uint32_t FCLKSEL5;
            __O  uint32_t FCLKSEL6;
            __O  uint32_t FCLKSEL7;
        };
    };
    __I  uint32_t RESERVED2D0[(0x2E0-0x2D0)/4];
    __IO uint32_t MCLKCLKSEL;
    __I  uint32_t RESERVED2E4[(0x2E8-0x2E4)/4];
    __IO uint32_t FRGCLKSEL;
    __IO uint32_t DMICCLKSEL;
    __I  uint32_t RESERVED2F0[(0x300-0x2F0)/4];
    __IO uint32_t SYSTICKCLKDIV;
    __IO uint32_t TRACECLKDIV;
    __I  uint32_t RESERVED308[(0x380-0x308)/4];
    __IO uint32_t AHBCLKDIV;
    __IO uint32_t CLKOUTDIV;
    __I  uint32_t RESERVED388[(0x394-0x388)/4];
    __IO uint32_t ADCCLKDIV;
    __IO uint32_t USBCLKDIV;
    __I  uint32_t RESERVED39C[(0x3A0-0x39C)/4];
    __IO uint32_t FRGCTRL;
    __I  uint32_t RESERVED3A4[(0x3A8-0x3A4)/4];
    __IO uint32_t DMICCLKDIV;
    __IO uint32_t I2SMCLKDIV;
    __I  uint32_t RESERVED3B0[(0x400-0x3B0)/4];
    __IO uint32_t FLASHCFG;
    __I  uint32_t RESERVED404[(0x40C-0x404)/4];
    __IO uint32_t USBCLKCTRL;
    __I  uint32_t USBCLKSTAT;
    __IO uint32_t EZHINT;
    __IO uint32_t FREQMECTRL;
    __I  uint32_t RESERVED41C[(0x420-0x41C)/4];
    __IO uint32_t MCLKIO;
    __I  uint32_t RESERVED424[(0x500-0x424)/4];
    __IO uint32_t FROCTRL;
    __I  uint32_t RESERVED504[(0x508-0x504)/4];
    __IO uint32_t WDTOSCCTRL;
    __IO uint32_t RTCOSCCTRL;
    __I  uint32_t RESERVED510[(0x580-0x510)/4];
    __IO uint32_t SYSPLLCTRL;
    __I  uint32_t SYSPLLSTAT;
    __IO uint32_t SYSPLLNDEC;
    __IO uint32_t SYSPLLPDEC;
    __IO uint32_t SYSPLLSSCTRL0;
    __IO uint32_t SYSPLLSSCTRL1;
    __I  uint32_t RESERVED598[(0x600-0x598)/4];
    __IO uint32_t PDSLEEPCFG0;
    __I  uint32_t RESERVED604[(0x610-0x604)/4];
    __IO uint32_t PDRUNCFG0;
    __I  uint32_t RESERVED614[(0x620-0x614)/4];
    __O  uint32_t PDRUNCFGSET0;
    __I  uint32_t RESERVED624[(0x630-0x624)/4];
    __O  uint32_t PDRUNCFGCLR0;
    __I  uint32_t RESERVED634[(0x680-0x634)/4];
    union {
        __IO uint32_t STARTER[2];
        struct {
            __IO uint32_t STARTER0;
            __IO uint32_t STARTER1;
        };
    };
    __I  uint32_t RESERVED688[(0x6A0-0x688)/4];
    union {
        __O  uint32_t STARTERSET[2];
        struct {
            __O  uint32_t STARTERSET0;
            __O  uint32_t STARTERSET1;
        };
    };
    __I  uint32_t RESERVED6A8[(0x6C0-0x6A8)/4];
    union {
        __O  uint32_t STARTERCLR[2];
        struct {
            __O  uint32_t STARTERCLR0;
            __O  uint32_t STARTERCLR1;
        };
    };
    __I  uint32_t RESERVED6C8[(0x780-0x6C8)/4];
    __IO uint32_t HWWAKE;
    __I  uint32_t RESERVED784[(0x800-0x784)/4];
    __IO uint32_t CPUCTRL;
    __IO uint32_t CPBOOT;
    __IO uint32_t CPSTACK;
    __I  uint32_t CPSTAT;
    __I  uint32_t RESERVED810[(0xE04-0x810)/4];
    __IO uint32_t AUTOCGOR;
    __I  uint32_t RESERVEDE08[(0xFF4-0xE08)/4];
    __I  uint32_t JTAGIDCODE;
    __I  uint32_t DEVICE_ID0;
    __I  uint32_t DEVICE_ID1;
} LPC_SYSCON_Type;


// ------------------------------------------------------------------------------------------------
// -----                                          USB                                         -----
// ------------------------------------------------------------------------------------------------

typedef struct {
    __IO uint32_t DEVCMDSTAT;
    __IO uint32_t INFO;
    __IO uint32_t EPLISTSTART;
    __IO uint32_t DATABUFSTART;
    __IO uint32_t LPM;
    __IO uint32_t EPSKIP;
    __IO uint32_t EPINUSE;
    __IO uint32_t EPBUFCFG;
    __IO uint32_t INTSTAT;
    __IO uint32_t INTEN;
    __IO uint32_t INTSETSTAT;
    __I  uint32_t RESERVED02C[(0x034-0x02C)/4];
    __I  uint32_t EPTOGGLE;
} LPC_USB_Type;


// ------------------------------------------------------------------------------------------------
// -----                                 Peripheral memory map                                -----
// ------------------------------------------------------------------------------------------------

#define LPC_FLASH_BASE        (0x00000000UL)
#define LPC_SRAM0_BASE        (0x20000000UL)
#define LPC_SRAM1_BASE        (0x20010000UL)
#define LPC_SRAM2_BASE        (0x20020000UL)
#define LPC_SRAMX_BASE        (0x04000000UL)
#define LPC_APB0_BASE         (0x40000000UL)
#define LPC_APB1_BASE         (0x40020000UL)
#define LPC_APBASYNCH_BASE    (0x40040000UL)
#define LPC_AHB_BASE          (0x40080000UL)
#define LPC_CM4_BASE          (0xE0000000UL)

/* APB0 peripherals                                                           */
#define LPC_SYSCON_BASE       (LPC_APB0_BASE + 0x00000)
#define LPC_IOCON_BASE        (LPC_APB0_BASE + 0x01000)
#define LPC_GINT0_BASE        (LPC_APB0_BASE + 0x02000)
#define LPC_GINT1_BASE        (LPC_APB0_BASE + 0x03000)
#define LPC_PINT_BASE         (LPC_APB0_BASE + 0x04000)
#define LPC_INPUTMUX_BASE     (LPC_APB0_BASE + 0x05000)
#define LPC_TIM0_BASE         (LPC_APB0_BASE + 0x08000)
#define LPC_TIM1_BASE         (LPC_APB0_BASE + 0x09000)
#define LPC_WWDT_BASE         (LPC_APB0_BASE + 0x0C000)
#define LPC_MRT_BASE          (LPC_APB0_BASE + 0x0D000)
#define LPC_UTICK_BASE        (LPC_APB0_BASE + 0x0E000)

/* APB1 peripherals                                                           */
#define LPC_SYSCONEXTRA_BASE  (LPC_APB1_BASE + 0x00000)
#define LPC_TIM2_BASE         (LPC_APB1_BASE + 0x08000)
#define LPC_RTC_BASE          (LPC_APB1_BASE + 0x0C000)
#define LPC_FLASHCTRL_BASE    (LPC_APB1_BASE + 0x14080)

/* APBASYNCH peripherals                                                           */
#define LPC_ASYNCSYSCON_BASE  (LPC_APBASYNCH_BASE + 0x00000)
#define LPC_TIM3_BASE         (LPC_APBASYNCH_BASE + 0x08000)
#define LPC_TIM4_BASE         (LPC_APBASYNCH_BASE + 0x09000)

/* AHB peripherals                                                            */
#define LPC_DMA_BASE          (LPC_AHB_BASE  + 0x02000)
#define LPC_USB_BASE          (LPC_AHB_BASE  + 0x04000)
#define LPC_SCT0_BASE         (LPC_AHB_BASE  + 0x05000)
#define LPC_FLEXCOMM0_BASE    (LPC_AHB_BASE  + 0x06000)
#define LPC_FLEXCOMM1_BASE    (LPC_AHB_BASE  + 0x07000)
#define LPC_FLEXCOMM2_BASE    (LPC_AHB_BASE  + 0x08000)
#define LPC_FLEXCOMM3_BASE    (LPC_AHB_BASE  + 0x09000)
#define LPC_FLEXCOMM4_BASE    (LPC_AHB_BASE  + 0x0A000)
#define LPC_MAILBOX_BASE      (LPC_AHB_BASE  + 0x0B000)
#define LPC_GPIO_BASE         (LPC_AHB_BASE  + 0x0C000)
#define LPC_DMIC_BASE         (LPC_AHB_BASE  + 0x10000)
#define LPC_CRC_BASE          (LPC_AHB_BASE  + 0x15000)
#define LPC_FLEXCOMM5_BASE    (LPC_AHB_BASE  + 0x16000)
#define LPC_FLEXCOMM6_BASE    (LPC_AHB_BASE  + 0x17000)
#define LPC_FLEXCOMM7_BASE    (LPC_AHB_BASE  + 0x18000)
#define LPC_ISPAP_BASE        (LPC_AHB_BASE  + 0x1C000)
#define LPC_ADC0_BASE         (LPC_AHB_BASE  + 0x20000)


// ------------------------------------------------------------------------------------------------
// -----                                Peripheral declaration                                -----
// ------------------------------------------------------------------------------------------------

#define LPC_ADC0              ((LPC_ADC_Type            *) LPC_ADC0_BASE     )
#define LPC_ASYNCSYSCON       ((LPC_ASYNCSYSCON_Type    *) LPC_ASYNCSYSCON_BASE)
#define LPC_CRC               ((LPC_CRC_Type            *) LPC_CRC_BASE      )
#define LPC_DMA               ((LPC_DMA_Type            *) LPC_DMA_BASE      )
#define LPC_DMIC              ((LPC_DMIC_Type           *) LPC_DMIC_BASE     )
#define LPC_FLASHCTRL         ((LPC_FLASHCTRL_Type      *) LPC_FLASHCTRL_BASE)
#define LPC_FLEXCOMM0         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM0_BASE)
#define LPC_FLEXCOMM1         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM1_BASE)
#define LPC_FLEXCOMM2         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM2_BASE)
#define LPC_FLEXCOMM3         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM3_BASE)
#define LPC_FLEXCOMM4         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM4_BASE)
#define LPC_FLEXCOMM5         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM5_BASE)
#define LPC_FLEXCOMM6         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM6_BASE)
#define LPC_FLEXCOMM7         ((LPC_FLEXCOMM_Type       *) LPC_FLEXCOMM7_BASE)
#define LPC_GINT0             ((LPC_GPIO_GROUP_INT_Type *) LPC_GINT0_BASE    )
#define LPC_GINT1             ((LPC_GPIO_GROUP_INT_Type *) LPC_GINT1_BASE    )
#define LPC_GPIO              ((LPC_GPIO_Type           *) LPC_GPIO_BASE     )
#define LPC_I2C0              ((LPC_I2C_Type            *) LPC_FLEXCOMM0_BASE)
#define LPC_I2C1              ((LPC_I2C_Type            *) LPC_FLEXCOMM1_BASE)
#define LPC_I2C2              ((LPC_I2C_Type            *) LPC_FLEXCOMM2_BASE)
#define LPC_I2C3              ((LPC_I2C_Type            *) LPC_FLEXCOMM3_BASE)
#define LPC_I2C4              ((LPC_I2C_Type            *) LPC_FLEXCOMM4_BASE)
#define LPC_I2C5              ((LPC_I2C_Type            *) LPC_FLEXCOMM5_BASE)
#define LPC_I2C6              ((LPC_I2C_Type            *) LPC_FLEXCOMM6_BASE)
#define LPC_I2C7              ((LPC_I2C_Type            *) LPC_FLEXCOMM7_BASE)
#define LPC_INPUTMUX          ((LPC_INPUTMUX_t          *) LPC_INPUTMUX_BASE )
#define LPC_IOCON             ((LPC_IOCON_Type          *) LPC_IOCON_BASE    )
#define LPC_MAILBOX           ((LPC_MAILBOX_Type        *) LPC_MAILBOX_BASE  )
#define LPC_MRT               ((LPC_MRT_t               *) LPC_MRT_BASE      )
#define LPC_OSTIMER           ((LPC_OSTIMER_Type        *) LPC_OSTIMER_BASE  )
#define LPC_PINT              ((LPC_GPIO_PIN_INT_Type   *) LPC_PINT_BASE     )
#define LPC_RTC               ((LPC_RTC_Type            *) LPC_RTC_BASE      )
#define LPC_SCT0              ((LPC_SCT0_Type           *) LPC_SCT0_BASE     )
#define LPC_SPI0              ((LPC_SPI_Type            *) LPC_FLEXCOMM0_BASE)
#define LPC_SPI1              ((LPC_SPI_Type            *) LPC_FLEXCOMM1_BASE)
#define LPC_SPI2              ((LPC_SPI_Type            *) LPC_FLEXCOMM2_BASE)
#define LPC_SPI3              ((LPC_SPI_Type            *) LPC_FLEXCOMM3_BASE)
#define LPC_SPI4              ((LPC_SPI_Type            *) LPC_FLEXCOMM4_BASE)
#define LPC_SPI5              ((LPC_SPI_Type            *) LPC_FLEXCOMM5_BASE)
#define LPC_SPI6              ((LPC_SPI_Type            *) LPC_FLEXCOMM6_BASE)
#define LPC_SPI7              ((LPC_SPI_Type            *) LPC_FLEXCOMM7_BASE)
#define LPC_SYSCON            ((LPC_SYSCON_Type         *) LPC_SYSCON_BASE   )
#define LPC_SYSCONEXTRA       ((LPC_SYSCONEXTRA_Type    *) LPC_SYSCONEXTRA_BASE)
#define LPC_TIMER0            ((LPC_TIMER_Type          *) LPC_TIM0_BASE     )
#define LPC_TIMER1            ((LPC_TIMER_Type          *) LPC_TIM1_BASE     )
#define LPC_TIMER2            ((LPC_TIMER_Type          *) LPC_TIM2_BASE     )
#define LPC_TIMER3            ((LPC_TIMER_Type          *) LPC_TIM3_BASE     )
#define LPC_TIMER4            ((LPC_TIMER_Type          *) LPC_TIM4_BASE     )
#define LPC_UART0             ((LPC_UART_Type           *) LPC_FLEXCOMM0_BASE)
#define LPC_UART1             ((LPC_UART_Type           *) LPC_FLEXCOMM1_BASE)
#define LPC_UART2             ((LPC_UART_Type           *) LPC_FLEXCOMM2_BASE)
#define LPC_UART3             ((LPC_UART_Type           *) LPC_FLEXCOMM3_BASE)
#define LPC_UART4             ((LPC_UART_Type           *) LPC_FLEXCOMM4_BASE)
#define LPC_UART5             ((LPC_UART_Type           *) LPC_FLEXCOMM5_BASE)
#define LPC_UART6             ((LPC_UART_Type           *) LPC_FLEXCOMM6_BASE)
#define LPC_UART7             ((LPC_UART_Type           *) LPC_FLEXCOMM7_BASE)
#define LPC_USART0            LPC_UART0
#define LPC_USART1            LPC_UART1
#define LPC_USART2            LPC_UART2
#define LPC_USART3            LPC_UART3
#define LPC_USART4            LPC_UART4
#define LPC_USART5            LPC_UART5
#define LPC_USART6            LPC_UART6
#define LPC_USART7            LPC_UART7
#define LPC_USB0              ((LPC_USB_Type            *) LPC_USB_BASE      )
#define LPC_UTICK             ((LPC_UTICK_Type          *) LPC_UTICK_BASE    )
#define LPC_WWDT              ((LPC_WWDT_Type           *) LPC_WWDT_BASE     )

#endif  // __LPC541xx_H__

