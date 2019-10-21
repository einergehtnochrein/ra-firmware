
#include <math.h>
#include <stdio.h>

#include "lpclib.h"
#include "bsp.h"
#include "config.h"


#if defined(__GNUC__)
    #define __stdout (*stdout)
    #define __stdin (*stdin)
    #define _Handle _file
#endif


#if !defined(__ICCARM__)
struct __FILE {
    UART_Handle _Handle;                    /**< Handle for UART */
};
#endif



UART_Handle blePort;

#define BLE_UART                            UART3
#define BLE_PORT_TXBUF_SIZE                 3072
#define BLE_PORT_RXBUF_SIZE                 512
static uint8_t blePortTxBuf[BLE_PORT_TXBUF_SIZE];
static uint8_t blePortRxBuf[BLE_PORT_RXBUF_SIZE];


static const UART_Config blePortConfig[] = {
    {.opcode = UART_OPCODE_SET_TX_BUFFER,
        {.buffer = {
            .pBuffer = blePortTxBuf,
            .size = BLE_PORT_TXBUF_SIZE, }}},

    {.opcode = UART_OPCODE_SET_RX_BUFFER,
        {.buffer = {
            .pBuffer = blePortRxBuf,
            .size = BLE_PORT_RXBUF_SIZE, }}},

    {.opcode = UART_OPCODE_SET_ASYNC_FORMAT,
        {.asyncFormat = {
            .databits = UART_DATABITS_8,
            .stopbits = UART_STOPBITS_1,
            .parity = UART_PARITY_NONE,}}},

    {.opcode = UART_OPCODE_SET_HARDWARE_HANDSHAKE,
        {.hardwareHandshake = LPCLIB_YES,}},

    {.opcode = UART_OPCODE_SET_BAUDRATE,
        {.baudrate = 115200,}},

    UART_CONFIG_END
};


static ADF7021_Config radioConfig[] = {
    {.opcode = ADF7021_OPCODE_SET_REFERENCE,
        {.referenceFrequency = 0, }},   /* Will be set at run-time from global config */

    ADF7021_CONFIG_END
};

ADF7021_Handle radio;



#if LPCLIB_DMA
DMA_Handle gpdma;
#endif


/* Ra2 vs Ra2fix dependencies */
GPIO_Pin GPIO_ADF7021_CE;


void BSP_init (void)
{
    GPIO_open();

    /* Check board version. "Fix" has port pin 0.19 grounded.  */
    GPIO_ADF7021_CE = GPIO_ADF7021_CE_org;
    if (GPIO_readBit(GPIO_0_19) == 0) {
        GPIO_ADF7021_CE = GPIO_ADF7021_CE_fix;
    }

    GPIO_setDirBit(GPIO_BLE_MODESEL, ENABLE);
    GPIO_setDirBit(GPIO_BLE_RESET, ENABLE);
    GPIO_setDirBit(GPIO_BLE_AUTORUN, ENABLE);
    GPIO_setDirBit(GPIO_ADF7021_CE, ENABLE);
    GPIO_setDirBit(GPIO_ADF7021_SLE, ENABLE);
    GPIO_setDirBit(GPIO_ENABLE_VDDA, ENABLE);
    GPIO_setDirBit(GPIO_LNA_GAIN, ENABLE);
    GPIO_setDirBit(GPIO_VBAT_ADC_ENABLE, ENABLE);
    GPIO_setDirBit(GPIO_0_23, ENABLE);                  /* Unused pin P0.23, output low */
    GPIO_setDirBit(GPIO_0_24, ENABLE);                  /* Unused pin P0.24, output low */
    GPIO_setDirBit(GPIO_0_25, ENABLE);                  /* Unused pin P0.25, output low */
    GPIO_setDirBit(GPIO_0_26, ENABLE);                  /* Unused pin P0.26, output low */

    GPIO_writeBit(GPIO_BLE_RESET, 0);
    GPIO_writeBit(GPIO_BLE_AUTORUN, 0);
    GPIO_writeBit(GPIO_BLE_MODESEL, 0);
    GPIO_writeBit(GPIO_ENABLE_VDDA, 0);
    GPIO_writeBit(GPIO_LNA_GAIN, 0);
    GPIO_writeBit(GPIO_ADF7021_CE, 0);
    GPIO_writeBit(GPIO_ADF7021_SLE, 0);
    GPIO_writeBit(GPIO_VBAT_ADC_ENABLE, 1);
    GPIO_writeBit(GPIO_0_23, 0);                        /* Unused pin P0.23, output low */
    GPIO_writeBit(GPIO_0_24, 0);                        /* Unused pin P0.24, output low */
    GPIO_writeBit(GPIO_0_25, 0);                        /* Unused pin P0.25, output low */
    GPIO_writeBit(GPIO_0_26, 0);                        /* Unused pin P0.26, output low */

#if LPCLIB_DMA
    DMA_open(DMA0, &gpdma);
#endif

//    CLKPWR_setDivider(CLKPWR_DIVIDER_UART3, CLKPWR_RATIO_1);

    UART_open(BLE_UART, &blePort);
    UART_ioctl(blePort, blePortConfig);

    // TODO need SPI driver
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SPI7);
    LPC_FLEXCOMM7->PSELID = 2;              /* Configure for SPI mode */
    LPC_SPI7->FIFOCFG = 0
                    | (1u << 0)         // SPI_FIFOCFG_ENABLETX_Pos
                    | (1u << 1)         // SPI_FIFOCFG_ENABLERX_Pos
                    | (0  << 12)        // SPI_FIFOCFG_DMATX_Pos
                    | (0  << 13)        // SPI_FIFOCFG_DMARX_Pos
                    ;
    LPC_SPI7->FIFOTRIG = 0
                    | (1u << 0)         // SPI_FIFOTRIG_TXLVLENA_Pos
                    | (1u << 1)         // SPI_FIFOTRIG_RXLVLENA_Pos
                    | (0  << 8)         // SPI_FIFOTRIG_TXLVL_Pos
                    | (0  << 16)        // SPI_FIFOTRIG_RXLVL_Pos
                    ;
    /* Install IRQ handler */
    FLEXCOMM_installHandler(FLEXCOMM7, (FLEXCOMM_IRQHandler_t)ADF7021_handleSpiEvent);
    ADF7021_open(LPC_SPI7, 2, GPIO_1_2, mrt, &radio);
    radioConfig[0].referenceFrequency = config_g->referenceFrequency;
    ADF7021_ioctl(radio, radioConfig);
}


/* Select pin multiplexers (and the electrical characteristics). */
static void BSP_initPins (void)
{
    IOCON_open();

    IOCON_configurePin(PIN_P0_0,  IOCON_makeConfigD(PIN_FUNCTION_0,             /* GPIO_0_0     VBAT_ADC_ENABLE */
                                                    PIN_PULL_NONE,
                                                    PIN_INPUT_NOT_INVERTED,
                                                    PIN_SLEWRATE_NORMAL,
                                                    PIN_FILTER_OFF,
                                                    PIN_OPENDRAIN_ON));
    IOCON_configurePinDefault(PIN_P0_2,  PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_2     BLE_MODESEL */
    IOCON_configurePinDefault(PIN_P0_6,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_6     ENABLE_VDDA */
    IOCON_configurePinDefault(PIN_P0_7,  PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_7     CHARGER_LED1 */
    IOCON_configurePinDefault(PIN_P0_8,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_0_8     BLE_AUTORUN */
    IOCON_configurePinDefault(PIN_P0_9,  PIN_FUNCTION_5, PIN_PULL_REPEATER);    /* FC3_CTS      BLE_CTS */
    IOCON_configurePinDefault(PIN_P0_10, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_10    BLE_EXTRA1 */
    IOCON_configurePinDefault(PIN_P0_11, PIN_FUNCTION_0, PIN_PULL_REPEATER);    /* GPIO_0_11    BLE_EXTRA2 */
    IOCON_configurePinDefault(PIN_P0_12, PIN_FUNCTION_1, PIN_PULL_UP);          /* FC3_RXD      BLE_RXD */
    IOCON_configurePinDefault(PIN_P0_13, PIN_FUNCTION_1, PIN_PULL_NONE);        /* FC3_TXD      BLE_TXD */
    IOCON_configurePinDefault(PIN_P0_14, PIN_FUNCTION_5, PIN_PULL_REPEATER);    /* FC1_SCK      SCLK_D_PDM_CLK */
    IOCON_configurePinDefault(PIN_P0_15, PIN_FUNCTION_1, PIN_PULL_NONE);        /* FC3_RTS      BLE_RTS */
    IOCON_configurePinDefault(PIN_P0_19, PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_19    Ra2 PCB version */
    IOCON_configurePinDefault(PIN_P0_22, PIN_FUNCTION_0, PIN_PULL_UP);          /* GPIO_0_22    FORCE_LOADER */
    IOCON_configurePin(PIN_P0_23, IOCON_makeConfigI(PIN_FUNCTION_0,             /* GPIO_0_23    Used as constant low output */
                                                   PIN_INPUT_INVERTED,
                                                   PIN_I2CMODE_GPIO_4MA,
                                                   PIN_FILTER_ON));
    IOCON_configurePin(PIN_P0_24, IOCON_makeConfigI(PIN_FUNCTION_0,             /* GPIO_0_24    Used as constant low output */
                                                   PIN_INPUT_NOT_INVERTED,
                                                   PIN_I2CMODE_GPIO_4MA,
                                                   PIN_FILTER_ON));
    IOCON_configurePin(PIN_P0_25, IOCON_makeConfigI(PIN_FUNCTION_0,             /* GPIO_0_25    Used as constant low output */
                                                   PIN_INPUT_NOT_INVERTED,
                                                   PIN_I2CMODE_GPIO_4MA,
                                                   PIN_FILTER_ON));
    IOCON_configurePin(PIN_P0_26, IOCON_makeConfigI(PIN_FUNCTION_0,             /* GPIO_0_26    Used as constant low output */
                                                   PIN_INPUT_NOT_INVERTED,
                                                   PIN_I2CMODE_GPIO_4MA,
                                                   PIN_FILTER_ON));
    IOCON_configurePinDefault(PIN_P0_29, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* FC1_MOSI     MOSI_D */
    IOCON_configurePinDefault(PIN_P0_30, PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* FC1_MISO     MISO_D */
    IOCON_configurePinDefault(PIN_P1_0,  PIN_FUNCTION_1, PIN_PULL_REPEATER);    /* PDM0_DATA    SYNC_PDM_DATA */
//    IOCON_configurePinDefault(PIN_P1_1,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* SWO          SWO */
    IOCON_configurePinDefault(PIN_P1_2,  PIN_FUNCTION_0, PIN_PULL_DOWN);        /* GPIO_1_2     MUXOUT */
    IOCON_configurePinDefault(PIN_P1_3,  PIN_FUNCTION_2, PIN_PULL_NONE);        /* FC7_SSEL2    SLE_C */
    if (GPIO_readBit(GPIO_0_19) == 0) {
        /* Ra2fix */
        IOCON_configurePin(PIN_P1_4, IOCON_makeConfigA(PIN_FUNCTION_0,          /* ADC0_7       VBAT_M_ADC */
                                                    PIN_PULL_NONE,
                                                    PIN_INPUT_NOT_INVERTED,
                                                    PIN_ADMODE_ANALOG,
                                                    PIN_FILTER_OFF,
                                                    PIN_OPENDRAIN_OFF));
    }
    else {
        /* Ra2 */
        IOCON_configurePinDefault(PIN_P1_4,  PIN_FUNCTION_0, PIN_PULL_NONE);    /* GPIO_1_4     ADF7021_CE */
    }
        /* Leave ADC input in digital mode until needed for measurement.
         * Pin draws 300µA in analog mode!
         */
    IOCON_configurePin(PIN_P1_5, IOCON_makeConfigA(PIN_FUNCTION_0,              /* ADC0_8       VBAT_P_ADC */
                                                   PIN_PULL_NONE,
                                                   PIN_INPUT_NOT_INVERTED,
                                                   PIN_ADMODE_ANALOG,
                                                   PIN_FILTER_OFF,
                                                   PIN_OPENDRAIN_OFF));
    IOCON_configurePinDefault(PIN_P1_6,  PIN_FUNCTION_2, PIN_PULL_NONE);        /* FC7_SCK      SCLK_C */
    IOCON_configurePinDefault(PIN_P1_7,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* FC7_MOSI     MOSI_C */
    IOCON_configurePinDefault(PIN_P1_8,  PIN_FUNCTION_2, PIN_PULL_REPEATER);    /* FC7_MISO     MISO_C */
    IOCON_configurePinDefault(PIN_P1_9,  PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_9     LNA_GAIN */
    IOCON_configurePinDefault(PIN_P1_11, PIN_FUNCTION_7, PIN_PULL_NONE);        /* USB0_VBUS    VBUS */
    IOCON_configurePinDefault(PIN_P1_12, PIN_FUNCTION_0, PIN_PULL_NONE);        /* GPIO_1_12    BLE_RESET */
    if (GPIO_readBit(GPIO_0_19) == 0) {
        IOCON_configurePinDefault(PIN_P1_15, PIN_FUNCTION_0, PIN_PULL_UP);      /* Ra2fix: pull-up */
    }
    else {
        IOCON_configurePinDefault(PIN_P1_15, PIN_FUNCTION_0, PIN_PULL_NONE);    /* Ra2: high impedance */
    }
    IOCON_configurePinDefault(PIN_P1_17, PIN_FUNCTION_4, PIN_PULL_REPEATER);    /* MCLK         SCLK_D_PDM_CLK */

    if (IOCON_checkErrors() != LPCLIB_SUCCESS) {
        while (1);  //DEBUGGING ONLY
    }
}



/* Port preparation for sleep mode */
void BSP_prepareSleep (void)
{
    GPIO_writeBit(GPIO_LNA_GAIN, 0);        /* LNA off */
    GPIO_writeBit(GPIO_ADF7021_CE, 0);      /* Radio chip off */
    GPIO_writeBit(GPIO_ADF7021_SLE, 0);
    GPIO_writeBit(GPIO_ENABLE_VDDA, 0);     /* VDDA off */
    GPIO_writeBit(GPIO_BLE_AUTORUN, 0);     /* Irrelevant pin once BLE module runs.
                                             * Set low because BL652.SIO_13 has an internal pull-down.
                                             * TODO: Move to BL652 driver
                                             */
    GPIO_writeBit(GPIO_BLE_MODESEL, 0);     /* Irrelevant pin once BLE module runs.
                                             * Set low because BL652.SIO_02 has an internal pull-down.
                                             * TODO: Move to BL652 driver
                                             */

    IOCON_configurePinDefault(PIN_P0_19, PIN_FUNCTION_0, PIN_PULL_DOWN);        /* PCB version sense: pull-down */
    IOCON_configurePinDefault(PIN_P0_29, PIN_FUNCTION_0, PIN_PULL_DOWN);        /* MOSI_D: pull-down */
    IOCON_configurePinDefault(PIN_P1_3,  PIN_FUNCTION_0, PIN_PULL_DOWN);        /* SLE_C: pull-down */
    IOCON_configurePinDefault(PIN_P1_6,  PIN_FUNCTION_0, PIN_PULL_DOWN);        /* SCLK_C: pull-down */
    IOCON_configurePinDefault(PIN_P1_7,  PIN_FUNCTION_0, PIN_PULL_DOWN);        /* MOSI_C: pull-down */
}



/* Port initialization after wakeup */
void BSP_wakeup (void)
{
    BSP_initPins();

    GPIO_writeBit(GPIO_ENABLE_VDDA, 1);     /* VDDA on */
    GPIO_writeBit(GPIO_ADF7021_SLE, 1);
    GPIO_writeBit(GPIO_ADF7021_CE, 1);      /* Radio chip on */
}



/* To be called as the first step in SystemInit(). */
void BSP_systemInit (void)
{
    uint32_t *pFrom;
    uint32_t *pTo;
    int length;


    /* Allow user mode code to pend interrupts via NVIC->STIR */
    SCB->CCR |= SCB_CCR_USERSETMPEND_Msk;

    /* Individual exception handlers for Usage Fault, Bus Fault, MemManage Fault */
    SCB->SHCSR |= 0
               | SCB_SHCSR_USGFAULTENA_Msk
               | SCB_SHCSR_BUSFAULTENA_Msk
               | SCB_SHCSR_MEMFAULTENA_Msk
               ;

    /* Enable the FPU (allow full access to coprocessors CP10 and CP11) */
    SCB->CPACR |= (3 << 2*10) | (3 << 2*11);

    /* Priority of peripheral interrupts depends on the application, and therefore the drivers
     * library does not set the priority.
     * After reset, all interrupts have highest priority, and this can lead to nasty failures
     * if your application uses an RTOS. Therefore it is a good idea to preset all interrupt
     * priorities to a safe default (the lowest priority). You may afterwards raise the
     * priority of individual interrupts as required by the application.
     */
    uint32_t numIrqLines = 32u * (1u + (uint32_t)((SCnSCB->ICTR & SCnSCB_ICTR_INTLINESNUM_Msk) >> SCnSCB_ICTR_INTLINESNUM_Pos));
    if (numIrqLines > 240u) {
        numIrqLines = 240u;
    }
    uint32_t n;
    for (n = 0; n < numIrqLines; n++) {
        NVIC_SetPriority((IRQn_Type)n, (1u << __NVIC_PRIO_BITS) - 1);
    }

    /* Pin initialization opens up complete address range for external flash
     * This is very likely a reuirement for the following scatter loading stage.
     */
    BSP_initPins();

    /* Do a manual scatter loading of vital RAM functions
     * Pins must be initialized already, because we almost certainly need the higher address
     * lines (beyond 16 KiB) to access the load region.
     */
    extern uint32_t Load$$LPCLIB_RAMCODE$$Base;
    extern uint32_t Image$$LPCLIB_RAMCODE$$Base;
    extern uint32_t Image$$LPCLIB_RAMCODE$$Length;

    pFrom = &Load$$LPCLIB_RAMCODE$$Base;
    pTo = &Image$$LPCLIB_RAMCODE$$Base;
    length = (int)&Image$$LPCLIB_RAMCODE$$Length;
    while (length > 0) {    /* The loop copies in multiples of 4! */
        *pTo = *pFrom;
        ++pFrom;
        ++pTo;
        length -= 4;
    }

    SystemCoreClock = 0;
}


/********** Standard Library I/O **********/

#if (defined(__GNUC__))

#include <sys/stat.h>

extern uint32_t __heap_start;

#if !SEMIHOSTING
int _fstat (int fd, struct stat *st);
int _fstat (int fd, struct stat *st)
{
    (void) fd;
    (void) st;

    return -1;
}


int _read (int fd, char *ptr, int len);
int _read (int fd, char *ptr, int len)
{
    (void) fd;

    int bytesUnRead;

    (void) ptr;
    bytesUnRead = 0;

    return len - bytesUnRead;
}


int _write (int fd, const char *ptr, int len);
int _write (int fd, const char *ptr, int len)
{
    (void) fd;

    while (len) {
        ITM_SendChar(*ptr);
        ++ptr;
        --len;
    }

    return 0;
}

int _close (int fd);
int _close (int fd)
{
    (void) fd;

    return 0;
}


int _lseek (int fd, int ptr, int dir);
int _lseek (int fd, int ptr, int dir)
{
    (void)fd;
    (void)ptr;
    (void)dir;

    return 0;
}


int _isatty (int fd);
int _isatty (int fd)
{
    return (fd <= 2) ? 1 : 0;   /* one of stdin, stdout, stderr */
}


caddr_t _sbrk (int incr);
caddr_t _sbrk (int incr)
{
    static caddr_t heap = NULL;
    caddr_t prev_heap;

    if (heap == NULL) {
        heap = (caddr_t)&__heap_start;
    }

    prev_heap = heap;
    heap += incr;

    return prev_heap;
}
#endif


#else


FILE __stdout;
FILE __stdin;


int fputc (int ch, FILE *f)
{
    int cr;

    if (f->_Handle != LPCLIB_INVALID_HANDLE) {
        if (ch == '\n') {
            cr = '\r';
//            UART_write(f->_Handle, &cr, 1);
        }
//        UART_write(f->_Handle, &ch, 1);
    }

    return ch;
}


int getc (FILE *f)
{
    int ch = 0;

    if (f->_Handle != LPCLIB_INVALID_HANDLE) {
//        if (UART_read((UART_Handle)(f->_Handle), &ch, 1) == 1) {
//            return ch;
//        }
    }

    return EOF;
}

int ferror (FILE *f)
{
    return EOF;
}
#endif

