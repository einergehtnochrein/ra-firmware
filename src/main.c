
#include <malloc.h>
#include <stdio.h>
#include <string.h>

#include "lpclib.h"
#include "bsp.h"
#include "app.h"
#include "bl652.h"
#include "mon.h"
#include "scanner.h"
#include "sys.h"
#include "ephemupdate.h"
#include "config.h"
#if (BOARD_RA == 2)
#include "usbuser_config.h"
#endif


MON_Handle monTask;
SYS_Handle sys;
SCANNER_Handle scanner;
SONDE_Handle sonde;
EPHEMUPDATE_Handle euTask;
MRT_Handle mrt;
BL652_Handle ble;

char s[100];
volatile struct mallinfo heapinfo;

int main (void)
{
    SystemCoreClock = CLKPWR_getBusClock(CLKPWR_CLOCK_CPU);

    osKernelInitialize();
    osKernelStart();

    CONFIG_open();

    //TODO: need mailbox driver
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_MAILBOX);
    MRT_open(&mrt);

    BSP_init();

    if (BL652_open(blePort, GPIO_BLE_AUTORUN, GPIO_BLE_MODESEL, GPIO_BLE_RESET, &ble) == LPCLIB_SUCCESS) {
        if (BL652_findBaudrate(ble) == LPCLIB_SUCCESS) {
            if (BL652_readParameters(ble) == LPCLIB_SUCCESS) {
#if (BOARD_RA == 2)
                if (BL652_updateParameters(ble)) {
                }
#endif
            }
        }

        _Bool hasPowerScript = false;
        BL652_hasPowerScript(ble, &hasPowerScript);
        if (hasPowerScript) {
            BL652_setMode(ble, BL652_MODE_COMMAND);
            BL652_runScript(ble, SMARTBASIC_SCRIPT);
        } else {
            BL652_setMode(ble, BL652_MODE_VSP_BRIDGE);
        }
    }

#if (BOARD_RA == 1)
    GPIO_writeBit(GPIO_ENABLE_VDDA, 1);
    GPIO_writeBit(GPIO_ADF7021_CE, 1);
    GPIO_writeBit(GPIO_ENABLE_DISP, 1);
#endif
#if (BOARD_RA == 2)
    GPIO_writeBit(GPIO_ENABLE_VDDA, 1);
    GPIO_writeBit(GPIO_ADF7021_CE, 1);
#endif

    SYS_open(&sys);
    SCANNER_open(&scanner);
    SONDE_open(&sonde);
    EPHEMUPDATE_open(&euTask);
#if (BOARD_RA == 2)
    USBUSER_open();
#endif
    MON_open(&monTask);

    /* Prepare M0 */
    extern uint32_t M0IMAGE_start;
    LPC_SYSCON->CPUCTRL = (LPC_SYSCON->CPUCTRL | 0xC0C40000) | (1 << 3) | (1 << 5);
    LPC_SYSCON->CPSTACK = ((volatile uint32_t *)&M0IMAGE_start)[0];
    LPC_SYSCON->CPBOOT = ((volatile uint32_t *)&M0IMAGE_start)[1];
    LPC_SYSCON->CPUCTRL = ((LPC_SYSCON->CPUCTRL | 0xC0C40000) | (1 << 3)) & ~(1 << 5);

    while (1) {
        heapinfo = mallinfo();
        SYS_thread(sys);
        SCANNER_thread(scanner);
        EPHEMUPDATE_thread(euTask);
#if (BOARD_RA == 2)
        USBUSER_worker();
#endif
        MON_thread(monTask);
        __WFI();
    }
}



void SystemInit (void)
{
    BSP_systemInit();

#if (BOARD_RA == 1)
    CLKPWR_setCpuClock(48000000, CLKPWR_CLOCK_IRC);
#endif
#if (BOARD_RA == 2)
    CLKPWR_setCpuClock(48000000, CLKPWR_CLOCK_FRO12);
#endif

    /* Enable Asynchronous APB bus */
    LPC_SYSCON->ASYNCAPBCTRL = 1;           /* on */
#if (BOARD_RA == 1)
    LPC_ASYNCSYSCON->ASYNCCLKDIV = 1;       /* Don't divide */
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELA = 0;   /* IRC */
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELB = 3;   /* Use CLKSELA */
#endif
#if (BOARD_RA == 2)
    LPC_ASYNCSYSCON->ASYNCAPBCLKSELA = 0;   /* FRO12M */
#endif

    /* Enable RAM blocks */
#if (BOARD_RA == 1)
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_FIFO);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_FRG0);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM1);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM2);
#endif
#if (BOARD_RA == 2)
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM1);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_SRAM2);
    CLKPWR_enableClock(CLKPWR_CLOCKSWITCH_USB);
    CLKPWR_unitPowerUp(CLKPWR_UNIT_USBPAD);
//    CLKPWR_setUsbClock();
#endif

#if (BOARD_RA == 1)
    /* Prepare system FIFO */
    LPC_FIFO->FIFOCFGUSART0 = 0x00000404;
    LPC_FIFO->FIFOUPDATEUSART = 0x000F000F;
    LPC_FIFO->FIFOCFGSPI1 = 0x00000404;
    LPC_FIFO->FIFOUPDATESPI = 0x00030003;
    LPC_FIFO->FIFOCTLSPI = 0;                           /* Allow SPI FIFO operations */
    LPC_SYSCON->FIFOCTRL = 0x00002020;                  /* SPI1 uses system FIFO */

    /* Fractional divider for UART */
    LPC_ASYNCSYSCON->FRGCTRL = (0 << 8) | (255 << 0);   /* No FRAC, i.e. UART clock = 12 MHz */
#endif
#if (BOARD_RA == 2)
    LPC_SYSCON->FCLKSEL[3] = 0;                         /* FLEXCOMM3 clock (BLE UART) = FRO12M */
    LPC_SYSCON->FCLKSEL[7] = 0;                         /* FLEXCOMM7 clock (Radio Control) = FRO12M */
    LPC_SYSCON->DMICCLKSEL = 3;                         /* MCLK pin */
    LPC_SYSCON->DMICCLKDIV = 0;
    LPC_SYSCON->USBCLKSEL = 0;                          /* USB clock = FROHF */
    LPC_SYSCON->USBCLKDIV = 0;                          /* USB clock divider = 1 (FROHF = 48 MHz) */
    LPC_SYSCON->FROCTRL |= (1u << 24) | (1u << 30);     /* USBCLKADJ=1, enable FROHF */
#endif

#if (BOARD_RA == 1)
    NVIC_EnableIRQ(PIN_INT2_IRQn);
    NVIC_EnableIRQ(MAILBOX_IRQn);
    NVIC_EnableIRQ(UART0_IRQn);
    NVIC_EnableIRQ(UART2_IRQn);
    NVIC_EnableIRQ(SPI1_IRQn);
    NVIC_EnableIRQ(MRT_IRQn);
#endif
#if (BOARD_RA == 2)
    NVIC_EnableIRQ(PIN_INT0_IRQn);
    NVIC_EnableIRQ(PIN_INT2_IRQn);
    NVIC_EnableIRQ(MAILBOX_IRQn);
    NVIC_EnableIRQ(UART3_IRQn);
    NVIC_EnableIRQ(SPI7_IRQn);
    NVIC_SetPriority(USB_IRQn, 6);     //TODO: Only a workaround. Don't do DSP in IRQ38!
    NVIC_EnableIRQ(USB_IRQn);
    NVIC_EnableIRQ(MRT_IRQn);
#endif
}



#if defined(__GNUC__)
void HardFault_Handler (void)
{
    /* Ignore debug events if not in debug mode */
    uint32_t sp = __get_MSP();
    uint32_t pc = ((volatile uint32_t *)sp)[6];
    uint16_t instr = ((volatile uint16_t *)pc)[0];
    if ((instr >> 8) == 0xBE) {
        /* BKPT instruction. Skip it. */
        ((volatile uint32_t *)sp)[6] = pc + 2;
        return;
    }

    __asm (
    "_HardFault_loop:                           \n\t"
    "   b _HardFault_loop                       \n\t"
    "   bx r14                                  \n\t"
    );
}


void BusFault_Handler (void)
{
    __asm (
    "_BusFault_loop:                            \n\t"
    "   b _BusFault_loop                        \n\t"
    "   bx r14                                  \n\t"
    );
}
#endif

