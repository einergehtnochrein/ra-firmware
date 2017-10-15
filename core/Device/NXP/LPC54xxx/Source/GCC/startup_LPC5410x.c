
#include "LPC5410x.h"

/* External functions */
extern int main (void);
__attribute__((weak)) void software_init_hook (void);   /* NEWLIB! */
extern void SystemInit (void);

/* These symbols are defined by the linker script */
extern uint32_t __stack_end__;
extern __attribute__((weak)) void __vector_checksum__ (void);


/* Default interrupt service routines.
 * Defined as weak symbols. Can be redefined in the application.
 */
#define ALIAS(def) __attribute__((weak,alias(#def)))

static void _NMI_Handler (void);
static void _HardFault_Handler (void);
static void _MPU_Handler (void);
static void _BusFault_Handler (void);
static void _UsageFault_Handler (void);
static void _SVC_Handler (void);
static void _DebugMon_Handler (void);
static void _PendSV_Handler (void);
static void _SysTick_Handler (void);
static void Default_IRQHandler (void);

void Reset_Handler (void);
void NMI_Handler (void) ALIAS(_NMI_Handler);
void HardFault_Handler (void) ALIAS(_HardFault_Handler);
void MPU_Handler (void) ALIAS(_MPU_Handler);
void BusFault_Handler (void) ALIAS(_BusFault_Handler);
void UsageFault_Handler (void) ALIAS(_UsageFault_Handler);
void SVC_Handler (void) ALIAS(_SVC_Handler);
void DebugMon_Handler (void) ALIAS(_DebugMon_Handler);
void PendSV_Handler (void) ALIAS(_PendSV_Handler);
void SysTick_Handler (void) ALIAS(_SysTick_Handler);

void WDT_IRQHandler (void) ALIAS(Default_IRQHandler);
void BOD_IRQHandler (void) ALIAS(Default_IRQHandler);
void DMA_IRQHandler (void) ALIAS(Default_IRQHandler);
void GINT0_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT0_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT1_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT2_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT3_IRQHandler (void) ALIAS(Default_IRQHandler);
void UTICK_IRQHandler (void) ALIAS(Default_IRQHandler);
void MRT_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER0_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER1_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER2_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER3_IRQHandler (void) ALIAS(Default_IRQHandler);
void TIMER4_IRQHandler (void) ALIAS(Default_IRQHandler);
void SCT0_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART0_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART1_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART2_IRQHandler (void) ALIAS(Default_IRQHandler);
void UART3_IRQHandler (void) ALIAS(Default_IRQHandler);
void PWM1_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C0_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C1_IRQHandler (void) ALIAS(Default_IRQHandler);
void I2C2_IRQHandler (void) ALIAS(Default_IRQHandler);
void SPI0_IRQHandler (void) ALIAS(Default_IRQHandler);
void SPI1_IRQHandler (void) ALIAS(Default_IRQHandler);
void ADC0_SEQA_IRQHandler (void) ALIAS(Default_IRQHandler);
void ADC0_SEQB_IRQHandler (void) ALIAS(Default_IRQHandler);
void ADC0_THCMP_IRQHandler (void) ALIAS(Default_IRQHandler);
void RTC_IRQHandler (void) ALIAS(Default_IRQHandler);
void EZH_IRQHandler (void) ALIAS(Default_IRQHandler);
void MAILBOX_IRQHandler (void) ALIAS(Default_IRQHandler);

void GINT1_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT4_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT5_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT6_IRQHandler (void) ALIAS(Default_IRQHandler);
void PIN_INT7_IRQHandler (void) ALIAS(Default_IRQHandler);
void RIT_IRQHandler (void) ALIAS(Default_IRQHandler);



/** Vector table. */
void (*vector_table[])(void) __attribute__ ((section(".vectors"))) =
{
    (void (*)(void))&__stack_end__,
    Reset_Handler,
    NMI_Handler,
    HardFault_Handler,
    MPU_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    __vector_checksum__,
    0,
    0,
    0,
    SVC_Handler,
    DebugMon_Handler,
    0,
    PendSV_Handler,
    SysTick_Handler,

    WDT_IRQHandler,
    BOD_IRQHandler,
    0,
    DMA_IRQHandler,
    GINT0_IRQHandler,
    PIN_INT0_IRQHandler,
    PIN_INT1_IRQHandler,
    PIN_INT2_IRQHandler,
    PIN_INT3_IRQHandler,
    UTICK_IRQHandler,
    MRT_IRQHandler,
    TIMER0_IRQHandler,
    TIMER1_IRQHandler,
    TIMER2_IRQHandler,
    TIMER3_IRQHandler,
    TIMER4_IRQHandler,

    SCT0_IRQHandler,
    UART0_IRQHandler,
    UART1_IRQHandler,
    UART2_IRQHandler,
    UART3_IRQHandler,
    I2C0_IRQHandler,
    I2C1_IRQHandler,
    I2C2_IRQHandler,
    SPI0_IRQHandler,
    SPI1_IRQHandler,
    ADC0_SEQA_IRQHandler,
    ADC0_SEQB_IRQHandler,
    ADC0_THCMP_IRQHandler,
    RTC_IRQHandler,
    EZH_IRQHandler,
    MAILBOX_IRQHandler,

    /* Next vectors exist on M4 only */
    GINT1_IRQHandler,
    PIN_INT4_IRQHandler,
    PIN_INT5_IRQHandler,
    PIN_INT6_IRQHandler,
    PIN_INT7_IRQHandler,
    0,
    0,
    0,
    RIT_IRQHandler,
};



/* If this code is running on the slave processor, check if stack and start address
 * have been set by the master processor. If yes, branch there. If not, go to sleep.
 */
#define SYSCON_CPUCTRL_MASTERCPU_Pos        0
#define SYSCON_CPUCTRL_MASTERCPU_Msk        (1u << SYSCON_CPUCTRL_MASTERCPU_Pos)
#define SYSCON_CPUCTRL_CM4CLKEN_Pos         2
#define SYSCON_CPUCTRL_CM4CLKEN_Msk         (1u << SYSCON_CPUCTRL_CM4CLKEN_Pos)
#define SYSCON_CPUCTRL_CM0CLKEN_Pos         3
#define SYSCON_CPUCTRL_CM0CLKEN_Msk         (1u << SYSCON_CPUCTRL_CM0CLKEN_Pos)
#define SYSCON_CPUCTRL_CM4RSTEN_Pos         4
#define SYSCON_CPUCTRL_CM4RSTEN_Msk         (1u << SYSCON_CPUCTRL_CM4RSTEN_Pos)
#define SYSCON_CPUCTRL_CM0RSTEN_Pos         5
#define SYSCON_CPUCTRL_CM0RSTEN_Msk         (1u << SYSCON_CPUCTRL_CM0RSTEN_Pos)
#define SYSCON_CPUCTRL_POWERCPU_Pos         6
#define SYSCON_CPUCTRL_POWERCPU_Msk         (1u << SYSCON_CPUCTRL_POWERCPU_Pos)
#define SYSCON_CPUCTRL_MAGIC                0xC0C40000

extern uint32_t __scattertable_start__;


void _boot_main (void);

__attribute__ ((section(".bootstrap")))
void _boot_main (void)
{
    uint32_t *pScatter;
    uint32_t scatterSource;
    uint32_t scatterDest;
    uint32_t scatterSize;


    SCB->VTOR = (uint32_t)vector_table;

    /* Init the clock tree */
    SystemInit();

    /* Scatter loading.
     * The scatter table contains three types of entries, marked by the first word of the entry:
     * 0 = end of scatter tabel
     * 1 = BSS section (initialize with zero)
     * 2 = DATA section. Initialze with non-zero values from flash.
     *
     * 1: Followed by two words for (1) start address and (b) length in bytes
     * 2. Followed by three words for (1) source address, (2) destination address,
     *    and (3) length in bytes
     */
//TODO Can we do word-wise copy?
    pScatter = &__scattertable_start__;
    while (*pScatter != 0) {
        switch (*pScatter++) {
        case 1:                             /* BSS section */
            scatterDest = *pScatter++;
            scatterSize = *pScatter++;
            while (scatterSize) {
                *((volatile uint8_t *)scatterDest) = 0;
                ++scatterDest;
                --scatterSize;
            }
            break;

        case 2:                             /* DATA section */
            scatterSource = *pScatter++;
            scatterDest = *pScatter++;
            scatterSize = *pScatter++;
            while (scatterSize) {
                *((volatile uint8_t *)scatterDest) = *((volatile uint8_t *)scatterSource);
                ++scatterSource;
                ++scatterDest;
                --scatterSize;
            }
            break;

        default:
            /* Illegal section type. Stop scatter loading. */
            break;
        }
    }

    /* Call a NEWLIB hook (if defined). Used by RTX to start main() as a task.
     * NOTE: If not defined, the linker will replace this call with a NOP!
     */
    software_init_hook();

    /* Now we can jump to the application entry point main() */
    main();

    /* Loop here forever, in case main() ever returns. */
    while (1)
        ;
}



/* If this file is part of the master image, it will be used by both M4 and M0+
 * processors! Both processors will therefore see the same start address and initial SP!
 * We must declare this function as naked (no stackframe) to ensure that the slave processor
 * does not mess up the master's stack.
 * Inside we use assembler code to have full control over the instruction set: Although usually
 * compiled for M4 (the standard master processor), the very same code will be executed by
 * the M0+ as well until we can decide whether or not to boot into the slave image.
 *
 * NOTE: Register addresses for CPUCTRL, CPSTACK, CPBOOT should not be formed by using a base
 * address (SYSCON, 0x40000000), and then an offset when using a LDR instruction.
 * The problem is that when using "ldr r0,=0x40000000", the M4 compiler might actually
 * optimize this into a "mov" instruction, which is not available on M0+. Using the full peripheral
 * register addresses (as in "ldr r0,=0x40000300"), the compiler has no such chance to optimize
 * for M4, and is forced to use "ldr" with literal, which the M0+ understands as well.
 */
__attribute__ ((naked, section(".bootstrap")))
void Reset_Handler (void)
{
    asm volatile (

#if defined(CHECK_SLAVE_BOOT)
        ".set       SCB_CPUID, 0xE000ED00\t\n"
        ".set       SCB_CPUID_PARTNO_Msk, 0xFF0FFFF0\t\n"
        ".set       M0PLUS_PARTNO, 0x410CC600\t\n"

        ".set       SYSCON_CPUCTRL, 0x40000300\t\n"
        ".set       SYSCON_CPUCTRL_MASTERCPU_Msk, 0x00000001\t\n"
        ".set       SYSCON_CPBOOT, 0x40000304\t\n"
        ".set       SYSCON_CPSTACK, 0x40000308\t\n"

        /* Preload CPUCTRL register */
        "ldr        r3, =SYSCON_CPUCTRL\t\n"
        "ldr        r3, [r3]\t\n"
        "movs       r2, #SYSCON_CPUCTRL_MASTERCPU_Msk\t\n"

        /* Is this the M0+ processor? */
        "ldr        r0, =SCB_CPUID\t\n"
        "ldr        r0, [r0]\t\n"
        "ldr        r1, =SCB_CPUID_PARTNO_Msk\t\n"
        "ands       r0, r1\t\n"
        "ldr        r1, =M0PLUS_PARTNO\t\n"
        "cmp        r0, r1\t\n"
        "beq.n      boot_m0\t\n"

    "boot_m4:\t\n"
        "ands       r3, r2\t\n"
        "bne.n      regular_boot\t\n"
        "b.n        check_slave_setup\t\n"

    "boot_m0:\t\n"
        "ands       r3, r2\t\n"
        "beq.n      regular_boot\t\n"

    "check_slave_setup:\t\n"
        "ldr        r0, =SYSCON_CPBOOT\t\n"
        "ldr        r0, [r0]\t\n"
        "ands       r0, r0\t\n"
        "beq.n      stop_boot\t\n"
        "ldr        r1, =SYSCON_CPSTACK\t\n"
        "ldr        r1, [r1]\t\n"
        "ands       r1, r1\t\n"
        "bne.n      slave_boot\t\n"

    "stop_boot:\t\n"
        "wfi\t\n"
        "b.n        stop_boot\t\n"

    "slave_boot:\t\n"
        "msr        msp, r1\t\n"
        "bx         r0\t\n"

    "regular_boot:\t\n"
#endif

        /* Continue with regular boot */
        "ldr     r3, =_boot_main\t\n"
        "bx      r3\t\n"
    );
}



/*------ Core exception handlers --------------------------------------------*/

__attribute__ ((section(".bootstrap")))
void _NMI_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _HardFault_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _MPU_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _BusFault_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _UsageFault_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _SVC_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _DebugMon_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _PendSV_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void _SysTick_Handler (void)
{
    while (1)
        ;
}

__attribute__ ((section(".bootstrap")))
void Default_IRQHandler (void)
{
    while (1)
        ;
}

/*------ NEWLIB stubs ------------------------------------------------*/

extern void _exit (int returnCode);
__attribute__((noreturn)) void _exit (int returnCode)
{
    (void) returnCode;

    while (1)
        ;
}

extern void _init (void);
void _init (void)
{
}

extern void _fini (void);
void _fini (void)
{
}

