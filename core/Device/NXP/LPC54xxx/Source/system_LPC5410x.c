            
#include <stdint.h>

#include "system_LPC5410x.h"



#if defined ( __CC_ARM )        /* ARM Compiler */
__attribute__((section("RW_RAM1"), zero_init))
#else
__attribute__((section(".noinit")))
#endif
uint32_t SystemCoreClock;   /*!< System Clock Frequency (Core Clock)*/


void SystemCoreClockUpdate (void)
{
    /* There is nothing to do here. Updating the core clock "SystemCoreClock"
     * is the responsibility of the application (PDL).
     */
}


#if defined ( __CC_ARM )        /* ARM Compiler */
__weak
#elif defined ( __ICCARM__ )    /* IAR Compiler */
__weak
#endif
void SystemInit (void)
{
}

