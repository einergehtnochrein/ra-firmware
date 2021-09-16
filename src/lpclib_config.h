#ifndef __LPCLIB_CONFIG_H__
#define __LPCLIB_CONFIG_H__

#define LPCLIB_CLKPWR_POWER_PROFILES                    1

#define LPCLIB_GPIO_INTERRUPTS                          1
#define LPCLIB_TIMER                                    1
#define LPCLIB_I2C                                      0
#define LPCLIB_SSP                                      0
#define LPCLIB_ADC                                      0

#define SEMIHOSTING                                     0
#define SEMIHOSTING_C34                                 (SEMIHOSTING && 0)
#define SEMIHOSTING_DFM                                 (SEMIHOSTING && 0)
#define SEMIHOSTING_IMET                                (SEMIHOSTING && 1)
#define SEMIHOSTING_RS41                                (SEMIHOSTING && 0)
#define SEMIHOSTING_RS92                                (SEMIHOSTING && 0)
#define T32_TERM_BLOCKED
#define T32_TERM_MEMORY_BLOCKED_SIZE                    128

#endif /* __LPCLIB_CONFIG_H__ */

