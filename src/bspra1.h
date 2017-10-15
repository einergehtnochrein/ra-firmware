#ifndef __BSP_H
#define __BSP_H

#include "lpclib.h"

/* GPIO functions */
#define GPIO_BLE_RESET                      GPIO_1_11
#define GPIO_BLE_AUTORUN                    GPIO_0_7
#define GPIO_BLE_RXD                        GPIO_0_0
#define GPIO_POWER_SWITCH                   GPIO_1_10
#define GPIO_BUTTON                         GPIO_1_9
#define GPIO_SSEL_DISP                      GPIO_1_4
#define GPIO_ENABLE_DISP                    GPIO_0_6
#define GPIO_ADF7021_CE                     GPIO_1_3
#define GPIO_ADF7021_SLE                    GPIO_1_5
#define GPIO_LNA_GAIN                       GPIO_0_19
#define GPIO_DEBUG1                         GPIO_0_22
#define GPIO_RX_CLK                         GPIO_0_11
#define GPIO_RX_DATA                        GPIO_0_12
#if PATCHLEVEL >= 1
#  define GPIO_ENABLE_VDDA                  GPIO_0_30
#else
#  define GPIO_ENABLE_VDDA                  GPIO_0_14
#endif


/* Sample rate for DSP processing (e.g. AFSK modem) */
#define DSP_SAMPLERATE                      16169.154f


extern UART_Handle blePort;


#include "adf7021.h"
extern ADF7021_Handle radio;


#if LPCLIB_DMA
extern DMA_Handle gpdma;
#endif


#define BSP_TIMER_GUI                       TIMER2
#define BSP_TIMER_SCOPE                     TIMER3


void BSP_init (void);

/** To be called as the first step in SystemInit(). */
void BSP_systemInit (void);

/** Port preparation for sleep mode */
void BSP_prepareSleep (void);

/** Port initialization after wakeup */
void BSP_wakeup (void);



#endif

