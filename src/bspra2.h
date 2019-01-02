#ifndef __BSP_H
#define __BSP_H

#include "lpclib.h"

/* GPIO functions */
#define GPIO_ADF7021_CE_org                 GPIO_1_4
#define GPIO_ADF7021_CE_fix                 GPIO_0_1
#define GPIO_ADF7021_SLE                    GPIO_1_3
#define GPIO_BLE_AUTORUN                    GPIO_0_8
#define GPIO_BLE_MODESEL                    GPIO_0_2
#define GPIO_BLE_RESET                      GPIO_1_12
#define GPIO_BLE_RXD                        GPIO_0_12
#define GPIO_ENABLE_VDDA                    GPIO_0_6
#define GPIO_LNA_GAIN                       GPIO_1_9
#define GPIO_VBAT_ADC_ENABLE                GPIO_0_0
#define GPIO_DETECT_RA2FIX                  GPIO_0_19

extern GPIO_Pin GPIO_ADF7021_CE;


/* Sample rate for DSP processing (e.g. AFSK modem) */
#define DSP_SAMPLERATE                      16000.0f


extern UART_Handle blePort;

#include "adf7021.h"
extern ADF7021_Handle radio;


#if LPCLIB_DMA
extern DMA_Handle gpdma;
#endif


void BSP_init (void);

/** To be called as the first step in SystemInit(). */
void BSP_systemInit (void);

/** Port preparation for sleep mode */
void BSP_prepareSleep (void);

/** Port initialization after wakeup */
void BSP_wakeup (void);



#endif

