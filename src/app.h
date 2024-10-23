#ifndef __APP_H
#define __APP_H

#include "bl652.h"
#include "ephemupdate.h"
#include "scanner.h"
#include "sys.h"

#define FIRMWARE_VERSION_MAJOR              56
#define FIRMWARE_VERSION_MINOR              4
#if !defined(FIRMWARE_NAME)
#  define FIRMWARE_NAME                     ""
#endif

#if (BOARD_RA == 1)
#define FIRMWARE_START_ADDRESS              0x00008000
#define FIRMWARE_END_ADDRESS                0x0006FFF0
#endif
#if (BOARD_RA == 2)
#define FIRMWARE_START_ADDRESS              0x00008000
#define FIRMWARE_END_ADDRESS                0x00037FF0
#endif


// Resources
#if (BOARD_RA == 2)
#define AUDIO_ADAPTER                       TIMER2
#define AUDIO_ADAPTER_CLOCK                 CLKPWR_CLOCK_TIMER2
#endif


// Define what hardware modifications have been done
// Ra1:
// 0: no change
// 1: CPU pin 49 disconnected from regulator (top trace cut)
//    CPU pin 49 connected to CPU pin 5
//    Regulator U7.EN connected to test pad TP6
#define PATCHLEVEL          1


// Name of the smartBASIC script inside BL652.
// If this script exists, it will be started to replace the built-in VSP handler.
#define SMARTBASIC_SCRIPT                   "ra_power"

extern EPHEMUPDATE_Handle euTask;
extern SCANNER_Handle scanner;
extern SYS_Handle sys;
extern SONDE_Handle sonde;
extern MRT_Handle mrt;
extern BL652_Handle ble;

#endif
