
#include "app.h"
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5410X
#include "bspra1.h"
#endif
#if LPCLIB_FAMILY == LPCLIB_FAMILY_LPC5411X
#include "bspra2.h"
#endif

#define usb                                 theUsb
#define sspAudio                            ssp0_bus
#define LED                                 LED_GREEN


