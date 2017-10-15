
#ifndef __CONFIG_H
#define __CONFIG_H

#include "lpclib.h"

__PACKED(struct _Config_t {
    /* 0000  0x000 */
    uint16_t version;
    uint16_t serialNumber;

    uint16_t usbVID;
    uint16_t usbPID;
    uint16_t usbVERSION;
    uint16_t __reserved036__;

    /* 0012  0x00C */
    float referenceFrequency;
    float rssiCorrectionLnaOn;
    float rssiCorrectionLnaOff;
    float __reserved018__[26];

    /* 0128  0x080 */
    char nameBluetooth[48];
    wchar_t usbVendorString[64];
    wchar_t usbProductString[64];
    uint8_t __reserved__[588];

    uint32_t crc;
});

typedef struct _Config_t Config_t;

extern const Config_t *config_g;

void CONFIG_open (void);

#endif
