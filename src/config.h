
#ifndef __CONFIG_H
#define __CONFIG_H

#include "lpclib.h"

/* <v> indicates parameter was introduced in config version v. */

__PACKED(struct _Config_t {
    /* 0000  0x000 */
    uint16_t version;
    uint16_t serialNumber;

    uint16_t usbVID;
    uint16_t usbPID;
    uint16_t usbVERSION;
    uint16_t __reserved00A__;

    /* 0012  0x00C */
    float referenceFrequencyFloat;
    float rssiCorrectionLnaOn;
    float rssiCorrectionLnaOff;
    float vbatTrim;                     /* VBAT measurement correction factor. <4>
                                         * Default: 1.0f
                                         */
    float __reserved01C__[24];
    uint32_t baudrate;

    /* 0128  0x080 */
    char nameBluetooth[48];
    wchar_t usbVendorString[64];
    wchar_t usbProductString[64];
    uint8_t __reserved1B0__[80];

    /* 0512  0x200 */
    uint16_t att_mtu;
    uint16_t att_data_length;
    uint16_t max_packet_length;
    uint8_t __reserved200__[494];

    /* 1012  0x3F4 */
    double referenceFrequency;          /* 'double' version of referenceFrequencyFloat. The old
                                         * float version lacks the required accuracy. <4>
                                         */

    uint32_t crc;
});

typedef struct _Config_t Config_t;

extern const Config_t *config_g;

void CONFIG_open (void);

/* Access functions (should replace direct structure access */
float CONFIG_getGeoidHeight (void);
float CONFIG_getVbatTrim (void);
double CONFIG_getReferenceFrequency (void);

#endif
