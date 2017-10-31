
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "config.h"

static const Config_t _factorySettingsDefault = {
#if (BOARD_RA == 1)
    .referenceFrequency = 13.0e6,
    .rssiCorrectionLnaOn = -16.0,
    .rssiCorrectionLnaOff = 13.0,
#endif

#if (BOARD_RA == 2)
    .usbVID = 0x169C,
    .usbPID = 0x05DC,
    .usbVERSION = 0x0100,

    .referenceFrequency = 12.8e6,
    .rssiCorrectionLnaOn = -13.0,
    .rssiCorrectionLnaOff = 16.0,

    .nameBluetooth = "LAIRD BL652",
    .usbVendorString = L"ra.leckasemmel.de",
    .usbProductString = L"Sondengott Ra",
#endif
};

__SECTION(".config")
static volatile const Config_t _factorySettingsUser;


/* Pointer to default configuration set at link time */
const Config_t *config_g = &_factorySettingsDefault;


/* Calculate the CRC32 checksum of the user configuration */
static uint32_t _CONFIG_getUserChecksum (void)
{
    CRC_Handle crc = LPCLIB_INVALID_HANDLE;
    CRC_Mode crcMode;
    uint32_t checksum = 0;

    crcMode = CRC_makeMode(
            CRC_POLY_CRC32,
            CRC_DATAORDER_NORMAL,
            CRC_SUMORDER_NORMAL,
            CRC_DATAPOLARITY_NORMAL,
            CRC_SUMPOLARITY_NORMAL
            );
    if (CRC_open(crcMode, &crc) == LPCLIB_SUCCESS) {
        CRC_seed(crc, 0xFFFFFFFF);
        CRC_write(
            crc,
            (void *)&_factorySettingsUser,
            sizeof(_factorySettingsUser) - sizeof(_factorySettingsUser.crc),
            NULL,
            NULL);
        checksum = CRC_read(crc);

        CRC_close(&crc);
    }

    return checksum;
}



void CONFIG_open (void)
{
    /* Use user supplied configuration if valid, otherwise fall back to factory defaults */
    if (_CONFIG_getUserChecksum() == _factorySettingsUser.crc) {
        config_g = (const Config_t *)&_factorySettingsUser;
    }
}

/************** Access functions **************/

float CONFIG_getGeoidHeight (void)
{
    //TODO
    return -49.0f;
}

