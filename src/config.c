
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "lpclib.h"
#include "config.h"

static const Config_t _factorySettingsDefault = {
    .version = 4,
    .baudrate = 115200.0f,

#if (BOARD_RA == 1)
    .referenceFrequencyFloat = 13.0e6f,
    .referenceFrequency = 13.0e6,
    .rssiCorrectionLnaOn = -16.0,
    .rssiCorrectionLnaOff = 13.0,
#endif

#if (BOARD_RA == 2)
    .usbVID = 0x169C,
    .usbPID = 0x05DC,
    .usbVERSION = 0x0100,

    .referenceFrequencyFloat = 12.8e6f,
    .referenceFrequency = 12.8e6,
    .rssiCorrectionLnaOn = -13.0f,
    .rssiCorrectionLnaOff = 16.0f,
    .vbatTrim = 1.0f,

    .nameBluetooth = "",
    .usbVendorString = L"leckasemmel.de/ra",
    .usbProductString = L"Sondengott Ra",
#endif
};

__SECTION(".config")
static volatile const Config_t _factorySettingsUser;


/* Pointer to default configuration set at link time */
const Config_t *config_g = &_factorySettingsDefault;


/* Calculate the CRC32 checksum of a configuration and validate the configuration */
static LPCLIB_Result _CONFIG_validateChecksum (volatile const Config_t *pConfig)
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
            (void *)pConfig,
            sizeof(*pConfig) - sizeof(pConfig->crc),
            NULL,
            NULL);
        checksum = CRC_read(crc);

        CRC_close(&crc);
    }

    return (checksum == pConfig->crc) ? LPCLIB_SUCCESS : LPCLIB_ERROR;
}



void CONFIG_open (void)
{
    /* Use user supplied configuration if valid, otherwise fall back to factory defaults */
    if (_CONFIG_validateChecksum(&_factorySettingsUser) == LPCLIB_SUCCESS) {
        config_g = (const Config_t *)&_factorySettingsUser;
    }
}

/************** Access functions **************/

float CONFIG_getGeoidHeight (void)
{
    //TODO
    return -49.0f;
}


double CONFIG_getReferenceFrequency (void)
{
    double ref = config_g->referenceFrequency;
    bool valid = config_g->version >= 4;    /* Minimum config version */
    valid = valid && (ref != 0);            /* Checks for all zeros */
    valid = valid && !isnan(ref);           /* Checks for NaN (includes all ones) */

    /* If invalid, default to old (inaccurate) float version. */
    return valid ? ref : config_g->referenceFrequencyFloat;
}


float CONFIG_getVbatTrim (void)
{
    float trim = config_g->vbatTrim;
    bool valid = config_g->version >= 4;    /* Minimum config version */
    valid = valid && (trim != 0);           /* Checks for all zeros */
    valid = valid && !isnan(trim);          /* Checks for NaN (includes all ones) */

    /* If invalid, default to trim=1 */
    return valid ? trim : 1.0f;
}

