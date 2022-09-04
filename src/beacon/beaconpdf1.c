
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "beacon.h"
#include "beaconprivate.h"


#define PDF1_FORMAT_FLAG                        ((25 << 8)  | 1)
#define PDF1_PROTOCOL_FLAG                      ((26 << 8)  | 1)
#define PDF1_COUNTRY_CODE                       ((27 << 8)  | 10)
#define PDF1_USER_PROTOCOL                      ((37 << 8)  | 3)
#define PDF1_LOCATION_PROTOCOL                  ((37 << 8)  | 4)

#define PDF1_USER_EPIRB_ID                      ((40 << 8)  | 36)
#define PDF1_USER_SERIAL_TYPE                   ((40 << 8)  | 3)
#define PDF1_USER_SERIAL_NUMBER                 ((44 << 8)  | 20)
#define PDF1_USER_AVIATION_ID                   ((40 << 8)  | 42)
#define PDF1_USER_AVIATION_AUX_RADIO_LOCATING   ((84 << 8)  | 2)

#define PDF1_LOC_STANDARD_LATITUDE              ((65 << 8)  | 10)
#define PDF1_LOC_STANDARD_LONGITUDE             ((75 << 8)  | 11)

#define PDF1_LOC_NATIONAL_ID                    ((41 << 8)  | 18)
#define PDF1_LOC_NATIONAL_LATITUDE              ((59 << 8)  | 13)
#define PDF1_LOC_NATIONAL_LONGITUDE             ((72 << 8)  | 14)


/* Modified Baudot Code */
static const char _BEACON_modifiedBaudot[64] = {
     0 ,'5', 0 ,'9', 0 , 0 , 0 , 0 , 
     0 , 0 ,'4', 0 ,'8','0', 0 , 0 , 
    '3', 0 , 0 , 0 , 0 ,'6', 0 ,'/', 
    '-','2', 0 , 0 ,'7','1', 0 , 0 , 
     0 ,'T', 0 ,'O',' ','H','N','M', 
     0 ,'L','R','G','I','P','C','V', 
    'E','Z','D','B','S','Y','F','X', 
    'A','W','J', 0 ,'U','Q','K', 0 , 
};


/* Convert modified Baudot into ASCII. */
static bool _BEACON_convertModifiedBaudot2ASCII (const uint8_t *baudot, char *ascii, int nChars)
{
    bool result = true;

    for (int i = 0; i < nChars; i++) {
        int firstBit = (6 * (nChars - 1 - i)) % 8;
        int b = baudot[(6 * (nChars - 1 - i)) / 8] >> firstBit;
        if (firstBit > 2) {
            b |= baudot[(6 * (nChars - 1 - i)) / 8 + 1] << (8 - firstBit);
        }
        b &= 0x3F;

        char c = _BEACON_modifiedBaudot[b];
        if (c == 0) {
            result = false;
        }
        ascii[i] = c;
    }
    ascii[nChars] = 0;

    return result;
}


LPCLIB_Result _BEACON_processPDF1 (BEACON_CookedPDF1 *cooked)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    uint8_t formatFlag;
    uint8_t protocolFlag;
    uint8_t protocol;
    int16_t coordinateCoarse;
    uint8_t id[6];
    char ascii[7+1];
    bool isLocationProtocol = false;

    _BEACON_getField(PDF1_FORMAT_FLAG, &formatFlag);
    cooked->longMessage = (formatFlag != 0);
    _BEACON_getField(PDF1_PROTOCOL_FLAG, &protocolFlag);
    cooked->protocolFlag = protocolFlag;
    _BEACON_getField(PDF1_COUNTRY_CODE, (uint8_t *)&cooked->countryCode);

    if (protocolFlag == 0) {        /* Location Protocols */
        /* It must be a long frame */
        if (formatFlag == 0) {
            return LPCLIB_ILLEGAL_PARAMETER;
        }

        /* Determine protocol type */
        _BEACON_getField(PDF1_LOCATION_PROTOCOL, (uint8_t *)&protocol);
        cooked->protocol = protocol;

        switch (protocol) {
            /* Extract coarse location */
            case PDF1_LOCATION_PROTOCOL_STANDARD_EPIRB:
            case PDF1_LOCATION_PROTOCOL_STANDARD_ELT:
            case PDF1_LOCATION_PROTOCOL_STANDARD_ELT_SERIAL:
            case PDF1_LOCATION_PROTOCOL_STANDARD_ELT_AIRCRAFT_OPERATOR:
            case PDF1_LOCATION_PROTOCOL_STANDARD_EPIRB_SERIAL:
            case PDF1_LOCATION_PROTOCOL_STANDARD_PLB_SERIAL:
            case PDF1_LOCATION_PROTOCOL_STANDARD_SHIP_SECURITY:
                _BEACON_getField(PDF1_LOC_STANDARD_LATITUDE, (uint8_t *)&coordinateCoarse);
                coordinateCoarse <<= 6; /* Shift sign bit into MSB */
                cooked->latitudeCoarse = (coordinateCoarse / 64.0f) * 0.25f;
                _BEACON_getField(PDF1_LOC_STANDARD_LONGITUDE, (uint8_t *)&coordinateCoarse);
                coordinateCoarse <<= 5; /* Shift sign bit into MSB */
                cooked->longitudeCoarse = (coordinateCoarse / 32.0f) * 0.25f;
                isLocationProtocol = true;
                break;

            case PDF1_LOCATION_PROTOCOL_NATIONAL_PLB:
                _BEACON_getField(PDF1_LOC_NATIONAL_ID, (uint8_t *)&cooked->serialNumber);
                _BEACON_getField(PDF1_LOC_NATIONAL_LATITUDE, (uint8_t *)&coordinateCoarse);
                if (coordinateCoarse == 0x0FE0){
                    cooked->latitudeCoarse = NAN;
                }
                else {
                    coordinateCoarse <<= 3; /* Shift sign bit into MSB */
                    cooked->latitudeCoarse = coordinateCoarse / 128.0f;
                    cooked->latitudeCoarse += ((coordinateCoarse >> 3) & 0x1F) / 30.0f;
                }
                _BEACON_getField(PDF1_LOC_NATIONAL_LONGITUDE, (uint8_t *)&coordinateCoarse);
                if (coordinateCoarse == 0x1FE0){
                    cooked->longitudeCoarse = NAN;
                }
                else {
                    coordinateCoarse <<= 2; /* Shift sign bit into MSB */
                    cooked->longitudeCoarse = coordinateCoarse / 128.0f;
                    cooked->longitudeCoarse += ((coordinateCoarse >> 3) & 0x1F) / 30.0f;
                }
                isLocationProtocol = true;
                break;
        }
    }
    else {                          /* User Protocols */
        _BEACON_getField(PDF1_USER_PROTOCOL, (uint8_t *)&protocol);
        cooked->protocol = protocol;

        /* Extract ID */
        cooked->hexID[0] = 0;
        switch (protocol) {
            case PDF1_USER_PROTOCOL_EPIRB_MARITIME:
            case PDF1_USER_PROTOCOL_EPIRB_RADIO_CALL_SIGN:
                _BEACON_getField(PDF1_USER_EPIRB_ID, (uint8_t *)&id);
                break;
            case PDF1_USER_PROTOCOL_AVIATION:
                _BEACON_getField(PDF1_USER_AVIATION_ID, (uint8_t *)&id);
                if (!_BEACON_convertModifiedBaudot2ASCII(id, ascii, 7)) {
                    result = LPCLIB_ILLEGAL_PARAMETER;
                }
                _BEACON_getField(PDF1_USER_AVIATION_AUX_RADIO_LOCATING, &cooked->auxRL);
                break;
            case PDF1_USER_PROTOCOL_SERIAL:
                _BEACON_getField(PDF1_USER_SERIAL_TYPE, &cooked->userSerialType);
                _BEACON_getField(PDF1_USER_SERIAL_NUMBER, (uint8_t *)&cooked->serialNumber);
                break;
            case PDF1_USER_PROTOCOL_TEST:
                break;
            case PDF1_USER_PROTOCOL_ORBITOGRAPHY:
                break;
            case PDF1_USER_PROTOCOL_NATIONAL:
                break;
        }

        isLocationProtocol = true;
    }

    /* Extract 15 Hex ID */
    int i;
    for (i = 0; i < 15; i++) {
        char c;
        _BEACON_getField(((26 + 4 * i) << 8) + 4, (uint8_t *)&c);

        if (isLocationProtocol) {
            switch (i) {
                case 9:     c &= 0x0E; break;
                case 10:    c = 0xF; break;
                case 11:    c = 0xF; break;
                case 12:    c = 0xB; break;
                case 13:    c = 0xF; break;
                case 14:    c = 0xF; break;
            }
        }

        c += '0';
        if (c > '9') {
            c += 7;
        }
        cooked->hexID[i] = c;
    }
    cooked->hexID[15] = 0;

    cooked->isLocationProtocol = isLocationProtocol;

    return result;
}

