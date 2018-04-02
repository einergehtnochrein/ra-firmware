
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "beacon.h"
#include "beaconprivate.h"


#define PDF2_LOC_STANDARD_LATITUDE_DELTA_SIGN   ((113 << 8) | 1)
#define PDF2_LOC_STANDARD_LATITUDE_DELTA_MIN    ((114 << 8) | 5)
#define PDF2_LOC_STANDARD_LATITUDE_DELTA_SEC    ((119 << 8) | 4)
#define PDF2_LOC_STANDARD_LONGITUDE_DELTA_SIGN  ((123 << 8) | 1)
#define PDF2_LOC_STANDARD_LONGITUDE_DELTA_MIN   ((124 << 8) | 5)
#define PDF2_LOC_STANDARD_LONGITUDE_DELTA_SEC   ((129 << 8) | 4)



LPCLIB_Result _BEACON_processPDF2 (const BEACON_CookedPDF1 *pdf1, BEACON_CookedPDF2 *cooked)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    uint8_t sign;
    int8_t min;
    int8_t sec;

    if (pdf1->protocolFlag == 0) {    /* Location Protocols */
        switch (pdf1->protocol) {
            case PDF1_LOCATION_PROTOCOL_STANDARD_EPIRB:
            case PDF1_LOCATION_PROTOCOL_STANDARD_ELT:
            case PDF1_LOCATION_PROTOCOL_STANDARD_ELT_SERIAL:
            case PDF1_LOCATION_PROTOCOL_STANDARD_ELT_AIRCRAFT_OPERATOR:
            case PDF1_LOCATION_PROTOCOL_STANDARD_EPIRB_SERIAL:
            case PDF1_LOCATION_PROTOCOL_STANDARD_PLB_SERIAL:
            case PDF1_LOCATION_PROTOCOL_STANDARD_SHIP_SECURITY:
                _BEACON_getField(PDF2_LOC_STANDARD_LATITUDE_DELTA_SIGN, &sign);
                _BEACON_getField(PDF2_LOC_STANDARD_LATITUDE_DELTA_MIN, (uint8_t *)&min);
                if (sign == 1) {
                    min = -min;
                }
                cooked->latitudeFine = pdf1->latitudeCoarse + min / 60.0f;
                _BEACON_getField(PDF2_LOC_STANDARD_LATITUDE_DELTA_SEC, (uint8_t *)&sec);
                if (sign == 1) {
                    sec = -sec;
                }
                cooked->latitudeFine += sec / (60.0 * 15.0);

                _BEACON_getField(PDF2_LOC_STANDARD_LONGITUDE_DELTA_SIGN, &sign);
                _BEACON_getField(PDF2_LOC_STANDARD_LONGITUDE_DELTA_MIN, (uint8_t *)&min);
                if (sign == 1) {
                    min = -min;
                }
                cooked->longitudeFine = pdf1->longitudeCoarse + min / 60.0f;
                _BEACON_getField(PDF2_LOC_STANDARD_LONGITUDE_DELTA_SEC, (uint8_t *)&sec);
                if (sign == 1) {
                    sec = -sec;
                }
                cooked->longitudeFine += sec / (60.0 * 15.0);
                break;
        }
    }
    else {                          /* User Protocols */
    }

    return result;
}

