
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

#define PDF2_LOC_USER_LATITUDE_SIGN             ((108 << 8) | 1)
#define PDF2_LOC_USER_LATITUDE_DEG              ((109 << 8) | 7)
#define PDF2_LOC_USER_LATITUDE_MIN              ((116 << 8) | 4)
#define PDF2_LOC_USER_LONGITUDE_SIGN            ((120 << 8) | 1)
#define PDF2_LOC_USER_LONGITUDE_DEG             ((121 << 8) | 8)
#define PDF2_LOC_USER_LONGITUDE_MIN             ((129 << 8) | 4)



LPCLIB_Result _BEACON_processPDF2 (const BEACON_CookedPDF1 *pdf1, BEACON_CookedPDF2 *cooked)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    uint8_t sign;
    uint8_t deg;
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
                cooked->latitude = pdf1->latitudeCoarse + min / 60.0f;
                _BEACON_getField(PDF2_LOC_STANDARD_LATITUDE_DELTA_SEC, (uint8_t *)&sec);
                if (sign == 1) {
                    sec = -sec;
                }
                cooked->latitude += sec / (60.0 * 15.0);

                _BEACON_getField(PDF2_LOC_STANDARD_LONGITUDE_DELTA_SIGN, &sign);
                _BEACON_getField(PDF2_LOC_STANDARD_LONGITUDE_DELTA_MIN, (uint8_t *)&min);
                if (sign == 1) {
                    min = -min;
                }
                cooked->longitude = pdf1->longitudeCoarse + min / 60.0f;
                _BEACON_getField(PDF2_LOC_STANDARD_LONGITUDE_DELTA_SEC, (uint8_t *)&sec);
                if (sign == 1) {
                    sec = -sec;
                }
                cooked->longitude += sec / (60.0 * 15.0);
                break;
        }
    }
    else {                          /* User Protocols */
        switch (pdf1->protocol) {
            case PDF1_USER_PROTOCOL_SERIAL:
                _BEACON_getField(PDF2_LOC_USER_LATITUDE_SIGN, &sign);
                _BEACON_getField(PDF2_LOC_USER_LATITUDE_DEG, &deg);
                cooked->latitude = deg;
                _BEACON_getField(PDF2_LOC_USER_LATITUDE_MIN, (uint8_t *)&min);
                cooked->latitude += (min / 15.0f);
                if (sign == 1) {
                    cooked->latitude = -cooked->latitude;
                }

                _BEACON_getField(PDF2_LOC_USER_LONGITUDE_SIGN, &sign);
                _BEACON_getField(PDF2_LOC_USER_LONGITUDE_DEG, &deg);
                cooked->longitude = deg;
                _BEACON_getField(PDF2_LOC_USER_LONGITUDE_MIN, (uint8_t *)&min);
                cooked->longitude += (min / 15.0f);
                if (sign == 1) {
                    cooked->longitude = -cooked->longitude;
                }
                break;
        }
    }

    return result;
}

