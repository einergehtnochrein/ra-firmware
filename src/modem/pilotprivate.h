
#ifndef __PILOTPRIVATE_H
#define __PILOTPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "pilot.h"
#include "gps.h"



/* A collection of all known packet types */
typedef struct {
    uint8_t packetLength;

    __PACKED(union {
        uint8_t rawData[50];

        __PACKED(struct {
            uint8_t sync[4];        /* First byte (0xAA) uses to allow for bit synchronisation.
                                     * Second to fourth byte (0xAA 0xAA 0x01) used as sync sequence.
                                     */
            __PACKED(struct _PILOT_Payload {
                uint8_t usedSats;   /* Number of used sats */
                uint8_t status;     /* 1=no fix, 2=2D fix, 3=3D fix */
                int32_t latitude;   /* Degrees * 10e6 */
                int32_t longitude;  /* Degrees * 10e6 */
                int32_t altitude;   /* Meters * 10e2 */
                int16_t speedEast;  /* m/s * 10e2 */
                int16_t speedNorth; /* m/s * 10e2 */
                int16_t climbRate;  /* m/s * 10e2 */
                uint32_t daytime;   /* 1000 * (10000*h + 100*m + s) */
                uint32_t date;      /* 10000*d + 100*m + y, year last two digits only (18 = 2018) */
                uint16_t hdop;      /* m * 100 (9998 = invalid) TODO: only a guess... */
                uint16_t vdop;      /* m * 100 (9998 = invalid) TODO:  " */
                uint8_t sats[12];   /* Satellite status (SNR?) */
                uint16_t crc;
            }) payload;
        });
    });
} PILOT_Packet;


typedef struct {
    LLA_Coordinate observerLLA;
    float hdop;
    float vdop;
    uint8_t usedSats;
    uint8_t visibleSats;
} PILOT_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _PILOT_InstanceData {
    struct _PILOT_InstanceData *next;
    uint32_t id;
    char hashName[20];                          /* Hashable sonde name */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    uint16_t frameCounter;

    PILOT_CookedGps gps;
} PILOT_InstanceData;


LPCLIB_Result _PILOT_processConfigBlock (
        const struct _PILOT_Payload *rawPayload,
        PILOT_InstanceData **instancePointer);

LPCLIB_Result _PILOT_processGpsBlock (
        const struct _PILOT_Payload *rawPayload,
        PILOT_CookedGps *cookedGps);

/* Iterate through instances */
bool _PILOT_iterateInstance (PILOT_InstanceData **instance);

/* Remove an instance from the chain */
void _PILOT_deleteInstance (PILOT_InstanceData *instance);

#endif
