
#ifndef __RS92PRIVATE_H
#define __RS92PRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "rs92.h"
#include "gps.h"


#define RS92_CLIMB_HISTORY_LENGTH       10

typedef enum {
    RS92_SUBFRAME_CALIB_CONFIG = 'e',
    RS92_SUBFRAME_METROLOGY = 'i',
    RS92_SUBFRAME_GPS = 'g',
    RS92_SUBFRAME_AUX = 'h',
    RS92_SUBFRAME_PADDING = 0xFF,
} RS92_SubFrameType;

typedef __PACKED(struct {
    unsigned int unknown:4;
    unsigned int snr:4;
}) RS92_GpsChannel;

typedef __PACKED(struct {
    int32_t val32;
    uint8_t val24[3];
    int8_t val8;
}) RS92_GpsSatData;


typedef __PACKED(union {
    RS92_RawData rawData;
    __PACKED(struct {
        __PACKED(struct _RS92_ConfigBlock {
            RS92_SubFrameType frameType;
            uint8_t length;
            uint16_t frameCounter;
            char name[10];
            uint16_t stateFlags;
            uint8_t nn;
            uint8_t thisCalibIndex;
            uint8_t calibFragment[16];
            uint16_t crc;
        }) config;  /* offset 0 */

        __PACKED(struct _RS92_MetrologyBlock {
            RS92_SubFrameType frameType;
            uint8_t length;
            uint8_t T[3];
            uint8_t U1[3];
            uint8_t U2[3];
            uint8_t REF1[3];
            uint8_t REF2[3];
            uint8_t P[3];
            uint8_t REF3[3];
            uint8_t REF4[3];
            uint16_t crc;
        }) metrology;   /* offset 36 */

        __PACKED(struct _RS92_GpsBlock {
            RS92_SubFrameType frameType;
            uint8_t length;
            int32_t timeMilliseconds;
            uint16_t nn;
            uint16_t prn[4];
            RS92_GpsChannel channel[12];
            RS92_GpsSatData satData[12];
            uint16_t crc;
        }) gps; /* offset 64 */

        __PACKED(struct {
            RS92_SubFrameType frameType;
            uint8_t length;
            uint16_t status;
            float aux1;
            float aux2;
            uint16_t crc;
        }) aux; /* offset 190 */

        __PACKED(struct {
            RS92_SubFrameType frameType;
            uint8_t length;
            uint16_t nn[2];
        }) unknown; /* offset 204 */

        uint8_t rscode[24]; /* offset 210 */
    });
}) RS92_Packet;


typedef struct {
    float temperature;
    float pressure;
    float pressureAltitude;
} RS92_CookedMetrology;


typedef struct {
    double gpstime;
    double rxClockOffset;
    double rxClockDrift;
    float hdop;
    uint8_t visibleSats;
    uint8_t usedSats;
    GPS_SatInfo sats[12];
    bool valid;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    float climbRate;
} RS92_CookedGps;


typedef struct _RS92_InstanceData {
    struct _RS92_InstanceData *next;
    uint32_t id;
    char hashName[20];                          /* Hashable sonde name */
    uint32_t fragmentValidFlags;                /* The 32 bits (if set) indicate validity of the corresponding fragment */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    uint16_t frameCounter;

    RS92_CookedGps gps;
    RS92_CookedMetrology metro;

    __PACKED(union {
        uint8_t rawData[32][16];
        __PACKED(struct {
            uint16_t nn0;                       /* 0 */
            uint16_t frequency;                 /* RF frequency (in 10 kHz steps above 400 MHz */
            uint16_t killTimer;                 /* Last frame # before kill. 65535 (-1) when kill timer disabled */
            uint16_t nn6;
            __PACKED(union {
                uint8_t nn8[10];
                __PACKED(struct {
                    uint8_t xx8;
                    float xx9;
                });
            });
            uint16_t sizeofCoeffEntry;          /* Size in bytes of one entry in the "coeff" array */
            uint16_t sizeofCoeffArray;          /* Size in bytes of the "coeff" array */
            char name[10];
            __PACKED(union {                    /* 2 */
                uint8_t x16[32];
                __PACKED(struct {
                    uint8_t nn16;
                    uint8_t nn17;
                    uint8_t nn18;
                    uint8_t nn19;
                    float nn20;
                    float nn24;
                    float nn28;
                    uint8_t nn32;
                    uint8_t nn33;
                    uint16_t nn34;
                    uint16_t nn36;
                    uint16_t nn38;
                    uint8_t nn40;
                    uint8_t nn41;
                    uint8_t nn42;
                    uint8_t nn43;
                    uint8_t nn44;
                    uint8_t nn45;
                    uint8_t nn46;
                    uint8_t nn47;
                });
            });
            __PACKED(struct {                   /* 4 */
                uint8_t index;
                float value;
            }) coeff[66];
        });
    });

    struct {
        float alt;
        int32_t frame;
    } lastAltitudes[RS92_CLIMB_HISTORY_LENGTH];
    int lastAltitudesWrIndex;
} RS92_InstanceData;



/* Process the config/calib block.
 * Sets calib==NULL in case of low memory
 * Otherwise sets calib to point to the calibration block for that sonde.
 * 
 * Returns LPCLIB_SUCCESS if the sonde name is valid and further blocks of the RS92 frame can be processed.
 */
LPCLIB_Result _RS92_processConfigBlock (
        const struct _RS92_ConfigBlock *rawConfig,
        RS92_InstanceData **instancePointer);


/* Check if the calibration block contains valid data for a given purpose */
#define CALIB_FREQUENCY             0x00000001
#define CALIB_TEMPERATURE           0x00003800
#define CALIB_PRESSURE              0x00000FF0
#define CALIB_KILLTIMER             0x00000001

bool _RS92_checkValidCalibration(RS92_InstanceData *instance, uint32_t purpose);

/* Iterate through instances */
bool _RS92_iterateInstance (RS92_InstanceData **instance);

/* Remove an instance from the chain */
void _RS92_deleteInstance (RS92_InstanceData *instance);

/* Process the metrology block.
 */
LPCLIB_Result _RS92_processMetrologyBlock (
        const struct _RS92_MetrologyBlock *rawMetro,
        RS92_CookedMetrology *cookedMetro,
        RS92_InstanceData *instance);


/* Process the GPS block.
 */
LPCLIB_Result _RS92_processGpsBlock (
        const struct _RS92_GpsBlock *rawGps,
        RS92_CookedGps *cookedGps,
        float pressureAltitude);

#endif
