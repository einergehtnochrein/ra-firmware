
#ifndef __RD41PRIVATE_H
#define __RD41PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "ncar.h"
#include "gps.h"


typedef __PACKED(struct {
    uint8_t sondeType;
    uint16_t frameCounter_BE;
    uint16_t crc;
}) RD41_SubFrameFrameNumber;

typedef __PACKED(struct {
    float pressure;     /* Pressure in hPa */
    float temperature;
    float f2;
    float f3;
    uint16_t crc;
}) RD41_SubFrameMetrology;

typedef __PACKED(struct {
    uint16_t speed_BE;
    int16_t D_BE;
    int16_t climb_rate_BE;
    int16_t altitude_b2b1_BE;
    uint8_t altitude_b0;
    uint8_t xxx_hour;
    uint8_t minute;
    uint8_t second;
    uint8_t second_hundredth;
    uint8_t reserved00D[4];
    uint16_t crc;
}) RD41_SubFrameGps1;

typedef __PACKED(struct {
    int32_t latitude_BE;
    int32_t longitude_BE;
    int16_t reserved008_BE;
    int16_t reserved00A_BE;
    uint16_t crc;
}) RD41_SubFrameGps2;

typedef __PACKED(struct {
    uint16_t speed_BE;
    int16_t D_BE;
    int16_t climb_rate_BE;
    int16_t altitude_b2b1_BE;
    uint8_t altitude_b0;
    int16_t reserved009_BE;
    int16_t reserved00B_BE;
    uint16_t crc;
}) RD41_SubFrameGps3;

typedef __PACKED(struct {
    uint8_t reserved000[27];
    uint16_t crc;
}) RD41_SubFrameGps4;

typedef __PACKED(struct {
    uint8_t reserved000[6];
    uint16_t battery_voltage_BE;
    float reserved008;
    char reserved00C[2];
    uint16_t crc;
}) RD41_SubFrameConfig;

typedef __PACKED(struct {
    uint8_t sondeType;
    uint16_t frameCounter;
    uint8_t sum1;
    uint8_t sum2;
}) RD94_SubFrameFrameNumber;


typedef __PACKED(struct {
    __PACKED(union {
        uint8_t rawData[120-4];
        __PACKED(struct {
            __PACKED(union {
                RD41_SubFrameFrameNumber frameNumberRD41;
                RD94_SubFrameFrameNumber frameNumberRD94;
            });
            __PACKED(struct {
                RD41_SubFrameMetrology metrology;
                RD41_SubFrameGps1 gps1;
                RD41_SubFrameGps2 gps2;
                RD41_SubFrameGps3 gps3;
                RD41_SubFrameGps4 gps4;
                RD41_SubFrameConfig config;
            });
        });
    });
}) RD41_RawFrame;


typedef struct {
    float T;                            /* Temperature [°C] main (air) sensor */
    float pressure;                     /* Atmospheric pressure [hPa] */
    float RH;                           /* Relative humidity [%] */
} NCAR_CookedMetrology;


typedef struct {
    double gpstime;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    uint8_t visibleSats;
    uint8_t usedSats;
} NCAR_CookedGps;


/* Data that needs to be stored for every RS41 instance. */
typedef struct _NCAR_InstanceData {
    struct _NCAR_InstanceData *next;
    uint32_t id;
    char name[20];                              /* Sonde name */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    float batteryVoltage;                       /* Battery voltage [V] */
    uint16_t frameCounter;
    NCAR_CookedGps gps;
    NCAR_CookedMetrology metro;
    NCAR_LogMode logMode;
    float rssi;
    uint64_t realTime;
} NCAR_InstanceData;



LPCLIB_Result _RD41_processConfigBlocks (
        const RD41_SubFrameFrameNumber *rawFrame,
        const RD41_SubFrameConfig *rawConfig,
        NCAR_InstanceData **instancePointer);


/* Iterate through instances */
bool _NCAR_iterateInstance (NCAR_InstanceData **instance);

/* Remove an instance from the chain */
void _NCAR_deleteInstance (NCAR_InstanceData *instance);

/* Process the metrology block(s).
 */
LPCLIB_Result _RD41_processMetrologyBlock (
        const RD41_SubFrameMetrology *rawMetro,
        NCAR_InstanceData *instance);

/* Process the GPS blocks
 */
LPCLIB_Result _RD41_processGps1Block (
        const RD41_SubFrameGps1 *rawGps1,
        NCAR_CookedGps *cookedGps);
LPCLIB_Result _RD41_processGps2_3Block (
        const RD41_SubFrameGps2 *rawGps2,
        const RD41_SubFrameGps3 *rawGps3,
        NCAR_CookedGps *cookedGps);
LPCLIB_Result _RD41_processGps4Block (
        const RD41_SubFrameGps4 *rawGps4,
        NCAR_CookedGps *cookedGps);


/* Check CRC of an RD41 sub-block */
_Bool _RD41_checkCRC (void *buffer, int length);
/* Check checksum of an RD94 sub-block */
_Bool _RD94_checkChecksum (void *buffer, int length, uint8_t checkSUM1, uint8_t checkSUM2);

#ifdef __cplusplus
}
#endif
#endif
