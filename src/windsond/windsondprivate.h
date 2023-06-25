
#ifndef __WINDSONDPRIVATE_H
#define __WINDSONDPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "windsond.h"
#include "gps.h"



typedef uint8_t WINDSOND_RawData[64];


typedef union {
    WINDSOND_RawData rawData;
} WINDSOND_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Humidity [%] */
    float pressure;                             /* Pressure [hPa] */
    float illuminance;
    float cpuTemperature;                       /* CPU temperature [°C] */
} WINDSOND_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} WINDSOND_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _WINDSOND_InstanceData {
    struct _WINDSOND_InstanceData *next;
    uint32_t id;
    uint16_t name_id;
    uint8_t name_sid;
    float rxFrequencyMHz;
    float rxOffset;
    uint32_t lastUpdated;
    uint16_t frameCounter;
    float rssi;
    uint64_t realTime;
    float vbat;

    uint64_t configValidFlags;                  /* Indicates valid fields in "config" */
    float config[64];

    int32_t latitude_reference;
    int32_t longitude_reference;
    uint32_t timestamp_reference;

    float ground_pressure;
    float ground_altitude;
    float ref_temperature;

    WINDSOND_CookedGps gps;
    WINDSOND_CookedMetrology metro;
} WINDSOND_InstanceData;



LPCLIB_Result _MEISEI_processFrame (
        WINDSOND_Packet *packet,
        WINDSOND_InstanceData **instancePointer,
        float rxFrequencyHz);


/* Get a new instance data structure for a new sonde */
WINDSOND_InstanceData *_WINDSOND_getInstanceDataStructure (float frequencyMHz, uint16_t id, uint8_t sid);

/* Iterate through instances */
bool _WINDSOND_iterateInstance (WINDSOND_InstanceData **instance);

/* Remove an instance from the chain */
void _WINDSOND_deleteInstance (WINDSOND_InstanceData *instance);

/* Search for correct CRC in the frame */
_Bool _WINDSOND_checkCRC (uint8_t *buffer, int length);

/* Remove data whitening */
void _WINDSOND_removeWhitening (uint8_t *buffer, int length);

/* Golay error correction. */
LPCLIB_Result _WINDSOND_checkFEC (uint8_t frame[], int *pNumCodewords);

/* Decode frame payload */
LPCLIB_Result _WINDSOND_processPayload (
        WINDSOND_InstanceData **instancePointer,
        const uint8_t *payload,
        int length,
        float rxFrequencyMHz);

#endif
