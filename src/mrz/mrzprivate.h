
#ifndef __MRZPRIVATE_H
#define __MRZPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mrz.h"
#include "gps.h"


typedef __PACKED(union {
    uint8_t dat8[47];
    uint32_t dat32[11];
}) MRZ_RawData;


typedef union {
    MRZ_RawData rawData;
    __PACKED(struct {
        uint8_t unk0;
        __PACKED(struct {
            uint8_t hour;
            uint8_t minute;
            uint8_t second;
        });
        uint8_t unk4;
        __PACKED(struct {
            uint32_t ecef_x;
            uint32_t ecef_y;
            uint32_t ecef_z;
            int16_t ecef_vx;
            int16_t ecef_vy;
            int16_t ecef_vz;
            uint8_t unk17;
            uint8_t unk18;
            uint8_t unk19;
        }) gps;
        int16_t unk1A;
        int16_t unk1C;
        int16_t unk1E;
        uint32_t rawTemperature;
        uint32_t rawHumidity;
        uint8_t thisCalibIndex;
        uint32_t calibFragment;
        uint16_t crc;
    });
} MRZ_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} MRZ_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} MRZ_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _MRZ_InstanceData {
    struct _MRZ_InstanceData *next;
    uint32_t id;
    char name[20];
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;
    uint32_t fragmentValidFlags;

    __PACKED(union {
        uint32_t rawCalib[16];
        __PACKED(struct {
            float calib1;
            float calib2;
            float calib3;
            float calib4;
            float calib5;
            float calib6;
            float calib7;
            float calib8;
            float calib9;
            uint32_t calib10;
            uint32_t calib11;
            uint32_t calib12;
            uint32_t serialSonde;
            uint32_t serialSensor;
            uint32_t ddmmyy_production;
            uint32_t ddmmyy_current;
        });
    }) calib;

    uint32_t lastGpsTime;
    MRZ_CookedGps gps;
    MRZ_CookedMetrology metro;
} MRZ_InstanceData;



LPCLIB_Result _MRZ_processConfigFrame (
        MRZ_Packet *packet,
        MRZ_InstanceData **instancePointer,
        float rxFrequencyHz);


/* Check if the calibration block contains valid data for a given purpose */
#define CALIB_SERIALSONDE           0x00004000l
#define CALIB_SERIALSENSOR          0x00008000l

bool _MRZ_checkValidCalibration(MRZ_InstanceData *instance, uint32_t purpose);

/* Iterate through instances */
bool _MRZ_iterateInstance (MRZ_InstanceData **instance);

/* Remove an instance from the chain */
void _MRZ_deleteInstance (MRZ_InstanceData *instance);

/* Process the GPS frame */
LPCLIB_Result _MRZ_processGpsFrame (
        MRZ_Packet *rawGps,
        MRZ_InstanceData *instance);

#endif
