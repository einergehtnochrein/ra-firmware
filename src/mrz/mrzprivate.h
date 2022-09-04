
#ifndef __MRZPRIVATE_H
#define __MRZPRIVATE_H


#include <math.h>
#include <stdlib.h>
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
            uint8_t usedSats;                   /* #sats in position solution. 0=position invalid */
            uint8_t pAcc_m_mod256;              /* pAcc [m] mod256. Max range 255m.
                                                 * Must have valid position.
                                                 */
            uint8_t pAcc_dm_mod256;             /* pAcc [dm] mod256. Max range 25m. Ignore if
                                                 * pAcc_m_mod256 > 24.
                                                 */
        }) gps;
        int16_t cookedTemperature;              /* Temperature [1/100 °C] */
        int16_t cookedHumidity;                 /* Humidity [1/100 %] */
        int16_t cookedPressure;                 /* [0.2 hPa], -1 if no sensor */
        int32_t rawTemperature;
        int32_t rawHumidity;
        uint8_t thisCalibIndex;                 /* 1...16 */
        uint32_t calibFragment;
        uint16_t crc;
    });
} MRZ_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Relative humidity [%] */
    float batteryVoltage;                       /* Battery voltage [V] */
    float pressure;                             /* Pressure [hPa] */
} MRZ_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    uint8_t usedSats;
    float pAcc;                                 /* Position Accuracy [m] */
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
            float calibNTC_A;
            float calibNTC_B;
            float calibNTC_C;
            float calibADC_T[3];
            float calibADC_U[3];
            uint32_t rawVbat;
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
#define CALIB_TEMPERATURE           0x0000003Fl
#define CALIB_HUMIDITY              0x000001FFl
#define CALIB_VBAT                  0x00000200l
#define CALIB_SERIALSONDE           0x00001000l
#define CALIB_SERIALSENSOR          0x00002000l

bool _MRZ_checkValidCalibration(MRZ_InstanceData *instance, uint32_t purpose);

/* Iterate through instances */
bool _MRZ_iterateInstance (MRZ_InstanceData **instance);

/* Remove an instance from the chain */
void _MRZ_deleteInstance (MRZ_InstanceData *instance);

/* Process the GPS frame */
LPCLIB_Result _MRZ_processGpsFrame (
        MRZ_Packet *rawGps,
        MRZ_InstanceData *instance);

/* Process the PTU measurements */
LPCLIB_Result _MRZ_processMetrology (
        MRZ_Packet *packet,
        MRZ_InstanceData *instance);

#endif
