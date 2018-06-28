
#ifndef __RS41PRIVATE_H
#define __RS41PRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "rs41.h"
#include "gps.h"


typedef enum {
    RS41_SUBFRAME_GAPFILL = 0x76,
    RS41_SUBFRAME_CRYPT78 = 0x78,
    RS41_SUBFRAME_CALIB_CONFIG = 0x79,
    RS41_SUBFRAME_METROLOGY = 0x7A,
    RS41_SUBFRAME_GPS_POSITION = 0x7B,
    RS41_SUBFRAME_GPS_INFO = 0x7C,
    RS41_SUBFRAME_7D = 0x7D,
    RS41_SUBFRAME_AUX = 0x7E,
    RS41_SUBFRAME_CRYPT80 = 0x80,
} RS41_SubFrameType;

#define RS41_CALIBRATION_MAX_INDEX      51



typedef __PACKED(struct {
    uint16_t frameCounter;
    char name[8];
    uint8_t batteryVoltage100mV;            /* Battery voltage in multiples of 100 mV */
    uint8_t reserved00B[2];
    uint8_t flags;                          /* Bit 1: 0=Ascent, 1=Descent */
    uint8_t reserved00E[8];
    uint8_t maxCalibIndex;                  /* Maximum index of calibration fragment */
    uint8_t thisCalibIndex;                 /* Index of calibration fragment in this frame */
    uint8_t calibFragment[16];
    uint16_t crc;
}) RS41_SubFrameCalibConfig;


typedef __PACKED(struct {
    __PACKED(struct {
        uint8_t current[3];
        uint8_t refmin[3];
        uint8_t refmax[3];
    }) adc[4];
    uint16_t val12_16;
    int16_t pressurePolyTwist;
    uint16_t val14_16;
    uint16_t crc;
}) RS41_SubFrameMetrology;


typedef __PACKED(struct {
    uint32_t ecefX;
    uint32_t ecefY;
    uint32_t ecefZ;
    int16_t speedX;
    int16_t speedY;
    int16_t speedZ;
    uint8_t reserved[3];
    uint16_t crc;
}) RS41_SubFrameGpsPosition;


typedef __PACKED(struct {
    uint16_t gpsWeek;
    uint32_t timeOfWeek;
    __PACKED(struct {
        uint8_t prn;
        uint8_t reserved;
    }) sats[12];
    uint16_t crc;
}) RS41_SubFrameGpsInfo;


typedef __PACKED(struct {
    uint32_t reserved000;
    uint8_t reserved004;
    __PACKED(struct {
        uint8_t reservedX0;
        uint8_t reservedX1[3];
        uint8_t reservedX4[3];  // signed 24 bit
    }) unknown005[12];
    uint16_t crc;
}) RS41_SubFrame7D;


typedef struct {
    uint8_t prn;
    int8_t info;
    uint32_t reserved000;
    uint32_t reserved001;
    int32_t reserved004;
} RS41_SatInfo;


typedef struct {
    float temperature;
    float temperature2;
    float humidity;
    float pressure;
    float pressureAltitude;

    bool hasO3;
} RS41_CookedMetrology;


typedef struct {
    double gpstime;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    uint8_t visibleSats;
} RS41_CookedGps;


/* Data that needs to be stored for every RS41 instance. */
typedef struct _RS41_InstanceData {
    struct _RS41_InstanceData *next;
    uint32_t id;
    char name[20];                              /* Sonde name */
    uint64_t fragmentValidFlags;                /* The 52 bits (if set) indicate validity of the corresponding fragment */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    float batteryVoltage;                       /* Battery voltage [V] */
    uint16_t frameCounter;
    bool encrypted;                             /* Set for RS41-SGM military version */
    bool onDescent;                             /* Descent phase detected */

    RS41_CookedGps gps;
    RS41_CookedMetrology metro;

    __PACKED(union {
        uint8_t rawData[RS41_CALIBRATION_MAX_INDEX + 1][16];
        __PACKED(struct {
            uint16_t reserved000;
            uint16_t frequency;                 /* TX is on 400 MHz + (frequency / 64) * 10 kHz */
            uint8_t reserved004[0x00D-0x004];
            uint8_t serial[8];                  /* Sonde ID, 8 char, not terminated */
            uint8_t reserved015[0x027-0x015];
            uint16_t killTimer;                 /* (probably) max frame counter before kill */
            uint8_t reserved029[0x02B-0x029];
            uint8_t burstKill;                  /* Burst kill (0=disabled, 1=enabled) */
            uint8_t reserved02C[0x03D-0x02C];
            float refResistorLow;               /* Reference resistor low (750 Ohms) */
            float refResistorHigh;              /* Reference resistor high (1100 Ohms) */
            float f045[62];
            uint8_t reserved13D[0x152-0x13D];
            float f152;
            uint8_t u156;
            float f157;
            uint8_t reserved15B[0x160-0x15B];
            float f160[35];
            uint8_t reserved1EC[0x1F0-0x1EC];
            float f1F0[8];
            float pressureLaunchSite[2];        /* Pressure [hPa] at launch site */
            uint8_t modelname218[10];           /* "RS41-SG" */
            uint8_t modelname222[10];           /* "RSM412" */
            uint8_t serial22C[9];               /* "L1123553" */
            uint8_t text235[14];                /* "0000000000" */
            uint8_t text243[10];                /* "00000000" */
            uint8_t c24D;
            uint32_t u24E[(0x25E - 0x24E)/4];
            float pressurePoly[18];             /* 0x25E: Coefficients for pressure sensor polynomial */
            float f2A6[17];
            uint8_t reserved2EA[0x2FA-0x2EA];
            uint16_t halfword2FA[9];
            uint8_t reserved30C[0x330-0x30C];
        });
    });
} RS41_InstanceData;



/* Process the config/calib block.
 * Sets calib==NULL in case of low memory
 * Otherwise sets calib to point to the calibration block for that sonde.
 * 
 * Returns LPCLIB_SUCCESS if the sonde name is valid
 */
LPCLIB_Result _RS41_processConfigBlock (
        const RS41_SubFrameCalibConfig *rawConfig,
        RS41_InstanceData **instancePointer);


/* Check if the calibration block contains valid data for a given purpose */
#define CALIB_FREQUENCY             0x0000000000000001ll
#define CALIB_TEMPERATURE1          0x0000000000000078ll
#define CALIB_TEMPERATURE2          0x00000000000C0018ll
#define CALIB_PRESSURE              0x000007E000000000ll
#define CALIB_KILLTIMER             0x0000000000000004ll

bool _RS41_checkValidCalibration(RS41_InstanceData *instance, uint64_t purpose);

/* Iterate through instances */
bool _RS41_iterateInstance (RS41_InstanceData **instance);

/* Remove an instance from the chain */
void _RS41_deleteInstance (RS41_InstanceData *instance);

/* Process the metrology block.
 */
LPCLIB_Result _RS41_processMetrologyBlock (
        const RS41_SubFrameMetrology *rawMetro,
        RS41_CookedMetrology *cookedMetro,
        RS41_InstanceData *instance);


/* Process the GPS block (position).
 */
LPCLIB_Result _RS41_processGpsPositionBlock (
        const RS41_SubFrameGpsPosition *rawGps,
        RS41_CookedGps *cookedGps);


/* Process the GPS block (sat info).
 */
LPCLIB_Result _RS41_processGpsInfoBlock (
        const RS41_SubFrameGpsInfo *rawGps,
        RS41_CookedGps *cookedGps);

#endif
