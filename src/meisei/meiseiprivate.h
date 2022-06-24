
#ifndef __MEISEIPRIVATE_H
#define __MEISEIPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "meisei.h"
#include "gps.h"


typedef enum {
    MEISEI_MODEL_UNKNOWN = 0,
    MEISEI_MODEL_RS11G,
    MEISEI_MODEL_IMS100,
} MEISEI_Model;

typedef uint8_t MEISEI_RawData[48];

typedef union {
    MEISEI_RawData rawData;
    uint64_t fields[6];
} MEISEI_RawPacket;

typedef struct {
    int16_t w[12];
} MEISEI_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Relative humidity [%] */
    float cpuTemperature;                       /* CPU temperature [°C] */
    float txTemperature;                        /* Radio (Si4032) temperature [°C] */
    float ana8_unknown;
} MEISEI_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;

    uint16_t milliseconds;
    uint16_t count1024;

    struct {
        uint8_t PRN;
        uint8_t snr;
        uint8_t xxx1;
        uint8_t xxx2;
    } sats[16];
    uint8_t usedSats;
    uint8_t visibleSats;
} MEISEI_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _MEISEI_InstanceData {
    struct _MEISEI_InstanceData *next;
    uint32_t id;
    uint32_t serialSonde;
    uint32_t serialPcb;
    uint32_t serialSensorBoom;
    MEISEI_Model model;
    float rxFrequencyMHz;
    float rxOffset;
    uint32_t lastUpdated;
    uint16_t frameCounter;

    uint64_t configValidFlags;                  /* Indicates valid fields in "config" */
    float config[64];

    float refFreq;                              /* Reference frequency for TU measurements */

    /* Raw data before processing */
    MEISEI_Packet configPacketEven;             /* Even frame number */
    MEISEI_Packet configPacketOdd;              /* Odd frame number */
    MEISEI_Packet gpsPacketEven;                /* Even frame number */
    MEISEI_Packet gpsPacketOdd;                 /* Odd frame number */
    uint16_t frameCounterEven;

    MEISEI_CookedGps gps;
    MEISEI_CookedMetrology metro;
} MEISEI_InstanceData;



LPCLIB_Result _MEISEI_processConfigFrame (
        MEISEI_Packet *packet,
        MEISEI_InstanceData **instancePointer,
        float rxFrequencyHz);
LPCLIB_Result _MEISEI_processGpsFrame (
        MEISEI_InstanceData *instance);
LPCLIB_Result _MEISEI_processMetrology (
        MEISEI_InstanceData *instance);
uint16_t _MEISEI_getPayloadHalfWord (const uint64_t *fields, int index);

/* Check if the calibration block contains valid data for a given purpose */
#define CALIB_SERIAL_SONDE1         0x0000000000000001ll
#define CALIB_SERIAL_SONDE2         0x0000000000010000ll
#define CALIB_SERIAL_SONDE3         0x0000000100000000ll
#define CALIB_SERIAL_SONDE4         0x0001000000000000ll
#define CALIB_SERIAL_SENSOR_BOOM    0x0000000000000010ll
#define CALIB_SERIAL_PCB            0x0000000000000004ll
#define CALIB_HUMIDITY              0x001E000000000000ll
#define CALIB_MAIN_TEMPERATURE      0x01E01FFE1FFE0000ll

bool _MEISEI_checkValidCalibration(MEISEI_InstanceData *instance, uint64_t purpose);

/* Iterate through instances */
bool _MEISEI_iterateInstance (MEISEI_InstanceData **instance);

/* Remove an instance from the chain */
void _MEISEI_deleteInstance (MEISEI_InstanceData *instance);

#endif
