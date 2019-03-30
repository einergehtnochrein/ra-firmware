
#ifndef __IMETPRIVATE_H
#define __IMETPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "imet.h"
#include "gps.h"


typedef enum {
    IMET_FRAME_PTU = 0x01,
    IMET_FRAME_PTUX = 0x04,
    IMET_FRAME_GPS = 0x02,
    IMET_FRAME_GPSX = 0x05,
    IMET_FRAME_XDATA = 0x03,
} IMET_FrameType;


typedef __PACKED(struct {
    uint8_t frameType;
    uint16_t packetNumber;
    uint8_t pressure24[3];
    int16_t temperature;
    int16_t humidity;
    uint8_t batteryVoltage;
    uint16_t crc;
}) IMET_FramePtu;


typedef __PACKED(struct {
    uint8_t frameType;
    uint16_t packetNumber;
    uint8_t pressure24[3];
    int16_t temperature;
    int16_t humidity;
    uint8_t batteryVoltage;
    int16_t internalTemperature;
    int16_t pressureSensorTemperature;
    int16_t humiditySensorTemperature;
    uint16_t crc;
}) IMET_FramePtux;


typedef __PACKED(struct {
    uint8_t frameType;
    float latitude;
    float longitude;
    uint16_t altitude;
    uint8_t numSats;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t crc;
}) IMET_FrameGps;


typedef __PACKED(struct {
    uint8_t frameType;
    float latitude;
    float longitude;
    uint16_t altitude;
    uint8_t numSats;
    float eastVelocity;
    float northVelocity;
    float verticalVelocity;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t crc;
}) IMET_FrameGpsx;


typedef struct {
    char name[20];
    uint32_t sondeType;
    uint16_t frameCounter;
    uint32_t frequencyKhz;
} IMET_CookedConfig;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    struct {
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
    } utc;

    uint8_t usedSats;                           /* # used sats */
    bool updated;
} IMET_CookedGps;


/* Data that needs to be stored for every instance.
 */
typedef struct _IMET_InstanceData {
    struct _IMET_InstanceData *next;
    uint32_t id;
    char name[20];
    float frequency;

    uint32_t lastUpdated;

    struct {
        uint32_t lastCh0Time;
        uint32_t lastTime;
        uint8_t maxChannel;
        uint8_t nDetections;
        uint8_t prevChannel;

        uint32_t prevSondeType;
        uint32_t sondeType;
        uint32_t prevSondeNumber;
        uint32_t sondeNumber;
    } confDetect;

    IMET_CookedConfig config;
    IMET_CookedGps gps;
} IMET_InstanceData;



/* Get a new instance data structure for a new sonde */
IMET_InstanceData *_IMET_getInstanceDataStructure (float frequency);


/* Iterate through instances */
bool _IMET_iterateInstance (IMET_InstanceData **instance);

/* Process a GPS block.
 */
LPCLIB_Result _IMET_processGpsFrame (
        const IMET_FrameGps *rawGps,
        IMET_CookedGps *cookedGps);
LPCLIB_Result _IMET_processGpsxFrame (
        const IMET_FrameGpsx *rawGps,
        IMET_CookedGps *cookedGps);


void IMET_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples);
void IMET_DSP_initAudio (void);
void IMET_DSP_reset (void);

#endif
