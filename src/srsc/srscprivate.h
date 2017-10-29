
#ifndef __SRSCPRIVATE_H
#define __SRSCPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "srsc.h"
#include "gps.h"


typedef enum {
    SRSC_FRAME_METRO_HUMIDITY = 1,

    SRSC_FRAME_GPS_DATE = 20,
    SRSC_FRAME_GPS_TIME = 21,
    SRSC_FRAME_GPS_LATITUDE = 22,
    SRSC_FRAME_GPS_LONGITUDE = 23,
    SRSC_FRAME_GPS_ALTITUDE = 24,
    SRSC_FRAME_GPS_SPEED = 25,
    SRSC_FRAME_GPS_TRACKMADEGOOD = 26,
    SRSC_FRAME_GPS_QUALITY = 27,

    SRSC_FRAME_CONFIG_NAME = 100,
    SRSC_FRAME_CONFIG_TYPE = 102,
    SRSC_FRAME_CONFIG_103 = 103,
    SRSC_FRAME_CONFIG_104 = 104,
    SRSC_FRAME_CONFIG_105 = 105,
    SRSC_FRAME_CONFIG_106 = 106,
    SRSC_FRAME_CONFIG_107 = 107,
    SRSC_FRAME_CONFIG_108 = 108,
    SRSC_FRAME_CONFIG_VBAT = 110,
    SRSC_FRAME_CONFIG_111 = 111,
    SRSC_FRAME_CONFIG_115 = 115,
    SRSC_FRAME_CONFIG_116 = 116,
    SRSC_FRAME_CONFIG_119 = 119,
    SRSC_FRAME_CONFIG_120 = 120,
} SRSC_FrameType;


__FORCEINLINE(bool SRSC_isGpsType (SRSC_FrameType type))
{
    return (type >= SRSC_FRAME_GPS_DATE) && (type <= SRSC_FRAME_GPS_QUALITY);
}


typedef enum {
    SRSC_DETECTOR_FIND_TYPE = 0,
    SRSC_DETECTOR_FIND_NAME = 1,
    SRSC_DETECTOR_READY = 2,
} SRSC_DetectorState;


typedef uint8_t SRSC_RawData[7];

typedef __PACKED(union {
    SRSC_RawData rawData;
    __PACKED(struct {
        uint8_t type;
        uint32_t d_bigendian;
        uint16_t parity;
    });
}) SRSC_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Humidity [%] */
} SRSC_CookedMetrology;


typedef struct {
    char name[20];
    uint32_t sondeType;
    uint32_t sondeNumber;
    bool sondeNameKnown;
    uint16_t frameCounter;
    uint32_t frequencyKhz;
    bool isC50;
    bool hasO3;

    uint32_t info103;
    uint32_t info104;
    uint32_t info105;
    uint32_t info106;
    uint32_t info107;
    uint32_t info108;
    float batteryVoltage;                   /* Battery voltage [V] */
    uint32_t info111;
    uint32_t info115;
    uint32_t info116;
    uint32_t info119;
    uint32_t info120;
} SRSC_CookedConfig;


typedef struct {
    double gpstime;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    float climbRate;
    float groundSpeed;
    float direction;
    struct {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
    } utc;

    uint8_t usedSats;                           /* # used sats */
    uint32_t usedSatsMask;                      /* Identifies sats used for position solution
                                                 * Bit31 (PRN32) ... Bit0 (PRN1)
                                                 */
    struct {
        uint8_t PRN;
        uint8_t snr;
    } sats[12];
    float hdop;                                 /* 2D quality indicator HDOP */

    int updateFlags;    // Bit 0: lat, bit 1: lon, bit 2: alt
} SRSC_CookedGps;


/* Data that needs to be stored for every SRSC instance.
 * In contrast to Vaisala sondes, SRSC frames so not contain a complete snapshot
 * of all sonde data. Therefore, it is necessary to store metrology and GPS here as well.
 */
typedef struct _SRSC_InstanceData {
    struct _SRSC_InstanceData *next;
    char name[20];
    float rxFrequencyMHz;
    float rxOffset;

    SRSC_DetectorState detectorState;
    uint32_t lastUpdated;

    struct {
        uint32_t lastCh0Time;
        uint32_t lastTime;
        uint8_t maxChannel;
        uint8_t nDetections;
        uint8_t numAnalog;
        uint8_t prevChannel;

        uint32_t prevSondeType;
        uint32_t sondeType;
        uint32_t prevSondeNumber;
        uint32_t sondeNumber;
    } confDetect;

    SRSC_CookedConfig config;
    SRSC_CookedMetrology metro;
    SRSC_CookedGps gps;
} SRSC_InstanceData;



/* Process the config/calib block.
 * Sets calib==NULL in case of low memory
 * Otherwise sets calib to point to the calibration block for that sonde.
 * 
 * Returns LPCLIB_SUCCESS if the sonde name is valid
 */
LPCLIB_Result _SRSC_processConfigFrame (
        const SRSC_Packet *rawConfig,
        SRSC_InstanceData **instancePointer,
        float rxFrequencyHz);


/* Iterate through instances */
bool _SRSC_iterateInstance (SRSC_InstanceData **instance);

/* Remove an instance from the chain */
void _SRSC_deleteInstance (SRSC_InstanceData *instance);

/* Process a GPS block.
 */
LPCLIB_Result _SRSC_processGpsFrame (
        const SRSC_Packet *rawGps,
        SRSC_CookedGps *cookedGps);


void SRSC_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples);
void SRSC_DSP_initAudio (void);

#endif
