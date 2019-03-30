
#ifndef __DFMPRIVATE_H
#define __DFMPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "dfm.h"
#include "gps.h"


/* Length of a DFM frame (32+528 bits @ 2500 bit/s) in milliseconds */
#define DFM_FRAME_LENGTH_MILLISEC           (224.0f)


typedef enum {
    DFM_MODEL_UNKNOWN = 0,
    DFM_MODEL_DFM06_OLD,
    DFM_MODEL_DFM06_NEW,
    DFM_MODEL_DFM09_OLD,
    DFM_MODEL_DFM09_NEW,
    DFM_MODEL_DFM09_AFRICA,
    DFM_MODEL_PS15,
    DFM_MODEL_DFM17,
} DFM_Model;


enum {
    DFM06_CHANNEL_CONFIG_NAME = 0,

    DFM09_CHANNEL_CONFIG_BATTERY_VOLTAGE = -5,
    DFM09_CHANNEL_CONFIG_CPU_TEMPERATURE = -4,
    DFM09_CHANNEL_CONFIG_NAME = 0,
} DFM_ConfigChannel;


typedef enum {
    DFM_DETECTOR_FIND_NANALOG = 0,
    DFM_DETECTOR_FIND_CONFIG_STRUCTURE = 1,
    DFM_DETECTOR_FIND_NAME = 2,
    DFM_DETECTOR_READY = 3,
} DFM_DetectorState;



typedef __PACKED(struct {
    uint8_t type;
    uint8_t h[6];
}) DFM_SubFrameConfig;


typedef __PACKED(struct {
    uint8_t d[12];
    uint8_t type;
}) DFM_SubFrameGps;


typedef uint8_t DFM_RawData[33];

typedef __PACKED(union {
    DFM_RawData rawData;
    __PACKED(struct {
        __PACKED(union _DFM_ConfigBlock {
            uint8_t h[7];
            DFM_SubFrameConfig raw;
        }) config;

        __PACKED(union _DFM_Data1Block {
            uint8_t d1[13];
            DFM_SubFrameGps raw;
        }) gps[2];
    });
}) DFM_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Humidity [%] */
    float pressure;
    float batteryVoltage;                       /* Battery voltage [V] */
    float cpuTemperature;                       /* CPU temperature [°C] */

    float _ref3;
    float _ref4;
} DFM_CookedMetrology;


typedef struct {
    uint32_t sondeNumber;
    bool sondeNameKnown;
    uint16_t frameCounter;
    uint32_t frequencyKhz;
} DFM_CookedConfig;


typedef struct {
    double gpstime;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    float climbRate;
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

    bool inBurkinaFaso;
    bool newPosition;
} DFM_CookedGps;


/* Data that needs to be stored for every DFM instance.
 * In contrast to Vaisala sondes, DFM frames do not contain a complete snapshot
 * of all sonde data. Therefore, it is necessary to store metrology here as well.
 */
typedef struct _DFM_InstanceData {
    struct _DFM_InstanceData *next;
    uint32_t id;
    DFM_Model model;
    char name[20];
    float rxFrequencyMHz;
    SONDE_Type platform;

    uint32_t lastUpdated;
    DFM_DetectorState detectorState;
    uint8_t numAnalog;                      /* Number of analog channels in each config frame */
    uint8_t maxConfigChannel;               /* Highest config channel used */
    uint8_t maxConfigChannelNibble0;
    struct {
        uint32_t lastCh0Time;
        uint32_t lastTime;
        uint8_t maxChannel;
        uint8_t nDetections;
        uint8_t numAnalog;
        uint8_t prevChannel;
        uint8_t prevNibble0;
        uint8_t maxChannelNibble0;
        uint32_t prevSondeNumber;
        uint32_t sondeNumber;
    } confDetect;

    DFM_CookedConfig config;
    DFM_CookedMetrology metro;
    DFM_CookedGps gps;
} DFM_InstanceData;



/* Process the config/calib block.
 * Sets calib==NULL in case of low memory
 * Otherwise sets calib to point to the calibration block for that sonde.
 * 
 * Returns LPCLIB_SUCCESS if the sonde name is valid
 */
LPCLIB_Result _DFM_processConfigBlock (
        const DFM_SubFrameConfig *rawConfig,
        DFM_InstanceData *instance,
        float rxFrequencyHz,
        uint32_t rxTime);


/* Check if the calibration block contains valid data for a given purpose */
#define CALIB_FREQUENCY             0x00000001

/* Iterate through instances */
bool _DFM_iterateInstance (DFM_InstanceData **instance);

/* Remove an instance from the chain */
void _DFM_deleteInstance (DFM_InstanceData *instance);

/* Process the GPS block.
 */
LPCLIB_Result _DFM_processGpsBlock (
        const DFM_SubFrameGps *rawGps,
        DFM_InstanceData **instancePointer,
        float rxFrequencyHz,
        uint32_t rxTime,
        SONDE_Type sondeType);

/* Process a frame with meteorological measurements. */
LPCLIB_Result _DFM_processMetrologyBlock (
        const DFM_SubFrameConfig *rawConfig,
        DFM_InstanceData *instance);

#endif
