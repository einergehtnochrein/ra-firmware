
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
    SRSC_FRAME_METRO_VOLTAGE_CU = 0,
    SRSC_FRAME_METRO_VOLTAGE_R2 = 1,
    SRSC_FRAME_METRO_TEMPERATURE_REFBLOCK = 2,
    SRSC_FRAME_METRO_TEMPERATURE_AIR = 3,
    SRSC_FRAME_METRO_TEMPERATURE_HUSENSOR = 4,
    SRSC_FRAME_METRO_TEMPERATURE_CHAMBER = 5,
    SRSC_FRAME_METRO_TEMPERATURE_O3INLET = 6,
    SRSC_FRAME_METRO_CURRENT_O3CELL = 7,

    SRSC_FRAME_METRO_HUMIDITY = 16,
    SRSC_FRAME_METRO_U_SENSOR_HEATING = 17,
    SRSC_FRAME_METRO_U_SENSOR_FREQUENCY = 18,

    SRSC_FRAME_GPS_DATE = 20,
    SRSC_FRAME_GPS_TIME = 21,
    SRSC_FRAME_GPS_LATITUDE = 22,
    SRSC_FRAME_GPS_LONGITUDE = 23,
    SRSC_FRAME_GPS_ALTITUDE = 24,
    SRSC_FRAME_GPS_25 = 25,
    SRSC_FRAME_GPS_26 = 26,
    SRSC_FRAME_GPS_QUALITY = 27,

    SRSC_FRAME_CONFIG_NAME = 100,
    SRSC_FRAME_CONFIG_TYPE = 102,
    SRSC_FRAME_CONFIG_103 = 103,
    SRSC_FRAME_CONFIG_104 = 104,
    SRSC_FRAME_CONFIG_105 = 105,
    SRSC_FRAME_CONFIG_106 = 106,
    SRSC_FRAME_CONFIG_107 = 107,
    SRSC_FRAME_CONFIG_FIRMWARE_VERSION = 108,
    SRSC_FRAME_CONFIG_VBAT = 110,
    SRSC_FRAME_CONFIG_RFPWRDETECT = 111,
    SRSC_FRAME_CONFIG_STATE = 115,
    SRSC_FRAME_CONFIG_ERROR_FLAGS = 116,
    SRSC_FRAME_CONFIG_VDOP = 119,
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
    union {
        float values[20];

        struct {
            float voltageCU;                            /* Voltage at CU mode [µV] */
            float voltageR2;                            /* Voltage at R2 mode [µV] */
            float temperatureRefBlock;                  /* Temperature in reference metal block [°C] */
            float temperatureAir;                       /* Temperature outside [°C] */
            float temperatureHuSensor;                  /* Temperature next to humidity sensor [°C] */
            float temperatureChamber;                   /* Temperature in separate chamber [°C] */
            float temperatureO3Inlet;                   /* Temperature air inlet ECC ozone sensor [°C] */
            float currentO3Cell;                        /* Cell current of ECC ozone sensor [µA] */
            float __reserved8__;
            float __reserved9__;
            float __reserved10__;
            float __reserved11__;
            float __reserved12__;
            float __reserved13__;
            float __reserved14__;
            float __reserved15__;
            float humidity;                             /* Humidity [%] */
            float USensorHeating;                       /* Heating for U sensor (0=off, 1=on) */
            float USensorFrequency;                     /* Frequency of oscillator using U sensor [Hz] */
            float __reserved19__;
        };
    };
} SRSC_CookedMetrology;


typedef struct {
    char name[20];
    uint32_t sondeType;
    uint32_t sondeNumber;
    bool sondeNameKnown;
    uint16_t frameCounter;
    uint32_t frequencyKhz;
    bool isC34;
    bool isC50;
    bool hasO3;

    uint32_t info103;
    uint32_t info104;
    uint32_t info105;
    uint32_t info106;
    uint32_t info107;
    uint32_t firmwareVersion;
    float batteryVoltage;                   /* Battery voltage [V] */
    float rfPwrDetect;
    int state;
    uint32_t errorFlags;
    uint32_t vdop;
    uint32_t info120;
} SRSC_CookedConfig;


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
