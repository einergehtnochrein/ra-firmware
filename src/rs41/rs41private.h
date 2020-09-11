
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
    RS41_SUBFRAME_GPS_RAW = 0x7D,
    RS41_SUBFRAME_XDATA = 0x7E,
    RS41_SUBFRAME_METROLOGY_SHORT = 0x7F,
    RS41_SUBFRAME_CRYPT80 = 0x80,
} RS41_SubFrameType;

#define RS41_CALIBRATION_MAX_INDEX      50



typedef __PACKED(struct {
    uint16_t frameCounter;
    char name[8];
    __PACKED(struct {
        uint8_t batteryVoltage100mV;        /* Battery voltage in multiples of 100 mV */
        uint16_t reserved00B;               /* Bit field (all 16 bits in use) */
        uint16_t flags;                     /* Bit 0: 0=Start phase, 1=Flight mode
                                             * Bit 1: 0=Ascent, 1=Descent
                                             * Bit 11: 0=VBATmin check disabled, 1=VBATmin check enabled
                                             * Bit 12: 0=VBAT ok, 1=VBAT too low
                                             */
        uint8_t reserved00F;                /* ? (can be either 0 or 6) */
        int8_t temperatureRef;              /* Reference temperature (@ PCB cutout) [°C] */
        uint16_t errorLog;                  /* Error flags:
                                             * 0: Low battery capacity
                                             * 1: No parameter setup
                                             * 2: TX init failure
                                             * 3: Not in TX state
                                             * 4: No add-on sensor data
                                             * 5: (various flash errors)
                                             * 6: PTU failure
                                             * 7: GPS init failure
                                             * 8: Invalid GPS messages
                                             * 9: Missing GPS messages
                                             * 10: T self-check failure
                                             * 11: U self-check failure
                                             * 12: Low regen temperature
                                             * 13: P-module not detected
                                             * 14: T, Tu or U check failed
                                             */
        uint16_t reserved013;
        uint8_t txPower;                    /* TX power level (0...7, see Si4032 data sheet) */
    });
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
    int16_t temperaturePressureSensor;
    uint16_t val14_16;
    uint16_t crc;
}) RS41_SubFrameMetrology;


/* The short metrology block contains only temperature and humidity values,
 * no pressure sensor data.
 * Never heard such a block myself, but it was reported by Zilog80:
 * https://www.fingers-welt.de/phpBB/viewtopic.php?f=14&t=43&start=2950
 */
typedef __PACKED(struct {
    __PACKED(struct {
        uint8_t current[3];
        uint8_t refmin[3];
        uint8_t refmax[3];
    }) adc[3];
    uint16_t crc;
}) RS41_SubFrameMetrologyShort;


/* The identification of fields in the following GPS structures
 * was inspired by the contributions of user "zilog80" in the "Finger" forum:
 * https://www.fingers-welt.de/phpBB/viewtopic.php?f=14&t=43&start=1975#p163997
 * Further information was taken from the following manual:
 * GPS.G6-SW-10018-F, www.u-blox.com
 */

typedef __PACKED(struct {
    uint32_t ecefX;
    uint32_t ecefY;
    uint32_t ecefZ;
    int16_t speedX;
    int16_t speedY;
    int16_t speedZ;
    uint8_t usedSats;
    uint8_t reserved;
    uint8_t dop;
    uint16_t crc;
}) RS41_SubFrameGpsPosition;


typedef __PACKED(struct {
    uint16_t gpsWeek;
    uint32_t timeOfWeek;
    __PACKED(struct {
        uint8_t prn;
        uint8_t cno_mesQI;
    }) sats[12];
    uint16_t crc;
}) RS41_SubFrameGpsInfo;


typedef __PACKED(struct {
    uint32_t minPrMes;                  /* Minimum PR measurement of all sats [m] (floor) */
    uint8_t reserved004;
    __PACKED(struct {
        uint32_t deltaPrMes;            /* PR measurement (delta to minPrMes) [1/256 m] */
                                        /* NOTE: In general also the satellite with the minimum PR
                                         *        will have a non-zero delta!
                                         */
        uint8_t doMes[3];               /* Doppler measurement [Hz], signed 24-bit int */
    }) sats[12];
    uint16_t crc;
}) RS41_SubFrameGpsRaw;


typedef struct {
    uint8_t prn;
    uint8_t cno;
    uint8_t mesQI;
    double pseudorange;
    int32_t doppler;
} RS41_SatInfo;


typedef struct {
    float T;                            /* Temperature [°C] main (air) sensor */
    float pressure;                     /* Atmospheric pressure [hPa] */
    float pressureAltitude;
    float temperaturePSensor;           /* Temperature [°C] pressure sensor */

    bool hasO3;

    // Humidity calculations. End results and intermediates
    float temperatureUSensor;           /* Temperature [°C] humidity sensor */
    float RH;                           /* Relative humidity [%] */
    float dewpoint;                     /* Dewpoint [°C] */
    float C;
    float Cp;
    float RHtu;
} RS41_CookedMetrology;


typedef struct {
    double gpstime;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
    float dop;
    uint8_t visibleSats;
    uint8_t usedSats;
} RS41_CookedGps;


typedef struct {
    RS41_SatInfo sats[12];
} RS41_RawGps;


/* Data that needs to be stored for every RS41 instance. */
typedef struct _RS41_InstanceData {
    struct _RS41_InstanceData *next;
    uint32_t id;
    char name[20];                              /* Sonde name */
    uint64_t fragmentValidFlags;                /* The 52 bits (if set) indicate validity of the corresponding fragment */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    float batteryVoltage;                       /* Battery voltage [V] */
    float temperatureTx;                        /* Temperature of Si4032 [°C] */
    float temperatureRef;                       /* Temperature of reference point on PCB (cutout) [°C] */
    uint16_t frameCounter;
    int8_t txPower_dBm;
    bool encrypted;                             /* Set for RS41-SGM military version */
    bool onDescent;                             /* Descent phase detected */
    int16_t killCounterRefFrame;                /* Last reference frame for kill counter */
    int16_t killCounterRefCount;                /* Kill counter in last reference frame */
    RS41_CookedGps gps;
    RS41_CookedMetrology metro;
    RS41_LogMode logMode;

    __PACKED(union {
        uint8_t rawData[RS41_CALIBRATION_MAX_INDEX + 1][16];
        __PACKED(struct {
            uint16_t crc16;                     /* CRC16 CCITT Checksum over range 0x002...0x31F */
            uint16_t frequency;                 /* 0x002: TX is on 400 MHz + (frequency / 64) * 10 kHz */
            uint8_t startupTxPower;             /* 0x004: TX power level at startup (1...7) */
            uint8_t reserved005;
            uint8_t reserved006;
            uint16_t reserved007;               /* 0x007:  ?? (some bitfield) [0],[1],[2],[3]. Init value = 0xE */
            uint16_t reserved009;               /* 0x009: ? */
            uint8_t reserved00B;
            uint8_t reserved00C;
            uint8_t serial[8];                  /* 0x00D: Sonde ID, 8 char, not terminated */
            uint16_t firmwareVersion;           /* 0x015: 10000*major + 100*minor + patch*/
            uint16_t reserved017;
            uint16_t minHeight4Flight;          /* 0x019: Height (meter above ground) where flight mode begins */
            uint8_t lowBatteryThreshold100mV;   /* 0x01B: (Default=18) Shutdown if battery voltage below this
                                                          threshold for some time (10s ?)
                                                */
            uint8_t nfcDetectorThreshold;       /* 0x01C: NFC detector threshold [25mV] (Default: 0x05 = 125mV) */
            uint8_t reserved01D;                /* 0x01D: ?? (Init value = 0xB4) */
            uint8_t reserved01E;                /* 0x01E: ?? (Init value = 0x3C) */
            uint16_t reserved01F;
            int8_t refTemperatureThreshold;     /* 0x021: Reference temperature threshold [°C] */
            uint8_t reserved022;
            uint16_t reserved023;
            uint16_t reserved025;
            int16_t flightKillFrames;           /* 0x027: Number of frames in flight until kill (-1 = disabled) */
            uint16_t reserved029;               /* 0x029: ? (Init value = 0) */
            uint8_t burstKill;                  /* 0x02B: Burst kill (0=disabled, 1=enabled) */
            uint8_t reserved02C;
            uint8_t reserved02D;
            uint16_t reserved02E;
            uint16_t reserved030;
            uint8_t reserved032;
            uint16_t reserved033;
            uint16_t reserved035;
            uint16_t reserved037;
            uint16_t reserved039;               /* 0x039: */
            uint8_t reserved03B;                /* 0x03B: */
            uint8_t reserved03C;                /* 0x03C: */
            float refResistorLow;               /* 0x03D: Reference resistor low (750 Ohms) */
            float refResistorHigh;              /* 0x041: Reference resistor high (1100 Ohms) */
            float refCapLow;                    /* 0x045: Reference capacitance low (0) */
            float refCapHigh;                   /* 0x049: Reference capacitance high (47 pF) */
            float taylorT[3];                   /* 0x04D: Tayor coefficients for main temperature calculation */
            float calT;                         /* 0x059: Calibration factor for main sensor */
            float polyT[6];                     /* 0x05D: */
            float calibU[2];                    /* 0x075: Calibration coefficients for humidity sensor */

            float matrixU[7][6];                /* 0x07D: Matrix for humidity sensor RH calculation */
            float taylorTU[3];                  /* 0x125: Coefficients for U sensor temperature calculation */
            float calTU;                        /* 0x131: Calibration factor for U temperature sensor */
            float polyTrh[6];                   /* 0x135:  */

            uint8_t reserved14D;                /* 0x14D: */
            uint32_t reserved14E;               /* 0x14E: */

            float f152;
            uint8_t u156;
            float f157;                         /* 0x157: ?? (Initialized by same value as calibU) */
            uint8_t reserved15B[0x160-0x15B];
            float f160[35];
            uint8_t startIWDG;                  /* 0x1EC: If ==0 or ==2: Watchdog IWDG will not be started */
            uint8_t parameterSetupDone;         /* 0x1ED: Set (!=0) if parameter setup was done */
            uint8_t reserved1EE;
            uint8_t reserved1EF;
            float f1F0[8];
            float pressureLaunchSite[2];        /* 0x210: Pressure [hPa] at launch site */
            __PACKED(struct {
                char variant[10];               /* 0x218: Sonde variant (e.g. "RS41-SG") */
                uint8_t mainboard[10];          /* 0x222: Name of mainboard (e.g. "RSM412") */
            }) names;
            __PACKED(struct {
                uint8_t mainboard[9];           /* 0x22C: Serial number of mainboard (e.g. "L1123553") */
                uint8_t text235[12];            /* 0x235: "0000000000" */
                uint16_t reserved241;           /* 0x241: */
                uint8_t pressureSensor[8];      /* 0x243: Serial number of pressure sensor (e.g. "N1310487") */
                uint16_t reserved24B;           /* 0x24B: */
            }) serials;
            uint16_t reserved24D;               /* 0x24D: */
            uint8_t reserved24F;
            uint8_t reserved250;
            uint16_t reserved251;               /* 0x251: (Init value = 0x21A = 538) */
            uint8_t xdataUartBaud;              /* 0x253: 1=9k6, 2=19k2, 3=38k4, 4=57k6, 5=115k2 */
            uint8_t reserved254;
            float cpuTempSensorVoltageAt25deg;  /* 0x255: CPU temperature sensor voltage at 25°C */
            uint8_t reserved259;
            uint8_t reserved25A[0x25E -0x25A];
            float matrixP[18];                  /* 0x25E: Coefficients for pressure sensor polynomial */
            float f2A6[17];
            uint8_t reserved2EA[0x2FA-0x2EA];
            uint16_t halfword2FA[9];
            float reserved30C;
            float reserved310;                  /* 0x310: */
            uint8_t reserved314;                /* 0x314: */
            uint8_t reserved315;                /* 0x315: */
            int16_t burstKillFrames;            /* 0x316: Number of active frames after burst kill */
            uint8_t reserved318[0x320-0x318];

            /* This is fragment 50. It only uses 14 valid bytes! */
            int16_t killCountdown;              /* 0x320: Counts frames remaining until kill (-1 = inactive) */
            uint8_t reserved322[6];
            int8_t intTemperatureCpu;           /* 0x328: Temperature [°C] of CPU */
            int8_t intTemperatureRadio;         /* 0x329: Temperature [°C] of radio chip */
            int8_t reserved32A;                 /* 0x32A: */
            uint8_t reserved32B;                /* 0x32B: */
            uint8_t reserved32C;                /* 0x32C: ? (the sum of two slow 8-bit counters) */
            uint8_t reserved32D;                /* 0x32D: ? (the sum of two slow 8-bit counters) */
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


/* Read 24-bit little-endian integer from memory */
uint32_t _RS41_readU24 (const uint8_t *p24);
int32_t _RS41_readS24 (const uint8_t *p24);


/* Check if the calibration block contains valid data for a given purpose */
#define CALIB_FREQUENCY             0x0000000000000001ll
#define CALIB_TEMPERATURE           0x00000000000000F8ll
#define CALIB_TEMPERATURE_U         0x00000000001C0018ll
#define CALIB_HUMIDITY              0x00000000001FFFF8ll
#define CALIB_PRESSURE              0x000007E000000000ll
#define CALIB_FLIGHTKILLTIMER       0x0000000000000004ll
#define CALIB_BURSTKILLTIMER        0x0002000000000000ll
#define CALIB_KILLCOUNTDOWN         0x0004000000000000ll
#define CALIB_MODELNAME             0x0000000600000000ll

bool _RS41_checkValidCalibration(RS41_InstanceData *instance, uint64_t purpose);

/* Iterate through instances */
bool _RS41_iterateInstance (RS41_InstanceData **instance);

/* Remove an instance from the chain */
void _RS41_deleteInstance (RS41_InstanceData *instance);

/* Process the metrology block(s).
 */
LPCLIB_Result _RS41_processMetrologyBlock (
        const RS41_SubFrameMetrology *rawMetro,
        RS41_CookedMetrology *cookedMetro,
        RS41_InstanceData *instance);

LPCLIB_Result _RS41_processMetrologyShortBlock (
        const RS41_SubFrameMetrologyShort *rawMetro,
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
        RS41_CookedGps *cookedGps,
        RS41_RawGps *raw);


/* Process the GPS raw satellite data.
 */
LPCLIB_Result _RS41_processGpsRawBlock (
        const RS41_SubFrameGpsRaw *p,
        RS41_RawGps *raw);

#endif
