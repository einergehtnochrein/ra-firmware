
#ifndef __IMET54PRIVATE_H
#define __IMET54PRIVATE_H


#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "imet54.h"
#include "gps.h"


#define IMET54_EXTRA_MAX_INDEX 10

typedef __PACKED(union {
    uint8_t dat8[432];
    uint32_t dat32[108];
}) IMET54_RawData;


typedef uint8_t IMET54_SubFrameMainRaw[112];

typedef __PACKED(struct {
    uint32_t serial;
    uint32_t time_milliseconds;
    int32_t latitude;
    int32_t longitude;
    int32_t altitude;
    uint32_t reserved14;
    uint32_t reserved18;
    float temperature;
    float humidity;
    float temperatureRH;
    uint16_t flags;
    uint8_t usedSats;
    uint8_t fragmentIndex;
    uint8_t fragment[8];
    uint32_t crc;
}) IMET54_SubFrameMain;

typedef __PACKED(struct {
    uint32_t reserved00;
    uint32_t reserved04;
    uint32_t reserved08;
    uint32_t reserved0C;
    uint32_t reserved10;
    uint32_t reserved14;
    uint32_t reserved18;
    uint32_t reserved1C;
    uint32_t reserved20;
    uint16_t reserved24;
    uint16_t reserved26;
    uint16_t reserved28;
    float reserved2A;
    uint16_t reserved2E;
    uint8_t reserved30;
    uint8_t reserved31;
}) IMET54_Block2_Type1_6;


typedef uint8_t IMET54_SubFrame2Raw[320];

typedef __PACKED(struct {
    uint8_t xxx[316];
    uint32_t crc;
}) IMET54_SubFrame2Long;

typedef __PACKED(struct {
    uint32_t reserved00;
    uint32_t reserved04;
    uint32_t reserved08;
    uint32_t reserved0C;
    uint32_t reserved10;
    uint32_t reserved14;
    uint32_t reserved18;
    uint32_t reserved1C;
    uint32_t reserved20;
    uint8_t xxx[120];
    uint32_t crc;
}) IMET54_SubFrame2Short;


typedef uint8_t IMET54_SubFrame3Raw[304];

typedef __PACKED(struct {
    uint8_t xxx[152];
}) IMET54_SubFrame3;


typedef union {
    IMET54_RawData rawData;
    __PACKED(struct {
        __PACKED(struct {
            IMET54_SubFrameMainRaw frameMainRaw;
            __PACKED(union {
                IMET54_SubFrame2Raw frame2Raw;
                IMET54_SubFrame3Raw frame3Raw;
            });
        });
    });
} IMET54_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
    float humidity;                             /* Relative humidity [%] */
    float temperatureRH;                        /* Temperature of humidity sensor [°C] */
} IMET54_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;
    uint8_t usedSats;
    uint8_t visibleSats;
} IMET54_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _IMET54_InstanceData {
    struct _IMET54_InstanceData *next;
    uint32_t id;
    char name[20];
    uint16_t frameCounter;
    float rxFrequencyMHz;
    uint32_t lastUpdated;
    float rssi;
    uint64_t realTime;

    uint32_t lastGpsTime;
    IMET54_CookedGps gps;
    IMET54_CookedMetrology metro;

    uint32_t fragmentValidFlags;
    __PACKED(union {
        uint8_t rawData[IMET54_EXTRA_MAX_INDEX + 1][8];
        __PACKED(struct {
            uint16_t firmwareVersion;
            uint16_t reserved02;
            uint16_t reserved04;
            uint8_t reserved06;
            uint8_t reserved07;
            uint8_t reserved08;
            uint8_t block2Type;
            uint16_t reserved0A;
            uint8_t reserved0C;
            uint8_t reserved0D;
            uint16_t reserved0E;
            uint16_t reserved10;
            uint16_t reserved12;
            __PACKED(struct {
                uint8_t prn;            /* Bit7: 1=sat used, 0=sat not used, Bit[6:0]: PRN */
                uint8_t system_snr;     /* Bit[7:6]: 00=GPS, 01=GLONASS, 10=Galileo, 11=Beidou, Bit[5:0]: SNR */
            }) sats[32];
            uint16_t reserved54;
            uint8_t visibleSats;        /* Number of visible satellites in above list */
            uint8_t usedSats;           /* Number of used sats in position solution */
        });
    }) extra;
} IMET54_InstanceData;



LPCLIB_Result _IMET54_prepare (
        IMET54_SubFrameMain *frameMain,
        IMET54_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _IMET54_iterateInstance (IMET54_InstanceData **instance);

/* Remove an instance from the chain */
void _IMET54_deleteInstance (IMET54_InstanceData *instance);

/* Deinterleave 8-byte block */
void _IMET54_deinterleave (uint8_t *block);

/* Check Hamming parity bits. Put the result (half of the input size) into the output buffer.
 * Length must be a multiple of two.
 */
_Bool _IMET54_checkParity (uint8_t *inBuffer, int length, uint8_t *outBuffer);

/* Check block CRC */
_Bool _IMET54_checkCRC (
        uint8_t *buffer1, int length1,
        uint8_t *buffer2, int length2,
        uint32_t receivedCRC);

/* Process the payload */
LPCLIB_Result _IMET54_processPayloadMain (
        const IMET54_SubFrameMain *payload,
        IMET54_InstanceData *instance);

#endif
