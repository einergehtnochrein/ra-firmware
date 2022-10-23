
#ifndef __JINYANGPRIVATE_H
#define __JINYANGPRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "jinyang.h"
#include "gps.h"


typedef enum {
    JINYANG_MODEL_UNKNOWN = 0,
    JINYANG_MODEL_RSG20A,
} JINYANG_Model;

typedef __PACKED(union {
    uint8_t dat8[40];
    uint32_t dat32[10];
}) JINYANG_RawData;


typedef __PACKED(struct {
    uint32_t time;
    float latitude;
    float longitude;
    float altitude;
    uint16_t unk1;
    uint16_t unk2;
    uint16_t unk3;
    uint16_t velocity;
    uint16_t direction;
    uint16_t unk4;
}) JINYANG_SubFrameGps;


typedef __PACKED(struct {
    int16_t data[14];
}) JINYANG_SubFrame0;


typedef __PACKED(struct {
    int16_t data[14];
}) JINYANG_SubFrame2;


typedef union {
    JINYANG_RawData rawData;
    __PACKED(struct {
        uint8_t unk0;
        uint8_t unk1;
        uint8_t unk2;
        uint8_t unk3;
        uint8_t unk4;
        uint16_t frameCounter;
        uint8_t unk7;
        uint8_t unk8;
        uint8_t subType;
        __PACKED(union {
            JINYANG_SubFrameGps frameGps;
            JINYANG_SubFrame0 frame0;
            JINYANG_SubFrame2 frame2;
        });
        uint16_t crc;
    });
} JINYANG_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} JINYANG_CookedMetrology;


typedef struct {
    LLA_Coordinate observerLLA;
uint16_t unk1;
uint16_t unk2;
uint16_t unk3;
uint16_t unk4;
} JINYANG_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _JINYANG_InstanceData {
    struct _JINYANG_InstanceData *next;
    uint32_t id;
    uint16_t frameCounter;
    JINYANG_Model model;
    float rxFrequencyMHz;
    uint32_t lastUpdated;
    float rssi;
    uint64_t realTime;

    uint32_t lastGpsTime;
    JINYANG_CookedGps gps;
    JINYANG_CookedMetrology metro;
JINYANG_SubFrameGps frameGps;
JINYANG_SubFrame0 frame0;
JINYANG_SubFrame2 frame2;
} JINYANG_InstanceData;



LPCLIB_Result _JINYANG_processConfigFrame (
        JINYANG_Packet *packet,
        JINYANG_InstanceData **instancePointer,
        float rxFrequencyHz);

/* Iterate through instances */
bool _JINYANG_iterateInstance (JINYANG_InstanceData **instance);

/* Remove an instance from the chain */
void _JINYANG_deleteInstance (JINYANG_InstanceData *instance);

/* Process the GPS frame */
LPCLIB_Result _JINYANG_processGpsFrame (
        JINYANG_SubFrameGps *rawGps,
        JINYANG_InstanceData *instance);

#endif
