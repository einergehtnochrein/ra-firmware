
#ifndef __JINYANGPRIVATE_H
#define __JINYANGPRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "jinyang.h"
#include "gps.h"


typedef enum {
    JINYANG_MODEL_UNKNOWN = 0,
    JINYANG_MODEL_RSG20A,
} JINYANG_Model;

typedef uint8_t JINYANG_RawData[40];


typedef __PACKED(struct {
    uint32_t time;
    float latitude;
    float longitude;
    float altitude;
    uint32_t unk1;
    uint16_t unk2;
    uint16_t velocity;
    uint16_t direction;
    uint16_t lastone;
}) JINYANG_SubFrameGps;


typedef __PACKED(struct {
    uint32_t data[7];
}) JINYANG_SubFrame1;


typedef __PACKED(struct {
    uint32_t data[7];
}) JINYANG_SubFrame3;


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
            JINYANG_SubFrame1 frame1;
            JINYANG_SubFrame3 frame3;
        });
        uint16_t crc;
    });
} JINYANG_Packet;


typedef struct {
    float temperature;                          /* Temperature [°C] */
} JINYANG_CookedMetrology;


typedef struct {
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} JINYANG_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _JINYANG_InstanceData {
    struct _JINYANG_InstanceData *next;
    uint32_t id;
    uint16_t frameCounter;
    JINYANG_Model model;
    float rxFrequencyMHz;
    uint32_t lastUpdated;

    JINYANG_CookedGps gps;
    JINYANG_CookedMetrology metro;
JINYANG_SubFrameGps frameGps;
JINYANG_SubFrame1 frame1;
JINYANG_SubFrame3 frame3;
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
