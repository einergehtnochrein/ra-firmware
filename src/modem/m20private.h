
#ifndef __M20PRIVATE_H
#define __M20PRIVATE_H


#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "m20.h"
#include "gps.h"



/* A collection of all known packet types */
typedef struct {
    uint8_t packetLength;

    __PACKED(struct _M20_Payload {
        uint8_t packetType;

        __PACKED(struct _M20_PayloadInner {
            uint16_t analog01;      // LE
            uint16_t analog03;      // LE
            uint16_t analog05;      // LE
            uint8_t altitude[3];    // BE Altitude [1/100 m]
            int16_t speedE;         // BE Speed eastwards [1/100 m/s]
            int16_t speedN;         // BE Speed northwards [1/100 m/s]
            uint8_t tow[3];         // BE GPS time of week [seconds since Sunday 00:00]
            uint8_t serial[3];
            uint8_t counter;
            uint16_t crc;
        }) inner;

        int16_t climbRate;      // BE Climb rate [1/100 m/s]
        uint16_t week;          // BE
        int32_t latitude;       // BE Latitude  [10e-6 degrees]
        int32_t longitude;      // BE Longitude [10e-6 degrees]
        uint8_t reserved23[9];
        uint8_t reserved2C[18];
        int16_t reserved3E;     // LE
        uint8_t reserved40[3];
        uint16_t crc;
    }) packet69;
} M20_Packet;


typedef struct {
    double gpstime;
    LLA_Coordinate observerLLA;
} M20_CookedGps;


typedef struct {
    float batteryVoltage;
    float temperature;
    float humidity;
} M20_CookedMetrology;


/* Data that needs to be stored for every instance. */
typedef struct _M20_InstanceData {
    struct _M20_InstanceData *next;
    uint32_t id;
    char hashName[20];                          /* Hashable sonde name */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    uint16_t frameCounter;

    M20_CookedGps gps;
    M20_CookedMetrology metro;
} M20_InstanceData;


LPCLIB_Result _M20_processConfigBlock (
        const struct _M20_Payload *rawConfig,
        M20_InstanceData **instancePointer);

LPCLIB_Result _M20_processPayloadInner (
        const struct _M20_PayloadInner *payload,
        M20_CookedGps *cookedGps,
        M20_CookedMetrology *cookedMetro);

LPCLIB_Result _M20_processPayload (
        const struct _M20_Payload *payload,
        _Bool valid,
        M20_CookedGps *cookedGps);

/* Iterate through instances */
bool _M20_iterateInstance (M20_InstanceData **instance);

/* Remove an instance from the chain */
void _M20_deleteInstance (M20_InstanceData *instance);

#endif
