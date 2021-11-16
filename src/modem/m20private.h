
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
            uint16_t humidity;          // LE
            uint16_t adc_temperature;   // LE, value in [11:0], range in [13:12]
            uint16_t analog05;          // LE
            uint8_t altitude[3];        // BE Altitude [1/100 m]
            int16_t speedE;             // BE Speed eastwards [1/100 m/s]
            int16_t speedN;             // BE Speed northwards [1/100 m/s]
            uint8_t tow[3];             // BE GPS time of week [seconds since Sunday 00:00]
            uint8_t serial[3];
            uint8_t counter;
            uint16_t crc;
        }) inner;

        int16_t climbRate;      // BE Climb rate [1/100 m/s]
        uint16_t week;          // BE
        int32_t latitude;       // BE Latitude  [10e-6 degrees]
        int32_t longitude;      // BE Longitude [10e-6 degrees]
        uint16_t reserved23;    // LE
        uint8_t vbat;           // Battery voltage, upper 8 bits of 12-bit ADC result (ADC_IN8, PB0)
        int8_t cpuTemperature;
        uint8_t adc_pb1_pc3[3]; // LE, ADC inputs from heater current sensor
        uint8_t reserved2A;
        uint16_t reserved2B;    // from EEPROM
        uint8_t reserved2D;     // from EEPROM
        uint16_t humidityCalibration;
        uint16_t reserved30;
        uint8_t satStatus[5];   // 3 bits each from 13 satellites (?)
        uint8_t adc_pc0_pc1[3]; // LE, ADC inputs from extension connector (if not in UART mode)
        uint8_t reserved3A;
        uint8_t reserved3B;
        uint8_t reserved3C;
        uint8_t reserved3D;
        uint16_t adc_pc2;       // LE, ADC input from unknown source
        uint16_t reserved40;
        uint8_t version;        // program/packet version number?
        uint16_t crc;
    }) packet69;
} M20_Packet;


typedef struct {
    double gpstime;
    LLA_Coordinate observerLLA;

    uint8_t sats[13];
} M20_CookedGps;


typedef struct {
    float batteryVoltage;
    float temperature;
    float humidity;
    float cpuTemperature;

    float humidityCalibration;
    float adc_pb1;
    float adc_pc3;
    float adc_pc0;
    float adc_pc1;
    float adc_pc2;
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
        M20_CookedGps *cookedGps,
        M20_CookedMetrology *cookedMetro);

/* Iterate through instances */
bool _M20_iterateInstance (M20_InstanceData **instance);

/* Remove an instance from the chain */
void _M20_deleteInstance (M20_InstanceData *instance);

#endif
