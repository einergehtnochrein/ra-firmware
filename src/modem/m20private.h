
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



typedef __PACKED(struct {
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
    uint16_t pressure;      // LE Pressure [1/16 hPa]
    uint8_t vbat;           // Battery voltage, upper 8 bits of 12-bit ADC result (ADC_IN8, PB0)
    int8_t cpuTemperature;  // CPU temperature sensor [0.4 °C]
    uint8_t adc_pb1_pc3[3]; // LE, ADC inputs from heater current sensor
    uint8_t reserved2A;
    uint16_t reserved2B;    // from EEPROM 024 [15:0]
    uint8_t reserved2D;     // from EEPROM 000 [23:16]
    uint16_t humidityCalibration;   // from EEPROM 01C [15:0]
    uint16_t reserved30;    // LE, Calibration value corresponding to temperature range
    uint8_t satStatus[5];   // 3 bits each from 13 satellites (?)
    uint8_t adc_pc0_pc1[3]; // LE, ADC inputs from extension connector (if not in UART mode)
    uint8_t reserved3A;     // from EEPROM 010 [23:16]
    uint8_t reserved3B;     // constant 0
    uint8_t flags;          // Status flags
    uint8_t heaterPower;    // Power in heater is: P=((heaterPower+145)/10.0) mW
    uint16_t adc_pc2;       // LE, ADC input from unknown source
    uint16_t reserved40;    // from EEPROM 024 [31:16]
    uint8_t version;        // program/packet version number?
    uint16_t crc;
}) M20_Packet;


typedef struct {
    double gpstime;
    LLA_Coordinate observerLLA;

    uint8_t sats[13];
} M20_CookedGps;


typedef struct {
    float batteryVoltage;
    float temperature;
    float humidity;
    float pressure;                 // Measured by LPS22HB sensor [hPa]
    float cpuTemperature;           // Temperature measured by STM32 sensor [°C]

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
        const M20_Packet *rawConfig,
        M20_InstanceData **instancePointer);

LPCLIB_Result _M20_processPayloadInner (
        const struct _M20_PayloadInner *payload,
        M20_CookedGps *cookedGps,
        M20_CookedMetrology *cookedMetro);

LPCLIB_Result _M20_processPayload (
        const M20_Packet *payload,
        _Bool valid,
        M20_CookedGps *cookedGps,
        M20_CookedMetrology *cookedMetro);

/* Iterate through instances */
bool _M20_iterateInstance (M20_InstanceData **instance);

/* Remove an instance from the chain */
void _M20_deleteInstance (M20_InstanceData *instance);

#endif
