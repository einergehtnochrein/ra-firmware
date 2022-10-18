
#ifndef __M20PRIVATE_H
#define __M20PRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "m20.h"
#include "gps.h"



typedef __PACKED(struct {
    uint8_t packetType;             // 00 (this is 0x20 for the M20)

    __PACKED(struct _M20_PayloadInner {
        uint16_t humidity;          // 01 LE
        uint16_t adc_T;             // 03 LE Temperature main sensor, value in [11:0], range in [13:12]
        uint16_t adc_TU;            // 05 LE Temperature U sensor
        uint8_t altitude[3];        // 07 BE Altitude [1/100 m]
        int16_t speedE;             // 0A BE Speed eastwards [1/100 m/s]
        int16_t speedN;             // 0C BE Speed northwards [1/100 m/s]
        uint8_t tow[3];             // 0E BE GPS time of week [seconds since Sunday 00:00]
        uint8_t serial[3];          // 11
        uint8_t counter;            // 14
        uint16_t crc;               // 15
    }) inner;

    int16_t climbRate;              // 17 BE Climb rate [1/100 m/s]
    uint16_t week;                  // 19 BE
    int32_t latitude;               // 1B BE Latitude  [10e-6 degrees]
    int32_t longitude;              // 1F BE Longitude [10e-6 degrees]
    uint16_t pressure;              // 23 LE Pressure [1/16 hPa]
    uint8_t vbat;                   // 25 Battery voltage, upper 8 bits of 12-bit ADC result (ADC_IN8, PB0)
    int8_t cpuTemperature;          // 26 CPU temperature sensor [0.4 °C]
    uint8_t adc_pb1_pc3[3];         // 27 LE, ADC inputs from heater current sensor
    uint8_t xdataLength;            // 2A Length of XDATA block at end of frame
    uint16_t reserved2B;            // 2B from EEPROM 024 [15:0]
    uint8_t reserved2D;             // 2D from EEPROM 000 [23:16]
    uint16_t humidityCalibration;   // 2E from EEPROM 01C [15:0]
    uint16_t reserved30;            // 30 LE, Calibration value corresponding to temperature range
    uint8_t satStatus[5];           // 32 3 bits each from 13 satellites (?)
    uint8_t adc_pc0_pc1[3];         // 37 LE, ADC inputs from extension connector (if not in UART mode)
    uint8_t reserved3A;             // 3A from EEPROM 010 [23:16]
    uint8_t reserved3B;             // 3B constant 0
    uint8_t flags;                  // 3C Status flags
    uint8_t heaterPower;            // 3D Power in heater is: P=((heaterPower+145)/10.0) mW
    uint16_t boardTemperature;      // 3E LE, NTC sensor for board temperature (ADC_IN12, PC2)
    uint8_t reserved40;             // 40 from EEPROM 024 [23:16]
    __PACKED(union {
        uint8_t reserved41;         // 41 from EEPROM 024 [31:24]
        char xdata;                 // 41 Placeholder for the begin of the XDATA block
    });
    uint8_t version;                // 42 program/packet version number?
    uint16_t crc;                   // 43
}) M20_Packet;


typedef struct {
    double gpstime;
    LLA_Coordinate observerLLA;

    uint8_t sats[13];
} M20_CookedGps;


typedef struct {
    float batteryVoltage;
    float T;                        // Main temperature [°C]
    float TU;                       // Temperature of humidity sensor [°C]
    float humidity;
    float pressure;                 // Measured by LPS22HB sensor [hPa]
    float cpuTemperature;           // Temperature measured by STM32 sensor [°C]
    float boardTemperature;         // Temperature of PCB, measured by NTC [°C]

    float humidityCalibration;
    float adc_pb1;
    float adc_pc3;
    float adc_pc0;
    float adc_pc1;
} M20_CookedMetrology;


/* Data that needs to be stored for every instance. */
typedef struct _M20_InstanceData {
    struct _M20_InstanceData *next;
    uint32_t id;
    char hashName[20];                          /* Hashable sonde name */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
    uint16_t frameCounter;
    float rssi;

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
