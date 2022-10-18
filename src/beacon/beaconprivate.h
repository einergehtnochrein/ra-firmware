
#ifndef __BEACONPRIVATE_H
#define __BEACONPRIVATE_H


#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "beacon.h"
#include "gps.h"


enum {
    PDF1_USER_PROTOCOL_EPIRB_MARITIME                       = 0x02,
    PDF1_USER_PROTOCOL_EPIRB_RADIO_CALL_SIGN                = 0x06,
    PDF1_USER_PROTOCOL_AVIATION                             = 0x01,
    PDF1_USER_PROTOCOL_SERIAL                               = 0x03,
    PDF1_USER_PROTOCOL_TEST                                 = 0x07,
    PDF1_USER_PROTOCOL_ORBITOGRAPHY                         = 0x00,
    PDF1_USER_PROTOCOL_NATIONAL                             = 0x04,
};

enum {
    PDF1_LOCATION_PROTOCOL_STANDARD_EPIRB                   = 0x02,
    PDF1_LOCATION_PROTOCOL_STANDARD_ELT                     = 0x03,
    PDF1_LOCATION_PROTOCOL_STANDARD_ELT_SERIAL              = 0x04,
    PDF1_LOCATION_PROTOCOL_STANDARD_ELT_AIRCRAFT_OPERATOR   = 0x05,
    PDF1_LOCATION_PROTOCOL_STANDARD_EPIRB_SERIAL            = 0x06,
    PDF1_LOCATION_PROTOCOL_STANDARD_PLB_SERIAL              = 0x07,
    PDF1_LOCATION_PROTOCOL_STANDARD_SHIP_SECURITY           = 0x0C,

    PDF1_LOCATION_PROTOCOL_NATIONAL_ELT                     = 0x08,
    PDF1_LOCATION_PROTOCOL_NATIONAL_EPIRB                   = 0x0A,
    PDF1_LOCATION_PROTOCOL_NATIONAL_PLB                     = 0x0B,

    PDF1_LOCATION_PROTOCOL_TEST_STANDARD                    = 0x0E,
    PDF1_LOCATION_PROTOCOL_TEST_NATIONAL                    = 0x0F,

    PDF1_LOCATION_PROTOCOL_RLS                              = 0x0D,
};


typedef uint8_t BEACON_RawData[15];

typedef __PACKED(union {
    BEACON_RawData rawData;
}) BEACON_Packet;


typedef struct {
    bool longMessage;
    uint16_t countryCode;
    uint8_t protocolFlag;
    uint8_t protocol;
    bool isLocationProtocol;

    uint8_t userSerialType;
    uint32_t serialNumber;

    float latitudeCoarse;
    float longitudeCoarse;
    uint8_t auxRL;
    char hexID[16];
} BEACON_CookedPDF1;

typedef struct {
    bool valid;
    double latitude;
    double longitude;
    bool have121MHzHoming;
} BEACON_CookedPDF2;


/* Data that needs to be stored for every instance. */
typedef struct _BEACON_InstanceData {
    struct _BEACON_InstanceData *next;
    uint32_t id;
    float rxFrequencyMHz;
    float rxOffset;
    bool emergency;
    float rssi;

    uint32_t lastUpdated;
    
    BEACON_CookedPDF1 pdf1;
    BEACON_CookedPDF2 pdf2;
} BEACON_InstanceData;



LPCLIB_Result _BEACON_processConfigFrame (BEACON_InstanceData **instancePointer);

/* Iterate through instances */
bool _BEACON_iterateInstance (BEACON_InstanceData **instance);

/* Remove an instance from the chain */
void _BEACON_deleteInstance (BEACON_InstanceData *instance);

void BEACON_DSP_processAudio (const int32_t *rawAudio, float *cookedAudio, int nSamples);
void BEACON_DSP_initAudio (void);

void _BEACON_getField (uint32_t fieldDescriptor, uint8_t *dest);
LPCLIB_Result _BEACON_processPDF1 (BEACON_CookedPDF1 *cooked);
LPCLIB_Result _BEACON_processPDF2 (const BEACON_CookedPDF1 *pdf1, BEACON_CookedPDF2 *cooked);

#endif
