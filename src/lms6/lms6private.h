
#ifndef __LMS6PRIVATE_H
#define __LMS6PRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "lms6.h"
#include "gps.h"


typedef struct {
    float temperature;                  /* Temperature [°C] main (air) sensor */
    float pressure;                     /* Atmospheric pressure [hPa] */
} LMS6_CookedMetrology;


typedef struct {
    double gpstime;
    ECEF_Coordinate observerECEF;
    LLA_Coordinate observerLLA;
} LMS6_CookedGps;


/* Data that needs to be stored for every instance. */
typedef struct _LMS6_InstanceData {
    struct _LMS6_InstanceData *next;
    uint32_t id;
    char name[20];                              /* Sonde name */
    uint32_t lastUpdated;
    float rxFrequencyMHz;
} LMS6_InstanceData;


/* Iterate through instances */
bool _LMS6_iterateInstance (LMS6_InstanceData **instance);

/* Remove an instance from the chain */
void _LMS6_deleteInstance (LMS6_InstanceData *instance);

/* Check CRC of a data frame */
_Bool _LMS6_checkCRC (uint8_t *buffer, int length, uint16_t receivedCRC);
/* Reed-Solomon error correction */
LPCLIB_Result _LMS6_checkReedSolomon (uint8_t rawFrame[], int *pNumErrors);

#ifdef __cplusplus
}
#endif
#endif
