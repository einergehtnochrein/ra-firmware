
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "app.h"
#include "dfm.h"
#include "dfmprivate.h"

struct {
    int32_t i32;
    int16_t i16;
    uint32_t n;
} dfm_gps_unknown[16];



#define DFM_MAX_SONDES         8


static struct _DFM_GpsDetect {
    struct {
        int32_t i32;
        int16_t i16;
    } fragment[16];
    uint32_t receivedFragmentsMask;
    uint32_t lastFragmentTime;          /* RX time of previous fragment */
    uint8_t prevChannel;
} _dfmGpsDetectContext;


/* Points to list of DFM instance structures */
static DFM_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static DFM_InstanceData *_DFM_getInstanceDataStructure (float frequencyMHz, LLA_Coordinate *pObserverLLA)
{
    DFM_InstanceData *p;
    DFM_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (p->rxFrequencyMHz == frequencyMHz) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= DFM_MAX_SONDES) {
        uint32_t oldest = (uint32_t)-1;

        p = instanceList;
        instance = instanceList;
        while (p) {
            if (p->lastUpdated < oldest) {
                oldest = p->lastUpdated;
                instance = p;
            }

            p = p->next;
        }
    }
    else {
        /* We need a new calibration structure */
        instance = (DFM_InstanceData *)calloc(1, sizeof(DFM_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        instance->rxFrequencyMHz = frequencyMHz;
        instance->detectorState = DFM_DETECTOR_FIND_NANALOG;
        instance->metro.temperature = NAN;
        instance->metro.humidity = NAN;
        instance->metro.pressure = NAN;
        instance->metro._ref3 = NAN;
        instance->metro._ref4 = NAN;
        instance->metro.batteryVoltage = NAN;
        instance->metro.cpuTemperature = NAN;

        /* Insert into list */
        p = instanceList;
        if (!p) {
            instanceList = instance;
        }
        else {
            while (p) {
                if (!p->next) {
                    p->next = instance;
                    break;
                }

                p = p->next;
            }
        }
    }

    return instance;
}



/* Process regular sonde */
static void _DFM_processGpsNormal (DFM_InstanceData *instance, struct _DFM_GpsDetect *pDetect)
{
    uint8_t n;
    int i;
    double d;
    float f;
    int32_t i32;
    int16_t i16;


    /* Regular message */
    d = ((double)pDetect->fragment[2].i32 / 1e7) * (M_PI / 180.0);
    if ((d >= -90.0) && (d <= 90.0)) {
        instance->gps.observerLLA.lat = d;
        i16 = pDetect->fragment[2].i16;
        instance->gps.observerLLA.velocity = (float)i16 / 100.0f;

        d = (double)pDetect->fragment[3].i32;
        instance->gps.observerLLA.lon = (d / 1e7) * (M_PI / 180.0);
        uint16_t dir = (uint16_t)pDetect->fragment[3].i16;
        if (dir >= 360*100) {
            dir = 360*100-1;
        }
        instance->gps.observerLLA.direction = (float)dir / 100.0f * (M_PI / 180.0f);
        d = (double)pDetect->fragment[4].i32;
        instance->gps.observerLLA.alt = d / 100.0;
        f = (float)pDetect->fragment[4].i16;
        instance->gps.climbRate = f / 100.0f;
        f = (float)((uint32_t)pDetect->fragment[5].i32 & 0xFFFF);
        if (instance->model == DFM_MODEL_PS15) {
            instance->gps.hdop = NAN;
        }
        else {
            instance->gps.hdop = f / 1000.0f;
        }

        i32 = pDetect->fragment[6].i32;
        i16 = pDetect->fragment[6].i16;
        if ((((uint32_t)i32 >> 30) & 3) == 1) {
            instance->gps.sats[0].PRN  = ((uint32_t)i32 >> 24) & 0x3F;
            instance->gps.sats[1].PRN  = ((uint32_t)i32 >> 16) & 0x3F;
            instance->gps.sats[2].PRN  = ((uint32_t)i32 >>  8) & 0x3F;
            instance->gps.sats[3].PRN  = ((uint32_t)i32 >>  0) & 0x3F;
            instance->gps.sats[4].PRN  = ((uint16_t)i16 >>  8) & 0x3F;
            instance->gps.sats[5].PRN  = ((uint16_t)i16 >>  0) & 0x3F;
        }
        if ((((uint32_t)i32 >> 30) & 3) == 2) {
            instance->gps.sats[0].snr  = ((uint32_t)i32 >> 24) & 0x3F;
            instance->gps.sats[1].snr  = ((uint32_t)i32 >> 16) & 0x3F;
            instance->gps.sats[2].snr  = ((uint32_t)i32 >>  8) & 0x3F;
            instance->gps.sats[3].snr  = ((uint32_t)i32 >>  0) & 0x3F;
            instance->gps.sats[4].snr  = ((uint16_t)i16 >>  8) & 0x3F;
            instance->gps.sats[5].snr  = ((uint16_t)i16 >>  0) & 0x3F;
        }

        i32 = pDetect->fragment[7].i32;
        i16 = pDetect->fragment[7].i16;
        if ((((uint32_t)i32 >> 30) & 3) == 1) {
            instance->gps.sats[6].PRN  = ((uint32_t)i32 >> 24) & 0x3F;
            instance->gps.sats[7].PRN  = ((uint32_t)i32 >> 16) & 0x3F;
            instance->gps.sats[8].PRN  = ((uint32_t)i32 >>  8) & 0x3F;
            instance->gps.sats[9].PRN  = ((uint32_t)i32 >>  0) & 0x3F;
            instance->gps.sats[10].PRN  = ((uint16_t)i16 >>  8) & 0x3F;
            instance->gps.sats[11].PRN  = ((uint16_t)i16 >>  0) & 0x3F;
        }
        if ((((uint32_t)i32 >> 30) & 3) == 2) {
            instance->gps.sats[6].snr  = ((uint32_t)i32 >> 24) & 0x3F;
            instance->gps.sats[7].snr  = ((uint32_t)i32 >> 16) & 0x3F;
            instance->gps.sats[8].snr  = ((uint32_t)i32 >>  8) & 0x3F;
            instance->gps.sats[9].snr  = ((uint32_t)i32 >>  0) & 0x3F;
            instance->gps.sats[10].snr  = ((uint16_t)i16 >>  8) & 0x3F;
            instance->gps.sats[11].snr  = ((uint16_t)i16 >>  0) & 0x3F;
        }

        i32 = pDetect->fragment[8].i32;
        instance->gps.utc.year = ((uint32_t)i32 >> 20) & 0xFFF;
        instance->gps.utc.month = ((uint32_t)i32 >> 16) & 0xF;
        instance->gps.utc.day = ((uint32_t)i32 >> 11) & 0x1F;
        instance->gps.utc.hour = ((uint32_t)i32 >> 6) & 0x1F;
        instance->gps.utc.minute = ((uint32_t)i32 >> 0) & 0x3F;

        i16 = pDetect->fragment[8].i16;
        if (instance->model != DFM_MODEL_PS15) {
//            instance->gps.usedSats = (uint32_t)i16 / 256;
        }

        // [1].i32 is a mask that indicates which PRN is used for this
        // position solution. Bit0=PRN1, bit31=PRN32
        i32 = pDetect->fragment[1].i32;
        instance->gps.usedSatsMask = (uint32_t)i32;
        // Count number of used satellites in bitmask
        n = 0;
        for (i = 0; i < 31; i++) {
            if (instance->gps.usedSatsMask & (1u << i)) {
                ++n;
            }
        }
        // In case the number of satellites is not transmitted in field [8].i16,
        // use counted satellites from bitmask instead.
if(1){//        if (n > instance->gps.usedSats) {
            instance->gps.usedSats = n;
        }

        if (instance->gps.usedSats == 0) {
            instance->gps.observerLLA.lat = NAN;
            instance->gps.observerLLA.lon = NAN;
        }
        else {
            GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
        }

        instance->gps.newPosition = true;
    }
}


/* Process Graw test sonde with "Africa bit" */
static void _DFM_processGpsBurkinaFaso (DFM_InstanceData *instance, struct _DFM_GpsDetect *pDetect)
{
    int i;
    double d;
    int32_t i32;


    d = ((double)pDetect->fragment[1].i32 / 1e7) * (M_PI / 180.0);
    if ((d >= -90.0) && (d <= 90.0)) {
        instance->gps.observerLLA.lat = d;
        d = (double)pDetect->fragment[2].i32;
        instance->gps.observerLLA.lon = (d / 1e7) * (M_PI / 180.0);
        instance->gps.observerLLA.velocity = NAN;
        instance->gps.observerLLA.direction = NAN;
        d = (double)pDetect->fragment[3].i32;
        instance->gps.observerLLA.alt = d / 100.0;
        instance->gps.climbRate = NAN;
        instance->gps.hdop = NAN;

        for (i = 0; i < 12; i++) {
            instance->gps.sats[i].PRN = 0;
            instance->gps.sats[i].snr = 0;
        }

        /* Get GPS time of that data field is available */
        if (pDetect->receivedFragmentsMask & (1u << 8)) {
            i32 = pDetect->fragment[8].i32;
            instance->gps.utc.year = ((uint32_t)i32 >> 20) & 0xFFF;
            instance->gps.utc.month = ((uint32_t)i32 >> 16) & 0xF;
            instance->gps.utc.day = ((uint32_t)i32 >> 11) & 0x1F;
            instance->gps.utc.hour = ((uint32_t)i32 >> 6) & 0x1F;
            instance->gps.utc.minute = ((uint32_t)i32 >> 0) & 0x3F;
        }

        instance->gps.usedSats = 0;
        instance->gps.usedSatsMask = 0;

        if ((instance->gps.observerLLA.lat == 0) && (instance->gps.observerLLA.lon == 0)) {
            instance->gps.observerLLA.lat = NAN;
            instance->gps.observerLLA.lon = NAN;
        }
        else {
            GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
        }

        instance->gps.newPosition = true;
    }
}


LPCLIB_Result _DFM_processGpsBlock (
        const DFM_SubFrameGps *rawGps,
        DFM_InstanceData **instancePointer,
        float rxFrequencyHz,
        uint32_t rxTime,
        SONDE_Type sondeType)
{
    int i;
    int32_t i32;
    int16_t i16;
    uint8_t channel = rawGps->type;
    struct _DFM_GpsDetect *p = &_dfmGpsDetectContext;
    LPCLIB_Result result = LPCLIB_PENDING;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Can we process the previously accumulated fragments now? Yes if...
     * 1) ... the received channel number is below the previously received channel number.
     * 2) ... there already is a fragment with the same channel number.
     * 3) ... the time elapsed between this and the previous fragment is more than
     *    a GPS frame (>= 4.5 DFM frames. A GPS frame is made of 9 GPS fragments,
     *    and there are two GPS fragments per DFM frame).
     */
    _Bool processNow = false;
    if (channel < p->prevChannel) {
        processNow = true;
    }
    if (p->receivedFragmentsMask & (1u << channel)) {
        processNow = true;
    }
    float delta = 10.0f * (rxTime - p->lastFragmentTime);   /* NOTE: 10ms OS tick */
    if (delta > 4.5f * DFM_FRAME_LENGTH_MILLISEC) {
        processNow = true;
    }

    if (processNow) {
        /* Insist on seeing fragments 1...7 (fragments 0 and 8 are less important) */
        if ((p->receivedFragmentsMask & 0x0FE) == 0x0FE) {
            /* Allocate new instance if new sonde! */
            DFM_InstanceData *instance = _DFM_getInstanceDataStructure(rxFrequencyHz / 1e6f, NULL);
            *instancePointer = instance;

            if (!instance) {
                return LPCLIB_ERROR;
            }

            /* Check for the Burkina Faso syndrom... */
            int africaEvidence = 0;
            if ((p->fragment[4].i32 == 0) && (p->fragment[4].i16 == 0)) {
                ++africaEvidence;
            }
            if (p->fragment[1].i32 == p->fragment[5].i32) {
                ++africaEvidence;
            }
            if (p->fragment[2].i32 == p->fragment[6].i32) {
                ++africaEvidence;
            }
            if (p->fragment[1].i16 == p->fragment[6].i16) {
                ++africaEvidence;
            }
            if (p->fragment[3].i32 == p->fragment[7].i32) {
                ++africaEvidence;
            }

            instance->gps.inBurkinaFaso = africaEvidence >= 3;

            if (instance->gps.inBurkinaFaso) {
                instance->model = DFM_MODEL_DFM09_AFRICA;
                _DFM_processGpsBurkinaFaso(instance, p);
            }
            else {
                if (sondeType == SONDE_DFM_INVERTED) {
                    instance->model = DFM_MODEL_DFM09_NEW;
                }
                _DFM_processGpsNormal(instance, p);
            }
        }

        /* This GPS frame has been processed */
        memset(p, 0, sizeof(*p));

        result = LPCLIB_SUCCESS;
    }

    /* Store new GPS fragment */
    i32 = 0;
    for (i = 0; i < 8; i++) {
        i32 = (i32 << 4) | rawGps->d[i];
    }
    i16 = 0;
    for (i = 8; i < 12; i++) {
        i16 = (i16 << 4) | rawGps->d[i];
    }

dfm_gps_unknown[channel].i32 = i32;
dfm_gps_unknown[channel].i16 = i16;
dfm_gps_unknown[channel].n++;

    p->fragment[channel].i32 = i32;
    p->fragment[channel].i16 = i16;
    p->receivedFragmentsMask |= 1u << channel;
    p->prevChannel = channel;
    p->lastFragmentTime = rxTime;

    return result;
}



/* Iterate through instances */
bool _DFM_iterateInstance (DFM_InstanceData **instance)
{
    bool result = false;

    if (instance) {
        if (*instance == NULL) {
            if (instanceList) {
                *instance = instanceList;
                result = true;
            }
        }
        else {
            *instance = (*instance)->next;
            if (*instance) {
                result = true;
            }
        }
    }

    return result;
}



/* Remove an instance from the chain */
void _DFM_deleteInstance (DFM_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    DFM_InstanceData **parent = &instanceList;
    DFM_InstanceData *p = NULL;
    while (_DFM_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}



