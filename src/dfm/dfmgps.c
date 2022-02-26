
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "app.h"
#include "config.h"
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
            /* Do we know the sonde position? */
            if (!pObserverLLA) {
                /* No. Match the sonde based on RX frequency alone. */
                return p;

                /* Sonde is a match if it is within 10km in either direction (assuming latitude 50°) */
                float hdist = 4094.6f * fabs(pObserverLLA->lon - p->gps.observerLLA.lon);
                float vdist = 6370.0f * fabs(pObserverLLA->lat - p->gps.observerLLA.lat);
                if ((hdist < 10000.0f) && (vdist < 10000.0f)) {
                    /* This must be the sonde we already know! */
                    return p;
                }
            }
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



/* Process GPS subframe */
static void _DFM_processGps (DFM_InstanceData *instance, struct _DFM_GpsDetect *pDetect)
{
    int i;
    int32_t i32;
    int16_t i16;
    double latitude = NAN;
    double longitude = NAN;
    float altitude = NAN;
    float direction = NAN;
    float climbrate = NAN;
    float velocity = NAN;

    switch (instance->gps.mode) {
        case 2:
            latitude = (double)pDetect->fragment[2].i32;
            longitude = (double)pDetect->fragment[3].i32;
            altitude = (float)pDetect->fragment[4].i32;
            direction = (float)pDetect->fragment[3].i16;
            climbrate = (float)pDetect->fragment[4].i16;
            velocity = (float)pDetect->fragment[2].i16;
            break;

        case 3: case 4:
            latitude = (double)pDetect->fragment[1].i32;
            longitude = (double)pDetect->fragment[2].i32;
            altitude = (float)pDetect->fragment[3].i32;
            direction = (float)pDetect->fragment[1].i16;
            climbrate = (float)pDetect->fragment[2].i16;
            velocity = (float)pDetect->fragment[0].i16;
            break;
    }

    latitude *= 1e-7;
    latitude = fmin(latitude, 90.0);
    latitude = fmax(latitude, -90.0);
    latitude *= M_PI / 180.0;
    longitude *= 1e-7;
    longitude = fmin(longitude, 180.0);
    longitude = fmax(longitude, -180.0);
    longitude *= M_PI / 180.0;
    altitude *= 1e-2f;
    direction *= 1e-2f;
    direction = fmodf(direction, 360.0f);
    direction *= M_PI / 180.0f;
    climbrate *= 1e-2f;
    velocity *= 1e-2f;

    instance->gps.observerLLA.lat = latitude;
    instance->gps.observerLLA.lon = longitude;
    instance->gps.observerLLA.alt = altitude;
    instance->gps.observerLLA.direction = direction;
    instance->gps.observerLLA.climbRate = climbrate;
    instance->gps.observerLLA.velocity = velocity;

    float ehpe = NAN;
    float evpe = NAN;
    float geoidCorrection = CONFIG_getGeoidHeight();

    if (instance->gps.mode == 2) {
        ehpe = (float)((uint32_t)pDetect->fragment[5].i32 & 0xFFFF) / 100.0f;
        evpe = (float)pDetect->fragment[5].i16 / 100.0f;
        geoidCorrection = (float)(pDetect->fragment[5].i32 / 65536) / 100.0f;

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
    }
    else {
         for (i = 0; i < 12; i++) {
             instance->gps.sats[i].PRN = 0;
             instance->gps.sats[i].snr = 0;
         }
    }

    instance->gps.ehpe = ehpe;
    instance->gps.evpe = evpe;
    instance->gps.geoidCorrection = geoidCorrection;
    /* Geoid height correction */
    instance->gps.observerLLA.alt += instance->gps.geoidCorrection;

    /* Convert to ECEF coordinate system */
    GPS_convertLLA2ECEF(&instance->gps.observerLLA, &instance->gps.observerECEF);
    instance->gps.newPosition = true;

    /* YYMMDD HHMM is the same for all modes */
    i32 = pDetect->fragment[8].i32;
    instance->gps.utc.year = ((uint32_t)i32 >> 20) & 0xFFF;
    instance->gps.utc.month = ((uint32_t)i32 >> 16) & 0xF;
    instance->gps.utc.day = ((uint32_t)i32 >> 11) & 0x1F;
    instance->gps.utc.hour = ((uint32_t)i32 >> 6) & 0x1F;
    instance->gps.utc.minute = ((uint32_t)i32 >> 0) & 0x3F;

    if (pDetect->receivedFragmentsMask & 0x100) {
        instance->gps.usedSatsMask = pDetect->fragment[8].i16 >> 8;
    }
}


/* Process XDATA information in GPS subframe */
static void _DFM_processXdata (DFM_InstanceData *instance, struct _DFM_GpsDetect *pDetect)
{
    _Bool haveXdata = false;

    /* XDATA is available in GPS mode 4 only */
    if (instance->gps.mode == 4) {
        /* Need a non-zero header */
        instance->xdata.header = pDetect->fragment[3].i16;
        if (instance->xdata.header != 0) {
            instance->xdata.x0_32 = pDetect->fragment[4].i32;
            instance->xdata.x0_16 = pDetect->fragment[4].i16;
            instance->xdata.x1_32 = pDetect->fragment[5].i32;
            instance->xdata.x1_16 = pDetect->fragment[5].i16;
            instance->xdata.x2_32 = pDetect->fragment[6].i32;
            instance->xdata.x2_16 = pDetect->fragment[6].i16;
            instance->xdata.x3_32 = pDetect->fragment[7].i32;
            instance->xdata.x3_16 = pDetect->fragment[7].i16;

            haveXdata = true;
        }
    }

    instance->haveXdata = haveXdata;
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
        /* Insist on seeing fragments 0...7 (fragment 8 is less important) */
        if ((p->receivedFragmentsMask & 0x0FF) == 0x0FF) {
            /* Allocate new instance if new sonde! */
            DFM_InstanceData *instance = _DFM_getInstanceDataStructure(rxFrequencyHz / 1e6f, NULL);
            *instancePointer = instance;

            if (!instance) {
                return LPCLIB_ERROR;
            }

            /* Interpretation of fields depends on DFM GPS mode */
            instance->gps.mode = (p->fragment[0].i32 >> 8) & 0xFF;
            if ((instance->gps.mode < 2) || (instance->gps.mode > 4)) {
                instance->gps.mode = 2;
            }

            if (sondeType == SONDE_DFM_INVERTED) {
                instance->model = DFM_MODEL_DFM09_NEW;
            }

            _DFM_processGps(instance, p);
            _DFM_processXdata(instance, p);
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



