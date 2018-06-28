
#include <math.h>
#include <stdlib.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "dfm.h"
#include "dfmprivate.h"

struct {
    int32_t i32;
    int16_t i16;
    uint32_t n;
} dfm_gps_unknown[16];



/* Process regular sonde */
static void _DFM_processGpsNormal (DFM_InstanceData *instance)
{
    uint8_t n;
    int i;
    double d;
    float f;
    int32_t i32;
    int16_t i16;


    /* Regular message */
    d = ((double)instance->gpsDetect.fragment[2].i32 / 1e7) * (M_PI / 180.0);
    if ((d >= -90.0) && (d <= 90.0)) {
        instance->gps.observerLLA.lat = d;
        i16 = instance->gpsDetect.fragment[2].i16;
        instance->gps.observerLLA.velocity = (float)i16 / 100.0f;

        d = (double)instance->gpsDetect.fragment[3].i32;
        instance->gps.observerLLA.lon = (d / 1e7) * (M_PI / 180.0);
        uint16_t dir = (uint16_t)instance->gpsDetect.fragment[3].i16;
        if (dir >= 360*100) {
            dir = 360*100-1;
        }
        instance->gps.observerLLA.direction = (float)dir / 100.0f * (M_PI / 180.0f);
        d = (double)instance->gpsDetect.fragment[4].i32;
        instance->gps.observerLLA.alt = d / 100.0;
        f = (float)instance->gpsDetect.fragment[4].i16;
        instance->gps.climbRate = f / 100.0f;
        f = (float)((uint32_t)instance->gpsDetect.fragment[5].i32 & 0xFFFF);
        if (instance->config.isPS15) {
            instance->gps.hdop = NAN;
        }
        else {
            instance->gps.hdop = f / 1000.0f;
        }

        i32 = instance->gpsDetect.fragment[6].i32;
        i16 = instance->gpsDetect.fragment[6].i16;
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

        i32 = instance->gpsDetect.fragment[7].i32;
        i16 = instance->gpsDetect.fragment[7].i16;
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

        i32 = instance->gpsDetect.fragment[8].i32;
        instance->gps.utc.year = ((uint32_t)i32 >> 20) & 0xFFF;
        instance->gps.utc.month = ((uint32_t)i32 >> 16) & 0xF;
        instance->gps.utc.day = ((uint32_t)i32 >> 11) & 0x1F;
        instance->gps.utc.hour = ((uint32_t)i32 >> 6) & 0x1F;
        instance->gps.utc.minute = ((uint32_t)i32 >> 0) & 0x3F;

        i16 = instance->gpsDetect.fragment[8].i16;
        if (!instance->config.isPS15) {
//            instance->gps.usedSats = (uint32_t)i16 / 256;
        }

        // [1].i32 is a mask that indicates which PRN is used for this
        // position solution. Bit0=PRN1, bit31=PRN32
        i32 = instance->gpsDetect.fragment[1].i32;
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
static void _DFM_processGpsBurkinaFaso (DFM_InstanceData *instance)
{
    int i;
    double d;
    int32_t i32;


    d = ((double)instance->gpsDetect.fragment[1].i32 / 1e7) * (M_PI / 180.0);
    if ((d >= -90.0) && (d <= 90.0)) {
        instance->gps.observerLLA.lat = d;
        d = (double)instance->gpsDetect.fragment[2].i32;
        instance->gps.observerLLA.lon = (d / 1e7) * (M_PI / 180.0);
        instance->gps.observerLLA.velocity = NAN;
        instance->gps.observerLLA.direction = NAN;
        d = (double)instance->gpsDetect.fragment[3].i32;
        instance->gps.observerLLA.alt = d / 100.0;
        instance->gps.climbRate = NAN;
        instance->gps.hdop = NAN;

        for (i = 0; i < 12; i++) {
            instance->gps.sats[i].PRN = 0;
            instance->gps.sats[i].snr = 0;
        }

        /* Get GPS time of that data field is available */
        if (instance->gpsDetect.receivedFragmentsMask & (1u << 8)) {
            i32 = instance->gpsDetect.fragment[8].i32;
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
        DFM_InstanceData *instance)
{
    int i;
    uint32_t t = os_time;

    int32_t i32;
    int16_t i16;
    uint8_t channel = rawGps->type;

    /* When we receive a channel number lower than the previous one, we process the message */
    if (channel < instance->gpsDetect.prevChannel) {
        /* Check duration of last message (sort out fragmented reception spanning multiple message times) */
        float delta = 10.0f * (instance->gpsDetect.lastTime - instance->gpsDetect.lastCh0Time); //TODO
        if (delta < 10.5f * DFM_FRAME_LENGTH_MILLISEC) {
            /* Insist on seeing fragments 1...7 (fragments 0 and 8 are less important) */
            if ((instance->gpsDetect.receivedFragmentsMask & 0x0FE) == 0x0FE) {
                /* Check for the Burkina Faso syndrom... */
                int africaEvidence = 0;
                if ((instance->gpsDetect.fragment[4].i32 == 0) && (instance->gpsDetect.fragment[4].i16 == 0)) {
                    ++africaEvidence;
                }
                if (instance->gpsDetect.fragment[1].i32 == instance->gpsDetect.fragment[5].i32) {
                    ++africaEvidence;
                }
                if (instance->gpsDetect.fragment[2].i32 == instance->gpsDetect.fragment[6].i32) {
                    ++africaEvidence;
                }
                if (instance->gpsDetect.fragment[1].i16 == instance->gpsDetect.fragment[6].i16) {
                    ++africaEvidence;
                }
                if (instance->gpsDetect.fragment[3].i32 == instance->gpsDetect.fragment[7].i32) {
                    ++africaEvidence;
                }

                instance->gps.inBurkinaFaso = africaEvidence >= 3;

                if (instance->gps.inBurkinaFaso) {
                    _DFM_processGpsBurkinaFaso(instance);
                }
                else {
                    _DFM_processGpsNormal(instance);
                }
            }
        }

        /* This message has been processed */
        memset(&instance->gpsDetect, 0, sizeof(instance->gpsDetect));
    }

    /* Store new message fragment */
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

    instance->gpsDetect.fragment[channel].i32 = i32;
    instance->gpsDetect.fragment[channel].i16 = i16;
    instance->gpsDetect.receivedFragmentsMask |= 1u << channel;
    instance->gpsDetect.prevChannel = channel;
    instance->gpsDetect.lastTime = t;
    if (channel == 0) {
        instance->gpsDetect.lastCh0Time = t;
    }

    return LPCLIB_SUCCESS;
}


