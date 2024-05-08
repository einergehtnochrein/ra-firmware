
#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "gps.h"
#include "lms6.h"
#include "lms6private.h"
#include "sys.h"
#include "xdata.h"


/** Context */
typedef struct LMS6_Context {
    uint8_t *pRawData;

    bool thisFrameNumberValid;

    float rxFrequencyHz;
    int nCorrectedErrors;

    LMS6_InstanceData *instance;
} LMS6_Context;


static LMS6_Context _lms6;


//TODO
LPCLIB_Result LMS6_open (LMS6_Handle *pHandle)
{
    *pHandle = &_lms6;

    return LPCLIB_SUCCESS;
}


/* Send position as a KISS packet */
static void _LMS6_sendKiss (LMS6_InstanceData *instance)
{
}


LPCLIB_Result LMS6_processBlock (
        LMS6_Handle handle,
        void *buffer,
        uint32_t numBits,
        float rxFrequencyHz,
        float rssi,
        uint64_t realTime)
{
    bool longFrame = false;


    /* Return if raw data is not the size of a RS(255,223) frame (in symbols!) */
    if (numBits != 2*255*8) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    handle->pRawData = buffer;

    /* Remember RX frequency (difference to nominal sonde frequency will be reported of frequency offset) */
    handle->rxFrequencyHz = rxFrequencyHz;

    return LPCLIB_SUCCESS;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result LMS6_resendLastPositions (LMS6_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    LMS6_InstanceData *instance = NULL;
    while (_LMS6_iterateInstance(&instance)) {
        _LMS6_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result LMS6_removeFromList (LMS6_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    LMS6_InstanceData *instance = NULL;
    while (_LMS6_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;
            
            /* Remove sonde */
            _LMS6_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}

