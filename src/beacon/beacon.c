
#include <math.h>
#if !defined(M_PI)
#  define M_PI 3.14159265358979323846
#endif
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "lpclib.h"
#include "bsp.h"
#include "sys.h"
#include "beacon.h"
#include "beaconprivate.h"


/** Context */
typedef struct BEACON_Context {
    BEACON_Packet packet;

    BEACON_InstanceData *instance;
    float rxFrequencyHz;
    float rxOffset;

} BEACON_Context;

static BEACON_Context _beacon;


//TODO
LPCLIB_Result BEACON_open (BEACON_Handle *pHandle)
{
    BEACON_Handle handle = &_beacon;

    *pHandle = handle;

    handle->rxOffset = NAN;
    BEACON_DSP_initAudio();

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _BEACON_sendKiss (BEACON_InstanceData *instance)
{
    (void)instance;
}



LPCLIB_Result BEACON_processBlock (BEACON_Handle handle, void *buffer, uint32_t length, float rxFrequencyHz)
{
    (void)handle;
    (void)buffer;
    (void)length;
    (void)rxFrequencyHz;

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result BEACON_removeFromList (BEACON_Handle handle, uint32_t id)
{
    (void)handle;

    BEACON_InstanceData *instance = NULL;
    while (_BEACON_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }
            /* Remove sonde */
            _BEACON_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


