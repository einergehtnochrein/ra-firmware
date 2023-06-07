
#include <inttypes.h>
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
#include "windsond.h"
#include "windsondprivate.h"
#include "bch.h"


/** Context */
typedef struct WINDSOND_Context {
    WINDSOND_Packet packet;

    WINDSOND_InstanceData *instance;
    float rxFrequencyHz;

} WINDSOND_Context;

static WINDSOND_Context _windsond;


//TODO
LPCLIB_Result WINDSOND_open (WINDSOND_Handle *pHandle)
{
    WINDSOND_Handle handle = &_windsond;

    *pHandle = handle;

    return LPCLIB_SUCCESS;
}



/* Send report */
static void _WINDSOND_sendKiss (WINDSOND_InstanceData *instance)
{
}


LPCLIB_Result WINDSOND_processBlock (
        WINDSOND_Handle handle,
        SONDE_Type sondeType,
        void *buffer,
        uint32_t length,
        float rxFrequencyHz)
{
    (void)rxFrequencyHz;
    int nErrors;
    LPCLIB_Result result = LPCLIB_ILLEGAL_PARAMETER;

    if (length == sizeof(WINDSOND_Packet)) {
    }

    return result;
}


/* Send KISS position messages for all known sondes */
LPCLIB_Result WINDSOND_resendLastPositions (WINDSOND_Handle handle)
{
    if (handle == LPCLIB_INVALID_HANDLE) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    WINDSOND_InstanceData *instance = NULL;
    while (_WINDSOND_iterateInstance(&instance)) {
        _WINDSOND_sendKiss(instance);
    }

    return LPCLIB_SUCCESS;
}


/* Remove entries from heard list */
LPCLIB_Result WINDSOND_removeFromList (WINDSOND_Handle handle, uint32_t id, float *frequency)
{
    (void)handle;

    WINDSOND_InstanceData *instance = NULL;
    while (_WINDSOND_iterateInstance(&instance)) {
        if (instance->id == id) {
            /* Remove reference from context if this is the current sonde */
            if (instance == handle->instance) {
                handle->instance = NULL;
            }

            /* Let caller know about sonde frequency */
            *frequency = instance->rxFrequencyMHz * 1e6f;

            /* Remove sonde */
            _WINDSOND_deleteInstance(instance);
            break;
        }
    }

    return LPCLIB_SUCCESS;
}


