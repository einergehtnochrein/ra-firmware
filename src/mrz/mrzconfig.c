
#include <inttypes.h>

#include "lpclib.h"
#include "app.h"
#include "mrz.h"
#include "mrzprivate.h"


#define MRZ_MAX_SONDES          2


/* Points to list of instance structures */
static MRZ_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static MRZ_InstanceData *_MRZ_getInstanceDataStructure (float frequencyMHz)
{
    MRZ_InstanceData *p;
    MRZ_InstanceData *instance;

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
    if (numSondes >= MRZ_MAX_SONDES) {
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
        instance = (MRZ_InstanceData *)calloc(1, sizeof(MRZ_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        instance->rxFrequencyMHz = frequencyMHz;

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



/* Process the config/calib block. */
LPCLIB_Result _MRZ_processConfigFrame (
        MRZ_Packet *packet,
        MRZ_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new calib space if new sonde! */
    MRZ_InstanceData *instance = _MRZ_getInstanceDataStructure(rxFrequencyHz / 1e6f);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    /* Add fragment to calibration data */
    int fragmentIndex = packet->thisCalibIndex - 1;

    if ((fragmentIndex >= 0) && (fragmentIndex < 16)) {
        instance->fragmentValidFlags |= (1ull << fragmentIndex);

        instance->calib.rawCalib[fragmentIndex] = packet->calibFragment;
    }

    /* Build name(s) if available */
    if (_MRZ_checkValidCalibration(instance, CALIB_SERIALSONDE)) {
        snprintf(instance->name, sizeof(instance->name), "%06"PRIu32, instance->calib.serialSonde);
    }

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    return result;
}


/* Check if the calibration block contains valid data for a given purpose */
bool _MRZ_checkValidCalibration(MRZ_InstanceData *instance, uint32_t purpose)
{
    if (!instance) {
        return false;
    }

    return (instance->fragmentValidFlags & purpose) == purpose;
}


/* Iterate through instances */
bool _MRZ_iterateInstance (MRZ_InstanceData **instance)
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
void _MRZ_deleteInstance (MRZ_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    MRZ_InstanceData **parent = &instanceList;
    MRZ_InstanceData *p = NULL;
    while (_MRZ_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}



