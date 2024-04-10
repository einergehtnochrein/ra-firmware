
#include "lpclib.h"
#include "app.h"
#include "mts01.h"
#include "mts01private.h"


#define MTS01_MAX_SONDES          2


/* Points to list of instance structures */
static MTS01_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static MTS01_InstanceData *_MTS01_getInstanceDataStructure (float frequencyMHz)
{
    MTS01_InstanceData *p;
    MTS01_InstanceData *instance;

    /* Count the number of sondes while traversing the list. */
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
    if (numSondes >= MTS01_MAX_SONDES) {
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
        instance = (MTS01_InstanceData *)calloc(1, sizeof(MTS01_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strcpy(instance->name, "MTS-01");

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
LPCLIB_Result _MTS01_prepare (
        MTS01_Packet *packet,
        MTS01_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    (void)packet;

    LPCLIB_Result result = LPCLIB_SUCCESS;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new calib space if new sonde! */
    MTS01_InstanceData *instance = _MTS01_getInstanceDataStructure(rxFrequencyHz / 1e6f);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    instance->rxFrequencyMHz = rxFrequencyHz / 1e6f;
//    instance->frameCounter = __REV16(packet->frameNumber);

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    return result;
}


/* Iterate through instances */
bool _MTS01_iterateInstance (MTS01_InstanceData **instance)
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
void _MTS01_deleteInstance (MTS01_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    MTS01_InstanceData **parent = &instanceList;
    MTS01_InstanceData *p = NULL;
    while (_MTS01_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

