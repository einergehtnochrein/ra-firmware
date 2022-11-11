
#include "lpclib.h"
#include "app.h"
#include "imet54.h"
#include "imet54private.h"


#define IMET54_MAX_SONDES          2


/* Points to list of instance structures */
static IMET54_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static IMET54_InstanceData *_IMET54_getInstanceDataStructure (float frequencyMHz, const char *name)
{
    IMET54_InstanceData *p;
    IMET54_InstanceData *instance;

    /* Count the number of sondes while traversing the list. */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if ((p->rxFrequencyMHz == frequencyMHz) && !strcmp(p->name, name)) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= IMET54_MAX_SONDES) {
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
        instance = (IMET54_InstanceData *)calloc(1, sizeof(IMET54_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strcpy(instance->name, name);
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
LPCLIB_Result _IMET54_prepare (
        IMET54_SubFrameMain *frameMain,
        IMET54_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;
    char name[20];

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    snprintf(name, sizeof(name), "%"PRIu32, frameMain->serial);

    /* Allocate new calib space if new sonde! */
    IMET54_InstanceData *instance = _IMET54_getInstanceDataStructure(rxFrequencyHz / 1e6f, name);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    /* Add parameter fragment */
    int fragmentIndex = frameMain->fragmentIndex;

    if (fragmentIndex <= IMET54_EXTRA_MAX_INDEX) {
        instance->fragmentValidFlags |= (1u << fragmentIndex);

        memcpy(instance->extra.rawData[fragmentIndex], frameMain->fragment, sizeof(instance->extra.rawData[fragmentIndex]));
    }
    instance->gps.visibleSats = instance->extra.visibleSats;

//    instance->frameCounter = __REV16(packet->frameNumber);

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    return result;
}


/* Iterate through instances */
bool _IMET54_iterateInstance (IMET54_InstanceData **instance)
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
void _IMET54_deleteInstance (IMET54_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    IMET54_InstanceData **parent = &instanceList;
    IMET54_InstanceData *p = NULL;
    while (_IMET54_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

