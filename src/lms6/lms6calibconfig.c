
#include "lpclib.h"
#include "app.h"
#include "lms6.h"
#include "lms6private.h"


#define LMS6_MAX_SONDES         2


/* Points to list of calibration structures */
static LMS6_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static LMS6_InstanceData *_LMS6_getInstanceDataStructure (uint32_t serial)
{
    LMS6_InstanceData *p;
    LMS6_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (serial == p->serial) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= LMS6_MAX_SONDES) {
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
        instance = (LMS6_InstanceData *)calloc(1, sizeof(LMS6_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        instance->serial = serial;

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
LPCLIB_Result _LMS6_processConfigBlock (
        const LMS6_RawFrame *rawConfig,
        LMS6_InstanceData **instancePointer)
{
    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new instance space if new sonde! */
    LMS6_InstanceData *instance = _LMS6_getInstanceDataStructure(__REV(rawConfig->serial));
    *instancePointer = instance;

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    return LPCLIB_SUCCESS;
}


/* Iterate through instances */
bool _LMS6_iterateInstance (LMS6_InstanceData **instance)
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
void _LMS6_deleteInstance (LMS6_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    LMS6_InstanceData **parent = &instanceList;
    LMS6_InstanceData *p = NULL;
    while (_LMS6_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

