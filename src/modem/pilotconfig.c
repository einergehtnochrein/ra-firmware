
#include "lpclib.h"
#include "app.h"
#include "pilot.h"
#include "pilotprivate.h"


#define PILOT_MAX_SONDES         2


/* Points to list of calibration structures */
static PILOT_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static PILOT_InstanceData *_PILOT_getInstanceDataStructure (const char *name)
{
    PILOT_InstanceData *p;
    PILOT_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (!strcmp(p->hashName, name)) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= PILOT_MAX_SONDES) {
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
        instance = (PILOT_InstanceData *)calloc(1, sizeof(PILOT_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strcpy(instance->hashName, name);

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
LPCLIB_Result _PILOT_processConfigBlock (
        const struct _PILOT_Payload *rawPayload,
        PILOT_InstanceData **instancePointer)
{
    (void)rawPayload;
    
    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new instance space if new sonde! */
    PILOT_InstanceData *instance = _PILOT_getInstanceDataStructure("PilotSonde");
    *instancePointer = instance;

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    return LPCLIB_SUCCESS;
}


/* Iterate through instances */
bool _PILOT_iterateInstance (PILOT_InstanceData **instance)
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
void _PILOT_deleteInstance (PILOT_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    PILOT_InstanceData **parent = &instanceList;
    PILOT_InstanceData *p = NULL;
    while (_PILOT_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

