
#include "lpclib.h"
#include "app.h"
#include "m20.h"
#include "m20private.h"


#define M20_MAX_SONDES         4


/* Points to list of calibration structures */
static M20_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static M20_InstanceData *_M20_getInstanceDataStructure (const char *name)
{
    M20_InstanceData *p;
    M20_InstanceData *instance;

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
    if (numSondes >= M20_MAX_SONDES) {
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
        instance = (M20_InstanceData *)calloc(1, sizeof(M20_InstanceData));
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
LPCLIB_Result _M20_processConfigBlock (
        const struct _M20_Payload *payload,
        M20_InstanceData **instancePointer)
{
    char s[20];

    /* Get the sonde name */
    snprintf(s, sizeof(s), "%02X-%02X-%02X",
            payload->inner.serial[0],
            payload->inner.serial[1],
            payload->inner.serial[2]
            );

    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new instance space if new sonde! */
    M20_InstanceData *instance = _M20_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    return LPCLIB_SUCCESS;
}


/* Iterate through instances */
bool _M20_iterateInstance (M20_InstanceData **instance)
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
void _M20_deleteInstance (M20_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    M20_InstanceData **parent = &instanceList;
    M20_InstanceData *p = NULL;
    while (_M20_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

