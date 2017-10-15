
#include "lpclib.h"
#include "m10.h"
#include "m10private.h"


#define M10_MAX_SONDES         2


/* Points to list of calibration structures */
static M10_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static M10_InstanceData *_M10_getInstanceDataStructure (const char *name)
{
    M10_InstanceData *p;
    M10_InstanceData *instance;

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
    if (numSondes >= M10_MAX_SONDES) {
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
        instance = (M10_InstanceData *)malloc(sizeof(M10_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        memset(instance, 0, sizeof(M10_InstanceData));
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
LPCLIB_Result _M10_processConfigBlock (
        const struct _M10_ConfigBlock *rawConfig,
        M10_InstanceData **instancePointer)
{
    char s[12];

    /* Get the sonde name */
    snprintf(s, sizeof(s), "%X%02d-%X-%d%04d",
            rawConfig->serial[2] / 16,
            rawConfig->serial[2] % 16,
            rawConfig->serial[0] % 16,
            rawConfig->serial[4] / 32,
            rawConfig->serial[3] + 256 * (rawConfig->serial[4] % 32)
            );

    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new instance space if new sonde! */
    M10_InstanceData *instance = _M10_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    /* Cook some other values */
//    instance->frameCounter = rawConfig->frameCounter;

    return LPCLIB_SUCCESS;
}


/* Iterate through instances */
bool _M10_iterateInstance (M10_InstanceData **instance)
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
void _M10_deleteInstance (M10_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    M10_InstanceData **parent = &instanceList;
    M10_InstanceData *p = NULL;
    while (_M10_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

