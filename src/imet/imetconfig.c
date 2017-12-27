
#include "lpclib.h"
#include "app.h"
#include "imet.h"
#include "imetprivate.h"


#define IMET_MAX_SONDES         1


/* Points to list of instance structures */
static IMET_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
IMET_InstanceData *_IMET_getInstanceDataStructure (float frequency)
{
    IMET_InstanceData *p;
    IMET_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (p->frequency == frequency) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= IMET_MAX_SONDES) {
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
        instance = (IMET_InstanceData *)calloc(1, sizeof(IMET_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        instance->frequency = frequency;
        instance->config.frequencyKhz = lroundf(frequency / 1000.0);
        strcpy(instance->name, "iMet-1");

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


/* Iterate through instances */
bool _IMET_iterateInstance (IMET_InstanceData **instance)
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

