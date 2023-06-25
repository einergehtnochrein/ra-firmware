
#include "lpclib.h"
#include "app.h"
#include "windsond.h"
#include "windsondprivate.h"


#define WINDSOND_MAX_SONDES         2


/* Points to list of instance structures */
static WINDSOND_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
WINDSOND_InstanceData *_WINDSOND_getInstanceDataStructure (float frequencyMHz, uint16_t id, uint8_t sid)
{
    WINDSOND_InstanceData *p;
    WINDSOND_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (p->rxFrequencyMHz == frequencyMHz) {
            if ((id != 0) && (id == p->name_id)) {
                /* Found it! */
                return p;
            }
            if ((sid != 0) && (sid == p->name_sid)) {
                /* Found it! */
                return p;
            }
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= WINDSOND_MAX_SONDES) {
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
        instance = (WINDSOND_InstanceData *)calloc(1, sizeof(WINDSOND_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        instance->rxFrequencyMHz = frequencyMHz;
        instance->name_id = id;
        instance->name_sid = sid;
        instance->metro.temperature = NAN;
        instance->metro.cpuTemperature = NAN;
        instance->gps.observerLLA.lat = NAN;
        instance->gps.observerLLA.lon = NAN;
        instance->gps.observerLLA.alt = NAN;

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
bool _WINDSOND_iterateInstance (WINDSOND_InstanceData **instance)
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
void _WINDSOND_deleteInstance (WINDSOND_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    WINDSOND_InstanceData **parent = &instanceList;
    WINDSOND_InstanceData *p = NULL;
    while (_WINDSOND_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}



