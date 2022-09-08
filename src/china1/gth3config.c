
#include "lpclib.h"
#include "app.h"
#include "gth3.h"
#include "gth3private.h"


#define GTH3_MAX_SONDES          2


/* Points to list of instance structures */
static GTH3_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static GTH3_InstanceData *_GTH3_getInstanceDataStructure (const char *name)
{
    GTH3_InstanceData *p;
    GTH3_InstanceData *instance;

    /* Count the number of sondes while traversing the list. */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (!strcmp(p->name, name)) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= GTH3_MAX_SONDES) {
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
        instance = (GTH3_InstanceData *)calloc(1, sizeof(GTH3_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strncpy(instance->name, name, sizeof(instance->name) - 1);

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



static uint8_t _GTH3_bcd2bin (uint8_t bcd)
{
    return 10 * (bcd / 16) + bcd % 16;
}



/* Process the config/calib block. */
LPCLIB_Result _GTH3_prepare (
        GTH3_Payload *payload,
        GTH3_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Format serial number */
    char s[20];
    snprintf(s, sizeof(s), "%02u%02u%02u%02u",
        _GTH3_bcd2bin(payload->serial[0]),
        _GTH3_bcd2bin(payload->serial[1]),
        _GTH3_bcd2bin(payload->serial[2]),
        _GTH3_bcd2bin(payload->serial[3]));

    /* Allocate new calib space if new sonde! */
    GTH3_InstanceData *instance = _GTH3_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    instance->rxFrequencyMHz = rxFrequencyHz / 1e6f;
    //instance->frameCounter = packet->frameCounter;

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    return result;
}


/* Iterate through instances */
bool _GTH3_iterateInstance (GTH3_InstanceData **instance)
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
void _GTH3_deleteInstance (GTH3_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    GTH3_InstanceData **parent = &instanceList;
    GTH3_InstanceData *p = NULL;
    while (_GTH3_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

