
#include "lpclib.h"
#include "app.h"
#include "cf06.h"
#include "cf06private.h"


#define CF06_MAX_SONDES          2


/* Points to list of instance structures */
static CF06_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static CF06_InstanceData *_CF06_getInstanceDataStructure (const char *name)
{
    CF06_InstanceData *p;
    CF06_InstanceData *instance;

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
    if (numSondes >= CF06_MAX_SONDES) {
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
        instance = (CF06_InstanceData *)calloc(1, sizeof(CF06_InstanceData));
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



/* Process the config/calib block. */
LPCLIB_Result _CF06_prepare (
        CF06_PayloadBlock1 *block1,
        CF06_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Format serial number */
    char s[20];
    snprintf(s, sizeof(s), "%lu%02lu%02lu%02lu",
             (block1->serial >>  0) & 0xFF,
             (block1->serial >>  8) & 0xFF,
             (block1->serial >> 16) & 0xFF,
             (block1->serial >> 24) & 0xFF);

    /* Allocate new calib space if new sonde! */
    CF06_InstanceData *instance = _CF06_getInstanceDataStructure(s);
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
bool _CF06_iterateInstance (CF06_InstanceData **instance)
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
void _CF06_deleteInstance (CF06_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    CF06_InstanceData **parent = &instanceList;
    CF06_InstanceData *p = NULL;
    while (_CF06_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

