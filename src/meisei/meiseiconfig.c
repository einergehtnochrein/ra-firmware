
#include "lpclib.h"
#include "app.h"
#include "meisei.h"
#include "meiseiprivate.h"


#define MEISEI_MAX_SONDES         4


/* Points to list of DFM instance structures */
static MEISEI_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static MEISEI_InstanceData *_MEISEI_getInstanceDataStructure (float frequencyMHz)
{
    MEISEI_InstanceData *p;
    MEISEI_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
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
    if (numSondes >= MEISEI_MAX_SONDES) {
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
        instance = (MEISEI_InstanceData *)calloc(1, sizeof(MEISEI_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
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
LPCLIB_Result _MEISEI_processConfigFrame (
        MEISEI_Packet *packet,
        MEISEI_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    LPCLIB_Result result = LPCLIB_SUCCESS;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new calib space if new sonde! */
    MEISEI_InstanceData *instance = _MEISEI_getInstanceDataStructure(rxFrequencyHz / 1e6f);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    /* Cook some other values */
    instance->rxFrequencyMHz = rxFrequencyHz / 1e6f;
    instance->frameCounter = _MEISEI_getPayloadHalfWord(packet->fields, 0);
    if ((instance->frameCounter % 2) == 0) {
        instance->_even1 = _MEISEI_getPayloadHalfWord(packet->fields, 1);
        instance->_even2 = _MEISEI_getPayloadHalfWord(packet->fields, 2);
        instance->_even3 = _MEISEI_getPayloadHalfWord(packet->fields, 3);
        instance->_even4 = _MEISEI_getPayloadHalfWord(packet->fields, 4);
        instance->_even5 = _MEISEI_getPayloadHalfWord(packet->fields, 5);
        instance->_even6 = _MEISEI_getPayloadHalfWord(packet->fields, 6);
        instance->_even7 = _MEISEI_getPayloadHalfWord(packet->fields, 7);
        instance->_even8 = _MEISEI_getPayloadHalfWord(packet->fields, 8);
        instance->_even9 = _MEISEI_getPayloadHalfWord(packet->fields, 9);
        instance->_even10 = _MEISEI_getPayloadHalfWord(packet->fields, 10);
        instance->_even11 = _MEISEI_getPayloadHalfWord(packet->fields, 11);
    }
    else {
        instance->_odd1 = _MEISEI_getPayloadHalfWord(packet->fields, 1);
        instance->_odd2 = _MEISEI_getPayloadHalfWord(packet->fields, 2);
        instance->_odd3 = _MEISEI_getPayloadHalfWord(packet->fields, 3);
        instance->_odd4 = _MEISEI_getPayloadHalfWord(packet->fields, 4);
        instance->_odd5 = _MEISEI_getPayloadHalfWord(packet->fields, 5);
        instance->_odd6 = _MEISEI_getPayloadHalfWord(packet->fields, 6);
        instance->_odd7 = _MEISEI_getPayloadHalfWord(packet->fields, 7);
        instance->_odd8 = _MEISEI_getPayloadHalfWord(packet->fields, 8);
        instance->_odd9 = _MEISEI_getPayloadHalfWord(packet->fields, 9);
        instance->_odd10 = _MEISEI_getPayloadHalfWord(packet->fields, 10);
        instance->_odd11 = _MEISEI_getPayloadHalfWord(packet->fields, 11);
    }

    return result;
}


/* Iterate through instances */
bool _MEISEI_iterateInstance (MEISEI_InstanceData **instance)
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
void _MEISEI_deleteInstance (MEISEI_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    MEISEI_InstanceData **parent = &instanceList;
    MEISEI_InstanceData *p = NULL;
    while (_MEISEI_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}



