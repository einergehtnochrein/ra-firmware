
#include "lpclib.h"
#include "app.h"
#include "ncar.h"
#include "ncarprivate.h"


#define RD41_MAX_SONDES         3


/* Points to list of calibration structures */
static NCAR_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static NCAR_InstanceData *_NCAR_getInstanceDataStructure (const char *name)
{
    NCAR_InstanceData *p;
    NCAR_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
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
    if (numSondes >= RD41_MAX_SONDES) {
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
        instance = (NCAR_InstanceData *)calloc(1, sizeof(NCAR_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strncpy(instance->name, name, sizeof(instance->name) - 1);
        instance->name[sizeof(instance->name) - 1] = 0;

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
LPCLIB_Result _RD41_processConfigBlocks (
        const RD41_SubFrameFrameNumber *rawFrame,
        const RD41_SubFrameConfig *rawConfig,
        NCAR_InstanceData **instancePointer)
{
    char s[11];
//    unsigned int i, j;

    strcpy(s, "RD41");
#if 0
    /* Get the sonde name (remove spaces) */
    j = 0;
    for (i = 0; i < sizeof(raw->name); i++) {
        /* Copy if not a space character */
        if (rawConfig->name[i] != ' ') {
            s[j] = rawConfig->name[i];
            ++j;
        }

        /* Probably never happens, but just in case: Test for premature end of string. */
        if (!rawConfig->name[i]) {
            break;
        }
    }
    s[j] = 0;
#endif
    /* We must have a valid name by now */
    if (strlen(s) == 0) {
        return LPCLIB_ERROR;
    }

    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new instance space if new sonde! */
    NCAR_InstanceData *instance = _NCAR_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;

        /* Cook some other values */
        instance->frameCounter = __REV16(rawFrame->frameCounter_BE);
        instance->batteryVoltage = __REV16(rawConfig->battery_voltage_BE) / 1e3f;
    }

    return LPCLIB_SUCCESS;
}


/* Iterate through instances */
bool _NCAR_iterateInstance (NCAR_InstanceData **instance)
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
void _NCAR_deleteInstance (NCAR_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    NCAR_InstanceData **parent = &instanceList;
    NCAR_InstanceData *p = NULL;
    while (_NCAR_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

