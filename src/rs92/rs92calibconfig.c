
#include "lpclib.h"
#include "app.h"
#include "rs92.h"
#include "rs92private.h"


#define RS92_MAX_SONDES         8


/* Points to list of calibration structures */
static RS92_InstanceData *instanceList;


/* Get a new instance data structure for a sonde */
static RS92_InstanceData *_RS92_getInstanceDataStructure (const char *name)
{
    RS92_InstanceData *p;
    RS92_InstanceData *instance;

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
    if (numSondes >= RS92_MAX_SONDES) {
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

        /* Remove entry */
        _RS92_deleteInstance(instance);
    }

    /* We need a new calibration structure */
    instance = (RS92_InstanceData *)calloc(1, sizeof(RS92_InstanceData));

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strncpy(instance->name, name, sizeof(instance->name) - 1);
        instance->name[sizeof(instance->name) - 1] = 0;
        instance->metro.currentO3Cell = NAN;
        instance->metro.currentO3Pump = NAN;
        instance->metro.humidity = NAN;
        instance->metro.pressure = NAN;
        instance->metro.temperature = NAN;
        instance->metro.temperatureO3 = NAN;

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
LPCLIB_Result _RS92_processConfigBlock (
        const struct _RS92_ConfigBlock *rawConfig,
        RS92_InstanceData **instancePointer)
{
    char s[11];
    unsigned int i, j;


    /* Get the sonde name (remove spaces) */
    j = 0;
    for (i = 0; i < sizeof(rawConfig->name); i++) {
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

    /* We must have a valid name by now */
    if (strlen(s) == 0) {
        return LPCLIB_ERROR;
    }

    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new calib space if new sonde! */
    RS92_InstanceData *instance = _RS92_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (instance) {
        /* Add fragment to calibration data */
        int fragmentIndex = rawConfig->thisCalibIndex;

        if (fragmentIndex < 32) {
            instance->fragmentValidFlags |= (1u << fragmentIndex);

            memcpy(instance->rawData[fragmentIndex], rawConfig->calibFragment, sizeof(instance->rawData[fragmentIndex]));
        }

        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    /* Cook some other values */
    instance->frameCounter = rawConfig->frameCounter;
/*
    if (_RS92_checkValidCalibration(instance, CALIB_FREQUENCY)) {
        instance->rxFrequencyMHz = 400.0f + instance->frequency * 0.01;
    }
*/
    return LPCLIB_SUCCESS;
}


/* Check if the calibration block contains valid data for a given purpose */
bool _RS92_checkValidCalibration(RS92_InstanceData *instance, uint32_t purpose)
{
    if (!instance) {
        return false;
    }

    return (instance->fragmentValidFlags & purpose) == purpose;
}


/* Iterate through instances */
bool _RS92_iterateInstance (RS92_InstanceData **instance)
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
void _RS92_deleteInstance (RS92_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    RS92_InstanceData **parent = &instanceList;
    RS92_InstanceData *p = NULL;
    while (_RS92_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

