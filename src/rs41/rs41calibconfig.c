
#include "lpclib.h"
#include "app.h"
#include "rs41.h"
#include "rs41private.h"


#define RS41_MAX_SONDES         8


/* Points to list of calibration structures */
static RS41_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static RS41_InstanceData *_RS41_getInstanceDataStructure (const char *name)
{
    RS41_InstanceData *p;
    RS41_InstanceData *instance;

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
    if (numSondes >= RS41_MAX_SONDES) {
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
        instance = (RS41_InstanceData *)calloc(1, sizeof(RS41_InstanceData));
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
LPCLIB_Result _RS41_processConfigBlock (
        const RS41_SubFrameCalibConfig *rawConfig,
        RS41_InstanceData **instancePointer)
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

    /* Allocate new instance space if new sonde! */
    RS41_InstanceData *instance = _RS41_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (instance) {
        /* Add fragment to calibration data */
        int fragmentIndex = rawConfig->thisCalibIndex;

        if (fragmentIndex <= RS41_CALIBRATION_MAX_INDEX) {
            instance->fragmentValidFlags |= (1ull << fragmentIndex);

            memcpy(instance->rawData[fragmentIndex], rawConfig->calibFragment, sizeof(instance->rawData[fragmentIndex]));
        }

        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;

        /* Cook some other values */
        instance->frameCounter = rawConfig->frameCounter;
        instance->batteryVoltage = rawConfig->batteryVoltage100mV / 10.0f;
        instance->onDescent = (rawConfig->flags & (1u << 1)) ? true : false;
        if (_RS41_checkValidCalibration(instance, CALIB_FREQUENCY)) {
            instance->rxFrequencyMHz = 400.0 + (instance->frequency * 10) / 64000.0;
        }
    }

    return LPCLIB_SUCCESS;
}


/* Read 24-bit little-endian unsigned integer from memory */
uint32_t _RS41_readU24 (const uint8_t *p24)
{
    return p24[0] + 256 * p24[1] + 65536 * p24[2];
}


/* Read 24-bit little-endian signed integer from memory */
int32_t _RS41_readS24 (const uint8_t *p24)
{
    return (int32_t)(_RS41_readU24(p24) << 8) / 256;
}


/* Check if the calibration block contains valid data for a given purpose */
bool _RS41_checkValidCalibration(RS41_InstanceData *instance, uint64_t purpose)
{
    if (!instance) {
        return false;
    }

    return (instance->fragmentValidFlags & purpose) == purpose;
}


/* Iterate through instances */
bool _RS41_iterateInstance (RS41_InstanceData **instance)
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
void _RS41_deleteInstance (RS41_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    RS41_InstanceData **parent = &instanceList;
    RS41_InstanceData *p = NULL;
    while (_RS41_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

