
#include "lpclib.h"
#include "app.h"
#include "lms6.h"
#include "lms6private.h"


#define LMS6_MAX_SONDES         2


/* Points to list of calibration structures */
static LMS6_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static LMS6_InstanceData *_LMS6_getInstanceDataStructure (const char *name)
{
    LMS6_InstanceData *p;
    LMS6_InstanceData *instance;

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
    if (numSondes >= LMS6_MAX_SONDES) {
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
        instance = (LMS6_InstanceData *)calloc(1, sizeof(LMS6_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strncpy(instance->name, name, sizeof(instance->name) - 1);
        instance->name[sizeof(instance->name) - 1] = 0;
        //instance->temperatureTx = NAN;

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


#if 0
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
        instance->temperatureRef = rawConfig->temperatureRef;
        instance->txPower_dBm = (rawConfig->txPower == 0) ? 1 : (-1 + 3 * rawConfig->txPower);
        if ((1 <= rawConfig->cryptoMode) && (rawConfig->cryptoMode <= 4)) {
            instance->is_SGM = true;
            if ((3 <= rawConfig->cryptoMode) && (rawConfig->cryptoMode <= 4)) {
                instance->encrypted = true;
            }
        }

        if (_RS41_checkValidCalibration(instance, CALIB_FREQUENCY)) {
            instance->rxFrequencyMHz = 400.0f + (instance->params.frequency * 10) / 64000.0f;
        }

        /* Last fragment contains volatile data */
        if (fragmentIndex == RS41_CALIBRATION_MAX_INDEX) {
            instance->killCounterRefFrame = instance->frameCounter;
            instance->killCounterRefCount = instance->params.killCountdown;
            instance->temperatureTx = instance->params.intTemperatureRadio;
        }
    }

    return LPCLIB_SUCCESS;
}
#endif

/* Iterate through instances */
bool _LMS6_iterateInstance (LMS6_InstanceData **instance)
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
void _LMS6_deleteInstance (LMS6_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    LMS6_InstanceData **parent = &instanceList;
    LMS6_InstanceData *p = NULL;
    while (_LMS6_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

