
#include "lpclib.h"
#include "app.h"
#include "dfm.h"
#include "dfmprivate.h"


#define DFM_MAX_SONDES         8


/* Points to list of DFM instance structures */
static DFM_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static DFM_InstanceData *_DFM_getInstanceDataStructure (float frequencyMHz)
{
    DFM_InstanceData *p;
    DFM_InstanceData *instance;

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
    if (numSondes >= DFM_MAX_SONDES) {
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
        instance = (DFM_InstanceData *)calloc(1, sizeof(DFM_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        instance->rxFrequencyMHz = frequencyMHz;
        instance->detectorState = DFM_DETECTOR_FIND_NANALOG;
        instance->metro.temperature = NAN;
        instance->metro.humidity = NAN;
        instance->metro.pressure = NAN;
        instance->metro._ref3 = NAN;
        instance->metro._ref4 = NAN;
        instance->metro.batteryVoltage = NAN;
        instance->metro.cpuTemperature = NAN;

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



struct {
    int32_t i32;
    uint32_t n;
} dfm_config_unknown[16];


/* Process the config/calib block. */
LPCLIB_Result _DFM_processConfigBlock (
        const DFM_SubFrameConfig *rawConfig,
        DFM_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    char s[11];
    unsigned int i;
    LPCLIB_Result result = LPCLIB_SUCCESS;


int32_t i32 = 0;
for (i = 0; i < 6; i++) {
    i32 = (i32 << 4) | rawConfig->h[i];
}
dfm_config_unknown[rawConfig->type].i32 = i32;
dfm_config_unknown[rawConfig->type].n++;

    /* Valid pointer to take the output value required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new calib space if new sonde! */
    DFM_InstanceData *instance = _DFM_getInstanceDataStructure(rxFrequencyHz / 1e6f);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    /* Config channel */
    uint8_t channel = rawConfig->type;

    /* Time markers */
    uint32_t thisTime = os_time;
    float delta = 10.0f * (thisTime - instance->confDetect.lastTime);
    float delta0 = 10.0f * (thisTime - instance->confDetect.lastCh0Time);
    instance->confDetect.lastTime = thisTime;

    /* See if the detector still needs to work out the exact type of sonde */
    switch (instance->detectorState) {
    /* Determine the number of fast analog channels */
    case DFM_DETECTOR_FIND_NANALOG:
        {
            /* Compare the interval between channel 0 transmissions */
            if (channel == 0) {
                /* If interval appears reasonable, remember config frame length */
                if ((delta0 > 1.5f*DFM_FRAME_LENGTH_MILLISEC) && (delta0 < 16.5*DFM_FRAME_LENGTH_MILLISEC)) {
                    instance->confDetect.numAnalog = lrintf(delta0 / DFM_FRAME_LENGTH_MILLISEC) - 1;
                    if (++instance->confDetect.nDetections >= 3) {
                        instance->numAnalog = instance->confDetect.numAnalog;
                        instance->detectorState = DFM_DETECTOR_FIND_CONFIG_STRUCTURE;

                        instance->confDetect.maxChannel = 0;
                        instance->confDetect.nDetections = 0;
                    }
                }
                else {
                    instance->confDetect.nDetections = 0;
                }
                instance->confDetect.lastCh0Time = thisTime;
            }
            else {
                /* Channels other than 0: If there is any gap (undetected subframe), reset detector */
                if (abs(delta - DFM_FRAME_LENGTH_MILLISEC) > DFM_FRAME_LENGTH_MILLISEC / 2.0f) {
                    instance->confDetect.lastCh0Time = 0;
                }
            }
        }
        break;

    /* Find the highes channel number used for config data. That channel carries the sonde number. */
    case DFM_DETECTOR_FIND_CONFIG_STRUCTURE:
        {
            /* Remember time of channel 0 */
            if (channel == 0) {
                instance->confDetect.lastCh0Time = thisTime;
            }

            /* The slow channels must appear at a certain distance from channel 0 */
            if ((channel >= instance->numAnalog) &&
                (delta0 < (instance->numAnalog + 0.5f) * DFM_FRAME_LENGTH_MILLISEC) &&
                (delta0 > (instance->numAnalog - 0.5f) * DFM_FRAME_LENGTH_MILLISEC)) {

                if (channel < instance->confDetect.prevChannel) {
                    /* Previous channel was the maximum. Wait until it's the same for a few cycles. */

                    if ((instance->confDetect.prevChannel == instance->confDetect.maxChannel) &&
                        (instance->confDetect.prevNibble0 == instance->confDetect.maxChannelNibble0)) {
                        ++instance->confDetect.nDetections;

                        if (instance->confDetect.nDetections >= 2) {
                            instance->maxConfigChannel = instance->confDetect.maxChannel;
                            instance->maxConfigChannelNibble0 = instance->confDetect.maxChannelNibble0;

                            /* We expect a serial number if the max channel is 6 or higher */
                            if (instance->maxConfigChannel >= 6) {
                                instance->detectorState = DFM_DETECTOR_FIND_NAME;
                            }
                            else {
                                /* Otherwise there is no serial number (DFM06 old) and we use a generic name */
                                instance->config.sondeNumber = 0;
                                strcpy(instance->name, "DFM-06");
                                instance->config.sondeNameKnown = true;
                                instance->config.isDFM06 = true;
                                instance->detectorState = DFM_DETECTOR_READY;
                            }

                            instance->confDetect.nDetections = 0;
                        }
                    }
                    else {
                        instance->confDetect.maxChannel = instance->confDetect.prevChannel;
                        instance->confDetect.maxChannelNibble0 = instance->confDetect.prevNibble0;
                        instance->confDetect.nDetections = 1;
                    }
                }
                instance->confDetect.prevChannel = channel;
                instance->confDetect.prevNibble0 = rawConfig->h[0];
            }
        }
        break;

    /* Get the sonde name (and sonde type, DFM-06/DFM-09) from the highest config channel */
    case DFM_DETECTOR_FIND_NAME:
        {
            if (instance->platform == SONDE_DFM09) {
                if (channel == instance->maxConfigChannel) {
                    if (rawConfig->h[5] == 0) {
                        instance->confDetect.sondeNumber = 0
                            | (rawConfig->h[1] << 28)
                            | (rawConfig->h[2] << 24)
                            | (rawConfig->h[3] << 20)
                            | (rawConfig->h[4] << 16)
                            ;
                    }
                    else if (rawConfig->h[5] == 1) {
                        instance->confDetect.sondeNumber |= 0
                            | (rawConfig->h[1] << 12)
                            | (rawConfig->h[2] << 8)
                            | (rawConfig->h[3] << 4)
                            | (rawConfig->h[4] << 0)
                            ;

                        if (instance->confDetect.sondeNumber == instance->confDetect.prevSondeNumber) {
                            ++instance->confDetect.nDetections;

                            if (instance->confDetect.nDetections >= 2) {
                                instance->config.sondeNumber = instance->confDetect.sondeNumber;
                                instance->detectorState = DFM_DETECTOR_READY;

                                /* Make sonde name */
                                sprintf(s, "%ld", instance->config.sondeNumber);
                                strcpy(instance->name, s);
                                instance->config.sondeNameKnown = true;
                            }
                        }
                        else {
                            instance->confDetect.prevSondeNumber = instance->confDetect.sondeNumber;
                            instance->confDetect.nDetections = 1;
                        }
                    }
                    else {
                        instance->confDetect.nDetections = 0;
                    }
                }
            }
            if (instance->platform == SONDE_DFM06) {
                if (channel == instance->maxConfigChannel) {
                    instance->confDetect.sondeNumber = 0;
                    for (i = 0; i < 6; i++) {
                        /* Detect illegal characters (must be a decimal number) */
                        if (rawConfig->h[i] > 9) {
                            break;
                        }

                        instance->confDetect.sondeNumber = (10 * instance->confDetect.sondeNumber) + rawConfig->h[i];
                    }
                    if ((i == 6) && (instance->confDetect.sondeNumber == instance->confDetect.prevSondeNumber)) {
                        ++instance->confDetect.nDetections;

                        if (instance->confDetect.nDetections >= 3) {
                            instance->config.sondeNumber = instance->confDetect.sondeNumber;
                            instance->detectorState = DFM_DETECTOR_READY;

                            /* Make sonde name */
                            sprintf(s, "%ld", instance->config.sondeNumber);
                            strcpy(instance->name, s);
                            instance->config.sondeNameKnown = true;
                        }
                    }
                    else {
                        instance->confDetect.prevSondeNumber = instance->confDetect.sondeNumber;
                        instance->confDetect.nDetections = 0;
                    }
                }
            }
        }
        break;

    case DFM_DETECTOR_READY:
        /* process metrology */
        result = _DFM_processMetrologyBlock(rawConfig, instance);
        break;
    }

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    /* Cook some other values */
    instance->config.frequencyKhz = lroundf(rxFrequencyHz / 1000.0f);

    return result;
}


/* Iterate through instances */
bool _DFM_iterateInstance (DFM_InstanceData **instance)
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
void _DFM_deleteInstance (DFM_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    DFM_InstanceData **parent = &instanceList;
    DFM_InstanceData *p = NULL;
    while (_DFM_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}



