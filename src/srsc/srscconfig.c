
#include "lpclib.h"
#include "srsc.h"
#include "srscprivate.h"


#define SRSC_MAX_SONDES         3


/* Points to list of SRSC-C instance structures */
static SRSC_InstanceData *instanceList;


/* Get a new instance data structure for a new sonde */
static SRSC_InstanceData *_SRSC_getInstanceDataStructure (float frequencyMHz)
{
    SRSC_InstanceData *p;
    SRSC_InstanceData *instance;

    /* Check if we already know a sonde on that frequency.
     * Count the number of sondes while traversing the list.
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

    /* Sonde not yet in list. If we have reached the maximum number of sondes
     * that we want to track in parallel, do a garbage collection now:
     * Identify the least recently used entry and delete it.
     */
    if (numSondes >= SRSC_MAX_SONDES) {
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
        _SRSC_deleteInstance(instance);
    }

    /* We need a new instance */
    instance = (SRSC_InstanceData *)malloc(sizeof(SRSC_InstanceData));

    if (instance) {
        /* Prepare structure */
        memset(instance, 0, sizeof(SRSC_InstanceData));
        instance->rxFrequencyMHz = frequencyMHz;
        instance->gps.observerLLA.lat = NAN;
        instance->gps.observerLLA.lon = NAN;
        instance->gps.observerLLA.alt = NAN;
        instance->gps.climbRate = NAN;
        int i;
        for (i = 0; i < 20; i++) {
            instance->metro.values[i] = NAN;
        }

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
LPCLIB_Result _SRSC_processConfigFrame (
        const SRSC_Packet *rawConfig,
        SRSC_InstanceData **instancePointer,
        float rxFrequencyHz)
{
    uint32_t data;
    LPCLIB_Result result = LPCLIB_SUCCESS;


    /* Valid pointers to take the output values are required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new calib space if new sonde! */
    SRSC_InstanceData *instance = _SRSC_getInstanceDataStructure(rxFrequencyHz / 1e6f);
    *instancePointer = instance;

    if (!instance) {
        return LPCLIB_ERROR;
    }

    data = __REV(rawConfig->d_bigendian);

    /* See if the detector still needs to work out the exact type of sonde */
    switch (instance->detectorState) {
        /* Determine the sonde type */
        case SRSC_DETECTOR_FIND_TYPE:
            {
                if (rawConfig->type == SRSC_FRAME_CONFIG_TYPE) {
                    instance->confDetect.sondeType = data;

                    if (instance->confDetect.sondeType == instance->confDetect.prevSondeType) {
                        ++instance->confDetect.nDetections;

                        if (instance->confDetect.nDetections >= 2) {
                            instance->config.sondeType = instance->confDetect.sondeType;
                            instance->detectorState = SRSC_DETECTOR_FIND_NAME;

                            instance->confDetect.nDetections = 0;

                            if ((instance->config.sondeType == 228) ||
                                (instance->config.sondeType == 229)) {
                                instance->config.isC34 = false;
                                instance->config.isC50 = true;
                            }
                            else {
                                instance->config.isC50 = false;
                                instance->config.isC34 = true;
                            }

                            instance->config.hasO3 = false;
                            if ((instance->config.sondeType == 56) ||
                                (instance->config.sondeType == 229)) {
                                instance->config.hasO3 = true;
                            }
                        }
                    }
                    else {
                        instance->confDetect.prevSondeType = instance->confDetect.sondeType;
                        instance->confDetect.nDetections = 1;
                    }
                }
            }
            break;

        /* Determine the sonde name */
        case SRSC_DETECTOR_FIND_NAME:
            {
                if (rawConfig->type == SRSC_FRAME_CONFIG_NAME) {
                    instance->confDetect.sondeNumber = data;

                    if (instance->confDetect.sondeNumber == instance->confDetect.prevSondeNumber) {
                        ++instance->confDetect.nDetections;

                        if (instance->confDetect.nDetections >= 2) {
                            instance->config.sondeNumber = instance->confDetect.sondeNumber;
                            instance->detectorState = SRSC_DETECTOR_READY;

                            /* Make sonde name */
                            if (instance->config.isC50) {
                                sprintf(instance->name, "C50-%05ld", instance->config.sondeNumber);
                            }
                            else {
                                sprintf(instance->name, "C34-%04ld", instance->config.sondeNumber);
                            }
                            instance->config.sondeNameKnown = true;
                        }
                    }
                    else {
                        instance->confDetect.prevSondeNumber = instance->confDetect.sondeNumber;
                        instance->confDetect.nDetections = 1;
                    }
                }
            }
            break;

        case SRSC_DETECTOR_READY:
            /* */
            break;
    }

    /* Once type is known we can collect the measurement results */
    if (instance->detectorState >= SRSC_DETECTOR_FIND_NAME) {
        switch (rawConfig->type) {
            case SRSC_FRAME_CONFIG_TYPE:
                /* Ignore once sonde type is fixed */
                break;

            case SRSC_FRAME_CONFIG_NAME:
                /* Ignore once sonde name is fixed */
                break;

            case SRSC_FRAME_CONFIG_103:
                instance->config.info103 = data;
                break;
            case SRSC_FRAME_CONFIG_104:
                instance->config.info104 = data;
                break;
            case SRSC_FRAME_CONFIG_105:
                instance->config.info105 = data;
                break;
            case SRSC_FRAME_CONFIG_106:
                instance->config.info106 = data;
                break;
            case SRSC_FRAME_CONFIG_107:
                instance->config.info107 = data;
                break;
            case SRSC_FRAME_CONFIG_FIRMWARE_VERSION:
                instance->config.firmwareVersion = data;
                break;
            case SRSC_FRAME_CONFIG_VBAT:
                instance->config.batteryVoltage = data / 1000.0f;
                break;
            case SRSC_FRAME_CONFIG_RFPWRDETECT:
                instance->config.rfPwrDetect = data / 1000.0f;
                break;
            case SRSC_FRAME_CONFIG_STATE:
                instance->config.state = data;
                break;
            case SRSC_FRAME_CONFIG_ERROR_FLAGS:
                instance->config.errorFlags = data;
                break;
            case SRSC_FRAME_CONFIG_VDOP:
                instance->config.vdop = data;
                break;
            case SRSC_FRAME_CONFIG_120:
                instance->config.info120 = data;
                break;

            default:
                if (rawConfig->type < 20) {
                    instance->metro.values[rawConfig->type] = ((float*)&data)[0];
                }
                break;
        }
    }

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    /* Cook some other values */
    strcpy(instance->config.name, instance->name);
    instance->config.frequencyKhz = lroundf(rxFrequencyHz / 1000.0);

//    cookedConfig->frameCounter = rawConfig->frameCounter;
//    if (_DFM_checkValidCalibration(calib, CALIB_FREQUENCY)) {
//        cookedConfig->frequencyKhz = 400000 + (calib->frequency * 10) / 64;
//    }

    return result;
}


/* Iterate through instances */
bool _SRSC_iterateInstance (SRSC_InstanceData **instance)
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
void _SRSC_deleteInstance (SRSC_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    SRSC_InstanceData **parent = &instanceList;
    SRSC_InstanceData *p = NULL;
    while (_SRSC_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

