
#include "lpclib.h"
#include "app.h"
#include "dfm.h"
#include "dfmprivate.h"


struct {
    int32_t i32;
    uint32_t n;
} dfm_config_unknown[16];


#define DFM_DATA_FIFO_LENGTH    10

/*static*/ struct _DFM_DataChannelFifo {
    struct {
        uint8_t channel;
        uint32_t data;
        uint32_t timestamp;
    } fifo[DFM_DATA_FIFO_LENGTH];
    int numEntries;
} _dfmDataChannelFifo;


uint32_t _dfmTotalFrames;


/* Process the config/calib block. */
LPCLIB_Result _DFM_processConfigBlock (
        const DFM_SubFrameConfig *rawConfig,
        DFM_InstanceData *instance,
        float rxFrequencyHz,
        uint32_t rxTime)
{
    char s[11];
    unsigned int i;
    uint32_t u32;
    uint8_t channel;
    int n;
    LPCLIB_Result result = LPCLIB_SUCCESS;


    /* Data channel */
    channel = rawConfig->type;

u32 = 0;
for (i = 0; i < 6; i++) {
    u32 = (u32 << 4) | rawConfig->h[i];
}
dfm_config_unknown[channel].i32 = u32;
dfm_config_unknown[channel].n++;

    /* Store the fragment for later processing */
    if (_dfmDataChannelFifo.numEntries >= DFM_DATA_FIFO_LENGTH) {
        for (i = 0; i < DFM_DATA_FIFO_LENGTH - 1; i++) {
            _dfmDataChannelFifo.fifo[i].data = _dfmDataChannelFifo.fifo[i + 1].data;
        }
        _dfmDataChannelFifo.numEntries = DFM_DATA_FIFO_LENGTH - 1;
    }

    /* Add new entry */
    _dfmDataChannelFifo.fifo[_dfmDataChannelFifo.numEntries].channel = channel;
    _dfmDataChannelFifo.fifo[_dfmDataChannelFifo.numEntries].data = u32;
    _dfmDataChannelFifo.fifo[_dfmDataChannelFifo.numEntries].timestamp = rxTime;
    ++_dfmDataChannelFifo.numEntries;

    /* If no instance is known, this is all we can do. */
    if (!instance) {
        return LPCLIB_PENDING;
    }

    /* Process all config fragments in FIFO */
    for (n = 0; n < _dfmDataChannelFifo.numEntries; n++) {
        /* Config channel */
        channel = _dfmDataChannelFifo.fifo[n].channel;
++_dfmTotalFrames;
        /* Time markers */
        uint32_t thisTime = _dfmDataChannelFifo.fifo[n].timestamp;
        float delta = 10.0f * (thisTime - instance->confDetect.lastTime);
        volatile float delta0 = 10.0f * (thisTime - instance->confDetect.lastCh0Time);
        u32 = _dfmDataChannelFifo.fifo[n].data;

        /* See if the detector still needs to work out the exact type of sonde */
        switch (instance->detectorState) {
        /* Determine the number of fast analog channels */
        case DFM_DETECTOR_FIND_NANALOG:
            {
                instance->confDetect.lastTime = thisTime;

                /* Compare the interval between channel 0 transmissions */
                if (channel == 0) {
                    /* If interval appears reasonable, remember config frame length */
                    if ((delta0 > 1.5f*DFM_FRAME_LENGTH_MILLISEC) && (delta0 < 16.5*DFM_FRAME_LENGTH_MILLISEC)) {
                        uint32_t numAnalog = lrintf(delta0 / DFM_FRAME_LENGTH_MILLISEC) - 1;
                        if (numAnalog == instance->confDetect.numAnalog) {
                            if (++instance->confDetect.nDetections >= 3) {
                                instance->numAnalog = instance->confDetect.numAnalog;
                                instance->confDetect.maxChannel = 0;
                                instance->confDetect.nDetections = 0;

                                /* TODO: Assume PS15 in case of a very high number of fast channels */
                                if (instance->numAnalog >= 7) {
                                    instance->detectorState = DFM_DETECTOR_FIND_NAME;
                                    instance->maxConfigChannel = instance->numAnalog;
                                    instance->model = DFM_MODEL_PS15;
                                }
                                else {
                                    instance->detectorState = DFM_DETECTOR_FIND_CONFIG_STRUCTURE;
                                }
                            }
                        }
                        else {
                            instance->confDetect.nDetections = 0;
                        }

                        instance->confDetect.numAnalog = numAnalog;
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

                /* The slow channels must appear at a specific distance from channel 0 */
                if ((channel >= instance->numAnalog) &&
                    (delta0 < (instance->numAnalog + 0.5f) * DFM_FRAME_LENGTH_MILLISEC) &&
                    (delta0 > (instance->numAnalog - 0.5f) * DFM_FRAME_LENGTH_MILLISEC)) {

                    /* We only look at consecutive slow channels (exactly numAnalogChannels apart) */
                    if ((delta < (instance->numAnalog + 1.5f) * DFM_FRAME_LENGTH_MILLISEC) &&
                        (delta > (instance->numAnalog - 1.5f) * DFM_FRAME_LENGTH_MILLISEC)) {

                        if (channel < instance->confDetect.prevChannel) {
                            /* Previous channel was the maximum. Wait until it's the same for a few cycles. */

                            if ((instance->confDetect.prevChannel == instance->confDetect.maxChannel) &&
                                (instance->confDetect.prevNibble0 == instance->confDetect.maxChannelNibble0)) {
                                ++instance->confDetect.nDetections;

                                if (instance->confDetect.nDetections >= 3) {
                                    instance->maxConfigChannel = instance->confDetect.maxChannel;
                                    instance->maxConfigChannelNibble0 = instance->confDetect.maxChannelNibble0;

                                    /* We expect a serial number if the max channel is 6 or higher */
                                    if (instance->maxConfigChannel >= 6) {
                                        instance->detectorState = DFM_DETECTOR_FIND_NAME;
                                        if (instance->maxConfigChannel >= 11) { //TODO: Any other way to detect DFM-17?
                                            instance->model = DFM_MODEL_DFM17;
                                        }
                                        else {
                                            instance->model = DFM_MODEL_DFM09_OLD;  //TODO: Could be DFM-06 new as well
                                        }
                                    }
                                    else {
                                        /* Otherwise there is no serial number (DFM06 old) and we use a generic name */
                                        instance->config.sondeNumber = 0;
                                        strcpy(instance->name, "DFM-06");
                                        instance->config.sondeNameKnown = true;
                                        instance->model = DFM_MODEL_DFM06_OLD;
                                        instance->detectorState = DFM_DETECTOR_READY;
                                    }

                                    instance->confDetect.nDetections = 0;
                                }
                            }
                            else {
                                instance->confDetect.maxChannel = instance->confDetect.prevChannel;
                                instance->confDetect.maxChannelNibble0 = instance->confDetect.prevNibble0;
                                instance->confDetect.nDetections = 0;
                            }
                        }
                    }
                    instance->confDetect.prevChannel = channel;
                    instance->confDetect.prevNibble0 = (u32 >> 20) & 0xF;
                    instance->confDetect.lastTime = thisTime;
                }
            }
            break;

        /* Get the sonde name (and sonde type, DFM-06/DFM-09) from the highest config channel */
        case DFM_DETECTOR_FIND_NAME:
            {
                if ((instance->maxConfigChannelNibble0 == 12) || (instance->model == DFM_MODEL_PS15)) {
                    if (channel == instance->maxConfigChannel) {
                        if ((u32 & 0xF) == 0) {
                            instance->confDetect.sondeNumber = ((u32 >> 4) & 0xFFFF) << 16;
                        }
                        else if ((u32 & 0xF) == 1) {
                            instance->confDetect.sondeNumber |= ((u32 >> 4) & 0xFFFF);

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
                else {
                    if (channel == instance->maxConfigChannel) {
                        instance->confDetect.sondeNumber = 0;
                        for (i = 0; i < 6; i++) {
                            instance->confDetect.sondeNumber *= 10;
                            instance->confDetect.sondeNumber += (u32 >> ((5 - i) * 4)) & 0x0F;
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
            break;
        }

        if (instance->detectorState >= DFM_DETECTOR_FIND_NAME) {
            /* process metrology */
            result = _DFM_processMetrologyBlock(rawConfig, instance);
        }
    }

    /* Set time marker to be able to identify old records */
    instance->lastUpdated = os_time;

    /* Cook some other values */
    instance->config.frequencyKhz = lroundf(rxFrequencyHz / 1000.0f);

    /* Flush FIFO */
    memset(&_dfmDataChannelFifo, 0, sizeof(_dfmDataChannelFifo));

    return result;
}

