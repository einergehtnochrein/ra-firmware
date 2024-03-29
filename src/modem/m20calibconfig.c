
#include "lpclib.h"
#include "app.h"
#include "m20.h"
#include "m20private.h"


#define M20_MAX_SONDES         4


/* Points to list of calibration structures */
static M20_InstanceData *instanceList;


/* Get a new calibration data structure for a new sonde */
static M20_InstanceData *_M20_getInstanceDataStructure (const char *name)
{
    M20_InstanceData *p;
    M20_InstanceData *instance;

    /* Check if we already have the calibration data. Count the number of sondes
     * while traversing the list.
     */
    int numSondes = 0;
    p = instanceList;
    while (p) {
        if (!strcmp(p->hashName, name)) {
            /* Found it! */
            return p;
        }

        ++numSondes;
        p = p->next;
    }

    /* If we have reached the maximum number of sondes that we want to track in parallel,
     * do a garbage collection now: Identify the least recently used entry and reuse it.
     */
    if (numSondes >= M20_MAX_SONDES) {
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
        instance = (M20_InstanceData *)calloc(1, sizeof(M20_InstanceData));
    }

    if (instance) {
        /* Prepare structure */
        instance->id = SONDE_getNewID(sonde);
        strcpy(instance->hashName, name);
        instance->metro.humidityCalibration = NAN;

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
LPCLIB_Result _M20_processConfigBlock (
        const M20_Packet *payload,
        M20_InstanceData **instancePointer)
{
    char s[20];

    /* Get the sonde name.
     *
     * It looks like there are three bytes B0, B1, B2 representing the serial number.
     * Two serial number encodings are known so far:
     * 002-2-01620    0x80 0x50 0x19 --> 10000001 01010000 00011001
     * 002-2-01621    0x80 0x54 0x19 --> 10000001 01010100 00011001
     * 911-2-00059    0xF6 0xEC 0x00 --> 11110110 11101100 00000000
     *
     * If we combine (B2(7:0) << 6) | B1(7:2), we get:
     *                00011001010100 = 01620
     *                00011001010101 = 01621
     *                00000000111011 = 00059
     *
     * Findings by OE5DXL for the first two numbers:
     *
     * B0(6:0) is 1 when the serial number starts with "002", and 118 when it starts with "911".
     * Following Meteomodem's old numbering scheme, this could indicate the month in a decade:
     *   1 % 12 = 1,    1 / 12 = 0  --> February 2020   "002"
     * 118 % 12 = 10, 118 / 12 = 9  --> November 2019   "911"
     *
     * B1[1:0]:B0[7] encodes the middle number: 1 --> "-2-", 2 --> "-3-"
     *
     * Other presentation:
     *    [2]      [1]       [0]
     * 00000000 111011|00 1|1110110    911-2-00059
     * ---------------|----|-------
     *        59         1    118
     *
     *    [2]      [1]       [0]
     * 00001111 101001|01 0|0000101    006-3-01001
     * ---------------|----|-------
     *       1001        2     5
     */
    snprintf(s, sizeof(s), "%d%02d-%d-%05d",
            (payload->inner.serial[0] % 128) / 12,              /* year (0...9) */
            1 + (payload->inner.serial[0] % 128) % 12,          /* month (1...12) */
            1 + (payload->inner.serial[0] / 128                 /* fab location (1/2=?, 3=Ury) */
                | (payload->inner.serial[1] & 3)),
            ((uint16_t)payload->inner.serial[2] << 6)           /* serial (0...16383) */
                | ((uint16_t)payload->inner.serial[1] >> 2)
            );

    /* Valid pointer to take the output values required */
    if (!instancePointer) {
        return LPCLIB_ILLEGAL_PARAMETER;
    }

    /* Allocate new instance space if new sonde! */
    M20_InstanceData *instance = _M20_getInstanceDataStructure(s);
    *instancePointer = instance;

    if (instance) {
        /* Set time marker to be able to identify old records */
        instance->lastUpdated = os_time;
    }

    return LPCLIB_SUCCESS;
}


/* Iterate through instances */
bool _M20_iterateInstance (M20_InstanceData **instance)
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
void _M20_deleteInstance (M20_InstanceData *instance)
{
    if ((instance == NULL) || (instanceList == NULL)) {
        /* Nothing to do */
        return;
    }

    M20_InstanceData **parent = &instanceList;
    M20_InstanceData *p = NULL;
    while (_M20_iterateInstance(&p)) {
        if (p == instance) {                /* Found */
            *parent = p->next;              /* Remove from chain */
            free(instance);                 /* Free allocated memory */
            break;
        }

        parent = &p->next;
    }
}

