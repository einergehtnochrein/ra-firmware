
#ifndef __SYNCDETECT_H
#define __SYNCDETECT_H

#include "common.h"

typedef enum {
    SYNC_STATE_HUNT = 0,
    SYNC_STATE_DATA_RAW,
    SYNC_STATE_DATA_UART_8N1,
    SYNC_STATE_DATA_BIPHASE_S,
    SYNC_STATE_DATA_BIPHASE_M,
    SYNC_STATE_DATA_MANCHESTER,
    SYNC_STATE_DATA_MANCHESTER_UART_8N1,
} SYNC_State;

typedef struct _SyncDetectorContext *SYNC_Handle;

typedef void (*ProcessFunc)(SYNC_Handle handle, volatile IPC_S2M *buffer, int writeIndex);

typedef struct {
    int nPatterns;

    struct {
        int id;                             /* Identifies this sync pattern. Reported to M4 on match. */
        uint64_t pattern[2];                /* The pattern. p[0].LSB=received last, p[1].MSB=received first */
        uint64_t patternMask[2];            /* Relevant bits in pattern */
        int nMaxDifference;                 /* Maximum # differences to received pattern */
        int frameLengthBits;                /* Number of payload bits to read after sync detect */
        int startOffset;                    /* Start offset in buffer for storing first payload byte */
        SYNC_State dataState;               /* State to jump to when receiving data */
        bool inverted;                      /* Invert bits if set */
        ProcessFunc postProcess;            /* Called after frame has been received */
        ProcessFunc byteProcess;            /* Called after each byte received */

        int nSubBlockBits;                  /* Limit sub-blocks to n bits */
        int nSubBlockBytes;                 /* Sub-block size (bytes) in data buffer */
    } conf[];
} SYNC_Config;


void SYNC_open (SYNC_Handle *pHandle);
void SYNC_configure(SYNC_Handle handle, const SYNC_Config *pConfig);

#endif
