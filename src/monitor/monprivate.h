
#ifndef __MONPRIVATE_H
#define __MONPRIVATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "lpclib.h"
#include "mon.h"

/* ADPCM encoder */
typedef struct _adpcm {
    int32_t yl;         // 0
    int32_t yu;         // 4
    int16_t dq[6];      // 8
    int32_t a[2];       // 20
    int32_t b[6];       // 28
    int32_t ap;         // 52
    int32_t dms;        // 56
    int32_t dml;        // 60
    int32_t td;         // 64
    int32_t pk[2];      // 68
    int32_t sr[2];      // 76

    /* Debug */
    int32_t y;
    int32_t d;
    int32_t dqsez;
    int32_t _dq;
    int32_t sez;
    int32_t se;
    int32_t _sr;
} adpcm_t;


/* Initialize ADPCM encoder context */
void ADPCM_init (adpcm_t *handle);
/* Process a single audio sample */
uint8_t ADPCM_processSample (adpcm_t *handle, int32_t sl);

/* Assembler implementation for following functions */
int32_t ADPCM_predictor_zero (int32_t b[6], int16_t dq[6]);
int32_t ADPCM_predictor_pole (int32_t a[2], int32_t sr[2]);

void _MON_sendAudio (const char *txData);

#ifdef __cplusplus
}
#endif
#endif
