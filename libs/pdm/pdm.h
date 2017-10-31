#ifndef __PDM_H
#define __PDM_H

typedef void (*PDM_Callback) (int32_t *samples, int nSamples);

typedef struct _PDM_Context *PDM_Handle;

void PDM_open (int dummy, PDM_Handle *pHandle);
void PDM_run (PDM_Handle handle, int decimationRatio, PDM_Callback callback);
void PDM_stop (PDM_Handle handle);
void PDM_close (PDM_Handle *pHandle);

/* This weak function should be defined by application.
 * The default implementation configures input muxers for "Ra1" board.
 */
void PDM_initUser (void);

/* Get DC offset (Ra2 only) */
float PDM_getDcOffset (PDM_Handle handle);

#endif
