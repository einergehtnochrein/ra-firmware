
#ifndef __TASK_SCANNER_H
#define __TASK_SCANNER_H

#include "pt.h"
#include "sonde.h"

typedef struct SCANNER_Context *SCANNER_Handle;

LPCLIB_Result SCANNER_open (SCANNER_Handle *pHandle);

void SCANNER_setManualMode (SCANNER_Handle handle, bool enable);
bool SCANNER_getManualMode (SCANNER_Handle handle);
void SCANNER_setManualAttenuator (SCANNER_Handle handle, bool enable);
bool SCANNER_getManualAttenuator (SCANNER_Handle handle);
void SCANNER_setManualFrequency (SCANNER_Handle handle, uint32_t frequencyHz);
void SCANNER_setManualSondeDetector (SCANNER_Handle handle, SONDE_Detector sondeDetector);
SONDE_Detector SCANNER_getManualSondeDetector (SCANNER_Handle handle);
void SCANNER_addListenFrequency (SCANNER_Handle handle, float frequencyMHz, SONDE_Detector sondeDetector);
void SCANNER_removeListenFrequency (SCANNER_Handle handle, float frequencyMHz);

/** Notify scanner engine that a valid frame was received */
void SCANNER_notifyValidFrame (SCANNER_Handle handle);

PT_THREAD(SCANNER_thread (SCANNER_Handle handle));

#endif
