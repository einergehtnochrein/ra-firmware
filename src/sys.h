
#ifndef __TASK_SYS_H
#define __TASK_SYS_H

#include "lpclib.h"
#include "pt.h"
#include "sonde.h"


/** Opcodes for application events. */
enum {
    APP_EVENT_SUSPEND,                      /**< System suspended */
    APP_EVENT_HEARD_SONDE,                  /**< Information about heard sonde */
    APP_EVENT_RAW_FRAME,                    /**< Raw frame received */
};


struct SYS_ConfigCallback {
    LPCLIB_Callback callback;               /**< New callback handler */
    LPCLIB_Callback *pOldCallback;          /**< Takes previously installed callback handler */
};


/** Submit a job for the system handler. */
LPCLIB_Result SYS_handleEvent (LPCLIB_Event event);

/** Install a callback to become informed about system events. */
void SYS_installCallback (struct SYS_ConfigCallback configCallback);

typedef struct SYS_Context *SYS_Handle;

LPCLIB_Result SYS_open (SYS_Handle *pHandle);
LPCLIB_Result SYS_enableDetector (SYS_Handle handle, float frequency, SONDE_Detector detector);

float SYS_getCurrentFrequency (SYS_Handle handle);

/* Read a new RSSI value in dBm. */
LPCLIB_Result SYS_readRssi (SYS_Handle handle, float *rssi);

/* Return last RSSI measurement in an RX frame */
float SYS_getFrameRssi (SYS_Handle handle);

#define HOST_CHANNEL_PING           0
#define HOST_CHANNEL_KISS           1
#define HOST_CHANNEL_INFO           2
#define HOST_CHANNEL_GUI            3
#define HOST_CHANNEL_EPHEMUPDATE    4
#define HOST_CHANNEL_SPECTRUM       5
#define HOST_CHANNEL_SWITCHES       7
#define HOST_CHANNEL_AUDIO          8
#define HOST_CHANNEL_FIRMWAREUPDATE 9
#define HOST_CHANNEL_DEBUG          77
#define HOST_CHANNEL_SMARTBASIC     81

LPCLIB_Result SYS_send2Host (int channel, const char *message);
LPCLIB_Result SYS_sendBreak (int durationMilliseconds);

/** System management task. */
PT_THREAD(SYS_thread (SYS_Handle handle));

#endif
