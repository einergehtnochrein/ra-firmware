
#ifndef __TASK_EPHEMUPDATE_H
#define __TASK_EPHEMUPDATE_H

#include "lpclib.h"
#include "pt.h"


typedef struct EPHEMUPDATE_Context *EPHEMUPDATE_Handle;

LPCLIB_Result EPHEMUPDATE_open (EPHEMUPDATE_Handle *pHandle);
LPCLIB_Result EPHEMUPDATE_processCommand (EPHEMUPDATE_Handle handle, const char *commandLine);

/** System management task. */
PT_THREAD(EPHEMUPDATE_thread (EPHEMUPDATE_Handle handle));

#endif
