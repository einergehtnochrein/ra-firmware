
#ifndef __PT_H__
#define __PT_H__

#define PT_BEGIN(pt) { 
#define PT_END(pt)  return 0; }
#define PT_THREAD(name_args) char name_args

#if 0
#define PT_INIT(pt)
#define PT_WAIT_UNTIL(pt, condition)
#define PT_WAIT_WHILE(pt, cond)
#define PT_WAIT_THREAD(pt, thread)
#define PT_SPAWN(pt, child, thread)
#define PT_RESTART(pt)
#define PT_EXIT(pt)	return 0
#define PT_SCHEDULE(f)
#define PT_YIELD(pt)
#define PT_YIELD_UNTIL(pt, cond)
#endif

#endif

