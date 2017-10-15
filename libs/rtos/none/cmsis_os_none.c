

#include "lpclib.h"
#include "cmsis_os.h"


/* Determine whether we are in thread mode or handler mode. */
static int inHandlerMode (void)
{
    return 0;
}


/* Timer related globals */
volatile uint32_t os_time;                              /**< Number of SysTick ticks */
static struct os_timer_custom *appTimers;               /**< Timer list */
static volatile uint32_t appTicksMult = 1024;
volatile uint32_t osDelayTimer;



#if defined(__TARGET_ARCH_4T) || defined(__ARM_ARCH_4T__) || defined(__TARGET_ARCH_5TE) || defined(__ARM_ARCH_5TE__)
static
#endif
void SysTick_Handler (void)
{
    struct os_timer_custom *p;

    ++os_time;

    /* Work on the list of timers */
    p = appTimers;
    while (p != NULL) {
        if (p->remainingTicks != 0) {
            --(p->remainingTicks);
            if (p->remainingTicks == 0) {
                p->parent->ptimer(p->argument);

                if (p->type == osTimerPeriodic) {
                    p->remainingTicks = p->period;
                }
            }
        }

        p = p->next;
    }

    if (osDelayTimer) {
        --osDelayTimer;
    }
}



#if defined(__TARGET_ARCH_4T) || defined(__ARM_ARCH_4T__)
#if defined (__CC_ARM)
__irq void SysTick_Handler_ARM7 (void)
#endif
#if defined (__GNUC__)
void SysTick_Handler_ARM7 (void) __attribute__((interrupt("IRQ")));
void SysTick_Handler_ARM7 (void)
#endif
{
    LPC_TIM0->IR = 1;
    SysTick_Handler();
    LPC_VIC->Address = 0;
}
#endif



#if defined(__TARGET_ARCH_5TE) || defined(__ARM_ARCH_5TE__)
static LPCLIB_Result systickCallback_ARM9 (LPCLIB_Event event)
{
    (void) event;

    SysTick_Handler();

    return LPCLIB_SUCCESS;
}
#endif



osStatus osKernelStart (void)
{
#if defined(__TARGET_ARCH_4T) || defined(__ARM_ARCH_4T__)

  #if (LPCLIB_FAMILY == LPCLIB_FAMILY_LPC2106) || (LPCLIB_FAMILY == LPCLIB_FAMILY_LPC212X) || (LPCLIB_FAMILY == LPCLIB_FAMILY_LPC213X)
    //TODO Make sure it gets lowest priority!
    VIC_SetPriority(TIMER0_IRQn, 15);
    VIC_SetHandler(TIMER0_IRQn, (uint32_t)SysTick_Handler_ARM7);
    VIC_EnableIRQ(TIMER0_IRQn);
  #else
    LPC_VIC->VectAddr[4] = (uint32_t)SysTick_Handler_ARM7;
    LPC_VIC->VectPriority[4] = 15;
    LPC_VIC->IntEnable = (1u << 4);
    LPC_SC->PCLKSEL[0] = (LPC_SC->PCLKSEL[0] & ~(3u << 2)) | (3u << 2); //CLK/8
  #endif
    LPC_TIM0->TCR = 2;
    LPC_TIM0->MR[0] = SystemCoreClock / (8 * 100) - 1;
    LPC_TIM0->MCR = 3;
    LPC_TIM0->TCR = 1;

#elif defined(__TARGET_ARCH_5TE) || defined(__ARM_ARCH_5TE__)

  #if 0
    VIC_SetHandler(TIMER0_IRQn, (uint32_t)SysTick_Handler_ARM9);
    VIC_EnableIRQ(TIMER0_IRQn);
    LPC_TIMER0->TCR = 2;
    LPC_TIMER0->MR[0] = CLKPWR_getBusClock(CLKPWR_CLOCK_TIMER0) / 100 - 1        + 10000;
    LPC_TIMER0->MCR = 1;                                /* MR0 causes reset */
    LPC_TIMER0->TCR = 1;
    LPC_TIMER0->INT_SET_ENABLE = (1u << 0);             /* Match0 interrupt */
  #else
    TIMER_Handle systickTimer;
    const TIMER_Config systickTimerConfig[] = {
        {.opcode = TIMER_OPCODE_CONFIG_MATCH,
         .match = {
             .channel = TIMER_MATCH0,
             .intOnMatch = LPCLIB_YES,
             .resetOnMatch = LPCLIB_YES, }},

        {.opcode = TIMER_OPCODE_SET_CALLBACK,
         .callback = {
             .callback = systickCallback_ARM9, }},

        TIMER_CONFIG_END
    };

    TIMER_open(TIMER0, &systickTimer);
    TIMER_ioctl(systickTimer, systickTimerConfig);
    TIMER_writeMatch(systickTimer, TIMER_MATCH0, CLKPWR_getBusClock(CLKPWR_CLOCK_TIMER0) / 100 - 1     +10000);
    TIMER_run(systickTimer);
  #endif
#else
    uint32_t rvr;
    uint32_t tenms;


    SysTick_Config(SystemCoreClock / 100);

    /* Calculate factor for ticks<->milliseconds transformation */
    rvr = (SysTick->LOAD & SysTick_LOAD_RELOAD_Msk) >> SysTick_LOAD_RELOAD_Pos;
    tenms = (SysTick->CALIB & SysTick_CALIB_TENMS_Msk) >> SysTick_CALIB_TENMS_Pos;
    if (tenms == 0) {
        /* TODO: Wild guess: 100 MHz core clock */
        tenms = 1000000;
    }
    appTicksMult = (((tenms * 256) / rvr) * 4) / 10;
#endif

    return osOK;
}


//  ==== Generic Wait Functions ====

/// Wait for Timeout (Time Delay)
/// \param[in]     millisec      time delay value
/// \return status code that indicates the execution status of the function.
osStatus osDelay (uint32_t millisec)
{
    /* Assume 10-ms tick...  TODO */
    osDelayTimer = (millisec + 5) / 10 + 1;
    while (osDelayTimer)
        ;

    return osOK;
}


#if (defined (osFeature_Wait)  &&  (osFeature_Wait != 0))     // Generic Wait available

/// Wait for Signal, Message, Mail, or Timeout
/// \param[in] millisec          timeout value or 0 in case of no time-out
/// \return event that contains signal, message, or mail information or error code.
/// \note MUST REMAIN UNCHANGED: \b osWait shall be consistent in every CMSIS-RTOS.
osEvent osWait (uint32_t millisec);

#endif  // Generic Wait available


//  ==== Timer Management Functions ====


/// Create a timer.
/// \param[in]     timer_def     timer object referenced with \ref osTimer.
/// \param[in]     type          osTimerOnce for one-shot or osTimerPeriodic for periodic behavior.
/// \param[in]     argument      argument to the timer call back function.
/// \return timer ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osTimerCreate shall be consistent in every CMSIS-RTOS.
osTimerId osTimerCreate (osTimerDef_t *timer_def, os_timer_type type, void *argument)
{
    struct os_timer_custom *p;


    /* Store context parameters */
    timer_def->custom->argument = argument;
    timer_def->custom->type = type;
    timer_def->custom->remainingTicks = 0;
    timer_def->custom->period = 0;
    timer_def->custom->parent = timer_def;

    /* Add to timer list */
    timer_def->custom->next = NULL;

    if (appTimers == NULL) {
        timer_def->custom->previous = NULL;
        appTimers = timer_def->custom;
    }
    else {
        p = appTimers;
        while (p->next) {
            p = p->next;
        }

        timer_def->custom->previous = p;
        p->next = timer_def->custom;
    }

    return timer_def->custom;
}



/// Start or restart a timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \param[in]     millisec      time delay value of the timer.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStart shall be consistent in every CMSIS-RTOS.
osStatus osTimerStart (osTimerId timer_id, uint32_t millisec)
{
    uint32_t ticks;


    if (timer_id == NULL) {
        return osErrorResource;
    }

    ticks = (appTicksMult * millisec) / 1024;
    if (ticks == 0) {
        ticks = 1;
    }

    ((struct os_timer_custom *)timer_id)->period = ticks;
    ((struct os_timer_custom *)timer_id)->remainingTicks = ticks;

    return osOK;
}



/// Stop the timer.
/// \param[in]     timer_id      timer ID obtained by \ref osTimerCreate.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osTimerStop shall be consistent in every CMSIS-RTOS.
osStatus osTimerStop (osTimerId timer_id)
{
    if (timer_id == NULL) {
        return osErrorResource;
    }

    ((struct os_timer_custom *)timer_id)->remainingTicks = 0;

    return osOK;
}



//  ==== Memory Pool Management Functions ====

#if (defined (osFeature_Pool)  &&  (osFeature_Pool != 0))  // Memory Pool Management available

/// \brief Define a Memory Pool.
/// \param         name          name of the memory pool.
/// \param         no            maximum number of objects (elements) in the memory pool.
/// \param         type          data type of a single object (element).
/// \note CAN BE CHANGED: The parameter to \b osPoolDef shall be consistent but the
///       macro body is implementation specific in every CMSIS-RTOS.

/* Create and Initialize a memory pool */
osPoolId osPoolCreate (osPoolDef_t *pool_def)
{
    return pool_def->cb;
}


/// Allocate a memory block from a memory pool
/// \param[in]     pool_id       memory pool ID obtain referenced with \ref osPoolCreate.
/// \return address of the allocated memory block or NULL in case of no memory available.
/// \note MUST REMAIN UNCHANGED: \b osPoolAlloc shall be consistent in every CMSIS-RTOS.
void *osPoolAlloc (osPoolId pool_id)
{
    void *p = NULL;
    uint32_t i;
    uint32_t index;

    if (inHandlerMode()) {
        __disable_irq();     //TODO: Save current status!
    }

    for (i = 0; i < pool_id->pool_sz; i++) {
        index = pool_id->currentIndex + i;
        if (index >= pool_id->pool_sz) {
            index = 0;
        }

        if (pool_id->markers[index] == 0) {
            pool_id->markers[index] = 1;
            p = (void *)((uint32_t)(pool_id->pool) + (index * pool_id->item_sz));
            pool_id->currentIndex = index;
            break;
        }
    }

    if (!inHandlerMode()) {
        __enable_irq();     //TODO: Restore previous status!
    }

    return p;
}


/// Allocate a memory block from a memory pool and set memory block to zero
/// \param[in]     pool_id       memory pool ID obtain referenced with \ref osPoolCreate.
/// \return address of the allocated memory block or NULL in case of no memory available.
/// \note MUST REMAIN UNCHANGED: \b osPoolCAlloc shall be consistent in every CMSIS-RTOS.
void *osPoolCAlloc (osPoolId pool_id)
{
    unsigned int i;
    void *p = osPoolAlloc(pool_id);

    for (i = 0; i < pool_id->item_sz; i++) {
        ((uint8_t *)p)[i] = 0;
    }

    return p;
}


/// Return an allocated memory block back to a specific memory pool
/// \param[in]     pool_id       memory pool ID obtain referenced with \ref osPoolCreate.
/// \param[in]     block         address of the allocated memory block that is returned to the memory pool.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osPoolFree shall be consistent in every CMSIS-RTOS.
osStatus osPoolFree (osPoolId pool_id, void *block)
{
    uint32_t index;

    if (pool_id == NULL) {
        return osErrorParameter;
    }

    if (block == NULL) {
        return osErrorParameter;
    }

    if (block < pool_id->pool) {
        return osErrorParameter;
    }

    index = (uint32_t)block - (uint32_t)(pool_id->pool);
    if (index % pool_id->item_sz) {
        return osErrorParameter;
    }
    index = index / pool_id->item_sz;
    if (index >= pool_id->pool_sz) {
        return osErrorParameter;
    }

    pool_id->markers[index] = 0;

    return osOK;
}


#endif   // Memory Pool Management available


//  ==== Message Queue Management Functions ====

#if (defined (osFeature_MessageQ)  &&  (osFeature_MessageQ != 0))     // Message Queues available


/// Create and Initialize a Message Queue.
/// \param[in]     queue_def     queue definition referenced with \ref osMessageQ.
/// \param[in]     thread_id     thread ID (obtained by \ref osThreadCreate or \ref osThreadGetId) or NULL.
/// \return message queue ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMessageCreate shall be consistent in every CMSIS-RTOS.
osMessageQId osMessageCreate (osMessageQDef_t *queue_def, osThreadId thread_id)
{
    (void) queue_def;
    (void) thread_id;

    return NULL;
}


/// Put a Message to a Queue.
/// \param[in]     queue_id      message queue ID obtained with \ref osMessageCreate.
/// \param[in]     info          message information.
/// \param[in]     millisec      timeout value or 0 in case of no time-out.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMessagePut shall be consistent in every CMSIS-RTOS.
osStatus osMessagePut (osMessageQId queue_id, uint32_t info, uint32_t millisec)
{
    (void) queue_id;
    (void) info;
    (void) millisec;

    return osOK;
}



/// Get a Message or Wait for a Message from a Queue.
/// \param[in]     queue_id      message queue ID obtained with \ref osMessageCreate.
/// \param[in]     millisec      timeout value or 0 in case of no time-out.
/// \return event information that includes status code.
/// \note MUST REMAIN UNCHANGED: \b osMessageGet shall be consistent in every CMSIS-RTOS.
osEvent osMessageGet (osMessageQId queue_id, uint32_t millisec);

#endif     // Message Queues available


//  ==== Mail Queue Management Functions ====

#if (defined (osFeature_MailQ)  &&  (osFeature_MailQ != 0))     // Mail Queues available


/// Create and Initialize mail queue
/// \param[in]     queue_def     reference to the mail queue definition obtain with \ref osMailQ
/// \param[in]     thread_id     thread ID (obtained by \ref osThreadCreate or \ref osThreadGetId) or NULL.
/// \return mail queue ID for reference by other functions or NULL in case of error.
/// \note MUST REMAIN UNCHANGED: \b osMailCreate shall be consistent in every CMSIS-RTOS.
osMailQId osMailCreate (osMailQDef_t *queue_def, osThreadId thread_id)
{
    (void) thread_id;

    return queue_def->cb;
}



/** Check if mail queue is full. */
static int _osMailQueueFull (osMailQId queue_id)
{
    if ((((queue_id->rdptr + queue_id->queueSize) - queue_id->wrptr) % queue_id->queueSize) == 1) {
        return 1;   /* Queue full */
    }

    return 0;       /* Queue not full */
}


/// Allocate a memory block from a mail
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     millisec      timeout value or 0 in case of no time-out
/// \return pointer to memory block that can be filled with mail or NULL in case error.
/// \note MUST REMAIN UNCHANGED: \b osMailAlloc shall be consistent in every CMSIS-RTOS.
void *osMailAlloc (osMailQId queue_id, uint32_t millisec)
{
    void *p = NULL;
    (void) millisec;


    if (queue_id == NULL) {
        return NULL;
    }

    if (!inHandlerMode()) {
        __disable_irq();
    }


    if (!_osMailQueueFull(queue_id)) {
        p = (void *)((uint32_t)(queue_id->data) + queue_id->wrptr * queue_id->itemSize);
        queue_id->wrptr = (queue_id->wrptr + 1) % queue_id->queueSize;
    }

    if (!inHandlerMode()) {
        __enable_irq();
    }

    return p;
}



/// Allocate a memory block from a mail and set memory block to zero
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     millisec      timeout value or 0 in case of no time-out
/// \return pointer to memory block that can shall filled with mail or NULL in case error.
/// \note MUST REMAIN UNCHANGED: \b osMailCAlloc shall be consistent in every CMSIS-RTOS.
void *osMailCAlloc (osMailQId queue_id, uint32_t millisec)
{
    uint32_t i;
    uint8_t *p = (uint8_t *)osMailAlloc(queue_id, millisec);

    if (p) {
        for (i = 0; i < sizeof(queue_id->itemSize); i++) {
            *p++ = 0;
        }
    }

    return p;
}



/// Put a mail to a queue
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     mail          memory block previously allocated with \ref osMailAlloc or \ref osMailCAlloc.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMailPut shall be consistent in every CMSIS-RTOS.
osStatus osMailPut (osMailQId queue_id, void *mail)
{
    if (queue_id == NULL) {
        return osErrorParameter;
    }

    if (mail == NULL) {
        return osErrorParameter;
    }

    //TODO:
    //The pointer returned by alloc() is a pointer to the queue!
    //Therefore the data is already where we want it!
    //Better to use a pool and a separate mail queue?

    return osOK;
}



/// Get a mail from a queue
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     millisec      timeout value or 0 in case of no time-out
/// \return event that contains mail information or error code.
/// \note MUST REMAIN UNCHANGED: \b osMailGet shall be consistent in every CMSIS-RTOS.
osEvent osMailGet (osMailQId queue_id, uint32_t millisec)
{
//TODO This function ignores the "millisec" parameter, and returns immediately

    osEvent rtosEvent;
    rtosEvent.status = osOK;
    (void) millisec;

    if (queue_id == NULL) {
        rtosEvent.status = osErrorParameter;
        return rtosEvent;
    }

    if (!inHandlerMode()) {
#if defined(__CC_ARM) && defined(__TARGET_ARCH_4T)
        #warning "!!! Cannot use __disable_irq() on Keil V5 for ARM7 !!!"
#else
        __disable_irq();
#endif
    }

    if (queue_id->rdptr != queue_id->wrptr) {
        rtosEvent.value.p = (void *)((uint32_t)queue_id->data + queue_id->rdptr * queue_id->itemSize);

        rtosEvent.status = osEventMail;
    }

    if (!inHandlerMode()) {
        __enable_irq();
    }

    return rtosEvent;
}



/// Free a memory block from a mail
/// \param[in]     queue_id      mail queue ID obtained with \ref osMailCreate.
/// \param[in]     mail          pointer to the memory block that was obtained with \ref osMailGet.
/// \return status code that indicates the execution status of the function.
/// \note MUST REMAIN UNCHANGED: \b osMailFree shall be consistent in every CMSIS-RTOS.
osStatus osMailFree (osMailQId queue_id, void *mail)
{
    int mailIndex;


    if (queue_id == NULL) {
        return osErrorParameter;
    }

    if (mail == NULL) {
        return osErrorParameter;
    }


    if (!inHandlerMode()) {
        __disable_irq();
    }

    /* Calculate index from pinter */
    mailIndex = ((uint32_t)mail - (uint32_t)queue_id->data) / queue_id->itemSize;

    /* Invalidate all queue items up to this item */
    queue_id->rdptr = (mailIndex + 1) % queue_id->queueSize;

    if (!inHandlerMode()) {
        __enable_irq();
    }

    return osOK;
}



osStatus osMutexWait (osMutexId mutex_id, uint32_t millisec)
{
    if (millisec == osWaitForever) {
        while (*mutex_id != 0) {
            ;
        }
    }
    else {
        if (millisec != 0) {
            osDelayTimer = millisec / 10;
            while (*mutex_id != 0) {
                if (osDelayTimer == 0) {
                    return osErrorTimeoutResource;
                }
            }
        }
    }
    *mutex_id = 1;

    return osOK;
}


int32_t osSemaphoreWait (osSemaphoreId semaphore_id, uint32_t millisec)
{
    if (*semaphore_id) {
        *semaphore_id = 0;

        return 1;
    }

    if (millisec == osWaitForever) {
        while (*semaphore_id == 0) {
            ;
        }
    }
    else {
        if (millisec != 0) {
            osDelayTimer = millisec / 10;
            while (*semaphore_id == 0) {
                if (osDelayTimer == 0) {
                    return 0;
                }
            }

            *semaphore_id = 0;

            return 1;
        }
    }

    return 0;
}

#endif  // Mail Queues available


