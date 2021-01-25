#ifndef SCHEDULER_H_INCLUDED
#define SCHEDULER_H_INCLUDED

#include "Tuareg.h"


#define SCHEDULER_PERIOD_US 8

//we are using a 32 bit timer now
//#define SCHEDULER_MAX_PERIOD_US (U32) ((0xFFFFFFFF * SCHEDULER_PERIOD_US) -1)

//witch maximum interval will make sense? the technically maximum possible one is not needed
#define SCHEDULER_MAX_PERIOD_US 500000

#define SCHEDULER_MIN_PERIOD_US 8

//scheduler watchdog
#define SCHEDULER_WATCHDOG_RESET_VALUE 250


typedef enum {

    SCHEDULER_CH_IGN1,
    SCHEDULER_CH_IGN2,
    SCHEDULER_CH_FUEL1,
    SCHEDULER_CH_FUEL2,
    SCHEDULER_CH_COUNT

} scheduler_channel_t;




typedef struct _scheduler_t {

    actor_control_t targets[SCHEDULER_CH_COUNT];

    VU8 watchdogs[SCHEDULER_CH_COUNT];

} scheduler_t;




void init_scheduler();
void scheduler_set_channel(scheduler_channel_t Channel, actor_control_t TargetState, U32 Delay_us);
void scheduler_reset_channel(scheduler_channel_t Channel);
void scheduler_update_watchdogs();

#endif // SCHEDULER_H_INCLUDED
