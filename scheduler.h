#ifndef SCHEDULER_H_INCLUDED
#define SCHEDULER_H_INCLUDED

#include "Tuareg.h"


#define SCHEDULER_PERIOD_US 1

//we are using a 32 bit timer now
#define SCHEDULER_MAX_PERIOD_US (U32) 0xFFFFFFFF * SCHEDULER_PERIOD_US -1



typedef enum {

    IGN_CH1,
    IGN_CH2,

    FUEL_CH1,
    FUEL_CH2

} scheduler_channel_t;




typedef struct _scheduler_t {

    U32 ign_ch1_action;
    U32 ign_ch2_action;
    U32 fuel_ch1_action;
    U32 fuel_ch2_action;

} scheduler_t;

void init_scheduler();
void scheduler_set_channel(scheduler_channel_t target_ch, U32 action, U32 delay_us);
void scheduler_reset_channel(scheduler_channel_t target_ch);

#endif // SCHEDULER_H_INCLUDED
