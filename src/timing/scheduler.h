#ifndef SCHEDULER_H_INCLUDED
#define SCHEDULER_H_INCLUDED

#include "Tuareg.h"

//#define SCHEDULER_DEBUG

#define SCHEDULER_PERIOD_US 8

//witch maximum interval will make sense? the technically maximum possible one is not needed
#define SCHEDULER_MAX_PERIOD_US 500000
#define SCHEDULER_MIN_PERIOD_US 8


typedef void (* scheduler_callback_func)(actor_control_t);
typedef void (* timer_alloc_func)(VU32, volatile bool, volatile bool);
typedef void (* timer_reset_func)();


typedef enum {

    SCHEDULER_CH_IGN1,
    SCHEDULER_CH_IGN2,
    SCHEDULER_CH_FUEL1,
    SCHEDULER_CH_FUEL2,
    SCHEDULER_CH_COUNT

} scheduler_channel_t;


typedef union
{
     U32 all_flags;

     struct
     {
        U32 complete_cycle_realloc :1;
        U32 interval2_enabled :1;
        U32 action1_power :1;
        U32 action2_power :1;
     };

} scheduler_activation_flags_t;


typedef struct _scheduler_activation_parameters_t {

    U32 interval1_us;
    U32 interval2_us;

    scheduler_activation_flags_t flags;

} scheduler_activation_parameters_t;


typedef union
{
     U32 all_flags;

     struct
     {
        U32 alloc :1;
        U32 interval1_expired :1;
     };

} scheduler_state_flags_t;



typedef struct _scheduler_channel_state_t {

    //channel data
    scheduler_activation_parameters_t parameters;
    scheduler_state_flags_t flags;

    //worker functions
    scheduler_callback_func callback;
    timer_alloc_func timer_alloc;
    timer_reset_func timer_reset;

} scheduler_channel_state_t;



typedef struct _scheduler_mgr_t {

    scheduler_channel_state_t channels[SCHEDULER_CH_COUNT];
    bool init_done;

} scheduler_mgr_t;


#ifdef SCHEDULER_DEBUG


typedef struct _scheduler_debug_set_flags_t
{
    //parameters
    U32 action1_power :1;
    U32 action2_power :1;
    U32 interval2_enabled :1;
    U32 complete_cycle_realloc :1;

    //allocation details
    U32 set_curr_cycle :1;
    U32 use_preload :1;
    U32 realloc :1;

} scheduler_debug_set_flags_t;


typedef struct _scheduler_debug_set_t {

    U32 now;
    U64 compare;

    U32 interval1_us;
    U32 interval2_us;

    scheduler_channel_t channel;

    scheduler_debug_set_flags_t flags;

} scheduler_debug_set_t;


typedef struct _scheduler_debug_compare_flags_t
{
    U16 sr_comp1 :1;
    U16 sr_comp2 :1;
    U16 sr_comp3 :1;
    U16 sr_comp4 :1;

    U16 sr_update :1;

    U16 dier_comp1 :1;
    U16 dier_comp2 :1;
    U16 dier_comp3 :1;
    U16 dier_comp4 :1;

    U16 dier_update :1;

} scheduler_debug_compare_flags_t;


typedef struct _scheduler_debug_compare_t {

    U32 CNT;

    U32 COMP1;
    U32 COMP2;
    U32 COMP3;
    U32 COMP4;

    scheduler_debug_compare_flags_t flags;

} scheduler_debug_compare_t;

#endif // SCHEDULER_DEBUG




void init_Vital_Scheduler();

void scheduler_set_channel(scheduler_channel_t Channel, volatile scheduler_activation_parameters_t * pParamaters);
void scheduler_reset_channel(scheduler_channel_t Channel);
void allocate_channel(scheduler_channel_t Channel, VU32 Delay_us);

#endif // SCHEDULER_H_INCLUDED
