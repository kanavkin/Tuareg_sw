#ifndef SCHEDULER_H_INCLUDED
#define SCHEDULER_H_INCLUDED

#include "Tuareg.h"


#define SCHEDULER_PERIOD_US 8

//witch maximum interval will make sense? the technically maximum possible one is not needed
#define SCHEDULER_MAX_PERIOD_US 500000
#define SCHEDULER_MIN_PERIOD_US 8



typedef enum {

    SCHEDULER_CH_IGN1,
    SCHEDULER_CH_IGN2,
    SCHEDULER_CH_FUEL1,
    SCHEDULER_CH_FUEL2,
    SCHEDULER_CH_COUNT

} scheduler_channel_t;


typedef union
{
     U8 all_flags;

     struct
     {
        U8 ign1_alloc :1;
        U8 ign2_alloc :1;
        U8 fuel1_alloc :1;
        U8 fuel2_alloc :1;
     };

} scheduler_state_t;


typedef struct _scheduler_t {

    actor_control_t target_controls[SCHEDULER_CH_COUNT];
    scheduler_state_t state;

} scheduler_t;


#ifdef SCHEDULER_DEBUG


typedef struct _scheduler_debug_set_flags_t
{
    U16 param_ch_ign1 :1;
    U16 param_ch_ign2 :1;
    U16 param_ch_fuel1 :1;
    U16 param_ch_fuel2 :1;

    U16 target_state_powered :1;

    U16 set_curr_cycle :1;
    U16 set_next_cycle_preload :1;

    U16 reactivated :1;

    U16 param_complete_realloc :1;

} scheduler_debug_set_flags_t;


typedef struct _scheduler_debug_set_t {

    U32 now;
    U64 compare;
    U32 param_delay_us;

    volatile scheduler_debug_set_flags_t flags;

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

extern void scheduler_allocate_ign1(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload);
extern void scheduler_reset_ign1();
extern void scheduler_allocate_ign2(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload);
extern void scheduler_reset_ign2();

extern void scheduler_allocate_fch1(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload);
extern void scheduler_reset_fch1();
extern void scheduler_allocate_fch2(VU32 Compare, volatile bool CurrentCycle, volatile bool EnablePreload);
extern void scheduler_reset_fch2();


void init_scheduler();

void scheduler_set_channel(scheduler_channel_t Channel, actor_control_t Controls, VU32 Delay_us, volatile bool Complete_on_realloc);
void scheduler_reset_channel(scheduler_channel_t Channel);

#endif // SCHEDULER_H_INCLUDED
