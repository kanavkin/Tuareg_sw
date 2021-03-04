#ifndef TIMERS_H
#define TIMERS_H

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


typedef union
{
     U8 all_flags;

     struct
     {
        U8 cycle_10_ms :1;
        U8 cycle_20_ms :1;
        U8 cycle_33_ms :1;
        U8 cycle_66_ms :1;
        U8 cycle_100_ms :1;
        U8 cycle_250_ms :1;
        U8 cycle_1000_ms :1;
     };

} systicks_flags_t;


typedef struct _systick_t {

    timestamp_t system_time;
    systicks_flags_t flags;

} systick_t;



typedef struct _systick_mgr_t {

    timestamp_t system_time;

    U32 counter_10_ms;
    U32 counter_20_ms;
    U32 counter_33_ms;
    U32 counter_66_ms;
    U32 counter_100_ms;
    U32 counter_250_ms;
    U32 counter_1000_ms;

    volatile systick_t out;

} systick_mgr_t;


volatile systick_t * init_systick_timer();

VU32 get_timestamp_fraction_us();

#endif // TIMERS_H
