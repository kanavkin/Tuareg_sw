#ifndef LOWPRIOSCHEDULER_H_INCLUDED
#define LOWPRIOSCHEDULER_H_INCLUDED

#include <Tuareg_platform.h>

//#define SCHEDULER_DEBUG

//we are using a 16 bit timer
#define LOWPRIO_SCHEDULER_MAX_PERIOD_US (U16) 0xFFFF * LOWPRIO_SCHEDULER_PERIOD_US -1

//witch maximum interval will make sense? the technically maximum possible one is not needed
#define LOWPRIO_SCHEDULER_PERIOD_US 200
#define LOWPRIO_SCHEDULER_MIN_PERIOD_US LOWPRIO_SCHEDULER_PERIOD_US

typedef void (* lowprio_scheduler_callback_func)(actor_control_t);
typedef void (* lowprio_timer_alloc_func)(U32, bool, bool);
typedef void (* lowprio_timer_reset_func)();


typedef enum {

    LOWPRIO_CH1,
    LOWPRIO_CH2,
    LOWPRIO_CH3,
    LOWPRIO_CH_TACH,

    LOWPRIO_CH_COUNT

} lowprio_scheduler_channel_t;


typedef enum {

    INTERVAL_A,
    INTERVAL_B,
    INTERVAL_PAUSE,

    INTERVAL_COUNT

} lowprio_scheduler_intervals_t;


typedef union
{
     U32 all_flags;

     struct
     {
        U32 interval_B_enabled :1;
        U32 interval_Pause_enabled :1;

        U32 power_after_interval_A :1;

        U32 free_running :1;

     };

} lowprio_scheduler_activation_flags_t;



/**
transfer object for lowprio scheduler activation
*/
typedef struct _lowprio_scheduler_activation_parameters_t {

    U32 intervals_us[INTERVAL_COUNT];

    //total amount of sequences to generate
    U32 cycles;

    //amount of high/low pulses in one sequence between pause
    U32 sequence_length;

    lowprio_scheduler_activation_flags_t flags;

} lowprio_scheduler_activation_parameters_t;





typedef struct _lowprio_scheduler_channel_state_t {

    //channel activation parameters
    lowprio_scheduler_activation_parameters_t parameters;

    //channel data
    lowprio_scheduler_intervals_t allocated_interval;
    U32 cycle_counter;
    U32 sequence_counter;

    //worker functions
    lowprio_scheduler_callback_func callback;
    lowprio_timer_alloc_func timer_alloc;
    lowprio_timer_reset_func timer_reset;

} lowprio_scheduler_channel_state_t;


//lowprio scheduler manager
typedef struct _lowprio_scheduler_mgr_t {

    lowprio_scheduler_channel_state_t channels[LOWPRIO_CH_COUNT];
    bool init_done;

} lowprio_scheduler_mgr_t;







void init_Lowprio_Scheduler();

void lowprio_scheduler_set_channel(lowprio_scheduler_channel_t Channel, lowprio_scheduler_activation_parameters_t * pParameters);
void lowprio_scheduler_reset_channel(lowprio_scheduler_channel_t Channel);

#endif // LOWPRIOSCHEDULER_H_INCLUDED
