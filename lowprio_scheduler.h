#ifndef LOWPRIOSCHEDULER_H_INCLUDED
#define LOWPRIOSCHEDULER_H_INCLUDED

#include "Tuareg.h"


#define LOWPRIO_SCHEDULER_PERIOD_US 200

//we are using a 16 bit timer
#define LOWPRIO_SCHEDULER_MAX_PERIOD_US (U16) 0xFFFF * LOWPRIO_SCHEDULER_PERIOD_US -1


#define TOGGLE_CH1_ENABLE_MASK 0
#define TOGGLE_CH2_ENABLE_MASK 1
#define TOGGLE_CH3_ENABLE_MASK 2
#define TOGGLE_CH4_ENABLE_MASK 3

#define TOGGLE_CH1_CYCLE_MASK 4
#define TOGGLE_CH2_CYCLE_MASK 5
#define TOGGLE_CH3_CYCLE_MASK 6
#define TOGGLE_CH4_CYCLE_MASK 7

#define TOGGLE_CH1_SEQUENCE_MASK 8
#define TOGGLE_CH2_SEQUENCE_MASK 9



typedef void (* lowprio_callback)(output_pin_t);


typedef enum {

    LOWPRIO_CH1,
    LOWPRIO_CH2,
    LOWPRIO_CH3,
    LOWPRIO_CH4

} lowprio_scheduler_channel_t;



typedef struct _lowprio_scheduler_t {

    lowprio_callback ch1_callback;
    lowprio_callback ch2_callback;
    lowprio_callback ch3_callback;
    lowprio_callback ch4_callback;

    output_pin_t ch1_action;
    output_pin_t ch2_action;
    output_pin_t ch3_action;
    output_pin_t ch4_action;

    VU32 ch1_delay1_us;
    VU32 ch1_delay2_us;
    VU32 ch1_delay3_us;
    VU32 ch2_delay1_us;
    VU32 ch2_delay2_us;
    VU32 ch2_delay3_us;
    VU32 ch3_delay1_us;
    VU32 ch3_delay2_us;
    VU32 ch4_delay1_us;
    VU32 ch4_delay2_us;

    VU32 toggle_ctrl;

    VU32 ch1_toggle_counter;
    VU32 ch2_toggle_counter;

    VU32 ch1_sequence_length;
    VU32 ch2_sequence_length;


} lowprio_scheduler_t;



void init_lowprio_scheduler();
void lowprio_scheduler_set_channel(lowprio_scheduler_channel_t target_ch, void (* callback_funct)(output_pin_t), output_pin_t action, U32 delay_us);
void lowprio_scheduler_togglemode_channel(lowprio_scheduler_channel_t target_ch, void (* callback_funct)(output_pin_t), U32 delay1_us, U32 delay2_us);
void lowprio_scheduler_reset_channel(lowprio_scheduler_channel_t target_ch);

#endif // LOWPRIOSCHEDULER_H_INCLUDED
