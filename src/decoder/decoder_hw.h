#ifndef DECODERHW_H_INCLUDED
#define DECODERHW_H_INCLUDED

#include "stm32_libs/boctok_types.h"

/**
decoder timer prescaler

WARNING:
when changing, adjust CRANK_NOISE_FILTER accordingly!
*/
#define DECODER_TIMER_PRESCALER 1000UL
#define DECODER_TIMER_PERIOD_US 10
#define DECODER_TIMER_OVERFLOW_MS 655




typedef enum {

    SENSING_DISABLED,
    SENSING_RISE,
    SENSING_FALL,
    SENSING_EDGE,
    SENSING_INVERT

} decoder_sensing_t;


/**
decoder_hw_state_t
*/
typedef struct {

     U8 timer_continuous_mode :1;
     U8 timer_reset_req :1;

} decoder_hw_state_t;


typedef struct {

    U32 timer_prescaler;
    U32 timer_period_us;
    U32 timer_overflow_ms;

    U32 current_timer_value;
    U32 prev1_timer_value;
    U32 prev2_timer_value;
    U32 captured_positions_cont;

    decoder_sensing_t crank_pickup_sensing;
    decoder_sensing_t cis_sensing;

    decoder_hw_state_t state;

} decoder_hw_t;


extern volatile decoder_hw_t Decoder_hw;

void init_decoder_hw();
void disable_decoder_hw();

void decoder_start_timer();
void decoder_set_timer_prescaler(U32 Prescaler, U32 Period_us, U32 Overflow_ms);
void update_decoder_timer_compare();
void decoder_stop_timer();
void decoder_mask_crank_irq();
void decoder_unmask_crank_irq();
void decoder_mask_cis_irq();
void decoder_unmask_cis_irq();
void decoder_set_crank_pickup_sensing(decoder_sensing_t sensing);
void decoder_set_cis_sensing(decoder_sensing_t sensing);
void trigger_decoder_irq();

U32 decoder_get_timestamp();

void decoder_set_timer_continuous_mode_on();
void decoder_set_timer_continuous_mode_off();
void decoder_request_timer_reset();

void update_crank_noisefilter();
void update_cam_noisefilter();


#endif // DECODERHW_H_INCLUDED
