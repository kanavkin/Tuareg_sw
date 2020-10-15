#ifndef DECODERHW_H_INCLUDED
#define DECODERHW_H_INCLUDED

#include "stm32_libs/boctok_types.h"

/**
decoder timer prescaler

decoder timer resolution affects the timestamp accuracy and the key/gap ratio and rotational speed calculation
its absolute accuracy has no effect on ignition/fuel timing!

for example: t_1° @ 9000 rpm := ~16 us

T.timer := 4 us @ 100 MHz

WARNING:
when changing, adjust CRANK_NOISE_FILTER accordingly!

this configuration parameter intentionally kept as built in config
*/
#define DECODER_TIMER_PSC 400UL
#define DECODER_TIMER_PERIOD_US 4
#define DECODER_TIMER_OVERFLOW_MS 262


typedef enum {

    SENSING_DISABLED,
    SENSING_RISE,
    SENSING_FALL,
    SENSING_EDGE,
    SENSING_INVERT

} sensing_t;

/**
set up the trigger edge detection mode required to detect a key begin or end event
RISE/FALL
*/
#define SENSING_KEY_BEGIN SENSING_FALL
#define SENSING_KEY_END SENSING_RISE


typedef struct {

    volatile sensing_t crank_pickup_sensing;
    volatile sensing_t cis_sensing;

} decoder_hw_t;

void init_decoder_hw();
void decoder_start_timer(U16 Noise_filter);
void decoder_stop_timer();
void decoder_mask_crank_irq();
void decoder_unmask_crank_irq();
void decoder_mask_cis_irq();
void decoder_unmask_cis_irq();
void decoder_set_crank_pickup_sensing(sensing_t sensing);
void decoder_set_cis_sensing(sensing_t sensing);
void trigger_decoder_irq();
VU32 decoder_get_data_age_us();


#endif // DECODERHW_H_INCLUDED
