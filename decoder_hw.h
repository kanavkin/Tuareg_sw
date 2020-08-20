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

    DISABLED,
    RISE,
    FALL,
    EDGE,
    INVERT

} sensing_t;

/**
set up the trigger edge detection mode required to detect a key begin or end event
RISE/FALL
*/
#define SENSING_KEY_BEGIN FALL
#define SENSING_KEY_END RISE


typedef struct {

    sensing_t crank_pickup_sensing;

} decoder_hw_t;

extern void init_decoder_hw();
extern void decoder_start_timer();
extern void decoder_stop_timer();
extern void decoder_mask_crank_irq();
extern void decoder_unmask_crank_irq();
extern void decoder_mask_cis_irq();
extern void decoder_unmask_cis_irq();
void decoder_set_crank_pickup_sensing(sensing_t sensing);
extern void trigger_decoder_irq();
VU32 decoder_get_data_age_us();


#endif // DECODERHW_H_INCLUDED
