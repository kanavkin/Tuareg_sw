#ifndef DECODERHW_H_INCLUDED
#define DECODERHW_H_INCLUDED

#include "stm32_libs/boctok_types.h"

/**
decoder timer prescaler
T.timer := 4 us @ 100 MHz

WARNING:
when changing, adjust the crank_rotation_period_us calculation in decoder_logic module accordingly!
*/
#define DECODER_TIMER_PSC 400UL


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


#endif // DECODERHW_H_INCLUDED
