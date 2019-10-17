#ifndef DECODERHW_H_INCLUDED
#define DECODERHW_H_INCLUDED

#include "stm32_libs/boctok_types.h"

//decoder timer prescaler
#define DECODER_TIMER_PSC 196UL


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

void init_decoder_hw();
void decoder_start_timer();
void decoder_stop_timer();
void decoder_mask_crank_irq();
void decoder_unmask_crank_irq();
void decoder_mask_cis_irq();
void decoder_unmask_cis_irq();
void decoder_set_crank_pickup_sensing(sensing_t sensing);
void trigger_decoder_irq();


#endif // DECODERHW_H_INCLUDED
