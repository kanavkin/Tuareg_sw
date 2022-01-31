#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "decoder_cis.h"
#include "decoder_debug.h"
#include "Tuareg_decoder.h"




/*********************************************************************************************************************************
internal data types
*********************************************************************************************************************************/

typedef enum {

    DSTATE_TIMEOUT,
    DSTATE_INIT,
    DSTATE_ASYNC,
    DSTATE_ASYNC_KEY,
    DSTATE_ASYNC_GAP,
    DSTATE_SYNC,
    DSTATE_COUNT

} decoder_internal_state_t;


/*********************************************************************************************************************************
internal data - not visible from outside
*********************************************************************************************************************************/

typedef struct {

    //exported data
    decoder_output_t out;

    //decoder internal state
    decoder_internal_state_t state;

    //timeout_count indicates how much consecutive timer update events have occurred
    U32 timeout_count;

    //crankshaft
    U32 last_crank_rpm;

    //cylinder identification sensor
    decoder_cis_t cis;

} Tuareg_decoder_t;


extern volatile Tuareg_decoder_t Decoder;

void init_decoder_logic();



/*
void decoder_set_state(decoder_internal_state_t NewState);

//timing data
void update_timing_data();
void reset_timing_data();

void reset_position_data();
void reset_internal_data();
void reset_timeout_counter();


//sync checker
bool check_sync_ratio();
bool check_sync_ratio_async();
*/



//Decoder_hw callback functions
void decoder_crank_handler();
void decoder_crank_noisefilter_handler();
void decoder_crank_timeout_handler();






#endif // DECODERLOGIC_H_INCLUDED
