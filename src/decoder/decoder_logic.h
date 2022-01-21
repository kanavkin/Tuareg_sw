#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg_decoder.h"


//#define DECODER_TIMING_DEBUG

/*********************************************************************************************************************************
decoder data types
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


typedef struct {

    VU8 lobe_begin_detected :1;
    VU8 lobe_end_detected :1;
    VU8 cis_failure :1;

} decoder_cis_state_t;



/*********************************************************************************************************************************
debug data
*********************************************************************************************************************************/

#ifdef DECODER_TIMING_DEBUG
typedef struct {

    U32 hw_period_us;
    U32 hw_timer_val;
    U32 period_us;
    U32 rpm;

    decoder_output_state_t out;

} decoder_timing_debug_t;
#endif // DECODER_TIMING_DEBUG


typedef union
{
     U8 all_flags;

     struct {

        VU8 got_sync :1;
        VU8 lost_sync :1;
        VU8 timer_overflow :1;
        VU8 standstill :1;
     };

} decoder_debug_flags_t;





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

    //occurred debug events to process
    decoder_debug_flags_t debug;

    //crankshaft
    U32 last_crank_rpm;

    /*
    CIS internals
    */
    U32 lobe_begin_timestamp;
    U32 lobe_end_timestamp;
    U32 cis_sync_counter;
    decoder_cis_state_t cis;

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
