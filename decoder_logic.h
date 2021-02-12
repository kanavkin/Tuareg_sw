#ifndef DECODERLOGIC_H_INCLUDED
#define DECODERLOGIC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "process_table.h"

#define DECODER_REQUIRED_CONFIG_VERSION 3

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


    VU8 position_valid :1;
    VU8 phase_valid :1;
    VU8 period_valid :1;
    VU8 rpm_valid :1;
    VU8 accel_valid :1;

    VU8 timeout :1;

} decoder_output_state_t;


typedef struct {

    VU8 lobe_begin_detected :1;
    VU8 lobe_end_detected :1;
    VU8 cis_failure :1;

} decoder_cis_state_t;


typedef struct {

    /*
    the following data is relevant for engine management
    its validity is indicated in the output state
    */

    //position data
    crank_position_t crank_position;
    engine_phase_t phase;

    //rotational period of the crankshaft
    U32 crank_period_us;
    U32 crank_rpm;
    U32 prev1_crank_rpm;

    //acceleration figure of the crankshaft
    VF32 crank_acceleration;

    //output data state
    decoder_output_state_t outputs;

    /*
    strictly internal data
    */

    //decoder internal state
    decoder_internal_state_t state;

    //decoder_timeout_thrs reflects the configured threshold in timer update events when the timeout shall occur
    U32 decoder_timeout_thrs;

    //timeout_count indicates how much consecutive timer update events have occurred
    U32 timeout_count;

    /*
    CIS internals
    */
    U32 lobe_begin_timestamp;
    U32 lobe_end_timestamp;
    U32 cis_sync_counter;
    decoder_cis_state_t cis;

} Tuareg_decoder_t;



volatile Tuareg_decoder_t * init_decoder_logic();

//extern volatile Tuareg_decoder_t Decoder;

extern void decoder_set_state(decoder_internal_state_t NewState);
extern void update_engine_speed();
extern void reset_internal_data();
extern void reset_timeout_counter();

extern bool check_sync_ratio();
extern bool check_sync_ratio_async();

extern void decoder_crank_handler();
extern void decoder_crank_noisefilter_handler();
extern void decoder_crank_timeout_handler();

extern void enable_cis();
extern void disable_cis();
extern void decoder_update_cis();
extern void decoder_cis_handler();
extern void decoder_cis_noisefilter_handler();


extern void sync_lost_debug_handler();
extern void got_sync_debug_handler();
extern void decoder_timeout_debug_handler();



#endif // DECODERLOGIC_H_INCLUDED
