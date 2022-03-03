#ifndef TUAREG_DECODER_H_INCLUDED
#define TUAREG_DECODER_H_INCLUDED

#include "Tuareg_types.h"


#define DECODER_REQUIRED_CONFIG_VERSION 3


/*********************************************************************************************************************************
data that describes the internal state of the decoder which is visible to other modules
*********************************************************************************************************************************/

typedef struct {

    VU8 position_valid :1;
    VU8 phase_valid :1;
    VU8 period_valid :1;
    VU8 rpm_valid :1;
    VU8 accel_valid :1;

    VU8 standstill :1;

} decoder_output_flags_t;


/**
the following data is relevant for engine management
its validity is indicated in the output flags
*/
typedef struct _decoder_output_t {

    //crankshaft
    U32 crank_period_us;
    U32 crank_rpm;
    F32 crank_acceleration;

    //position data
    crank_position_t crank_position;
    engine_phase_t phase;

    //output data state
    decoder_output_flags_t flags;

} decoder_output_t;


volatile decoder_output_t * init_Decoder();
void disable_Decoder();

VU32 decoder_get_position_data_age_us();


#endif // TUAREG_DECODER_H_INCLUDED
