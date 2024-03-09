#ifndef TUAREG_DECODER_H_INCLUDED
#define TUAREG_DECODER_H_INCLUDED

#include <Tuareg_platform.h>



#define DECODER_REQUIRED_CONFIG_VERSION 4


/*********************************************************************************************************************************
data that describes the internal state of the decoder which is visible to other modules
*********************************************************************************************************************************/

typedef struct {

    U8 position_valid :1;
    U8 phase_valid :1;
    U8 period_valid :1;
    U8 rpm_valid :1;
    U8 accel_valid :1;

    U8 standstill :1;

    U8 updated :1;

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


void Init_Decoder();
void disable_Decoder();

U32 decoder_get_position_data_age_us();


/**
provide includes for all decoder related code
*/
#include "decoder_hw.h"
#include "decoder_cis.h"
#include "decoder_debug.h"
#include "decoder_logic.h"
#include "decoder_config.h"
#include "decoder_debug.h"
#include "decoder_syslog_locations.h"


#endif // TUAREG_DECODER_H_INCLUDED
