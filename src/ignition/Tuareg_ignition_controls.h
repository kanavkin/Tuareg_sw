#ifndef TUAREG_IGNITION_CONTROLS_H_INCLUDED
#define TUAREG_IGNITION_CONTROLS_H_INCLUDED

#include <Tuareg_platform.h>




/**
ignition_logic_state_t
*/
typedef union
{
     U8 all_flags;

     struct
     {
        U8 valid :1;
        U8 dynamic_controls :1;
        U8 sequential_mode :1;
        U8 cold_idle :1;
     };

} ignition_logic_flags_t;



/**
ignition controls
*/
typedef struct _ignition_control_t {

    //dynamic ignition
    F32 ignition_timing_us;
    crank_position_t ignition_pos;

    //dynamic dwell
    F32 dwell_us;
    F32 dwell_timing_us;
    crank_position_t dwell_pos;

    //status data
    ignition_logic_flags_t flags;

} ignition_controls_t;





/**
export rpm requirements for dynamic ignition
*/
extern const U32 cIgnition_min_dyn_rpm;


void Tuareg_update_ignition_controls();
void clear_ignition_controls();

#endif // TUAREG_IGNITION_CONTROLS_H_INCLUDED
