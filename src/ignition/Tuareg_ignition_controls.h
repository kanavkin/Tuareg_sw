#ifndef TUAREG_IGNITION_CONTROLS_H_INCLUDED
#define TUAREG_IGNITION_CONTROLS_H_INCLUDED

#include <Tuareg_platform.h>





/**
cranking ignition controls reported values
*/
#define CRANKING_REPORTED_IGNITION_ADVANCE_DEG 3
#define CRANKING_REPORTED_DWELL_US 10000

/**
rev limiter ignition controls reported values
*/
#define REVLIMITER_REPORTED_IGNITION_ADVANCE_DEG 0
#define REVLIMITER_REPORTED_DWELL_US 0


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
    F32 ignition_advance_deg;
    F32 ignition_timing_us;
    crank_position_t ignition_pos;

    //dynamic dwell
    F32 dwell_us;
    F32 dwell_timing_us;
    crank_position_t dwell_pos;

    //status data
    ignition_logic_flags_t flags;

} ignition_controls_t;




void Tuareg_update_ignition_controls(volatile ignition_controls_t * pTarget);

void default_ignition_controls(volatile ignition_controls_t * pTarget);
void revlimiter_ignition_controls(volatile ignition_controls_t * pTarget);
void cranking_ignition_controls(volatile ignition_controls_t * pTarget);

void dynamic_ignition_controls(volatile ignition_controls_t * pTarget);



#endif // TUAREG_IGNITION_CONTROLS_H_INCLUDED
