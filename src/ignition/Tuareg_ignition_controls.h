#ifndef TUAREG_IGNITION_CONTROLS_H_INCLUDED
#define TUAREG_IGNITION_CONTROLS_H_INCLUDED

#include <Tuareg_platform.h>


/**
default ignition controls
*/
#define DEFAULT_DWELL_POSITION CRK_POSITION_D2
#define DEFAULT_IGNITION_POSITION CRK_POSITION_B1
#define DEFAULT_IGNITION_ADVANCE_DEG 10
#define DEFAULT_REPORTED_DWELL_US 10000


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
        U8 advance_map :1;
     };

} ignition_logic_flags_t;



/**
ignition controls
*/
typedef struct _ignition_control_t {

    //dynamic ignition
    U32 ignition_advance_deg;
    U32 ignition_timing_us;
    crank_position_t ignition_pos;

    //dynamic dwell
    U32 dwell_us;
    U32 dwell_timing_us;
    crank_position_t dwell_pos;

    //status data
    ignition_logic_flags_t flags;

} ignition_controls_t;




void Tuareg_update_ignition_controls();

void default_ignition_controls(volatile ignition_controls_t * pTarget);
void revlimiter_ignition_controls(volatile ignition_controls_t * pTarget);
void cranking_ignition_controls(volatile ignition_controls_t * pTarget);

void dynamic_ignition_controls(volatile ignition_controls_t * pTarget);



#endif // TUAREG_IGNITION_CONTROLS_H_INCLUDED
