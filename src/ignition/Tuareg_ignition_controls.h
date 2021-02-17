#ifndef TUAREG_IGNITION_CONTROLS_H_INCLUDED
#define TUAREG_IGNITION_CONTROLS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


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



#define IGNITION_REQUIRED_CONFIG_VERSION 1


/**
ignition_logic_state_t
*/
typedef union
{
     U16 all_flags;

     struct
     {
        U16 valid :1;
        U16 default_controls :1;
        U16 cranking_controls :1;
        U16 dynamic_controls :1;
        U16 rev_limiter :1;
        U16 sequential_mode :1;
        U16 cold_idle :1;
        U16 advance_map :1;
        U16 advance_tps :1;
     };

} ignition_logic_state_t;



/**
ignition_control_t defines a transfer object
*/
typedef struct _ignition_control_t {

    //dynamic ignition
    U32 ignition_advance_deg;
    U32 ignition_timing_us;
    crank_position_t ignition_pos;

    //dynamic dwell
    U32 dwell_timing_sequential_us;
    U32 dwell_timing_batch_us;
    U32 dwell_sequential_us;

    //fallback dwell
    crank_position_t dwell_pos;
    U32 dwell_batch_us;

    //status data
    ignition_logic_state_t state;

} ignition_control_t;




void Tuareg_update_ignition_controls();

extern void default_ignition_controls();
extern void cranking_ignition_controls();
extern void revlimiter_ignition_controls();
extern exec_result_t dynamic_ignition_controls();



#endif // TUAREG_IGNITION_CONTROLS_H_INCLUDED
