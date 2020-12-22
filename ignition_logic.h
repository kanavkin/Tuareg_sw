#ifndef IGNITIONLOGIC_H_INCLUDED
#define IGNITIONLOGIC_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_process_data.h"
#include "decoder_logic.h"


/**
essential config section
*/

/**
ignition timing setup

remember that there will be a significant voltage drop
while cranking -> needs longer dwell!
    (D2..B2 is about 180° dwell)

adjust DYNAMIC_MIN_RPM to a value higher than your desired idle rpm if you want fixed ignition operation there
(rough idling, ...)
*/
#define DEFAULT_DWELL_POSITION CRK_POSITION_D2
#define DEFAULT_IGNITION_POSITION CRK_POSITION_B1
#define DEFAULT_IGNITION_ADVANCE_DEG 10
#define DEFAULT_REPORTED_DWELL_MS 10

/**
dynamic ignition calculation dwell position setup

fit_position() will select a dwell position in between DYNAMIC_DWELL_LATEST_POSITION and DYNAMIC_DWELL_EARLIEST_POSITION

The proper selected DYNAMIC_DWELL_EARLIEST_POSITION allows the spark to burn for at least 1ms after ignition event.
Dwell position will default to DYNAMIC_DWELL_EARLIEST_POSITION.

DYNAMIC_DWELL_LATEST_POSITION shall be selected to not interfere with the scheduler allocation, but allow for maximum dwell.
*/
#define DYNAMIC_DWELL_LATEST_POSITION_COMP CRK_POSITION_D2
#define DYNAMIC_DWELL_EARLIEST_POSITION_UNPHASED CRK_POSITION_B2


/**
values to report during cranking
*/
#define CRANKING_REPORTED_IGNITION_ADVANCE_DEG 3
#define CRANKING_REPORTED_DWELL_MS 10

/**
values to report during rev limiter action
*/
#define REVLIMITER_REPORTED_IGNITION_ADVANCE_DEG 0
#define REVLIMITER_REPORTED_DWELL_MS 0

/**
ignition_logic_state_t
*/
typedef union
{
     U8 all_flags;

     struct
     {
        U8 default_timing :1;
        U8 cranking_timing :1;
        U8 rev_limiter :1;
        U8 dynamic :1;
        U8 cold_idle :1;
        U8 advance_map :1;
        U8 advance_tps :1;
        U8 extended_dwell :1;
     };

} ignition_logic_state_t;



/**
ignition_control_t defines a transfer object

It keeps the relevant data for the timing of all ignition channels.
*/
typedef struct _ignition_control_t {

    //ignition setup
    U16 ignition_advance_deg;
    U32 ignition_timing_us;
    crank_position_t ignition_pos;

    //dwell setup in phased mode
    crank_position_t dwell_pos_phased;
    engine_phase_t dwell_phase_cyl1;
    engine_phase_t dwell_phase_cyl2;
    U8 dwell_ms_phased;

    //dwell setup in unphased mode
    crank_position_t dwell_pos_unphased;
    U8 dwell_ms_unphased;

    //status data
    ignition_logic_state_t state;

} ignition_control_t;


exec_result_t calculate_dynamic_ignition_controls(volatile process_data_t * pImage, volatile ignition_control_t * pTarget);
exec_result_t calculate_ignition_alignment( VU32 Ignition_AD, VU32 Dwell_target_us, VU32 Crank_T_us, volatile ignition_control_t * pTarget);

void default_ignition_controls(volatile ignition_control_t * pTarget);
void cranking_ignition_controls(volatile ignition_control_t * pTarget);
void revlimiter_ignition_controls(volatile ignition_control_t * pTarget);

void trigger_coil_by_timer(VU32 delay_us, VU32 level);





#endif // IGNITIONLOGIC_H_INCLUDED
