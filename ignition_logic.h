#ifndef IGNITIONLOGIC_H_INCLUDED
#define IGNITIONLOGIC_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_process_data.h"
#include "decoder_logic.h"
#include "trigger_wheel_layout.h"


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
#define DYNAMIC_DWELL_LATEST_POSITION CRK_POSITION_D2
#define DYNAMIC_DWELL_EARLIEST_POSITION CRK_POSITION_B2


/**
values to report during cranking
*/
#define CRANKING_REPORTED_IGNITION_ADVANCE_DEG 3
#define CRANKING_REPORTED_DWELL_MS 10



/**

*/
typedef enum {

    IGNLOG_DEFAULT_TIMING,
    IGNLOG_CRANKING_TIMING,
    IGNLOG_REV_LIMITER,
    IGNLOG_DYNAMIC,
    IGNLOG_COLD_IDLE,
    IGNLOG_ADVANCE_MAP,
    IGNLOG_ADVANCE_TPS,
    IGNLOG_COUNT

} ignition_logic_state_bits_t;




/**
ignition_timing_t defines a transfer object

keeps the relevant data for the timing of an ignition channel
*/
typedef struct _ignition_timing_t {

    //functional data
    U16 ignition_advance_deg;
    U8 dwell_ms;

    //functional timing
    U32 coil_dwell_timing_us;
    U32 coil_ignition_timing_us;
    crank_position_t coil_dwell_pos;
    crank_position_t coil_ignition_pos;

    //status data
    ignition_logic_state_bits_t state;

} ignition_timing_t;



void fit_position( VU32 Ign_advance_deg, VU32 Dwell_target_us, volatile process_data_t * pImage, volatile ignition_timing_t * pTarget);
void update_ignition_timing(volatile process_data_t * pImage, volatile ignition_timing_t * pTarget);
void default_ignition_timing(volatile ignition_timing_t * pTarget);
extern void cranking_ignition_timing(volatile ignition_timing_t * pTarget);
void trigger_coil_by_timer(VU32 delay_us, VU32 level);





#endif // IGNITIONLOGIC_H_INCLUDED
