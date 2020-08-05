#ifndef IGNITIONLOGIC_H_INCLUDED
#define IGNITIONLOGIC_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
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
#define DEFAULT_DWELL_DEG 180
#define DEFAULT_ADVANCE_DEG 10


/**
ignition_timing_t defines a transfer object

keeps the relevant data for the timing of an ignition channel
*/
typedef struct _ignition_timing_t {

    //status data
    U16 ignition_advance_deg;
    U16 dwell_deg;

    //timing
    U32 coil_dwell_timing_us;
    U32 coil_ignition_timing_us;
    crank_position_t coil_dwell_pos;
    crank_position_t coil_ignition_pos;

} ignition_timing_t;



void fit_position( VU32 Period_us, VU32 Crank_angle_deg, volatile crank_position_t * pTarget_position, VU32 * pTarget_delay_us);
void calculate_ignition_timing(volatile ignition_timing_t * pTarget, VU32 Period_us, VU32 Rpm);
void default_ignition_timing(volatile ignition_timing_t * pTarget);
extern void cranking_ignition_timing(volatile ignition_timing_t * pTarget);
void trigger_coil_by_timer(VU32 delay_us, VU32 level);





#endif // IGNITIONLOGIC_H_INCLUDED
