#ifndef IGNITIONLOGIC_H_INCLUDED
#define IGNITIONLOGIC_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "decoder_logic.h"
#include "trigger_wheel_layout.h"


/**
trigger wheel geometry
*/
#define POSITION_C1_ADVANCE 280
#define POSITION_C2_ADVANCE 275
#define POSITION_D1_ADVANCE 190
#define POSITION_D2_ADVANCE 185
#define POSITION_A1_ADVANCE 100
#define POSITION_A2_ADVANCE 60
#define POSITION_B1_ADVANCE 10
#define POSITION_B2_ADVANCE 0


/**
deprecated?
crank sensor setup
logical state when a key triggers the sensor

#define KEY_SIGNAL_POLARITY OFF
#define IDLE_SIGNAL ON
*/

/**
ignition timing setup

remember that there will be a significant voltage drop
while cranking -> needs longer dwell!
    (D2..B2 is about 180° dwell)

adjust DYNAMIC_MIN_RPM to a value higher than your
desired idle rpm if you want fixed ignition operation there
(rough idling, ...)
*/
#define DEFAULT_DWELL_POSITION POSITION_D1
#define DEFAULT_IGNITION_POSITION POSITION_B1
#define DEFAULT_DWELL_DEG 180
#define DEFAULT_ADVANCE_DEG 10


//#define DYNAMIC_MIN_RPM 1000
#define DYNAMIC_MIN_RPM 10000
#define DYNAMIC_DWELL_US 4000

/**
safety margin for positioning
*/
#define MARGIN_US 15


typedef struct _ignition_timing_t {

    //status data
    U16 ignition_advance_deg;
    U16 dwell_deg;

    //timing
    U32 coil_dwell_timing_us;
    U32 coil_ignition_timing_us;
    engine_position_t coil_dwell_pos;
    engine_position_t coil_ignition_pos;

} ignition_timing_t;



//U32 get_advance(U32 Rpm);
//U32 calc_rot_duration(U32 Angle_deg, U32 Period_us);
//U32 calc_rot_angle(U32 Interval_us, U32 Period_us);

//void init_ignition_logic(volatile ignition_timing_t * initial_timing);

void fit_position( U32 Period_us, U32 Crank_angle_deg, engine_position_t * pTarget_position, U32 * pTarget_delay_us);
void calc_ignition_timing(volatile ignition_timing_t * pTarget, U32 Period_us, U32 Rpm);
void default_ignition_timing(volatile ignition_timing_t * pTarget);
void trigger_coil_by_timer(U32 delay_us, U32 level);





#endif // IGNITIONLOGIC_H_INCLUDED
