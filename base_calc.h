#ifndef ROTATIONCALC_H_INCLUDED
#define ROTATIONCALC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "trigger_wheel_layout.h"

U32 calc_rot_duration_us(U32 Angle_deg, U32 Period_us);
U32 calc_rot_angle_deg(U32 Interval_us, U32 Period_us);
U32 calc_rpm(U32 Period_us);

void sub_VU32(VU32 * pMin, VU32 Subtr);
VU32 subtract_VU32(VU32 Min, VU32 Subtr);
VU32 abs_delta_VU32(VU32 Val1, VU32 Val2);

volatile crank_position_t next_crank_position(crank_position_t Current_Position);
#endif // ROTATIONCALC_H_INCLUDED
