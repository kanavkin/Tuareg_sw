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

void setBit_U8(U32 Pos, VU8 * pTarget);
void setBit_U16(U32 Pos, VU16 * pTarget);


void increment_crank_position(volatile crank_position_t * pPosition);
#endif // ROTATIONCALC_H_INCLUDED
