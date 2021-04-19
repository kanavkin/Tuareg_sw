#ifndef BASECALC_H_INCLUDED
#define BASECALC_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

U32 calc_rot_duration_us(U32 Angle_deg, U32 Period_us);
U32 calc_rot_angle_deg(U32 Interval_us, U32 Period_us);
U32 calc_rpm(U32 Period_us);

void sub_VU32(VU32 * pMin, VU32 Subtr);
VU32 subtract_VU32(VU32 Min, VU32 Subtr);
VU32 abs_delta_VU32(VU32 Val1, VU32 Val2);

VU32 divide_VU32(VU32 Dividend, VU32 Divisor);
VF32 divide_VF32(VU32 Dividend, VU32 Divisor);
VF32 divide_float(VF32 Dividend, VF32 Divisor);

crank_position_t crank_position_after(crank_position_t Position);
engine_phase_t opposite_phase(engine_phase_t Phase_in);


VF32 solve_linear(VF32 Y, VF32 M, VF32 N);

/*

//Handy bitsetting macros
#define BIT_SET(a,b) ((a) |= (1<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1<<(b)))
#define BIT_CHECK(var,pos) !!((var) & (1<<(pos)))
*/

U16 word(U8 high, U8 low);
U8 lowByte(U16);
U8 highByte(U16);
U32 dword(U8 Msb, U8 Mid1, U8 Mid2, U8 Lsb);


void memclr_boctok(void * pBegin, U32 Length);

#endif // BASECALC_H_INCLUDED
