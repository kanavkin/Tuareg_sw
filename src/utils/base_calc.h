#ifndef BASECALC_H_INCLUDED
#define BASECALC_H_INCLUDED

#include <Tuareg_platform.h>

U32 calc_rot_duration_us(U32 Angle_deg, U32 Period_us);
U32 calc_rot_angle_deg(U32 Interval_us, U32 Period_us);
U32 calc_rpm(U32 Period_us);

void sub_VU32(VU32 * pMin, U32 Subtr);
U32 subtract_U32(U32 Min, U32 Subtr);
U32 abs_delta_U32(U32 Val1, U32 Val2);

U32 divide_U32(U32 Dividend, U32 Divisor);
F32 divide_F32(U32 Dividend, U32 Divisor);
F32 divide_float(F32 Dividend, F32 Divisor);

crank_position_t crank_position_after(crank_position_t Position);
engine_phase_t opposite_phase(engine_phase_t Phase_in);


F32 solve_linear(F32 Y, F32 M, F32 N);

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

U32 floor_boctok(F32 Argument);
U32 ceiling_boctok(F32 Argument);

F32 calc_ema(F32 Alpha, F32 Last_value, F32 New_value);
F32 calc_derivative_s(F32 Last_Value, F32 New_Value, U32 Interval_us);


F32 calc_pow_float(F32 Base, U32 Exp);
U32 calc_pow_U32(U32 Base, U32 Exp);

F32 calc_expf(F32 Arg, S32 Base, U32 MaxOrder);


#endif // BASECALC_H_INCLUDED
