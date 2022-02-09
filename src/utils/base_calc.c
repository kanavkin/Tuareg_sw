/**

subject to refactoring:

"safe" calculation means, that an argument error will lead to FATAL state (rather than NMI Irq)

"clip" calculation means that there are some tolerances

TBD!

*/
#include "Tuareg.h"

#include "syslog.h"
#include "base_calc_syslog_locations.h"


/**
calculate the duration (in us) corresponding to an rotation angle
e.g. how long will it take the crank shaft to rotate by xx deg?
*/
U32 calc_rot_duration_us(U32 Angle_deg, U32 Period_us)
{
    return (Angle_deg * Period_us) / 360;
}

/**
calculate the angle (in deg) that the crank shaft will rotate in
a given interval at a given rpm
*/
U32 calc_rot_angle_deg(U32 Interval_us, U32 Period_us)
{
    Assert(Period_us > 0, TID_BASE_CALC, BASECALC_LOC_CALC_ROT_ANGLE_DEG_DIV0);

    return (360 * Interval_us) / Period_us;
}


/**
calculate the rpm figure from rotational period
*/
U32 calc_rpm(U32 Period_us)
{
    Assert(Period_us > 0, TID_BASE_CALC, BASECALC_LOC_CALC_RPM_DIV0);

    return (60000000UL) / Period_us;
}


/**
safe subtraction with clipping
*/
void sub_VU32(VU32 * pMin, VU32 Subtr)
{
    if(*pMin > Subtr)
    {
        *pMin -= Subtr;
    }
    else
    {
        *pMin= 0;
    }
}


/**
safe subtraction with clipping
*/
VU16 subtract_VU16(VU16 Min, VU16 Subtr)
{
    if(Min > Subtr)
    {
        return (Min - Subtr);
    }
    else
    {
         return 0;
    }
}


/**
safe subtraction with clipping
*/
VU32 subtract_VU32(VU32 Min, VU32 Subtr)
{

    if(Min > Subtr)
    {
        return (Min - Subtr);
    }
    else
    {
         return 0;
    }
}


/**
safe absolute difference
*/
VU32 abs_delta_VU32(VU32 Val1, VU32 Val2)
{

    if(Val1 > Val2)
    {
        return (Val1 - Val2);
    }
    else if(Val2 > Val1)
    {
        return (Val2 - Val1);
    }
    else
    {
         return 0;
    }
}


/***************************************************************************************************************
*   safe division
*   this division functions will prevent the DIV0 crash
****************************************************************************************************************/

/**
unsigned int ->  unsigned int
*/
VU32 divide_VU32(VU32 Dividend, VU32 Divisor)
{
    if(Divisor == 0)
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_DIVIDE_VU32_DIV0);
        return 0;
    }

    return (Dividend / Divisor);
}


/**
unsigned int -> float
*/
VF32 divide_VF32(VU32 Dividend, VU32 Divisor)
{

    if(Divisor == 0)
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_DIVIDE_VF32_DIV0);
        return 0.0;
    }

    return ((VF32) Dividend) / ((VF32) Divisor);
}


/**
safe float division
*/
VF32 divide_float(VF32 Dividend, VF32 Divisor)
{
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"

    if(Divisor == 0.0)
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_DIVIDE_FLOAT_DIV0);
        return 0.0;
    }

    #pragma GCC diagnostic pop

    return (Dividend / Divisor);
}


/***************************************************************************************************************
*   fault tolerance crank position / phase calculations
****************************************************************************************************************/

/**
returns the crank position that will follow the indicated position, when the crank rotates in nominal direction

CRK_POSITION_UNDEFINED is a properly defined crank position, so calculations on it shall be valid, too.

implemented logic:
UNDEF -> UNDEF

*/
crank_position_t crank_position_after(crank_position_t Position)
{
    if(Position >= CRK_POSITION_COUNT)
    {
        return CRK_POSITION_UNDEFINED;
    }

    if(Position == 0)
    {
        //after the first position in cycle follows the last one
        return CRK_POSITION_COUNT -1;
    }

    return (Position -1);
}

/**
returns the opposite phase to the given Phase

PHASE_UNDEFINED is a properly defined engine phase, so calculations on it shall be valid, too.

implemented logic:

COMP -> EX
EX -> COMP
UNDEF -> UNDEF

*/
volatile engine_phase_t opposite_phase(volatile engine_phase_t Phase_in)
{
    if(Phase_in == PHASE_CYL1_COMP)
    {
        return PHASE_CYL1_EX;
    }
    else if(Phase_in == PHASE_CYL1_EX)
    {
        return PHASE_CYL1_COMP;
    }

    return PHASE_UNDEFINED;
}






/*
void setBit_U8(U32 Pos, VU8 * pTarget)
{
    if(Pos < 8)
    {
        *pTarget |= (1 << Pos);
    }
}

void setBit_U16(U32 Pos, VU16 * pTarget)
{
    if(Pos < 16)
    {
        *pTarget |= (1 << Pos);
    }
}

void setBit_U32(U32 Pos, VU32 * pTarget)
{
    if(Pos < 32)
    {
        *pTarget |= (1 << Pos);
    }
}


U8 lowByte(U16 in)
{
    return (U8)(in & 0x00FF);
}

U8 highByte(U16 in)
{
    return (U8)(in >> 8);
}

U16 word(U8 high, U8 low)
{
    return ((high << 8) | low);
}

U32 dword(U8 Msb, U8 Mid1, U8 Mid2, U8 Lsb)
{
    return ( (Msb << 24) | (Mid1 << 16) | (Mid2 << 8) | Lsb );
}


*/





void memclr_boctok(void * pBegin, U32 Length)
{
    VU8 * pData= (VU8 *) pBegin;
    U32 i;

    for(i=0; i < Length; i++)
    {
        *(pData + i)= 0;
    }
}


U32 floor_boctok(VF32 Argument)
{
    return (U32)(Argument + 32768.0) - 32768;
}


U32 ceiling_boctok(VF32 Argument)
{
    return 32768 - (U32)(32768.0 - Argument);
}


/****************************************************************************************************************************************
*   mathematical functions
*   providing invalid arguments to these functions will cause fatal error
****************************************************************************************************************************************/


/**
solves the linear equation y = mx + n
*/
VF32 solve_linear(VF32 Y, VF32 M, VF32 N)
{
    VF32 inverse;


    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"

    Assert(M != 0.0, TID_BASE_CALC, BASECALC_LOC_SOLVE_LINEAR_ARGS);

/*
    if(M == 0.0)
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_SOLVE_LINEAR_ARGS);
        return 0.0;
    }
*/

    #pragma GCC diagnostic pop

    //x = ( y - n)  / m
    inverse= ((Y - N) / M);

    return inverse;
}


/**
implements the exponential moving average filter
EMA: y[n]= y[n−1] * (1−α) + x[n] * α
*/
VF32 calc_ema(VF32 Alpha, VF32 Last_value, VF32 New_value)
{
    VF32 ema;

    Assert(Alpha > 0.0, TID_BASE_CALC, BASECALC_LOC_EMA_ARGS);
    Assert(Alpha < 1.01, TID_BASE_CALC, BASECALC_LOC_EMA_ARGS);

/*
    //check alpha range
    if((Alpha <= 0) || (Alpha > 1.0))
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_EMA_ARGS);
        return 0.0;
    }
*/

    // EMA: y[n]= y[n−1] * (1−α) + x[n] * α
    ema= (1 - Alpha) * Last_value + Alpha * New_value;

    return ema;
}


/**
calculates d/dt
result is in #/s
*/
VF32 calc_derivative_s(VF32 Last_Value, VF32 New_Value, VU32 Interval_us)
{
    VF32 diff, deriv;

    Assert(Interval_us > 0, TID_BASE_CALC, BASECALC_LOC_DERIVATIVE_ARGS);

/*
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"

    if(Interval_us <= 0.0)
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_DERIVATIVE_ARGS);
        return 0.0;
    }

    #pragma GCC diagnostic pop
*/

    //calculate the current change rate and scale to a one second interval
    diff= 1000000 * (New_Value - Last_Value);
    deriv= diff / (VF32) Interval_us;

    return deriv;
}





