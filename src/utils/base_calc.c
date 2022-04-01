/**

subject to refactoring:

"safe" calculation means, that an argument error will lead to FATAL state (rather than NMI Irq)

"clip" calculation means that there are some tolerances

TBD!

*/
#include "Tuareg.h"

#include "syslog.h"
#include "base_calc.h"
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
    VitalAssert(Period_us > 0, TID_BASE_CALC, BASECALC_LOC_CALC_ROT_ANGLE_DEG_DIV0);

    return (360 * Interval_us) / Period_us;
}


/**
calculate the rpm figure from rotational period
*/
U32 calc_rpm(U32 Period_us)
{
    VitalAssert(Period_us > 0, TID_BASE_CALC, BASECALC_LOC_CALC_RPM_DIV0);

    return (60000000UL) / Period_us;
}


/**
safe subtraction with clipping
*/
void sub_VU32(VU32 * pMin, U32 Subtr)
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
U16 subtract_VU16(U16 Min, U16 Subtr)
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
U32 subtract_U32(U32 Min, U32 Subtr)
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
U32 abs_delta_U32(U32 Val1, U32 Val2)
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
U32 divide_U32(U32 Dividend, U32 Divisor)
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
F32 divide_F32(U32 Dividend, U32 Divisor)
{

    if(Divisor == 0)
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_DIVIDE_VF32_DIV0);
        return 0.0;
    }

    return ((F32) Dividend) / ((F32) Divisor);
}


/**
safe float division
*/
F32 divide_float(F32 Dividend, F32 Divisor)
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
engine_phase_t opposite_phase(engine_phase_t Phase_in)
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
F32 solve_linear(F32 Y, F32 M, F32 N)
{
    VF32 inverse;


    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"

    VitalAssert(M != 0.0, TID_BASE_CALC, BASECALC_LOC_SOLVE_LINEAR_ARGS);

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
F32 calc_ema(F32 Alpha, F32 Last_value, F32 New_value)
{
    F32 ema;

/*
    using bogus arguments will not lead to hard fault
    fatal state will prevent setting proper values through tuner studio
    wrong values will lead to wrong results
    sw shall be robust

    Assert(Alpha >= 0.0, TID_BASE_CALC, BASECALC_LOC_EMA_ARGS);
    Assert(Alpha < 1.01, TID_BASE_CALC, BASECALC_LOC_EMA_ARGS);
*/

    //check alpha range
    if((Alpha < 0) || (Alpha > 1.0))
    {
        Syslog_Error(TID_BASE_CALC, BASECALC_LOC_EMA_ARGS);
        return New_value;
    }

    // EMA: y[n]= y[n−1] * (1−α) + x[n] * α
    ema= (1 - Alpha) * Last_value + Alpha * New_value;

    return ema;
}


/**
calculates d/dt
result is in #/s
*/
F32 calc_derivative_s(F32 Last_Value, F32 New_Value, U32 Interval_us)
{
    F32 diff, deriv;

    VitalAssert(Interval_us > 0, TID_BASE_CALC, BASECALC_LOC_DERIVATIVE_ARGS);

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
    deriv= diff / (F32) Interval_us;

    return deriv;
}

/*


constexpr float expf_taylor_impl(float x, uint8_t n)
{
	if (x < -2)
	{
		return 0.818f;
	}
	else if (x > 0)
	{
		return 1;
	}

	x = x + 1;

	float x_power = x;
	int fac = 1;
	float sum = 1;

	for (int i = 1; i <= n; i++)
	{
		fac *= i;
		sum += x_power / fac;

		x_power *= x;
	}

	return sum / constant_e;
}



*/

/**
calculates the exponential function e^x
implements taylor expansion of "Order", centered at "Base"

E(x, a, N) := ( 1 + (x-a)/1 + (x-a)/2 + (x-a)/6 + (x-a)/24 ) * e^a
E(x, a, N) := ( 1 + SUM_N( x_shift_pow / factorial ) * e^a

*/
F32 calc_expf(F32 Arg, S32 Base, U32 MaxOrder)
{
    F32 x_shift= 0.0, x_shift_pow= 1.0, p_sum= 1.0;
    U32 factorial= 1, order= 0;

	/**
	shifted argument x
	*/
	x_shift= Arg - Base;

    //0 order - per initialization
    //x_shift_pow= 1.0;
    //factorial= 1;
    //p_sum= 1.0;


    //summands 1..N order
	for(order= 1; order <= MaxOrder; order++)
	{
        //taylor series summands
		x_shift_pow *= x_shift;
        factorial *= order;

        //partial sum n
        p_sum += x_shift_pow / factorial;
	}

	/**
	handle e^a
	*/
	if(Base == 0)
	{
        return p_sum;
	}

	if(Base < 0)
	{
        return divide_float(p_sum, calc_pow_float(cEuler, -Base));
	}
    else
    {
        return p_sum * calc_pow_float(cEuler, Base);
    }

}




F32 calc_pow_float(F32 Base, U32 Exp)
{
    F32 result;
    U32 n;

    if(Exp == 0)
    {
        return 1.0;
    }
    else if(Exp == 1)
    {
        return Base;
    }

    result= Base;

    for(n= 1; n < Exp; n++)
	{
        result *= Base;
    }

    return result;
}


U32 calc_pow_U32(U32 Base, U32 Exp)
{
    U32 n, result= 1;

    for(n= 0; n < Exp; n++)
	{
        result *= Base;
    }

    return result;
}

