/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"


/// TODO (oli#1#): add range check and clipping

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
    if(Period_us == 0)
    {
        return 0;
    }

    return (360 * Interval_us) / Period_us;
}


/**
calculate the rpm figure from rotational period
*/
U32 calc_rpm(U32 Period_us)
{
    if(Period_us > 0)
    {
        return (60000000UL) / Period_us;
    }
    else
    {
        return 0;
    }
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



/**
safe division with halt
*/
VU32 divide_VU32(VU32 Dividend, VU32 Divisor)
{
/// TODO (oli#1#): add assert

    if(Divisor == 0)
    {
        print(DEBUG_PORT, "\r\n *** ERRORO *** DIV0!");
        return 0;
    }

    return (Dividend / Divisor);
}



/*
returns the crank position that will follow the indicated position, when the crank rotates in nominal direction
*/
crank_position_t crank_position_after(crank_position_t Position)
{
    if(Position >= CRK_POSITION_COUNT)
    {
        /// TODO (oli#3#): use assert for this argument error
        return CRK_POSITION_UNDEFINED;
    }

    if(Position == 0)
    {
        //after the first position in cycle follows the last one
        return CRK_POSITION_COUNT -1;
    }

    return (Position -1);
}

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
    else
    {
        return PHASE_UNDEFINED;
    }
}



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





