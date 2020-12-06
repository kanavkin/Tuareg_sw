/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg.h"


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
    if(Period_us > 0)
    {
        return (360 * Interval_us) / Period_us;
    }
    else
    {
        return 0;
    }
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


crank_position_t next_crank_position(crank_position_t Position)
{
    switch(Position)
    {
    case CRK_POSITION_A1:
        return CRK_POSITION_A2;
        break;

    case CRK_POSITION_A2:
        return  CRK_POSITION_B1;
        break;

    case CRK_POSITION_B1:
        return  CRK_POSITION_B2;
        break;

    case CRK_POSITION_B2:
        return  CRK_POSITION_C1;
        break;

    case CRK_POSITION_C1:
        return  CRK_POSITION_C2;
        break;

    case CRK_POSITION_C2:
        return  CRK_POSITION_D1;
        break;

    case CRK_POSITION_D1:
        return  CRK_POSITION_D2;
        break;

    case CRK_POSITION_D2:
        return  CRK_POSITION_A1;
        break;

    default:
        return  CRK_POSITION_UNDEFINED;
        break;
    }
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





