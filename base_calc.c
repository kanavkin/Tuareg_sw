/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg.h"


#warning TODO (oli#1#): add range check and clipping

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
VU32 sub_U32(VU32 Min, VU32 Subtr)
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



