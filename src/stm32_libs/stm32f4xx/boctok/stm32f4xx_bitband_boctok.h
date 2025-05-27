#ifndef STM32_BITBAND_H_INCLUDED
#define STM32_BITBAND_H_INCLUDED

#include "../boctok_types.h"



/**
Access bit banded peripheral register by direct address.

https://www.mikrocontroller.net/articles/ARM_Bitbanding

usage:


    EXTI->FTSR |= EXTI_FTSR_TR0;
    EXTI->RTSR &= ~EXTI_RTSR_TR0;

    translates to:

    BBPeriphMask(EXTI->FTSR, EXTI_FTSR_TR0)= 1;
    BBPeriphMask(EXTI->RTSR, EXTI_RTSR_TR0)= 0;

    or

    BBPeriphBit(EXTI->FTSR, 0)= 1;
    BBPeriphBit(EXTI->RTSR, 0)= 0;
*/

#define BBPeriphBit(perReg, bit)    (*(__typeof__(perReg)*) ((PERIPH_BB_BASE + (((unsigned)&(perReg) - PERIPH_BASE) << 5) + ((bit) << 2))))
#define BBPeriphMask(perReg, mask)  BBPeriphBit(perReg, BBitOfMask(mask))
#define BBitOfMask(mask)    (31 - __builtin_clz(mask))




#endif // STM32_BITBAND_H_INCLUDED
