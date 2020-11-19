/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"


#include <stdbool.h>

void setBit_BF8(U32 Pos, VBF8 * pTarget)
{
    if(Pos < BF8_LEN)
    {
        *pTarget |= (1 << Pos);
    }
}

void setBit_BF16(U32 Pos, VBF16 * pTarget)
{
    if(Pos < BF16_LEN)
    {
        *pTarget |= (1 << Pos);
    }
}

void setBit_BF32(U32 Pos, VBF32 * pTarget)
{
    if(Pos < BF32_LEN)
    {
        *pTarget |= (1 << Pos);
    }
}


bool getBit_BF8(U32 Pos, VBF8 * pTarget)
{
    if(Pos < BF8_LEN)
    {
        return (*pTarget & (1 << Pos))? true : false;
    }
}


bool getBit_BF16(U32 Pos, VBF16 * pTarget)
{
    if(Pos < BF16_LEN)
    {
        return (*pTarget & (1 << Pos))? true : false;
    }
}

bool getBit_BF32(U32 Pos, VBF32 * pTarget)
{
    if(Pos < BF32_LEN)
    {
        return (*pTarget & (1 << Pos))? true : false;
    }
}



