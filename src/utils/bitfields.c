/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"
#include "Tuareg_ID.h"
#include "Tuareg_errors.h"


#include <stdbool.h>

void setBit_BF8(U32 Pos, VBF8 * pTarget)
{
    Assert(Pos < BF8_LEN, TID_BITFIELDS, 0);

    *pTarget |= (1 << Pos);

}

void setBit_BF16(U32 Pos, VBF16 * pTarget)
{
    Assert(Pos < BF16_LEN, TID_BITFIELDS, 1);

    *pTarget |= (1 << Pos);
}

void setBit_BF32(U32 Pos, VBF32 * pTarget)
{
    Assert(Pos < BF32_LEN, TID_BITFIELDS, 2);

    *pTarget |= (1 << Pos);
}


bool getBit_BF8(U32 Pos, VBF8 Target)
{
    Assert(Pos < BF8_LEN, TID_BITFIELDS, 3);

    return (Target & (1 << Pos))? true : false;
}


bool getBit_BF16(U32 Pos, VBF16 Target)
{
    Assert(Pos < BF16_LEN, TID_BITFIELDS, 4);

    return (Target & (1 << Pos))? true : false;
}

bool getBit_BF32(U32 Pos, VBF32 Target)
{
    Assert(Pos < BF32_LEN, TID_BITFIELDS, 5);

    return (Target & (1 << Pos))? true : false;
}



