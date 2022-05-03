/**



*/
#include <Tuareg_platform.h>
#include <Tuareg.h>


void setBit_BF8(U32 Pos, VBF8 * pTarget)
{
    VitalAssert(Pos < BF8_LEN, TID_BITFIELDS, 0);

    *pTarget |= (1 << Pos);

}

void setBit_BF16(U32 Pos, VBF16 * pTarget)
{
    VitalAssert(Pos < BF16_LEN, TID_BITFIELDS, 1);

    *pTarget |= (1 << Pos);
}

void setBit_BF32(U32 Pos, VBF32 * pTarget)
{
    VitalAssert(Pos < BF32_LEN, TID_BITFIELDS, 2);

    *pTarget |= (1 << Pos);
}


bool getBit_BF8(U32 Pos, VBF8 Target)
{
    VitalAssert(Pos < BF8_LEN, TID_BITFIELDS, 3);

    return (Target & (1 << Pos))? true : false;
}


bool getBit_BF16(U32 Pos, VBF16 Target)
{
    VitalAssert(Pos < BF16_LEN, TID_BITFIELDS, 4);

    return (Target & (1 << Pos))? true : false;
}

bool getBit_BF32(U32 Pos, VBF32 Target)
{
    VitalAssert(Pos < BF32_LEN, TID_BITFIELDS, 5);

    return (Target & (1 << Pos))? true : false;
}



