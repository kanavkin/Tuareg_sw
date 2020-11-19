#ifndef BITFIELDS_H_INCLUDED
#define BITFIELDS_H_INCLUDED

#include "stm32_libs/boctok_types.h"

void setBit_BF8(U32 Pos, VBF8 * pTarget);
void setBit_BF16(U32 Pos, VBF16 * pTarget);
void setBit_BF32(U32 Pos, VBF32 * pTarget);

BOOL32 getBit_BF8(U32 Pos, VBF8 * pTarget);
BOOL32 getBit_BF16(U32 Pos, VBF16 * pTarget);
BOOL32 getBit_BF32(U32 Pos, VBF32 * pTarget)



#endif // BITFIELDS_H_INCLUDED
