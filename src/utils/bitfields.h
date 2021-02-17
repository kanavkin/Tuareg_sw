#ifndef BITFIELDS_H_INCLUDED
#define BITFIELDS_H_INCLUDED

#include <stdbool.h>
#include "stm32_libs/boctok_types.h"

void setBit_BF8(U32 Pos, VBF8 * pTarget);
void setBit_BF16(U32 Pos, VBF16 * pTarget);
void setBit_BF32(U32 Pos, VBF32 * pTarget);

bool getBit_BF8(U32 Pos, VBF8 Target);
bool getBit_BF16(U32 Pos, VBF16 Target);
bool getBit_BF32(U32 Pos, VBF32 Target);



#endif // BITFIELDS_H_INCLUDED
