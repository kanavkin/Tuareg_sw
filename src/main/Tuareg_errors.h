#ifndef TUAREG_ERRORS_H_INCLUDED
#define TUAREG_ERRORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_ID.h"


extern void Assert(bool Condition, Tuareg_ID Id, U8 Location);
void Fatal(Tuareg_ID Id, U8 Location);
void Limp(Tuareg_ID Id, U8 Location);


#endif // TUAREG_ERRORS_H_INCLUDED
