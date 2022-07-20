#ifndef TUAREG_ERRORS_H_INCLUDED
#define TUAREG_ERRORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_ID.h"


#define VITAL_ASSERT(condition, module, location) if((condition) != true) Fatal((module), (location)); return
#define ASSERT_EXEC_OK_VOID(result) if((result) != EXEC_OK) return

void VitalAssert(bool Condition, Tuareg_ID Id, U8 Location);
void FunctionalAssert(bool Condition, Tuareg_ID Id, U8 Location);
void Fatal(Tuareg_ID Id, U8 Location);
void Limp(Tuareg_ID Id, U8 Location);


#endif // TUAREG_ERRORS_H_INCLUDED
