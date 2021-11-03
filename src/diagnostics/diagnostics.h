#ifndef DIAGNOSTICS_H_INCLUDED
#define DIAGNOSTICS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#define USE_DIAGNOSTICS

#include "decoder_diag.h"
#include "ignition_diag.h"
#include "fueling_diag.h"
#include "Tuareg_diag.h"
#include "scheduler_diag.h"
#include "sensors_diag.h"


exec_result_t copy_diag_data(VU32 * pSource, U32 Length);
VU32 get_diag_data(VU32 Index);
void release_diag_shadow();



#endif // DIAGNOSTICS_H_INCLUDED
