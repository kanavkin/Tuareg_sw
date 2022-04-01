#ifndef ACCEL_COMP_H_INCLUDED
#define ACCEL_COMP_H_INCLUDED


#include "Tuareg_types.h"
#include "Tuareg_fueling_controls.h"

void update_legacy_AE(volatile fueling_control_t * pTarget);
void disable_legacy_AE(volatile fueling_control_t * pTarget);

void update_load_transient_comp(volatile fueling_control_t * pTarget);
void disable_load_transient_comp(volatile fueling_control_t * pTarget);

#endif // ACCEL_COMP_H_INCLUDED
