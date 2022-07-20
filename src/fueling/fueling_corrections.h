#ifndef FUELING_CORRECTIONS_H_INCLUDED
#define FUELING_CORRECTIONS_H_INCLUDED


#include "Tuareg_types.h"
#include "Tuareg_fueling_controls.h"

void update_fuel_mass_corrections(volatile fueling_control_t * pTarget);

void update_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget);
void disable_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget);

void update_fuel_mass_warmup_correction(volatile fueling_control_t * pTarget);
void update_fuel_mass_barometric_correction(volatile fueling_control_t * pTarget);

#endif // FUELING_CORRECTIONS_H_INCLUDED
