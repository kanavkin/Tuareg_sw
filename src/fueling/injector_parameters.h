#ifndef INJECTOR_PARAMETERS_H_INCLUDED
#define INJECTOR_PARAMETERS_H_INCLUDED


#include "Tuareg_types.h"


void update_injector_deadtime(volatile fueling_control_t * pTarget);

void update_injector_intervals_batch(volatile fueling_control_t * pTarget);
void update_injector_intervals_sequential(volatile fueling_control_t * pTarget);

void update_injection_begin_batch(volatile fueling_control_t * pTarget);
void update_injection_begin_sequential(volatile fueling_control_t * pTarget);

#endif // INJECTOR_PARAMETERS_H_INCLUDED
