#ifndef FUELINGHW_H_INCLUDED
#define FUELINGHW_H_INCLUDED

#include "Tuareg_types.h"

void init_fueling_hw();

void set_injector1(actor_control_t level);
void set_injector2(actor_control_t level);
void set_fuel_pump(actor_control_t level);

#endif // FUELINGHW_H_INCLUDED
