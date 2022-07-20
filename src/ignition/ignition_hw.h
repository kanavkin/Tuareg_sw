#ifndef IGNITIONHW_H_INCLUDED
#define IGNITIONHW_H_INCLUDED

#include <Tuareg_platform.h>


void init_ignition_hw();

void set_ignition_ch1(actor_control_t level);
void set_ignition_ch2(actor_control_t level);

#endif // IGNITIONHW_H_INCLUDED
