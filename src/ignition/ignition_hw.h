#ifndef IGNITIONHW_H_INCLUDED
#define IGNITIONHW_H_INCLUDED

#include <Tuareg_platform.h>


void init_ignition_hw();

void set_ignition_ch1(actor_control_t level);
void set_ignition_ch2(actor_control_t level);


//helper functions
void set_coil1_powered();
void set_coil1_unpowered();
void set_coil2_powered();
void set_coil2_unpowered();

void trigger_ignition_irq();



#endif // IGNITIONHW_H_INCLUDED
