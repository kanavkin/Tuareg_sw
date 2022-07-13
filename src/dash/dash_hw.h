#ifndef DASHHW_H_INCLUDED
#define DASHHW_H_INCLUDED

#include "Tuareg_platform.h"
#define TACH_PWM_RESOLUTION 5000


void init_dash_hw();

void set_mil_hw(actor_control_t level);

void set_tachometer_compare(U32 Compare);

#endif // DASHHW_H_INCLUDED
