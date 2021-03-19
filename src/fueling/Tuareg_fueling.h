#ifndef TUAREG_FUELING_H_INCLUDED
#define TUAREG_FUELING_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "fueling_hw.h"
#include "Tuareg_fueling_controls.h"

#define FUELLING_REQUIRED_CONFIG_VERSION 1


void init_Fueling();

void Tuareg_fueling_update_crankpos_handler();
void Tuareg_fueling_irq_handler();

#endif // TUAREG_FUELING_H_INCLUDED
