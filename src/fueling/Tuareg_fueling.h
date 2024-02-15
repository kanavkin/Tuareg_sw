#ifndef TUAREG_FUELING_H_INCLUDED
#define TUAREG_FUELING_H_INCLUDED

#include "Tuareg_types.h"

#include "fueling_hw.h"
#include "fueling_config.h"
#include "Tuareg_fueling_controls.h"
#include "Fueling_syslog_locations.h"

#define FUELING_REQUIRED_CONFIG_VERSION 9


void init_Fueling();

void Tuareg_fueling_update_crankpos_handler();

#endif // TUAREG_FUELING_H_INCLUDED
