#ifndef TUAREG_IGNITION_H_INCLUDED
#define TUAREG_IGNITION_H_INCLUDED

#include <Tuareg_platform.h>

#include "ignition_config.h"
#include "ignition_diag.h"
#include "ignition_hw.h"
#include "Tuareg_ignition_controls.h"
#include "Ignition_syslog_locations.h"

void init_Ignition();

void Tuareg_ignition_update_crankpos_handler();
void Tuareg_ignition_irq_handler();

#endif // TUAREG_IGNITION_H_INCLUDED
