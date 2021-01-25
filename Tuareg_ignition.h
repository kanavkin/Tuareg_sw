#ifndef TUAREG_IGNITION_H_INCLUDED
#define TUAREG_IGNITION_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_ignition_controls.h"


typedef enum {

    COILS_SHARED,
    COILS_SEPARATE,

    COILS_COUNT

} coil_setup_t;


void Tuareg_ignition_update_crankpos_handler();
void Tuareg_ignition_irq_handler();



#endif // TUAREG_IGNITION_H_INCLUDED
