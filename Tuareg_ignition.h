#ifndef TUAREG_IGNITION_H_INCLUDED
#define TUAREG_IGNITION_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "ignition_logic.h"


typedef enum {

    COILS_SHARED,
    COILS_SEPARATE,

    COILS_COUNT

} coil_setup_t;



void Tuareg_update_ignition_controls();
void Tuareg_trigger_ignition_actors(volatile crank_position_t CrankPosition, volatile engine_phase_t Phase, volatile ignition_control_t * pIgnitionControls);


#endif // TUAREG_IGNITION_H_INCLUDED
