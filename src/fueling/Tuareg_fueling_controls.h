#ifndef TUAREG_FUELING_CONTROLS_H_INCLUDED
#define TUAREG_FUELING_CONTROLS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"





/**
ignition_logic_state_t
*/
typedef union
{
     U16 all_flags;

     struct
     {
        U16 valid :1;

        U16 VEfromMAP :1;
        U16 sequential_mode :1;

        U16 injector_overload :1;


     };

} fueling_logic_flags_t;



/**
ignition_control_t defines a transfer object
*/
typedef struct _fueling_control_t {

    //VE is a factor from 0..1
    VF32 volumetric_efficency_pct;

    //air density is in micro gram per cubic centimeter
    VF32 air_density;

    //fuel mass to be injected into each cylinder
    U32 fuel_mass_ug;

    U32 injector1_interval_us;
    U32 injector2_interval_us;

    VF32 AFR_target;


    //status data
    fueling_logic_flags_t flags;

} fueling_control_t;




void Tuareg_update_fueling_controls();

void update_volumetric_efficiency(volatile fueling_control_t * pTarget);
void update_air_density(volatile fueling_control_t * pTarget);
void update_AFR_target(volatile fueling_control_t * pTarget);
void update_fuel_mass(volatile fueling_control_t * pTarget);
void update_injector_intervals(volatile fueling_control_t * pTarget);
void update_injector_timings(volatile fueling_control_t * pTarget);





#endif // TUAREG_FUELING_CONTROLS_H_INCLUDED
