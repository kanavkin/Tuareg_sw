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

        U16 sequential_mode :1;


        U16 VE_valid :1;
        U16 SPD_active :1;

        U16 AFR_target_valid :1;

        U16 injector_dc_clip :1;

        U16 injection_begin_valid :1;

        U16 accel_comp_active :1;
        U16 warmup_comp_active :1;
        U16 afterstart_comp_active :1;

        U16 dry_cranking :1;

     };

} fueling_logic_flags_t;



/**
ignition_control_t defines a transfer object
*/
typedef struct _fueling_control_t {

    //VE in %
    VF32 VE_pct;

    //air density is in micro gram per cubic centimeter
    VF32 air_density;

    F32 AFR_target;

    U32 injector_deadtime_us;
    U32 injector_target_dc;
    U32 injector1_interval_us;
    U32 injector2_interval_us;

    //fuel mass to be injected into each cylinder
    F32 base_fuel_mass_ug;
    U32 target_fuel_mass_ug;

    //accel pump
    F32 fuel_mass_accel_corr_pct;
    U32 fuel_mass_accel_corr_cycles_left;

    //warmup compensation
    F32 fuel_mass_warmup_corr_pct;

    //afterstart compensation
    F32 fuel_mass_afterstart_corr_pct;
    U32 fuel_mass_afterstart_corr_cycles_left;

    //injection phase data
    crank_position_t injection_begin_pos;
    engine_phase_t seq_injector1_begin_phase;
    engine_phase_t seq_injector2_begin_phase;
    U32 injector1_timing_us;
    U32 injector2_timing_us;


    //status data
    fueling_logic_flags_t flags;

} fueling_control_t;


void Tuareg_notify_fueling_cranking_end();

void Tuareg_update_fueling_controls();

void invalid_fueling_controls(volatile fueling_control_t * pTarget);

void update_mode(volatile fueling_control_t * pTarget);
void update_strategy(volatile fueling_control_t * pTarget);

void update_volumetric_efficiency(volatile fueling_control_t * pTarget);
void update_air_density(volatile fueling_control_t * pTarget);
void update_AFR_target(volatile fueling_control_t * pTarget);

void update_base_fuel_mass(volatile fueling_control_t * pTarget);
void update_base_fuel_mass_cranking(volatile fueling_control_t * pTarget);

void update_fuel_mass_accel_correction(volatile fueling_control_t * pTarget);
void disable_fuel_mass_accel_correction(volatile fueling_control_t * pTarget);

void update_fuel_mass_warmup_correction(volatile fueling_control_t * pTarget);

void update_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget);
void disable_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget);

void update_target_fuel_mass(volatile fueling_control_t * pTarget);

void update_injector_deadtime(volatile fueling_control_t * pTarget);

void update_injector_intervals_sequential(volatile fueling_control_t * pTarget);
void update_injector_intervals_batch(volatile fueling_control_t * pTarget);

void update_injection_begin_batch(volatile fueling_control_t * pTarget);
void update_injection_begin_sequential(volatile fueling_control_t * pTarget);
void earliest_sequential_injection_begin(volatile fueling_control_t * pTarget);





#endif // TUAREG_FUELING_CONTROLS_H_INCLUDED
