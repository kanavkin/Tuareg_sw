#ifndef TUAREG_FUELING_CONTROLS_H_INCLUDED
#define TUAREG_FUELING_CONTROLS_H_INCLUDED


#include "Tuareg_types.h"


/**
fueling_control_flags_t
*/
typedef union
{
     U16 all_flags;

     struct
     {
        U16 valid :1;

        U16 MAP_nTPS :1;
        U16 AFR_fallback :1;
        U16 injection_begin_valid :1;

        U16 dry_cranking :1;
        U16 sequential_mode :1;
        U16 injector_dc_clip :1;

        U16 WUE_active :1;
        U16 ASE_active :1;
        U16 BARO_corr_active :1;
        U16 legacy_AE_active :1;
        U16 legacy_AE_trig_MAP_accel :1;
        U16 legacy_AE_trig_MAP_decel :1;
        U16 legacy_AE_trig_TPS_accel :1;
        U16 legacy_AE_trig_TPS_decel :1;
        U16 load_transient_comp :1;
     };

} fueling_control_flags_t;



/**
fueling_control_t defines a transfer object
*/
typedef struct _fueling_control_t {

    //basic parameters
    F32 VE_pct;
    F32 charge_temp_K;
    F32 air_density;
    F32 air_flowrate_gps;
    F32 AFR_target;
    F32 base_fuel_mass_ug;

    //warmup correction
    F32 WUE_pct;

    //afterstart correction
    F32 ASE_pct;
    U32 ASE_cycles_left;

    //BARO correction
    F32 BARO_pct;

    //legacy load transient compensation
    F32 legacy_AE_ug;
    U32 legacy_AE_cycles_left;

    //corrected fuel mass
    F32 target_fuel_mass_ug;

    // X-Tau load compensation
    F32 wall_fuel_mass_ug;
    F32 cmd_fuel_mass_ug;

    //injector parameters
    F32 injector_deadtime_us;
    F32 injector_target_dc;

    U32 injector1_interval_us;
    U32 injector2_interval_us;
    U32 injector1_timing_us;
    U32 injector2_timing_us;
    engine_phase_t seq_injector2_begin_phase;
    engine_phase_t seq_injector1_begin_phase;
    crank_position_t injection_begin_pos;

    //status data
    fueling_control_flags_t flags;

} fueling_control_t;


void Tuareg_update_fueling_controls();
void invalid_fueling_controls(volatile fueling_control_t * pTarget);


void update_air_flow(volatile fueling_control_t * pTarget);
void update_AFR_target(volatile fueling_control_t * pTarget);

void update_base_fuel_mass(volatile fueling_control_t * pTarget);
void update_fuel_mass_cranking(volatile fueling_control_t * pTarget);

void update_target_fuel_mass(volatile fueling_control_t * pTarget);

#endif // TUAREG_FUELING_CONTROLS_H_INCLUDED
