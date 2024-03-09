#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg_console.h"
#include "TunerStudio.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"

#include "table.h"


#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "base_calc.h"
#include "diagnostics.h"
#include "bitfields.h"

#include "process_table.h"


Output_Channels_t Out;

volatile U8 * const pOut_data= (volatile U8 *) &Out;
const U32 cOut_size= sizeof(Out);

/**
helper functions
*/

// prepare OutputChannel "tuareg bits"
void ts_tuareg_bits()
{
    Out.tuareg_bits.run_allow= Tuareg.flags.run_allow;
    Out.tuareg_bits.crash_sensor_triggered= Tuareg.flags.crash_sensor_triggered;
    Out.tuareg_bits.run_switch_deactivated= Tuareg.flags.run_switch_deactivated;
    Out.tuareg_bits.sidestand_sensor_triggered= Tuareg.flags.sidestand_sensor_triggered;
    Out.tuareg_bits.overheat_detected= Tuareg.flags.overheat_detected;
    Out.tuareg_bits.service_mode= Tuareg.flags.service_mode;
    Out.tuareg_bits.limited_op= Tuareg.flags.limited_op;
    Out.tuareg_bits.rev_limiter= Tuareg.flags.rev_limiter;
    Out.tuareg_bits.standby= Tuareg.flags.standby;
    Out.tuareg_bits.cranking= Tuareg.flags.cranking;
    Out.tuareg_bits.fuel_pump= Tuareg.flags.fuel_pump;
    Out.tuareg_bits.mil= Tuareg.flags.mil;
    Out.tuareg_bits.syslog_update= Tuareg.flags.syslog_update;
    Out.tuareg_bits.datalog_update= Tuareg.flags.datalog_update;
    Out.tuareg_bits.highspeedlog_update= Tuareg.flags.highspeedlog_update;
    Out.tuareg_bits.fatal_error= Tuareg.errors.fatal_error;
    Out.tuareg_bits.decoder_config_error= Tuareg.errors.decoder_config_error;
    Out.tuareg_bits.ignition_config_error= Tuareg.errors.ignition_config_error;
    Out.tuareg_bits.tuareg_config_error= Tuareg.errors.tuareg_config_error;
    Out.tuareg_bits.fueling_config_error= Tuareg.errors.fueling_config_error;
    Out.tuareg_bits.sensor_calibration_error= Tuareg.errors.sensor_calibration_error;
    Out.tuareg_bits.sensor_O2_error= Tuareg.errors.sensor_O2_error;
    Out.tuareg_bits.sensor_TPS_error= Tuareg.errors.sensor_TPS_error;
    Out.tuareg_bits.sensor_IAT_error= Tuareg.errors.sensor_IAT_error;
    Out.tuareg_bits.sensor_CLT_error= Tuareg.errors.sensor_CLT_error;
    Out.tuareg_bits.sensor_VBAT_error= Tuareg.errors.sensor_VBAT_error;
    Out.tuareg_bits.sensor_KNOCK_error= Tuareg.errors.sensor_KNOCK_error;
    Out.tuareg_bits.sensor_BARO_error= Tuareg.errors.sensor_BARO_error;
    Out.tuareg_bits.sensor_GEAR_error= Tuareg.errors.sensor_GEAR_error;
    Out.tuareg_bits.sensor_MAP_error= Tuareg.errors.sensor_MAP_error;
    Out.tuareg_bits.sensor_CIS_error= Tuareg.errors.sensor_CIS_error;
}

// prepare OutputChannel "control bits"
void ts_control_bits()
{
    //control strategy
    Out.control_bits.ctrl_SPD= Tuareg.Controls.Flags.SPD_ctrl;
    Out.control_bits.ctrl_trans= Tuareg.Controls.Flags.smooth_transition;
    Out.control_bits.ctrl_AFR_fb= Tuareg.Controls.Flags.AFR_fallback;
    Out.control_bits.ctrl_val= Tuareg.Controls.Flags.valid;

    //ignition
    Out.control_bits.ign_val= Tuareg.Controls.Ignition.flags.valid;
    Out.control_bits.ign_dyn= Tuareg.Controls.Ignition.flags.dynamic_controls;
    Out.control_bits.ign_seq= Tuareg.Controls.Ignition.flags.sequential_mode;
    Out.control_bits.ign_cld_idl= Tuareg.Controls.Ignition.flags.cold_idle;

    //fueling
    Out.control_bits.fue_val= Tuareg.Controls.Fueling.flags.valid;
    Out.control_bits.fue_dry_crk= Tuareg.Controls.Fueling.flags.dry_cranking;
    Out.control_bits.fue_seq= Tuareg.Controls.Fueling.flags.sequential_mode;
    Out.control_bits.fue_dc_clip= Tuareg.Controls.Fueling.flags.injector_dc_clip;
    Out.control_bits.fue_WUE= Tuareg.Controls.Fueling.flags.WUE_active;
    Out.control_bits.fue_ASE= Tuareg.Controls.Fueling.flags.ASE_active;
    Out.control_bits.fue_BARO= Tuareg.Controls.Fueling.flags.BARO_corr_active;
    Out.control_bits.fue_AE= Tuareg.Controls.Fueling.flags.legacy_AE_active;
    Out.control_bits.fue_AE_MAP_acc= Tuareg.Controls.Fueling.flags.legacy_AE_trig_MAP_accel;
    Out.control_bits.fue_AE_MAP_dec= Tuareg.Controls.Fueling.flags.legacy_AE_trig_MAP_decel;
    Out.control_bits.fue_AE_TPS_acc= Tuareg.Controls.Fueling.flags.legacy_AE_trig_TPS_accel;
    Out.control_bits.fue_AE_TPS_dec= Tuareg.Controls.Fueling.flags.legacy_AE_trig_TPS_decel;
    Out.control_bits.fue_load_trans= Tuareg.Controls.Fueling.flags.load_transient_comp;

    //set
    Out.control_bits.ctrl_set= Tuareg.Controls.Set;
}



/*****************************************************
send Output channels to TunerStudio
*****************************************************/
void ts_sendOutputChannels(USART_TypeDef * Port)
{
    /**
    the TS connection timer ts_secl shall be zero right after connection establishment
    */
    if(Tuareg_console.ts_connected == false)
    {
        Tuareg_console.ts_secl= 0;
        Tuareg_console.ts_connected= true;
    }

    Out.secl= Tuareg_console.ts_secl;

    ts_tuareg_bits();
    Out.com_bits.all_flags= Tuareg_console.cli_permissions.all_flags;

    Out.rpm= Tuareg.Decoder.crank_rpm;
    Out.ddt_rpm= Tuareg.Decoder.crank_acceleration;

    Out.MAP_kPa= Tuareg.process.MAP_kPa;
    Out.ddt_MAP= Tuareg.process.ddt_MAP;
    Out.BARO_kPa= Tuareg.process.Baro_kPa;
    Out.TPS_deg= Tuareg.process.TPS_deg;
    Out.ddt_TPS= Tuareg.process.ddt_TPS;
    Out.IAT_K= Tuareg.process.IAT_K;
    Out.CLT_K= Tuareg.process.CLT_K;
    Out.BAT_V= Tuareg.process.VBAT_V;
    Out.AFR= Tuareg.process.O2_AFR;
    Out.Knock= Tuareg.process.Knock_level;
    Out.IVT_K= Tuareg.process.IVT_K;
    Out.Gear= Tuareg.process.Gear;

    Out.speed_kmh= Tuareg.process.speed_kmh;
    Out.engine_runtime_ms= Tuareg.process.engine_runtime;

    ts_control_bits();

    //control set data
    Out.IgnAdv_deg= Tuareg.Controls.IgnAdv_deg;
    Out.VE_pct= Tuareg.Controls.VE_pct;
    Out.AFRtgt= Tuareg.Controls.AFRtgt;

    //ignition
    Out.dwell_us= Tuareg.Controls.Ignition.dwell_us;

    //fueling
    Out.charge_temp_K= Tuareg.Controls.Fueling.charge_temp_K;
    Out.air_rate_gps= Tuareg.Controls.Fueling.air_flowrate_gps;

    Out.base_fuel_mass_ug= Tuareg.Controls.Fueling.base_fuel_mass_ug;
    Out.target_fuel_mass_ug= Tuareg.Controls.Fueling.target_fuel_mass_ug;
    Out.cmd_fuel_mass_ug= Tuareg.Controls.Fueling.cmd_fuel_mass_ug;
    Out.wall_fuel_mass_ug= Tuareg.Controls.Fueling.wall_fuel_mass_ug;

    Out.inj1_interval_us= Tuareg.Controls.Fueling.injector1_interval_us;
    Out.inj2_interval_us= Tuareg.Controls.Fueling.injector2_interval_us;
    Out.inj_delay_us= Tuareg.Controls.Fueling.injector_deadtime_us;
    Out.inj_dc_pct= Tuareg.Controls.Fueling.injector_target_dc;

    //eficiency
    Out.fuel_rate_gps= Tuareg.process.fuel_rate_gps;
    Out.fuel_eff_mpg= Tuareg.process.fuel_eff_mpg;


    /**
    send output channels
    */
    UART_send_data(Port, pOut_data, cOut_size);
}







