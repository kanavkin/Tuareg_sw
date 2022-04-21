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

    ts_tuareg_bits(&(Out.tuareg_bits));
    Out.com_bits= Tuareg_console.cli_permissions.all_flags;

    Out.rpm= (Tuareg.pDecoder->flags.rpm_valid == true)? Tuareg.pDecoder->crank_rpm : 0;
    Out.ddt_rpm= (Tuareg.pDecoder->flags.accel_valid == true)? 1 : 0;

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

    Out.ground_speed_mmps= Tuareg.process.ground_speed_mmps;
    Out.engine_runtime_ms= Tuareg.engine_runtime;

    Out.ignition_bits= Tuareg.ignition_controls.flags.all_flags;
    Out.ignition_adv_deg= Tuareg.ignition_controls.ignition_advance_deg;
    Out.ignition_dwell_us= Tuareg.ignition_controls.dwell_us;

    Out.fueling_bits= Tuareg.fueling_controls.flags.all_flags;
    Out.VE_pct= Tuareg.fueling_controls.VE_pct;
    Out.AFR_target= Tuareg.fueling_controls.AFR_target;
    Out.air_rate_gps= Tuareg.fueling_controls.air_flowrate_gps;

    Out.base_fuel_mass_ug= Tuareg.fueling_controls.base_fuel_mass_ug;
    Out.target_fuel_mass_ug= Tuareg.fueling_controls.target_fuel_mass_ug;
    Out.cmd_fuel_mass_ug= Tuareg.fueling_controls.cmd_fuel_mass_ug;
    Out.wall_fuel_mass_ug= Tuareg.fueling_controls.wall_fuel_mass_ug;

    Out.inj1_interval_us= Tuareg.fueling_controls.injector1_interval_us;
    Out.inj2_interval_us= Tuareg.fueling_controls.injector2_interval_us;
    Out.inj_dc_pct= Tuareg.fueling_controls.injector_target_dc;
    Out.inj_delay_us= Tuareg.fueling_controls.injector_deadtime_us;

    Out.conspumtion_ugps= Tuareg.fuel_consumpt_1s_ug;


    /**
    send output channels
    */
    UART_send_data(Port, pOut_data, cOut_size);
}



/**
prepare OutputChannel "tuareg" field
*/
void ts_tuareg_bits(ts_tuareg_bits_t * pTarget)
{
    pTarget->run_inhibit= Tuareg.flags.run_inhibit;
    pTarget->crash_sensor_triggered= Tuareg.flags.crash_sensor_triggered;
    pTarget->run_switch_deactivated= Tuareg.flags.run_switch_deactivated;
    pTarget->sidestand_sensor_triggered= Tuareg.flags.sidestand_sensor_triggered;
    pTarget->overheat_detected= Tuareg.flags.overheat_detected;
    pTarget->service_mode= Tuareg.flags.service_mode;
    pTarget->limited_op= Tuareg.flags.limited_op;
    pTarget->rev_limiter= Tuareg.flags.rev_limiter;
    pTarget->standby= Tuareg.flags.standby;
    pTarget->cranking= Tuareg.flags.cranking;
    pTarget->fuel_pump= Tuareg.flags.fuel_pump;
    pTarget->mil= Tuareg.flags.mil;
    pTarget->syslog_update= Tuareg.flags.syslog_update;
    pTarget->datalog_update= Tuareg.flags.datalog_update;
    pTarget->highspeedlog_update= Tuareg.flags.highspeedlog_update;
    pTarget->fatal_error= Tuareg.errors.fatal_error;
    pTarget->decoder_config_error= Tuareg.errors.decoder_config_error;
    pTarget->ignition_config_error= Tuareg.errors.ignition_config_error;
    pTarget->tuareg_config_error= Tuareg.errors.tuareg_config_error;
    pTarget->fueling_config_error= Tuareg.errors.fueling_config_error;
    pTarget->sensor_calibration_error= Tuareg.errors.sensor_calibration_error;
    pTarget->sensor_O2_error= Tuareg.errors.sensor_O2_error;
    pTarget->sensor_TPS_error= Tuareg.errors.sensor_TPS_error;
    pTarget->sensor_IAT_error= Tuareg.errors.sensor_IAT_error;
    pTarget->sensor_CLT_error= Tuareg.errors.sensor_CLT_error;
    pTarget->sensor_VBAT_error= Tuareg.errors.sensor_VBAT_error;
    pTarget->sensor_KNOCK_error= Tuareg.errors.sensor_KNOCK_error;
    pTarget->sensor_BARO_error= Tuareg.errors.sensor_BARO_error;
    pTarget->sensor_GEAR_error= Tuareg.errors.sensor_GEAR_error;
    pTarget->sensor_MAP_error= Tuareg.errors.sensor_MAP_error;
    pTarget->sensor_CIS_error= Tuareg.errors.sensor_CIS_error;
}



