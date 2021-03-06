#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "decoder_hw.h"
#include "decoder_logic.h"
#include "Tuareg_ignition.h"
#include "Tuareg_ignition_controls.h"
#include "ignition_hw.h"
#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "systick_timer.h"
#include "TunerStudio.h"
#include "ignition_config.h"
#include "table.h"
#include "eeprom.h"
#include "Tuareg_sensors.h"
#include "sensors.h"
#include "fueling_hw.h"
#include "fueling_logic.h"




//#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"



/****************************************************************************************************************************************
*   Process data controls update
****************************************************************************************************************************************/


void Tuareg_update_process_data()
{
    //collect diagnostic information
    //tuareg_diag_log_event(TDIAG_PROCESSDATA_CALLS);


    //process analog sensor data
    Tuareg.process.MAP_kPa= Tuareg_update_MAP_sensor();
    Tuareg.process.Baro_kPa= Tuareg_update_BARO_sensor();
    Tuareg.process.TPS_deg= Tuareg_update_TPS_sensor();
    Tuareg.process.IAT_K= Tuareg_update_IAT_sensor();
    Tuareg.process.CLT_K= Tuareg_update_CLT_sensor();
    Tuareg.process.VBAT_V= Tuareg_update_VBAT_sensor();
    Tuareg.process.ddt_TPS= Tuareg_update_ddt_TPS();
    Tuareg.process.O2_AFR= Tuareg_update_O2_sensor();
    Tuareg.process.Gear= Tuareg_update_GEAR_sensor();


    #ifdef TUAREG_LOAD_CODE
    /*
    At least one method to determine engine load is required to operate the engine

    Hints:
    - while booting all errors will be present
    - non-running modes are not affected
    - while cranking a static ignition profile and fueling is used

    if((Tuareg.flags.cranking == false) && (Tuareg.errors.sensor_MAP_error == true) && (Tuareg.errors.sensor_TPS_error == true))
    {
        //LIMP
        Tuareg.flags.limited_op= true;
    }

    //load figures
    Tuareg_update_load(&(Tuareg.process));
    */
    #else
    Tuareg.process.load_pct=0;
    #endif


    //ground speed
    if((Tuareg.pDecoder->outputs.rpm_valid) && (Tuareg.process.Gear < GEAR_NEUTRAL))
    {
        Tuareg.process.ground_speed_mmps= Tuareg.pDecoder->crank_rpm * Tuareg_Setup.gear_ratio[Tuareg.process.Gear];
    }
    else
    {
        Tuareg.process.ground_speed_mmps= 0;
    }



}


/****************************************************************************************************************************************
*   Process data controls update - load
****************************************************************************************************************************************/

#ifdef TUAREG_LOAD_CODE
/**
calculates the current engine load from MAP, BARO and TPS sensor inputs
*/
void Tuareg_update_load(volatile process_data_t * pProcess)
{
    //collect diagnostic information
    //tuareg_diag_log_event(TDIAG_PROCESSDATA_CALLS);

    VF32 load_vacuum_pct =0, load_throttle_pct =0;

    U32 load_from_map_min_rpm= 2000;
    U32 load_from_map_max_rpm= 6000;


    //check preconditions
    if((Tuareg.errors.sensor_MAP_error == true) && (Tuareg.errors.sensor_TPS_error == true))
    {
        pProcess->load_pct= TUAREG_DEFAULT_LOAD_PCT;
        return;
    }

    //calculate vacuum load
    if(Tuareg.errors.sensor_MAP_error == false)
    {
        /*
        calculation can be valid even with the BARO default value
        load by vacuum :=  MAP / BARO
        */
        load_vacuum_pct= divide_float(pProcess->MAP_kPa, pProcess->Baro_kPa);
    }

    //calculate throttle load
    if(Tuareg.errors.sensor_TPS_error == false)
    {
        /*
        load by throttle is scaled to throttle geometry :=  TPS / TPS(WOT - IDLE)
        */
        load_throttle_pct= divide_float(pProcess->TPS_deg, 90.0);
    }

    /*
    check witch value shall be the load source:

    use vacuum based load figure if

    the calculated load value is valid
    AND
    vacuum based load has been requester

    OR

    if throttle based load calculation is not possible
    */
    if(
       ((Tuareg.errors.sensor_MAP_error == false) && (Tuareg.errors.sensor_BARO_error == false) &&
       (Tuareg.pDecoder->outputs.rpm_valid == true) && (Tuareg.pDecoder->crank_rpm >= load_from_map_min_rpm) && (Tuareg.pDecoder->crank_rpm <= load_from_map_max_rpm))
       ||
       (Tuareg.errors.sensor_TPS_error == true)
    )
    {
        //use vacuum based load
        pProcess->load_pct= load_vacuum_pct;
    }
    else
    {
        //use throttle based load
        pProcess->load_pct= load_throttle_pct;
    }
}

#endif // TUAREG_LOAD_CODE






