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


inline void Tuareg_update_process_data()
{




    //collect diagnostic information
    //tuareg_diag_log_event(TDIAG_PROCESSDATA_CALLS);


    //analog sensors
    Tuareg.process.MAP_kPa= Tuareg_update_MAP_sensor();
    Tuareg.process.Baro_kPa= Tuareg_update_BARO_sensor();
    Tuareg.process.TPS_deg= Tuareg_update_TPS_sensor();
    Tuareg.process.IAT_K= Tuareg_update_IAT_sensor();
    Tuareg.process.CLT_K= Tuareg_update_CLT_sensor();
    Tuareg.process.VBAT_V= Tuareg_update_VBAT_sensor();
    Tuareg.process.ddt_TPS= Tuareg_update_ddt_TPS();
    Tuareg.process.O2_AFR= Tuareg_update_O2_sensor();
    Tuareg.process.Gear= Tuareg_update_GEAR_sensor();

    if(Tuareg.pDecoder->outputs.rpm_valid)
    {
        Tuareg.process.ground_speed_kmh= Tuareg.pDecoder->crank_rpm * Tuareg_Setup.gear_ratio[Tuareg.process.Gear];
    }
    else
    {
        Tuareg.process.ground_speed_kmh= 0;
    }



}







