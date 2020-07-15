#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "ignition_logic.h"
#include "ignition_hw.h"
#include "scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "lowspeed_timers.h"
#include "TunerStudio.h"
#include "config.h"
#include "table.h"
#include "eeprom.h"
#include "sensors.h"
#include "fuel_hw.h"
#include "fuel_logic.h"

#include "dash_hw.h"
#include "dash_logic.h"

#include "debug.h"
#include "Tuareg.h"




void Tuareg_stop_engine()
{
    set_fuelpump(OFF);
    set_injector_ch1(OFF);
    set_injector_ch2(OFF);
    set_ignition_ch1(COIL_POWERDOWN);
    set_ignition_ch2(COIL_POWERDOWN);
}




void Tuareg_trigger_ignition()
{
    //collect diagnostic information
    Tuareg.diag[TDIAG_TRIG_IGN_CALLS] += 1;

    /**
    its a normal situation that ignition and dwell are triggered from the same position
    */
    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_ignition_pos)
    {
        //collect diagnostic information
        Tuareg.diag[TDIAG_TRIG_COIL_IGN] += 1;

        trigger_coil_by_timer(Tuareg.ignition_timing.coil_ignition_timing_us, COIL_IGNITION);
    }

    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_dwell_pos)
    {
        //collect diagnostic information
        Tuareg.diag[TDIAG_TRIG_COIL_DWELL] += 1;

        trigger_coil_by_timer(Tuareg.ignition_timing.coil_dwell_timing_us, COIL_DWELL);
    }


}




U32 Tuareg_get_asensor(asensors_t sensor)
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.asensor_validity & (1<< sensor)) && (Tuareg.sensors->asensors_health & (1<< sensor)) )
    {
        //use live value
        return Tuareg.sensors->asensors[sensor];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< sensor)) && (Tuareg.sensors->asensors_valid[sensor] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.asensor_validity |= (1<< sensor);

            //use live value
            return Tuareg.sensors->asensors[sensor];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.asensor_validity &= ~(1<< sensor);

            //use default value
            return Tuareg.asensor_defaults[sensor];
        }



    }
}


/**
writes the collected diagnostic data to the memory a pTarget
*/
void Tuareg_export_diag(VU32 * pTarget)
{
    U32 count;

    for(count=0; count < TDIAG_COUNT; count++)
    {
        pTarget[count]= Tuareg.diag[count];
    }
}
