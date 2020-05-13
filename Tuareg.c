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
    //ignition control
    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_dwell_pos)
    {
        trigger_coil_by_timer(Tuareg.ignition_timing.coil_dwell_timing_us, COIL_DWELL);
    }
    else if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_ignition_pos)
    {
        trigger_coil_by_timer(Tuareg.ignition_timing.coil_ignition_timing_us, COIL_IGNITION);
    }
}



void Tuareg_check_health()
{






}


U32 Tuareg_get_MAP()
{
    //if sensor is available
    if(Tuareg.sensors->asensors_sync_health & (1<< ASENSOR_SYNC_MAP))
    {
        return Tuareg.sensors->asensors_sync[ASENSOR_SYNC_MAP];
    }
    else
    {
        //sensor possibly disturbed
        return MAP_DEFAULT_KPA;
    }
}
