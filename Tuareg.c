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
    set_ignition_ch1(OFF);
    set_ignition_ch2(OFF);
}





void Tuareg_check_health()
{






}
