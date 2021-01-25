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
#include "lowspeed_timers.h"
#include "TunerStudio.h"
#include "ignition_config.h"
#include "table.h"
#include "eeprom.h"
#include "sensors.h"
#include "fuel_hw.h"
#include "fuel_logic.h"




#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"



/****************************************************************************************************************************************
*   Process data controls update
****************************************************************************************************************************************/


inline void Tuareg_update_process_data()
{

    //this information is included for diagnostic purposes only
    Tuareg.process.crank_rpm= (Tuareg.decoder->state.rpm_valid == true)? (Tuareg.decoder->crank_rpm): 0;


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


}




/****************************************************************************************************************************************
*   evaluate the analog sensors
****************************************************************************************************************************************/



/**
checks the health state of the MAP sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_MAP_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_MAP_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_MAP)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_MAP];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_MAP)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_MAP] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_MAP_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_MAP];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_MAP_error= true;

            //use default value
            return MAP_DEFAULT_KPA;
        }

    }
}


/**
checks the health state of the O2 sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_O2_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_O2_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_O2)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_O2];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_O2)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_O2] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_O2_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_O2];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_O2_error= true;

            //use default value
            return O2_DEFAULT_AFR;
        }

    }
}


/**
checks the health state of the TPS sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_TPS_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_TPS_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_TPS)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_TPS];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_TPS)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_TPS] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_TPS_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_TPS];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_TPS_error= true;

            //use default value
            return TPS_DEFAULT_DEG;
        }

    }
}


VF32 Tuareg_update_ddt_TPS()
{
    /// must be executed after TPS sensor update!
    if(Tuareg.Errors.sensor_TPS_error == false)
    {
        //use live value
        return Tuareg.sensors->ddt_TPS;
    }
    else
    {
        return 0;
    }

}


/**
checks the health state of the IAT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_IAT_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_IAT_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_IAT)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_IAT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_IAT)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_IAT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_IAT_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_IAT];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_IAT_error= true;

            //use default value
            return IAT_DEFAULT_C;
        }

    }
}


/**
checks the health state of the CLT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_CLT_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_CLT_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_CLT)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_CLT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_CLT)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_CLT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_CLT_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_CLT];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_CLT_error= true;

            //use default value
            return CLT_DEFAULT_C;
        }

    }
}


/**
checks the health state of the VBAT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_VBAT_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_VBAT_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_VBAT)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_VBAT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_VBAT)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_VBAT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_VBAT_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_VBAT];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_VBAT_error= true;

            //use default value
            return VBAT_DEFAULT_V;
        }

    }
}


/**
checks the health state of the KNOCK sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_KNOCK_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_KNOCK_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_KNOCK)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_KNOCK];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_KNOCK)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_KNOCK] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_KNOCK_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_KNOCK];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_KNOCK_error= true;

            //use default value
            return KNOCK_DEFAULT;
        }

    }
}


/**
checks the health state of the BARO sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_BARO_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_BARO_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_BARO)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_BARO];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_BARO)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_BARO] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_BARO_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_BARO];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_BARO_error= true;

            //use default value
            return BARO_DEFAULT_KPA;
        }

    }
}


/**
checks the health state of the GEAR sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_GEAR_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_GEAR_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_GEAR)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_GEAR];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_GEAR)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_GEAR] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_GEAR_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_GEAR];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_GEAR_error= true;

            //use default value
            return GEAR_DEFAULT;
        }

    }
}





