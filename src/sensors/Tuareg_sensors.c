#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg.h"
#include "Tuareg_sensors.h"

#include "sensor_calibration.h"

#include "uart.h"
#include "uart_printf.h"




/******************************************************************************************************************************
Sensors initialization
 ******************************************************************************************************************************/
volatile sensor_interface_t * init_Sensors()
{
    exec_result_t result;
    volatile sensor_interface_t * pInterface;

    //start with error state
    Tuareg.Errors.sensor_calibration_error= true;

    //load calibration data
    result= load_Sensor_Calibration();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load sensor calibration
        print(DEBUG_PORT, "\r\nEE Failed to load Sensor Calibration!");
    }
    else if(Sensor_Calibration.Version != SENSORS_REQUIRED_CALIBRATION_VERSION)
    {
        //loaded wrong sensor calibration version
        print(DEBUG_PORT, "\r\nEE Sensor Calibration version does not match");
    }
    else
    {
        //loaded sensor calibration with correct Version
        Tuareg.Errors.sensor_calibration_error= false;

        print(DEBUG_PORT, "\r\nII Sensor Calibration has been loaded");
    }

    //init logic part
    pInterface= init_sensor_inputs(ASENSOR_VALIDITY_FASTINIT);

    return pInterface;
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
    if( (Tuareg.Errors.sensor_MAP_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_MAP)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_MAP];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_MAP)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_MAP] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_MAP_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_MAP];
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
    if( (Tuareg.Errors.sensor_O2_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_O2)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_O2];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_O2)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_O2] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_O2_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_O2];
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
    if( (Tuareg.Errors.sensor_TPS_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_TPS)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_TPS];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_TPS)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_TPS] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_TPS_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_TPS];
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
        return Tuareg.pSensors->ddt_TPS;
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
    if( (Tuareg.Errors.sensor_IAT_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_IAT)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_IAT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_IAT)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_IAT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_IAT_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_IAT];
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
    if( (Tuareg.Errors.sensor_CLT_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_CLT)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_CLT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_CLT)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_CLT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_CLT_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_CLT];
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
    if( (Tuareg.Errors.sensor_VBAT_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_VBAT)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_VBAT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_VBAT)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_VBAT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_VBAT_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_VBAT];
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
    if( (Tuareg.Errors.sensor_KNOCK_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_KNOCK)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_KNOCK];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_KNOCK)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_KNOCK] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_KNOCK_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_KNOCK];
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
    if( (Tuareg.Errors.sensor_BARO_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_BARO)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_BARO];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_BARO)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_BARO] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_BARO_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_BARO];
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
gears_t Tuareg_update_GEAR_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_GEAR_error == false) && (Tuareg.pSensors->asensors_health & (1<< ASENSOR_GEAR)) && (Tuareg.Errors.sensor_calibration_error == false))
    {
        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_GEAR];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.pSensors->asensors_health & (1<< ASENSOR_GEAR)) && (Tuareg.pSensors->asensors_valid_samples[ASENSOR_GEAR] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_GEAR_error= false;

            //use live value
            return Tuareg.pSensors->asensors[ASENSOR_GEAR];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_GEAR_error= true;

            //use default value
            return GEAR_NEUTRAL;
        }

    }
}

