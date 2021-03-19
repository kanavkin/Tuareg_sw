#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "bitfields.h"

#include "Tuareg.h"
#include "Tuareg_sensors.h"

#include "sensor_calibration.h"

#include "syslog.h"
#include "sensors_syslog_locations.h"

#include "debug_port_messages.h"

#define SENSORS_DEBUG_OUTPUT

#ifdef SENSORS_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // SENSORS_DEBUG_OUTPUT


/******************************************************************************************************************************
Sensors initialization
 ******************************************************************************************************************************/
volatile sensor_interface_t * init_Sensors()
{
    exec_result_t result;
    volatile sensor_interface_t * pInterface;

    //start with all errors set
    Tuareg.Errors.sensor_calibration_error= true;

    Tuareg.Errors.sensor_O2_error= true;
    Tuareg.Errors.sensor_TPS_error= true;
    Tuareg.Errors.sensor_IAT_error= true;
    Tuareg.Errors.sensor_CLT_error= true;
    Tuareg.Errors.sensor_VBAT_error= true;
    Tuareg.Errors.sensor_KNOCK_error= true;
    Tuareg.Errors.sensor_BARO_error= true;
    Tuareg.Errors.sensor_GEAR_error= true;
    Tuareg.Errors.sensor_MAP_error= true;
    Tuareg.Errors.sensor_CIS_error= true;

    //load calibration data
    result= load_Sensor_Calibration();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load sensor calibration
        Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_CONFIG_LOAD_FAIL);

        #ifdef SENSORS_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Sensor Calibration!");
        #endif // SENSORS_DEBUG_OUTPUT
    }
    else if(Sensor_Calibration.Version != SENSORS_REQUIRED_CALIBRATION_VERSION)
    {
        //loaded wrong sensor calibration version
        Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_CONFIG_VERSION_MISMATCH);

        #ifdef SENSORS_DEBUG_OUTPUT
        DebugMsg_Error("Sensor Calibration version does not match");
        #endif // SENSORS_DEBUG_OUTPUT
    }
    else
    {
        //loaded sensor calibration with correct Version
        Tuareg.Errors.sensor_calibration_error= false;

        Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_CONFIG_LOAD_SUCCESS);
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
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_MAP] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_MAP_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_MAP_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_MAP_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_MAP];
    }
    else
    {
        if(Tuareg.Errors.sensor_MAP_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_MAP_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_MAP_FAILED);
        }

        //use default value
        return MAP_DEFAULT_KPA;
    }
}


/**
checks the health state of the O2 sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_O2_sensor()
{
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_O2] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_O2_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_O2_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_O2_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_O2];
    }
    else
    {
        if(Tuareg.Errors.sensor_O2_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_O2_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_O2_FAILED);
        }

        //use default value
        return O2_DEFAULT_AFR;
    }
}


/**
checks the health state of the TPS sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_TPS_sensor()
{
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_TPS] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_TPS_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_TPS_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_TPS_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_TPS];
    }
    else
    {
        if(Tuareg.Errors.sensor_TPS_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_TPS_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_TPS_FAILED);
        }

        //use default value
        return TPS_DEFAULT_DEG;
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
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_IAT] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_IAT_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_IAT_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_IAT_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_IAT];
    }
    else
    {
        if(Tuareg.Errors.sensor_IAT_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_IAT_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_IAT_FAILED);
        }

        //use default value
        return IAT_DEFAULT_C + cKelvin_offset;
    }

}


/**
checks the health state of the CLT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_CLT_sensor()
{
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_CLT] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_CLT_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_CLT_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_CLT_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_CLT];
    }
    else
    {
        if(Tuareg.Errors.sensor_CLT_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_CLT_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_CLT_FAILED);
        }

        //use default value
        return CLT_DEFAULT_C+ cKelvin_offset;
    }

}


/**
checks the health state of the VBAT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_VBAT_sensor()
{
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_VBAT] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_VBAT_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_VBAT_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_VBAT_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_VBAT];
    }
    else
    {
        if(Tuareg.Errors.sensor_VBAT_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_VBAT_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_VBAT_FAILED);
        }

        //use default value
        return VBAT_DEFAULT_V;
    }

}


/**
checks the health state of the KNOCK sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_KNOCK_sensor()
{
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_KNOCK] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_KNOCK_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_KNOCK_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_KNOCK_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_KNOCK];
    }
    else
    {
        if(Tuareg.Errors.sensor_KNOCK_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_KNOCK_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_KNOCK_FAILED);
        }

        //use default value
        return KNOCK_DEFAULT;
    }
}


/**
checks the health state of the BARO sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_BARO_sensor()
{
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_BARO] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_BARO_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_BARO_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_BARO_VALIDATED);

        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_BARO];
    }
    else
    {
        if(Tuareg.Errors.sensor_BARO_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_BARO_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_BARO_FAILED);
        }

        //use default value
        return BARO_DEFAULT_KPA;
    }
}



/**
checks the health state of the GEAR sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
gears_t Tuareg_update_GEAR_sensor()
{
    //check if sensor can be validated in this cycle
    if(Tuareg.pSensors->asensors_valid_samples[ASENSOR_GEAR] > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.Errors.sensor_GEAR_error == true)
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_GEAR_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_GEAR_VALIDATED);
        }

        //use live value
        return Tuareg.pSensors->asensors[ASENSOR_GEAR];
    }
    else
    {
        if(Tuareg.Errors.sensor_GEAR_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_GEAR_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_GEAR_FAILED);
        }
            //use default value
            return GEAR_NEUTRAL;
    }
}

