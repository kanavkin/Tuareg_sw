
#include "Tuareg.h"

#include "Tuareg_sensors.h"
#include "sensor_calibration.h"
#include "sensors_syslog_locations.h"

#include "syslog.h"

//#define SENSORS_DEBUGMSG

#ifdef SENSORS_DEBUGMSG
#include "debug_port_messages.h"
#warning debug outputs enabled
#endif // SENSORS_DEBUGMSG


/******************************************************************************************************************************
Sensors initialization
 ******************************************************************************************************************************/
void init_Sensors()
{
    exec_result_t result;

    //start with all errors set
    Tuareg.errors.sensor_O2_error= true;
    Tuareg.errors.sensor_TPS_error= true;
    Tuareg.errors.sensor_IAT_error= true;
    Tuareg.errors.sensor_CLT_error= true;
    Tuareg.errors.sensor_VBAT_error= true;
    Tuareg.errors.sensor_KNOCK_error= true;
    Tuareg.errors.sensor_BARO_error= true;
    Tuareg.errors.sensor_GEAR_error= true;
    Tuareg.errors.sensor_MAP_error= true;

    //load calibration data
    result= load_Sensor_Calibration();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        /**
        failed to load sensor calibration
        */
        Tuareg.errors.sensor_calibration_error= true;

        //enter limp mode
        Limp(TID_TUAREG_SENSORS, SENSORS_LOC_CONFIGLOAD_ERROR);

        #ifdef SENSORS_DEBUGMSG
        DebugMsg_Error("Failed to load Sensor Calibration!");
        #endif // SENSORS_DEBUGMSG
    }
    else if(Sensor_Calibration.Version != SENSORS_REQUIRED_CALIBRATION_VERSION)
    {
        /**
        loaded wrong sensor calibration version
        */
        Tuareg.errors.sensor_calibration_error= true;

        //enter limp mode
        Limp(TID_TUAREG_SENSORS, SENSORS_LOC_CONFIGVERSION_ERROR);

        #ifdef SENSORS_DEBUGMSG
        DebugMsg_Error("Sensor Calibration version does not match");
        #endif // SENSORS_DEBUGMSG
    }
    else
    {
        //loaded sensor calibration with correct Version
        Tuareg.errors.sensor_calibration_error= false;

    }

    //init analog part
    init_analog_sensors(ASENSOR_VALIDITY_THRES -1);

    //init digital sensors
    init_digital_sensors();

    Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_READY);
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
    if(Analog_Sensors[ASENSOR_MAP].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_MAP_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_MAP_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_MAP_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_MAP].out;
    }
    else
    {
        if(Tuareg.errors.sensor_MAP_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_MAP_error= true;
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
    if(Analog_Sensors[ASENSOR_O2].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_O2_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_O2_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_O2_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_O2].out;
    }
    else
    {
        if(Tuareg.errors.sensor_O2_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_O2_error= true;
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
    if(Analog_Sensors[ASENSOR_TPS].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_TPS_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_TPS_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_TPS_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_TPS].out;
    }
    else
    {
        if(Tuareg.errors.sensor_TPS_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_TPS_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_TPS_FAILED);
        }

        //use default value
        return TPS_DEFAULT_DEG;
    }
}


/**
checks the health state of the IAT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_IAT_sensor()
{
    if(Analog_Sensors[ASENSOR_IAT].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_IAT_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_IAT_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_IAT_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_IAT].out;
    }
    else
    {
        if(Tuareg.errors.sensor_IAT_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_IAT_error= true;
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
    if(Analog_Sensors[ASENSOR_CLT].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_CLT_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_CLT_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_CLT_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_CLT].out;
    }
    else
    {
        if(Tuareg.errors.sensor_CLT_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_CLT_error= true;
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
    if(Analog_Sensors[ASENSOR_VBAT].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_VBAT_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_VBAT_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_VBAT_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_VBAT].out;
    }
    else
    {
        if(Tuareg.errors.sensor_VBAT_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_VBAT_error= true;
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
    if(Analog_Sensors[ASENSOR_KNOCK].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_KNOCK_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_KNOCK_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_KNOCK_VALIDATED);
        }

        //use live value
        return Analog_Sensors[ASENSOR_KNOCK].out;
    }
    else
    {
        if(Tuareg.errors.sensor_KNOCK_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_KNOCK_error= true;
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
    if(Analog_Sensors[ASENSOR_BARO].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_BARO_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_BARO_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_BARO_VALIDATED);

        }

        //use live value
        return Analog_Sensors[ASENSOR_BARO].out;
    }
    else
    {
        if(Tuareg.errors.sensor_BARO_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_BARO_error= true;
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

const F32 cGearsRoundOffset= 0.5;

gears_t Tuareg_update_GEAR_sensor()
{
    U32 converted_gear;

    //check if sensor can be validated in this cycle
    if(Analog_Sensors[ASENSOR_GEAR].valid_count > ASENSOR_VALIDITY_THRES)
    {
        if(Tuareg.errors.sensor_GEAR_error == true)
        {
            //sensor successfully validated
            Tuareg.errors.sensor_GEAR_error= false;
            Syslog_Info(TID_TUAREG_SENSORS, SENSORS_LOC_GEAR_VALIDATED);
        }

        /**
        use live value
        the conversion of float value to gears_t shall be carried out with rounding to the nearest value
        with offset 0.5 the 6th gear is reported for input value 320
        */
        converted_gear= Analog_Sensors[ASENSOR_GEAR].out + cGearsRoundOffset;

        if(converted_gear >= GEAR_COUNT)
        {
            return GEAR_COUNT -1;
        }
        else
        {
            return converted_gear;
        }

    }
    else
    {
        if(Tuareg.errors.sensor_GEAR_error == false)
        {
            //sensor temporarily disturbed
            Tuareg.errors.sensor_GEAR_error= true;
            Syslog_Error(TID_TUAREG_SENSORS, SENSORS_LOC_GEAR_FAILED);
        }
            //use default value
            return GEAR_NEUTRAL;
    }
}

