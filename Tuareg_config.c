

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"

#include "Tuareg_config.h"
#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"
#include "legacy_config.h"

#include "config_tables.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

//DEBUG
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"



/****************************************************************************************************************************************************
*
* Load configuration data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t config_load()
{
    exec_result_t result;


    /****************
    * legacy config *
    *****************/
    result= load_legacy_config();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO legacy config has been loaded");


    /********************
    * Tuareg config     *
    ********************/


    result= load_Ignition_Config();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO Ignition Config has been loaded");

    result= load_Sensor_Calibration();

    print(DEBUG_PORT, "\r\nINFO Sensor Calibration has been loaded");

    ASSERT_EXEC_OK(result);


    /********************
    * tables            *
    ********************/

    //ignition tables
    result= load_3D_table(&ignitionTable_TPS, EEPROM_CONFIG3_MAP, 100, 2);

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO ignition table TPS has been loaded");

    result= load_3D_table(&ignitionTable_MAP, EEPROM_IGNITIONTABLE_MAP_Z, 100, 1);

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO ignition table MAP has been loaded");

    //init all 2d tables
    init_2Dtables();


    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* Takes the current configuration (config pages and maps) and writes them to EEPROM as per the layout defined in eeprom_layout.h
*
****************************************************************************************************************************************************/
exec_result_t config_write()
{
    exec_result_t result;

    /********************
    * legacy config  *
    ********************/
    result= write_legacy_config();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO legacy config has been written");

    /********************
    * Tuareg config     *
    ********************/
    result= write_Decoder_Config();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO Decoder Config has been written");

    result= write_Ignition_Config();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO Ignition Config has been written");

    result= write_Sensor_Calibration();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO Sensor Calibration has been written");


    /********************
    * tables            *
    ********************/

    //ignition tables
    result= write_3D_table(&ignitionTable_TPS, EEPROM_CONFIG3_MAP, 100, 2);

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO ignition table TPS has been written");

    result= write_3D_table(&ignitionTable_MAP, EEPROM_IGNITIONTABLE_MAP_Z, 100, 2);

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO ignition table MAP has been written");


    return EXEC_OK;
}



/****************************************************************************************************************************************************

 ****************************************************************************************************************************************************/
exec_result_t check_config()
{
    /*
    U8 eeprom_data;
    U32 eeprom_code;



    check DATA_VERSION
    (Brand new eeprom)

    eeprom_code= eeprom_read_byte(EEPROM_DATA_VERSION, &eeprom_data);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }
/// TODO (oli#4#): Add an eeprom layout and config version check (at config load)

    if( (eeprom_data == 0) || (eeprom_data == 255) )
    {
        eeprom_code= eeprom_update(EEPROM_DATA_VERSION, CURRENT_DATA_VERSION);
    }

    return eeprom_code;
    */

    return EXEC_OK;
}







/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void config_load_essentials()
{
    load_essential_Decoder_Config();
    load_essential_Ignition_Config();

}
