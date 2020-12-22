

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


    /********************
    * Tuareg config     *
    ********************/


    result= load_Ignition_Config();

    ASSERT_EXEC_OK(result);

    print(DEBUG_PORT, "\r\nINFO Ignition Config has been loaded");

    result= load_Sensor_Calibration();

    print(DEBUG_PORT, "\r\nINFO Sensor Calibration has been loaded");

    ASSERT_EXEC_OK(result);




    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* Takes the current configuration tables and writes them to EEPROM as per the layout defined in eeprom_layout.h
*
****************************************************************************************************************************************************/
exec_result_t config_tables_write()
{
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
    load_essential_Decoder_Setup();
    load_essential_Ignition_Config();

}
