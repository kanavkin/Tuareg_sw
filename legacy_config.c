#include "Tuareg_types.h"

#include "legacy_config.h"

#include "uart.h"
#include "conversion.h"



/****************************************************************************************************************************************************
*
* Load configuration data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t load_legacy_config()
{


    //success
    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* Takes the current configuration (config pages and maps)
* and writes them to EEPROM as per the layout defined in eeprom_layout.h
*
TODO remove map dimensions from eeprom
****************************************************************************************************************************************************/
exec_result_t write_legacy_config()
{


    return EXEC_OK;
}


/**
    Repoint the 2D table structs to the config pages
    (initialise the 8 table2D structs)


    to be reconfigured!

/// TODO (oli#3#): find out which kind of 2d table will make sense for us

*/
void init_2Dtables()
{

}
