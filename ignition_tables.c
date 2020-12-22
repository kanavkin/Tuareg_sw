

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
#include "ignition_config.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

//DEBUG
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"








/**
*
* reads ignition table data from eeprom
*
*/
exec_result_t load_Ignition_Tables()
{
   exec_result_t load_result;



   ASSERT_EXEC_OK(load_result);

    //all done
    return EXEC_OK;
}

/**
*
* writes ignition tables data to eeprom
*
*/
exec_result_t store_Ignition_Tables()
{
    exec_result_t store_result;

    store_result= store_3Dt_data(&(ignAdvTable_TPS.data), EEPROM_IGNITION_ADVTPSMAP_BASE);

    ASSERT_EXEC_OK(store_result);

    //all done
    return EXEC_OK;
}





