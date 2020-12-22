#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg_console.h"
#include "TunerStudio.h"
#include "TunerStudio_legacy.h"

#include "utils.h"
#include "table.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


#include "config_pages.h"
#include "config_tables.h"
#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"
#include "legacy_config.h"
#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "sensors.h"
#include "debug.h"
#include "base_calc.h"
#include "diagnostics.h"
#include "bitfields.h"

#include "process_table.h"

#define TS_DEBUG


/**
this function implements the TS interface binary config page read command
*/
void ts_readPage_legacy(U32 Page)
{
    U32 i, configPage_size =0;

    switch (Page)
    {

        case VEMAPPAGE:
            configPage_size= 288;
            break;

        case VESETPAGE:
            configPage_size= VESETPAGE_SIZE;
            break;

        case IGNMAPPAGE:
            configPage_size= 288;
            break;

        case IGNSETPAGE:
            configPage_size= IGNSETPAGE_SIZE;
            break;

        case AFRMAPPAGE:
            configPage_size= 288;
            break;

        case AFRSETPAGE:
            configPage_size= AFRSETPAGE_SIZE;
            break;

        case IACPAGE:
            configPage_size= IACPAGE_SIZE;
            break;

        case BOOSTVVCPAGE:

            configPage_size= BOOSTVVCPAGE_SIZE;
            break;

        case SEQFUELPAGE:

            configPage_size= SEQFUELPAGE_SIZE;
            break;

        case WARMUPPAGE:
            configPage_size= WARMUPPAGE_SIZE;
            break;


        default:
            return;
            break;
    }

    ///just fake!
    for (i= 0; i < configPage_size; i++)
    {
        UART_Tx(TS_PORT, i);
    }
}



/**
ts_diagPage() prints the content of the currently selected page for inspection
*/
void ts_showPage_legacy(U32 Page)
{


}




/**
a byte sent by TunerStudio shall replace our
current configuration value
*/
exec_result_t ts_valueWrite_legacy(U32 Page, U32 Offset, U32 Value)
{
  return EXEC_OK;
}

