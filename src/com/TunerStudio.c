#include <boctok_types.h>
#include "Tuareg_types.h"

#include "Tuareg_console.h"
#include "TunerStudio.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"

#include "table.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

//#include "decoder_config.h"
//#include "ignition_config.h"
//#include "sensor_calibration.h"
#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "sensors.h"
//#include "debug.h"
#include "base_calc.h"
#include "diagnostics.h"
#include "bitfields.h"

#include "process_table.h"

#include "syslog.h"

#define TS_DEBUG


/**
this function implements the TS interface binary config page read command
*/
void ts_readPage(U32 Page)
{
    switch (Page)
    {
        case CALIBPAGE:
            send_Sensor_Calibration(TS_PORT);
            break;

        case DECODERPAGE:
            send_Decoder_Setup(TS_PORT);
            break;

        case TSETUP_PAGE:
            send_Tuareg_Setup(TS_PORT);
            break;


        case IGNITIONPAGE:
            send_Ignition_Setup(TS_PORT);
            break;

        case IGNITIONMAP_TPS:
            send_ignAdvTable_TPS(TS_PORT);
            break;

        case SYSLOG_PAGE:
            send_syslog(TS_PORT);
            break;

        default:
            break;
    }
}



/**
this function implements the TS interface valueWrite command
*/
void ts_valueWrite(U32 Page, U32 Offset, U32 Value)
{
    exec_result_t result= false;

    switch(Page)
    {
        case DECODERPAGE:

            if(Tuareg_console.cli_permissions.decoder_mod_permission == false)
            {
                print(DEBUG_PORT, "\r\n*** decoder config modification rejected (permission) ***\r\n");
                return;
            }

            result= modify_Decoder_Setup(Offset, Value);
            break;

        case TSETUP_PAGE:

            if(Tuareg_console.cli_permissions.tsetup_mod_permission == false)
            {
                print(DEBUG_PORT, "\r\n*** tuareg setup modification rejected (permission) ***\r\n");
                return;
            }

            result= modify_Tuareg_Setup(Offset, Value);
            break;

        case IGNITIONPAGE:

            if(Tuareg_console.cli_permissions.ignition_mod_permission == false)
            {
                print(DEBUG_PORT, "\r\n*** ignition config modification rejected (permission) ***\r\n");
                return;
            }

            result= modify_Ignition_Setup(Offset, Value);
            break;

        case CALIBPAGE:

            if(Tuareg_console.cli_permissions.calib_mod_permission == false)
            {
                print(DEBUG_PORT, "\r\n*** calibration modification rejected (permission) ***\r\n");
                return;
            }

            result= modify_Sensor_Calibration(Offset, Value);
            break;


        case IGNITIONMAP_TPS:

            if(Tuareg_console.cli_permissions.ignition_mod_permission == false)
            {
                print(DEBUG_PORT, "\r\n*** ignition config modification rejected (permission) ***\r\n");
                return;
            }

            result= modify_ignAdvTable_TPS(Offset, Value);
            break;

        default:
            break;

    }

    if(result != EXEC_OK)
    {
        print(DEBUG_PORT, "\r\nWARNING Page ");
        printf_U(DEBUG_PORT, Tuareg_console.ts_active_page, NO_PAD);
        print(DEBUG_PORT, "could not be updated");
    }

    #ifdef TS_DEBUG
    /// TODO (oli#1#): debug action enabled
    else
    {
        print(DEBUG_PORT, "\r\nMOD Page ");
        printf_U(DEBUG_PORT, Tuareg_console.ts_active_page, NO_PAD);
        print(DEBUG_PORT, "at ");
        printf_U(DEBUG_PORT, Offset, NO_PAD);
        Print_U8Hex(DEBUG_PORT, Value);
    }
    #endif //TS_DEBUG
}


/**
this function implements the TS interface burnPage command
*/
void ts_burnPage(U32 Page)
{
    exec_result_t result= EXEC_ERROR;

    switch(Page)
    {
       case CALIBPAGE:

            result= store_Sensor_Calibration();
            break;

        case DECODERPAGE:

            result= store_Decoder_Setup();
            break;

        case TSETUP_PAGE:

            result= store_Tuareg_Setup();
            break;

        case IGNITIONPAGE:

            result= store_Ignition_Setup();
            break;

        case IGNITIONMAP_TPS:

            result= store_ignAdvTable_TPS();
            break;

       default:
            print(DEBUG_PORT, "\r\nWARNING invalid page specified");
            break;

    }


    if(result == EXEC_OK)
    {
        print(DEBUG_PORT, "\r\nINFO Page ");
        printf_U(DEBUG_PORT, Page, NO_PAD);
        print(DEBUG_PORT, "has been written");
    }
    else
    {
        print(DEBUG_PORT, "\r\nWARNING Page ");
        printf_U(DEBUG_PORT, Page, NO_PAD);
        print(DEBUG_PORT, "write failed");
    }

}

