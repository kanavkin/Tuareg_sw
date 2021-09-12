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

#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "sensors.h"

#include "debug_port_messages.h"

#include "base_calc.h"
#include "diagnostics.h"
#include "bitfields.h"

#include "fueling_config.h"

#include "process_table.h"

#include "syslog.h"
#include "fault_log.h"

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

        case IGNITIONMAP_DWELL:
            send_ignDwellTable(TS_PORT);
            break;


        case FUELINGPAGE:
            send_Fueling_Setup(TS_PORT);
            break;

        case VEMAP_TPS:
            send_VeTable_TPS(TS_PORT);
            break;

        case VEMAP_MAP:
            send_VeTable_MAP(TS_PORT);
            break;

        case AFRMAP_TPS:
            send_AfrTable_TPS(TS_PORT);
            break;

        case ACCELCOMP_TPS:
            send_AccelCompTableTPS(TS_PORT);
            break;

        case ACCELCOMP_MAP:
            send_AccelCompTableMAP(TS_PORT);
            break;

        case WARMUPCOMP_TABLE:
            send_WarmUpCompTable(TS_PORT);
            break;

        case INJ_TIMING_TABLE:
            send_InjectorTimingTable(TS_PORT);
            break;

        case INJ_PHASE_TABLE:
            send_InjectorPhaseTable(TS_PORT);
            break;

        case CRANKINGFUEL_TABLE:
            send_CrankingFuelTable(TS_PORT);
            break;


        case SYSLOG_PAGE:
            send_syslog(TS_PORT);
            break;

        case FAULTLOG_PAGE:
            send_Fault_Log(TS_PORT);
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
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** decoder config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_Decoder_Setup(Offset, Value);
            break;

        case TSETUP_PAGE:

            if(Tuareg_console.cli_permissions.tsetup_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** tuareg setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_Tuareg_Setup(Offset, Value);
            break;

        case IGNITIONPAGE:

            if(Tuareg_console.cli_permissions.ignition_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** ignition setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_Ignition_Setup(Offset, Value);
            break;

        case FUELINGPAGE:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_Fueling_Setup(Offset, Value);
            break;

        case CALIBPAGE:

            if(Tuareg_console.cli_permissions.calib_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** calibration modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_Sensor_Calibration(Offset, Value);
            break;


        case IGNITIONMAP_TPS:

            if(Tuareg_console.cli_permissions.ignition_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** ignition config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_ignAdvTable_TPS(Offset, Value);
            break;


        case IGNITIONMAP_DWELL:

            if(Tuareg_console.cli_permissions.ignition_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** ignition config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_ignDwellTable(Offset, Value);
            break;


        case VEMAP_TPS:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_VeTable_TPS(Offset, Value);
            break;

        case VEMAP_MAP:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_VeTable_MAP(Offset, Value);
            break;

        case AFRMAP_TPS:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_AfrTable_TPS(Offset, Value);
            break;

        case ACCELCOMP_TPS:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_AccelCompTableTPS(Offset, Value);
            break;

        case ACCELCOMP_MAP:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_AccelCompTableMAP(Offset, Value);
            break;

        case WARMUPCOMP_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_WarmUpCompTable(Offset, Value);
            break;

        case INJ_TIMING_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_InjectorTimingTable(Offset, Value);
            break;

        case INJ_PHASE_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_InjectorPhaseTable(Offset, Value);
            break;

        case CRANKINGFUEL_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod_permission == false)
            {
                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return;
            }

            result= modify_CrankingFuelTable(Offset, Value);
            break;

        default:
            break;

    }

    #ifdef TS_DEBUG
    if(result != EXEC_OK)
    {
        print(DEBUG_PORT, "\r\nWARNING Page ");
        printf_U(DEBUG_PORT, Tuareg_console.ts_active_page, NO_PAD);
        print(DEBUG_PORT, "could not be updated");
    }
    else
    {
        print(DEBUG_PORT, "\r\nMOD Page ");
        printf_U(DEBUG_PORT, Tuareg_console.ts_active_page, NO_PAD);
        print(DEBUG_PORT, "at ");
        printf_U(DEBUG_PORT, Offset, NO_PAD);
        printf_U8hex(DEBUG_PORT, Value, 0);
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

        case IGNITIONMAP_DWELL:

            result= store_ignDwellTable();
            break;

        case FUELINGPAGE:

            result= store_Fueling_Setup();
            break;

        case VEMAP_TPS:

            result= store_VeTable_TPS();
            break;

        case VEMAP_MAP:

            result= store_VeTable_MAP();
            break;

        case AFRMAP_TPS:

            result= store_AfrTable_TPS();
            break;

        case ACCELCOMP_TPS:

            result= store_AccelCompTableTPS();
            break;

        case ACCELCOMP_MAP:

            result= store_AccelCompTableMAP();
            break;

        case WARMUPCOMP_TABLE:

            result= store_WarmUpCompTable();
            break;

        case INJ_TIMING_TABLE:

            result= store_InjectorTimingTable();
            break;

        case INJ_PHASE_TABLE:

            result= store_InjectorPhaseTable();
            break;

        case CRANKINGFUEL_TABLE:

            result= store_CrankingFuelTable();
            break;



       default:
            #ifdef TS_DEBUG
            DebugMsg_Warning("invalid page specified");
            #endif // TS_DEBUG
            break;

    }

    #ifdef TS_DEBUG
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
    #endif

}

