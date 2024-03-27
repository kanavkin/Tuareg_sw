#include <Tuareg_platform.h>
#include <Tuareg.h>


#include "TunerStudio.h"


//#define TS_DEBUG

#ifdef TS_DEBUG
#include "uart_printf.h"
#warning Tuner Studio debugging enabled
#endif // TS_DEBUG


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

        case INVCLT_TABLE:
            send_InvTableCLT(TS_PORT);
            break;

        case DECODERPAGE:
            send_Decoder_Setup(TS_PORT);
            break;

        case TSETUP_PAGE:
            send_Tuareg_Setup(TS_PORT);
            break;

        case TACH_TABLE:
            send_TachTable(TS_PORT);
            break;

        case IGNITIONPAGE:
            send_Ignition_Setup(TS_PORT);
            break;

        case IGNITIONMAP_DWELL:
            send_ignDwellTable(TS_PORT);
            break;

        case FUELINGPAGE:
            send_Fueling_Setup(TS_PORT);
            break;

        case CTRLSET_MAP_PAGE:

            send_Control_MAP(TS_PORT);
            break;

        case CTRLSET_TPS_PAGE:

            send_Control_TPS(TS_PORT);
            break;

        case CTRLSET_TPSLIMP_PAGE:

            send_Control_TPS_Limp(TS_PORT);
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

        case CRANKINGFUEL_TABLE:
            send_CrankingFuelTable(TS_PORT);
            break;

        case BAROCORR_TABLE:
            send_BAROtable(TS_PORT);
            break;

        case CHARGETEMP_TABLE:
            send_ChargeTempMap(TS_PORT);
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
exec_result_t ts_valueWrite(U32 Page, U32 Offset, U32 Value)
{
    exec_result_t result= EXEC_ERROR;

    switch(Page)
    {
        case DECODERPAGE:

            if(Tuareg_console.cli_permissions.decoder_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** decoder config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_Decoder_Setup(Offset, Value);
            break;

        case TSETUP_PAGE:

            if(Tuareg_console.cli_permissions.tsetup_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** tuareg setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_Tuareg_Setup(Offset, Value);
            break;

        case TACH_TABLE:

            if(Tuareg_console.cli_permissions.tsetup_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** tachometer table modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_TachTable(Offset, Value);
            break;

        case IGNITIONPAGE:

            if(Tuareg_console.cli_permissions.ignition_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** ignition setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_Ignition_Setup(Offset, Value);
            break;

        case IGNITIONMAP_DWELL:

            if(Tuareg_console.cli_permissions.ignition_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** ignition setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_ignDwellTable(Offset, Value);
            break;

        case FUELINGPAGE:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling setup modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_Fueling_Setup(Offset, Value);
            break;

        case CALIBPAGE:

            if(Tuareg_console.cli_permissions.calib_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** calibration modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_Sensor_Calibration(Offset, Value);
            break;


        case INVCLT_TABLE:

            if(Tuareg_console.cli_permissions.calib_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** LCT table modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_InvTableCLT(Offset, Value);
            break;


        case CTRLSET_MAP_PAGE:

            if(Tuareg_console.cli_permissions.ctrlset_map_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM_CTRLSET_MAP);
                return result;
            }

            result= modify_Control_MAP(Offset, Value);
            break;

        case CTRLSET_TPS_PAGE:

            if(Tuareg_console.cli_permissions.ctrlset_tps_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM_CTRLSET_TPS);
                return result;
            }

            result= modify_Control_TPS(Offset, Value);
            break;

        case CTRLSET_TPSLIMP_PAGE:

            if(Tuareg_console.cli_permissions.ctrlset_tps_limp_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM_CTRLSET_TPSLIMP);
                return result;
            }

            result= modify_Control_TPS_Limp(Offset, Value);
            break;


        case ACCELCOMP_TPS:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_AccelCompTableTPS(Offset, Value);
            break;

        case ACCELCOMP_MAP:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_AccelCompTableMAP(Offset, Value);
            break;

        case WARMUPCOMP_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_WarmUpCompTable(Offset, Value);
            break;

        case INJ_TIMING_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_InjectorTimingTable(Offset, Value);
            break;


        case CRANKINGFUEL_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_CrankingFuelTable(Offset, Value);
            break;

        case BAROCORR_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_BAROtable(Offset, Value);
            break;

        case CHARGETEMP_TABLE:

            if(Tuareg_console.cli_permissions.fueling_mod == false)
            {
                Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOMODPERM);

                #ifdef TS_DEBUG
                DebugMsg_Warning("*** fueling config modification rejected (permission) ***");
                #endif // TS_DEBUG
                return result;
            }

            result= modify_ChargeTempMap(Offset, Value);
            break;

        default:
            break;

    }

    if(result != EXEC_OK)
    {
        Syslog_Error(TID_TUNERSTUDIO, TS_LOC_MOD_ERROR);

        #ifdef TS_DEBUG
        print(DEBUG_PORT, "\r\nERROR Page ");
        printf_U(DEBUG_PORT, Tuareg_console.ts_active_page, NO_PAD);
        print(DEBUG_PORT, "could not be updated");
        #endif //TS_DEBUG
    }
    #ifdef TS_DEBUG
    else
    {
        print(DEBUG_PORT, "\r\nMOD Page ");
        printf_U(DEBUG_PORT, Tuareg_console.ts_active_page, NO_PAD);
        print(DEBUG_PORT, "at ");
        printf_U(DEBUG_PORT, Offset, NO_PAD);
        printf_U8hex(DEBUG_PORT, Value, 0);
    }
    #endif //TS_DEBUG

    return result;
}


/**
this function implements the TS interface burnPage command
*/
exec_result_t ts_burnPage(U32 Page)
{
    exec_result_t result= EXEC_ERROR;


    if(Tuareg_console.cli_permissions.burn_permission == false)
    {
        Syslog_Warning(TID_TUNERSTUDIO, TS_LOC_NOBRNPERM);

        #ifdef TS_DEBUG
        DebugMsg_Warning("*** page write rejected rejected (permission) ***");
        #endif // TS_DEBUG

        return result;
    }


    switch(Page)
    {
       case CALIBPAGE:

            result= store_Sensor_Calibration();
            break;

        case INVCLT_TABLE:

            result= store_InvTableCLT();
            break;

        case DECODERPAGE:

            result= store_Decoder_Setup();
            break;

        case TSETUP_PAGE:

            result= store_Tuareg_Setup();
            break;

        case TACH_TABLE:

            result= store_TachTable();
            break;

        case IGNITIONPAGE:

            result= store_Ignition_Setup();
            break;

        case IGNITIONMAP_DWELL:

            result= store_ignDwellTable();
            break;

        case FUELINGPAGE:

            result= store_Fueling_Setup();
            break;

        case CTRLSET_MAP_PAGE:

            result= store_Control_MAP();
            break;

        case CTRLSET_TPS_PAGE:

            result= store_Control_TPS();
            break;

        case CTRLSET_TPSLIMP_PAGE:

            result= store_Control_TPS_Limp();
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

        case CRANKINGFUEL_TABLE:

            result= store_CrankingFuelTable();
            break;

        case BAROCORR_TABLE:

            result= store_BAROtable();
            break;

        case CHARGETEMP_TABLE:

            result= store_ChargeTempMap();
            break;


       default:
            #ifdef TS_DEBUG
            DebugMsg_Warning("invalid page specified");
            #endif // TS_DEBUG
            break;

    }

    if(result != EXEC_OK)
    {
        Syslog_Error(TID_TUNERSTUDIO, TS_LOC_BRN_ERROR);

        #ifdef TS_DEBUG
        print(DEBUG_PORT, "\r\nERROR Page ");
        printf_U(DEBUG_PORT, Page, NO_PAD);
        print(DEBUG_PORT, "write failed");
        #endif
    }
    #ifdef TS_DEBUG
    else
    {
        print(DEBUG_PORT, "\r\nINFO Page ");
        printf_U(DEBUG_PORT, Page, NO_PAD);
        print(DEBUG_PORT, "has been written");
    }
    #endif

    return result;
}

