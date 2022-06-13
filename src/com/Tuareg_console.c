#include <Tuareg_platform.h>
#include <Tuareg.h>

#include "serial_buffer.h"

#include "console_syslog_locations.h"

#include "TunerStudio.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"
#include "TunerStudio_syslog_locations.h"


//#define CONSOLE_DEBUG

#ifdef CONSOLE_DEBUG
#warning Tuareg Console debugging enabled
#endif // CONSOLE_DEBUG

//#define CONSOLE_DEBUGMSG

#ifdef CONSOLE_DEBUGMSG
#warning Tuareg Console debug port messages enabled
#endif // CONSOLE_DEBUGMSG


volatile Tuareg_console_t Tuareg_console;


const U32 cTSmaxChunkLen= 50;
const U32 cTSmaxOffset= 320;

/**
This is the periodic, re-entrant function that processes received input data.
The (current) active command will be reset, once the function reaches its end.
*/
void Tuareg_update_console()
{
    VU32 offset, count, value, on, off;
    exec_result_t result;

    // reset active command when timeout occurred
    if((Tuareg_console.active_cmd > 0) && (Tuareg_console.ts_cmd_watchdog == 0))
    {
        Tuareg_console.active_cmd =0;

        #ifdef CONSOLE_DEBUG
        print(DEBUG_PORT, "---x");
        #endif // CONSOLE_DEBUG

    }


    //nothing to do if no new rx data is available
    if( ComRx_Buffer_avail() == 0)
    {
        return;
    }

    //collect diagnostic information
    //wrong diagnostic domain
    //tuareg_diag_log_event(TDIAG_TSTUDIO_CALLS);

    //fetch new command
    if(Tuareg_console.active_cmd == 0)
    {
        ComRx_Buffer_pull(&(Tuareg_console.active_cmd));

        Tuareg_console.ts_cmd_watchdog= TS_CMD_WATCHDOG_S;
        Tuareg_console.param_count= 0;
        Tuareg_console.param_offset= 0;
        Tuareg_console.params_valid= false;


        #ifdef CONSOLE_DEBUG
        if(Tuareg_console.active_cmd != 'A')
        {
            print(DEBUG_PORT, "\r\n>");
            UART_Tx(DEBUG_PORT, Tuareg_console.active_cmd);
        }
        #endif // CONSOLE_DEBUG

    }

    switch(Tuareg_console.active_cmd)
    {

    case 'A':
        //send output channels
        ts_sendOutputChannels(TS_PORT);
        break;


    case 'B':

        //write current configuration data to eeprom
        ts_burnPage(Tuareg_console.ts_active_page);
        break;


    case 'C':
        //response to Tuner Studio ping
        UART_Tx(TS_PORT, '1');
        break;


    case 'D':

        //received debug feature command takes 2 bytes of input data
        if(ComRx_Buffer_avail() < 2)
        {
            return;
        }

        //MSB, LSB (Tuareg format: Little endian)
        result= ComRx_Buffer_pull_U16(&value, false);

        ASSERT_EXEC_OK_VOID(result);


        #ifdef CONSOLE_DEBUG
        print(DEBUG_PORT, "@");
        printf_U(DEBUG_PORT, value, NO_PAD | NO_TRAIL);
        #endif // CONSOLE_DEBUG

        //run the desired feature
        ts_debug_features(value);
        break;

    case 'F':

        //send serial protocol version
        print(TS_PORT, "001");
        break;

    case 'H':
        send_highspeedlog(TS_PORT);
        break;


    case 'I':

        //received service info command takes 2 bytes of input data
        if(ComRx_Buffer_avail() < 2)
        {
            return;
        }

        //MSB, LSB, (Tuareg format: Little endian)
        result= ComRx_Buffer_pull_U16(&value, false);

        ASSERT_EXEC_OK_VOID(result);

        #ifdef CONSOLE_DEBUG
        print(DEBUG_PORT, "@");
        printf_U(DEBUG_PORT, value, NO_PAD | NO_TRAIL);
        #endif // CONSOLE_DEBUG

        //run the desired feature
        ts_debug_info(value, TS_PORT);
        break;


    case 'J':

        /**
        user permission management
        4 bytes following
        */
        if(ComRx_Buffer_avail() < 4)
        {
            return;
        }

        //Tuareg format: Little endian
        result= ComRx_Buffer_pull_U32(&value, false);

        ASSERT_EXEC_OK_VOID(result);

        //check requested permissions
        cli_setPermissions(value);

        break;

    case 'K':

        show_syslog(TS_PORT);
        break;

    case 'L':
        //show the content of the current page in human readable form
        cli_showPage(Tuareg_console.ts_active_page);
        break;

    case 'M':

        if(ComRx_Buffer_avail() < 4)
        {
            return;
        }

        /**
        byte format: <actor> <on> <off> <end>

        variable allocation:

                <offset> <on> <off> <value>
        */
        result= ComRx_Buffer_pull(&offset);

        ASSERT_EXEC_OK_VOID(result);

        result= ComRx_Buffer_pull(&on);

        ASSERT_EXEC_OK_VOID(result);

        result= ComRx_Buffer_pull(&off);

        ASSERT_EXEC_OK_VOID(result);

        result= ComRx_Buffer_pull(&value);

        ASSERT_EXEC_OK_VOID(result);

        //check if the received command is a "service mode request"
        if( (offset == 0xFF) && (on == 0xFF) && (off == 0x00) && (value == 0x00) )
        {
            request_service_mode();
        }
        else
        {
            //activate the desired actor
            request_service_activation(offset, on, off, value);
        }
        break;


    case 'P':

        /**
        set the (current) active Tuner Studio page
        Data format: P\ Page ID MSB (ascii)\ Page ID LSB (ascii)
        */
        if(ComRx_Buffer_avail() < 2)
        {
            return;
        }

        //ID MSB
        result= ComRx_Buffer_pull(&offset);

        //convert ascii number to decimal
        value= 10* subtract_U32(offset, 0x30);

        //ID LSB
        result= ComRx_Buffer_pull(&offset);

        //ID LSB - convert ascii number to decimal
        value += subtract_U32(offset, 0x30);

        if((value > 0) && (value < TSPAGE_COUNT))
        {
            Tuareg_console.ts_active_page= value;

            #ifdef CONSOLE_DEBUG
            print(DEBUG_PORT, "@");
            printf_U(DEBUG_PORT, value, NO_PAD);
            #endif // CONSOLE_DEBUG
        }
        else
        {
            DebugMsg_Warning("TS cmd -P- invalid page specified");
        }

        break;


        case 'Q':
            /**
            send code version
            */
            //print(TS_PORT, "speeduino 201708");
            print(TS_PORT, "rusEFI v1.04");
            break;


        case 'R':
            // Tuner Studio readPage command
            ts_readPage(Tuareg_console.ts_active_page);
            break;


        case 'S':
            /**
            send code version
            */
            print_flash(TS_PORT, Tuareg_Version);

            //This is required in TS3 due to its stricter timings
            Tuareg_console.ts_secl = 0;
            break;


        case 'U':

            /**
            A defined command to write a block of bytes to a Controller
            Data format: U%2o%2c%2v -> U\ Offset MSB\ Offset LSB\ Count MSB\ Count LSB\ Value MSB ... Value LSB
            */

            /*
            try to parse the required command parameters
            */
            if(Tuareg_console.params_valid == false)
            {

                if(ComRx_Buffer_avail() < 4)
                {
                    return;
                }

                //offset <LSB> <MSB>, (TunerStudio format: Big endian)
                result= ComRx_Buffer_pull_U16(&offset, true);

                ASSERT_EXEC_OK_VOID(result);

                if(offset < cTSmaxOffset)
                {
                    Tuareg_console.param_offset= offset;
                }
                else
                {
                    #ifdef CONSOLE_DEBUG
                    print(DEBUG_PORT, "@ INVALID o:");
                    printf_U(DEBUG_PORT, Tuareg_console.param_offset, NO_PAD);
                    #endif // CONSOLE_DEBUG

                    break;
                }

                //count <LSB> <MSB>, (TunerStudio format: Big endian)
                result= ComRx_Buffer_pull_U16(&count, true);

                ASSERT_EXEC_OK_VOID(result);

                if(count <= cTSmaxChunkLen)
                {
                    Tuareg_console.param_count= count;
                    Tuareg_console.params_valid= true;
                }
                else
                {
                    #ifdef CONSOLE_DEBUG
                    print(DEBUG_PORT, "@ o:");
                    printf_U(DEBUG_PORT, Tuareg_console.param_offset, NO_PAD);
                    print(DEBUG_PORT, "INVALID c:");
                    printf_U(DEBUG_PORT, Tuareg_console.param_count, NO_PAD);
                    #endif // CONSOLE_DEBUG

                    break;
                }

                #ifdef CONSOLE_DEBUG
                print(DEBUG_PORT, "@ o:");
                printf_U(DEBUG_PORT, Tuareg_console.param_offset, NO_PAD);
                print(DEBUG_PORT, "c:");
                printf_U(DEBUG_PORT, Tuareg_console.param_count, NO_PAD);
                #endif // CONSOLE_DEBUG
            }

            /**
            pause until the announced amount of data has been received
            */
            if(ComRx_Buffer_avail() < Tuareg_console.param_count)
            {
                return;
            }


            /**
            process received data bytes
            */
            for(count=0; count < Tuareg_console.param_count; count++)
            {
                result= ComRx_Buffer_pull(&value);

                ASSERT_EXEC_OK_VOID(result);

                ts_valueWrite(Tuareg_console.ts_active_page, Tuareg_console.param_offset + count, value);



                //can destroy timing
                #ifdef CONSOLE_DEBUG
                print(DEBUG_PORT, "\r\nv:");
                printf_U32hex(DEBUG_PORT, value);
                #endif // CONSOLE_DEBUG

            }


            break; // W cmd


        case 'W':

            /**
            pageValueWrite writes one byte to the currently selected page
            Data format: W%2o%v -> U\ Offset MSB\ Offset LSB\ Value
            */
            if(ComRx_Buffer_avail() < 3)
            {
                return;
            }

            //offset <LSB> <MSB>,(Big Endian)
            result= ComRx_Buffer_pull_U16(&offset, true);
            ASSERT_EXEC_OK_VOID(result);

            //value
            result= ComRx_Buffer_pull(&value);
            ASSERT_EXEC_OK_VOID(result);

            #ifdef CONSOLE_DEBUG
            print(DEBUG_PORT, "@ o:");
            printf_U(DEBUG_PORT, offset, NO_PAD);
            print(DEBUG_PORT, "v:");
            printf_U32hex(DEBUG_PORT, value);
            #endif // CONSOLE_DEBUG

            ts_valueWrite(Tuareg_console.ts_active_page, offset, value);

            break; // W cmd



    case '?':

            cli_show_help();


            break;

        default:
            break;

    } //switch active cmd


    //finish command execution
    Tuareg_console.active_cmd =0;
}




/**
prints the content of the currently selected page for manual inspection
*/
void cli_showPage(U32 Page)
{
    switch(Page)
    {
        case CALIBPAGE:

                show_Sensor_Calibration(TS_PORT);
                break;

        case INVCLT_TABLE:

                show_InvTableCLT(TS_PORT);
                break;

        case DECODERPAGE:

                show_Decoder_Setup(TS_PORT);
                break;


        case IGNITIONPAGE:

            show_Ignition_Setup(TS_PORT);
            break;

        case IGNITIONMAP_TPS:
            show_ignAdvTable_TPS(TS_PORT);
            break;

        case IGNITIONMAP_DWELL:
            show_ignDwellTable(TS_PORT);
            break;

        case FUELINGPAGE:
            show_Fueling_Setup(TS_PORT);
            break;


        case VEMAP_TPS:

            show_VeTable_TPS(TS_PORT);
            break;

        case VEMAP_MAP:

            show_VeTable_MAP(TS_PORT);
            break;

        case AFRMAP_TPS:

            show_AfrTable_TPS(TS_PORT);
            break;

        case AFRMAP_MAP:

            show_AfrTable_MAP(TS_PORT);
            break;

        case ACCELCOMP_TPS:

            show_AccelCompTableTPS(TS_PORT);
            break;

        case ACCELCOMP_MAP:

            show_AccelCompTableMAP(TS_PORT);
            break;

        case WARMUPCOMP_TABLE:

            show_WarmUpCompTable(TS_PORT);
            break;

        case INJ_TIMING_TABLE:

            show_InjectorTimingTable(TS_PORT);
            break;

        case INJ_PHASE_TABLE:

            show_InjectorPhaseTable(TS_PORT);
            break;

        case CRANKINGFUEL_TABLE:

            show_CrankingFuelTable(TS_PORT);
            break;

        case BAROCORR_TABLE:

            show_BAROtable(TS_PORT);
            break;

        case CHARGETEMP_TABLE:

            show_ChargeTempTable(TS_PORT);
            break;


        case TSETUP_PAGE:

                show_Tuareg_Setup(TS_PORT);
                break;

        case SYSLOG_PAGE:

                show_syslog(TS_PORT);
                break;


        default:
            show_syslog(TS_PORT);
            break;
    }
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmultichar"

/*

*/
void cli_setPermissions(U32 Value)
{
    switch(Value)
    {
    case 'cal#':
        Tuareg_console.cli_permissions.calib_mod_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_SENSORCALIB_MOD_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked calibration modification");
        #endif // CONSOLE_DEBUGMSG

        break;

    case 'dec#':
        Tuareg_console.cli_permissions.decoder_mod_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_DECODERCONF_MOD_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked decoder config modification");
        #endif // CONSOLE_DEBUGMSG
        break;

    case 'ign#':
        Tuareg_console.cli_permissions.ignition_mod_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_IGNCONF_MOD_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked ignition config modification");
        #endif // CONSOLE_DEBUGMSG
        break;

    case 'fue#':
        Tuareg_console.cli_permissions.fueling_mod_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_FUELCONF_MOD_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked fueling config modification");
        #endif // CONSOLE_DEBUGMSG
        break;

    case 'tua#':
        Tuareg_console.cli_permissions.tsetup_mod_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_TUAREGSETUP_MOD_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked tuareg setup modification");
        #endif // CONSOLE_DEBUGMSG
        break;

    case 'brn!':
        Tuareg_console.cli_permissions.burn_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_BURN_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked config burn");
        #endif // CONSOLE_DEBUGMSG
        break;

    case 'lock':
        Tuareg_console.cli_permissions.burn_permission = false;
        Tuareg_console.cli_permissions.calib_mod_permission = false;
        Tuareg_console.cli_permissions.decoder_mod_permission = false;
        Tuareg_console.cli_permissions.fueling_mod_permission = false;
        Tuareg_console.cli_permissions.ignition_mod_permission = false;
        Tuareg_console.cli_permissions.tsetup_mod_permission = false;
        Tuareg_console.cli_permissions.faultlog_permission = false;

        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_LOCK_CONFIG);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("config locked");
        #endif // CONSOLE_DEBUGMSG
        break;

    case 'faul':
        Tuareg_console.cli_permissions.faultlog_permission = true;
        Syslog_Info(TID_TUAREG_CONSOLE, CONSOLE_LOC_FAULTLOG_PERM_GIVEN);

        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Info("unlocked fault log");
        #endif // CONSOLE_DEBUGMSG
        break;

    default:
        #ifdef CONSOLE_DEBUGMSG
        DebugMsg_Error("invalid permission request");
        #endif // CONSOLE_DEBUGMSG
        break;

    }
}

#pragma GCC diagnostic pop

/**
this function implements the TS interface binary config page read command
*/
void cli_show_help()
{
/// TODO (oli#9#): keep command list up to date

    print(TS_PORT, "\r\n\r\n\r\n *** ");
    print_flash(TS_PORT, Tuareg_Version);
    print(TS_PORT, " CLI ***\n\r");
    print(TS_PORT, "List of all commands for human interaction on CLI:\n\r");
    print(TS_PORT, "(all input in ascii)\n\r");

    print(TS_PORT, "B - Burn current page\n\r");
    print(TS_PORT, "L - Show current page data\n\r");
    print(TS_PORT, "J - Set permissions <mod#>, <cal#>, <ign#>, <dec#> or <brn!> \n\r");
    print(TS_PORT, "P - Set current page number: P<tens><ones>\n\r");
    print(TS_PORT, "S - Show signature number\n\r");
    print(TS_PORT, "? - Displays this help page\n\r");
    print(TS_PORT, "I - Show debug information: I<a><b>\n\r");
}



/*
this function shall be called every second
*/
void cli_cyclic_update()
{
    sub_VU32(&(Tuareg_console.ts_cmd_watchdog), 1);

    /**
    the connection timer is allowed to roll over since 2010

    if(Tuareg_console.secl < 255)
    {
        Tuareg_console.secl++;
    }
    else
    {
        Tuareg_console.secl= 1;
    }
    */
    Tuareg_console.ts_secl++;
}




void Tuareg_init_console()
{
    UART_TS_PORT_Init();
    UART_DEBUG_PORT_Init();

    Tuareg_console.ts_connected= false;
}


