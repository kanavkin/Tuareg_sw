#include "Tuareg_types.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg_console.h"
#include "console_syslog_locations.h"

#include "TunerStudio.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"

#include "Tuareg_service_functions.h"

#include "table.h"

#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"
#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "base_calc.h"

#include "fueling_config.h"

#include "diagnostics.h"
#include "bitfields.h"

#include "process_table.h"

#include "debug_port_messages.h"
#include "syslog.h"
#include "fault_log.h"

#include "highspeed_loggers.h"

#define CONSOLE_DEBUG

#ifdef CONSOLE_DEBUG
#warning Tuareg Console debugging enabled
#endif // CONSOLE_DEBUG

#define CONSOLE_DEBUGMSG

#ifdef CONSOLE_DEBUGMSG
#warning Tuareg Console debug port messages enabled
#endif // CONSOLE_DEBUGMSG


const char Tuareg_Version [] __attribute__((__section__(".rodata"))) = "Tuareg V0.23 2022.01";


volatile Tuareg_console_t Tuareg_console;


const U32 cTSmaxChunkLen= 4;

/**
This is the periodic, re-entrant function that processes received input data.
The (current) active command will be reset, once the function reaches its end.
*/
void Tuareg_update_console()
{
    VU32 offset, count, value, on, off;

    // reset active command when timeout occurred
    if((Tuareg_console.active_cmd > 0) && (Tuareg_console.ts_cmd_watchdog == 0))
    {
        Tuareg_console.active_cmd =0;

        #ifdef CONSOLE_DEBUG
        print(DEBUG_PORT, "---x");
        #endif // CONSOLE_DEBUG

    }


    //nothing to do if no new rx data is available
    if( UART_available() == 0)
    {
        return;
    }

    //collect diagnostic information
    //wrong diagnostic domain
    //tuareg_diag_log_event(TDIAG_TSTUDIO_CALLS);

    //fetch new command
    if(Tuareg_console.active_cmd == 0)
    {
        Tuareg_console.active_cmd= UART_getRX();
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
        if(UART_available() < 2)
        {
            return;
        }

        /**
        format:
        MSB, LSB
        */
        value= UART_getRX();
        value <<= 8;
        value |= UART_getRX();

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


    case 'G':

        //show Fault Log
        show_Fault_Log(TS_PORT);
        break;


    case 'H':
        send_highspeedlog(TS_PORT);
        break;


    case 'I':

        //received service info command takes 2 bytes of input data
        if(UART_available() < 2)
        {
            return;
        }

        /**
        format:
        MSB, LSB
        */
        value= UART_getRX();
        value <<= 8;
        value |= UART_getRX();

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
        if(UART_available() < 4)
        {
            return;
        }

        value= UART_getRX();
        value <<= 8;
        value |= UART_getRX();
        value <<= 8;
        value |= UART_getRX();
        value <<= 8;
        value |= UART_getRX();

        //check requested permissions
        cli_checkPermissions(value);

        break;

    case 'K':

        show_syslog(TS_PORT);
        break;

    case 'L':
        //show the content of the current page in human readable form
        cli_showPage(Tuareg_console.ts_active_page);
        break;

    case 'M':

        if(UART_available() < 4)
        {
            return;
        }

        /**
        byte format: <actor> <on> <off> <end>

        variable allocation:

                <offset> <on> <off> <value>
        */
        offset= UART_getRX();

        on= UART_getRX();
        off= UART_getRX();

        value= UART_getRX();

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
        if(UART_available() < 2)
        {
            return;
        }

        //ID MSB - convert ascii number to decimal
        value= 10* subtract_VU32( UART_getRX(), 0x30);

        //ID LSB - convert ascii number to decimal
        value += subtract_VU32( UART_getRX(), 0x30);

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
            //print(TS_PORT, "Tuareg V0.22 2021.10");
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

                if(UART_available() < 4)
                {
                    return;
                }

                //offset <LSB> <MSB>
                offset= UART_getRX();
                offset |= UART_getRX() << 8;

                if(offset < 0xFFFF)
                {
                    Tuareg_console.param_offset= offset;
                }

                //count <LSB> <MSB>
                count= UART_getRX();
                count |= UART_getRX() << 8;

                if(count <= cTSmaxChunkLen)
                {
                    Tuareg_console.param_count= count;
                    Tuareg_console.params_valid= true;
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
            if(UART_available() < Tuareg_console.param_count)
            {
                return;
            }


            /**
            add up and shift data bytes for
            value <LSB> <1> <2> <MSB>
            */
            for(count=0; count < Tuareg_console.param_count; count++)
            {
                value= UART_getRX();
                //value |= UART_getRX() << 8;

                #ifdef CONSOLE_DEBUG
                print(DEBUG_PORT, "\r\nv:");
                printf_U32hex(DEBUG_PORT, value);
                #endif // CONSOLE_DEBUG
            }

            //ts_valueWrite(Tuareg_console.ts_active_page, Tuareg_console.param_offset, value);

            break; // W cmd


        case 'W':

            /**
            pageValueWrite writes one byte to the currently selected page
            Data format: W%2o%v -> U\ Offset MSB\ Offset LSB\ Value
            */
            if(UART_available() < 3)
            {
                return;
            }

            //offset <LSB> <MSB>
            offset= UART_getRX();
            offset |= UART_getRX() << 8;

            //value
            value= UART_getRX();

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
inline void cli_showPage(U32 Page)
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
            print(TS_PORT, "\r\nIgnition Map TPS (in boctok 3D coordinate system)\r\n");
            show_ignAdvTable_TPS(TS_PORT);
            break;
/*
        case IGNITIONMAP_MAP:
            print(TS_PORT, "\r \nIgnition Map MAP (in boctok 3D coordinate system)\r\n");
            //show_ignAdvTable_MAP(TS_PORT);
            break;
*/
        case IGNITIONMAP_DWELL:
            print(TS_PORT, "\r\nIgnition Dwell table \r\n");
            show_ignDwellTable(TS_PORT);
            break;

        case FUELINGPAGE:
            show_Fueling_Setup(TS_PORT);
            break;


        case VEMAP_TPS:

            print(TS_PORT, "\r\nVE Map - TPS (in boctok 3D coordinate system)\r\n");
            show_VeTable_TPS(TS_PORT);
            break;

        case VEMAP_MAP:

            print(TS_PORT, "\r\nVE Map - MAP (in boctok 3D coordinate system)\r\n");
            show_VeTable_MAP(TS_PORT);
            break;

        case AFRMAP_TPS:

            print(TS_PORT, "\r\nAFR Map - TPS (in boctok 3D coordinate system)\r\n");
            show_AfrTable_TPS(TS_PORT);
            break;

        case AFRMAP_MAP:

            print(TS_PORT, "\r\nAFR Map - MAP (in boctok 3D coordinate system)\r\n");
            show_AfrTable_MAP(TS_PORT);
            break;

        case ACCELCOMP_TPS:

            print(TS_PORT, "\r\nAcceleration compensation - TPS based\r\n");
            show_AccelCompTableTPS(TS_PORT);
            break;

        case ACCELCOMP_MAP:

            print(TS_PORT, "\r\nAcceleration compensation - MAP based\r\n");
            show_AccelCompTableMAP(TS_PORT);
            break;

        case WARMUPCOMP_TABLE:

            print(TS_PORT, "\r\nWarm up compensation\r\n");
            show_WarmUpCompTable(TS_PORT);
            break;

        case INJ_TIMING_TABLE:

            print(TS_PORT, "\r\nInjector Timing\r\n");
            show_InjectorTimingTable(TS_PORT);
            break;

        case INJ_PHASE_TABLE:

            print(TS_PORT, "\r\nInjector Phase\r\n");
            show_InjectorPhaseTable(TS_PORT);
            break;

        case CRANKINGFUEL_TABLE:

            print(TS_PORT, "\r\nCranking Fuel mass\r\n");
            show_CrankingFuelTable(TS_PORT);
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
void cli_checkPermissions(U32 Value)
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
        break;

    }
}

#pragma GCC diagnostic pop

/**
this function implements the TS interface binary config page read command
*/
void cli_show_help()
{
/// TODO (oli#9#): update command list

    UART_Tx(TS_PORT, '\n');
    print(TS_PORT, "===Command Help===\n\r");
    print(TS_PORT, "All commands are single character and are concatenated with their parameters \n\r");
    print(TS_PORT, "without spaces. Some parameters are binary and cannot be entered through this \n\r");
    print(TS_PORT, "prompt by conventional means. \n\r");
    print(TS_PORT, "Syntax:  <command>+<parameter1>+<parameter2>+<parameterN>\n\r");
    print(TS_PORT, "===List of Commands===\n\r");
    print(TS_PORT, "A - Send Tuareg output channels (live data)\n\r");
    print(TS_PORT, "B - Burn current map and configPage values to eeprom\n\r");
    print(TS_PORT, "C - Test COM port.  Used by Tunerstudio to see whether an ECU is on a given serial \n\r");
    print(TS_PORT, "    port. Returns a binary number.\n\r");
    print(TS_PORT, "L - Displays map page (aka table) or configPage values.  Use P to change page (not \n\r");
    print(TS_PORT, "    every page is a map)\n\r");
    print(TS_PORT, "J - get permissions <mod#>, <cal#>, <ign#>, <dec#> or <brn!> \n\r");
    print(TS_PORT, "N - Print new line.\n\r");
    print(TS_PORT, "P - Set current page.  Syntax:  P+<pageNumber>\n\r");
    print(TS_PORT, "S - Display signature number\n\r");
    print(TS_PORT, "U - update configuration / calibration value U<Offset MSB, LSB><Value MSB, x, x, LSB> \n\r");
    print(TS_PORT, "Q - Same as S command\n\r");
    print(TS_PORT, "V - Display map or configPage values in binary\n\r");
    print(TS_PORT, "W - Set one byte in map or configPage.  Expects binary parameters. \n\r");
    print(TS_PORT, "    Syntax:  W+<offset>+<new byte>\n\r");
    print(TS_PORT, "? - Displays this help page\n\r");


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


