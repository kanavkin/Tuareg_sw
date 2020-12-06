/**
TODO
- implement diag buttons
    (ts_commandButtons)
*/

#include "stm32_libs/boctok_types.h"

#include "utils.h"
#include "table.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include "TunerStudio.h"

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



volatile tuners_cli_t TS_cli;

/**
This is called when a command is received over serial from TunerStudio / Megatune
It parses the command and calls the relevant function
A detailed description of each call can be found at: http://www.msextra.com/doc/ms1extra/COM_RS232.htm

All commands that require more than one byte of rx data shall set the TS_cli.State.cmd_pending flag to true, until the command has been processed properly.
-> The ts_communication function can be entered many times for the same active command
*/
void ts_communication()
{
    VU32 data_1, data_2, data_3, data_4, offset, value;
    exec_result_t result;


    // reset cmd pending when timeout occurred
    if((TS_cli.State.cmd_pending == true) && (TS_cli.command_duration == 0))
    {
        TS_cli.State.cmd_pending = false;

        #ifdef TS_DEBUG
        /// TODO (oli#1#): debug action enabled
        UART_Tx(DEBUG_PORT, '!');
        #endif // TS_DEBUG

    }

    //nothing to do if no new rx data is available
    if( UART_available() == 0)
    {
        return;
    }

    //if we have no currently active command the received byte shall be a command
    if(TS_cli.State.cmd_pending == false)
    {
        TS_cli.currentCommand= UART_getRX();
        TS_cli.command_duration= COMMAND_MAX_DURATION_S;

        #ifdef TS_DEBUG
        /// TODO (oli#1#): debug action enabled
        UART_Tx(DEBUG_PORT, '*');
        #endif // TS_DEBUG

    }
/// TODO (oli#4#): export diagnostics


    switch(TS_cli.currentCommand)
    {

        case 'A':
            /**
            send OutputChannels
            */
            ts_sendOutputChannels();

            break;


        case 'B':
            /**
            Burn current configuration data to eeprom if permission has been given
            */
            if( TS_cli.State.burn_permission == false )
            {
                print(DEBUG_PORT, "\r\nWARNING config write rejected");
            }
            else
            {
                /// TODO (oli#8#): evaluate return value!
                print(DEBUG_PORT, "\r\nINFO writing config to eeprom");

                result= config_write();

                if(result == EXEC_OK)
                {
                    print(DEBUG_PORT, "\r\nINFO config has been written");
                }
                else
                {
                    print(DEBUG_PORT, "\r\nWARNING config write failed");
                }

            }

            break;


        case 'C':
            /**
            test communication
            This is used by Tunerstudio to see whether there is an ECU on a given serial port
            */
            UART_Tx(TS_PORT, '1');

            break;


        case 'D':

            /**
            received debug feature command
            */
            TS_cli.State.cmd_pending = true;

            //taking 2 bytes of input data

            if(UART_available() >= 2)
            {
                /**
                format:
                MSB, LSB
                */
                data_1= UART_getRX();
                data_2= UART_getRX();

                //run the desired feature
                ts_debug_features( word(data_1, data_2) );

                TS_cli.State.cmd_pending = false;
            }
            break;

        case 'F':
                /**
                send serial protocol version
                */
                print(TS_PORT, "001");
                break;


        case 'J':

                /**
                user permission management
                */
                TS_cli.State.cmd_pending= true;

                if(UART_available() >= 4)
                {
                    //read input
                    data_1= UART_getRX();
                    data_2= UART_getRX();
                    data_3= UART_getRX();
                    data_4= UART_getRX();

                    //get modification permission
                    if((data_1 == 'm') && (data_2 == 'o') && (data_3 == 'd') && (data_4 == '#'))
                    {
                        TS_cli.State.mod_permission = true;
                        print(DEBUG_PORT, "\r\nINFO unlocked config modification");
                    }
                    else if((data_1 == 'c') && (data_2 == 'a') && (data_3 == 'l') && (data_4 == '#'))
                    {
                        TS_cli.State.calib_mod_permission = true;
                        print(DEBUG_PORT, "\r\nINFO unlocked calibration modification");
                    }
                    else if((data_1 == 'd') && (data_2 == 'e') && (data_3 == 'c') && (data_4 == '#'))
                    {
                        TS_cli.State.decoder_mod_permission = true;
                        print(DEBUG_PORT, "\r\nINFO unlocked decoder config modification");
                    }
                    else if((data_1 == 'i') && (data_2 == 'g') && (data_3 == 'n') && (data_4 == '#'))
                    {
                        TS_cli.State.ignition_mod_permission = true;
                        print(DEBUG_PORT, "\r\nINFOunlocked ignition config modification");
                    }
                    else if((data_1 == 'b') && (data_2 == 'r') && (data_3 == 'n') && (data_4 == '!'))
                    {
                        TS_cli.State.burn_permission = true;
                        print(DEBUG_PORT, "\r\nINFO unlocked config burn");
                    }

                    else if((data_1 == 'l') && (data_2 == 'o') && (data_3 == 'c') && (data_4 == 'k'))
                    {
                        TS_cli.State.burn_permission = false;
                        TS_cli.State.mod_permission = false;
                        TS_cli.State.calib_mod_permission = false;
                        TS_cli.State.decoder_mod_permission = false;
                        TS_cli.State.ignition_mod_permission = false;
                        print(DEBUG_PORT, "\r\nINFO config locked");
                    }

                    //ready
                    TS_cli.State.cmd_pending = false;
                }

            break;


        case 'L':
            /**
            List the contents of current page in human readable form
            */
            ts_diagPage();
            break;


        case 'N':
            /**
            Displays a new line.
            */
            print(TS_PORT, "\r\n");
            break;


        case 'P':

            /**
            set the current page
            (A 2nd byte of data is required after the 'P' specifying the new page number)
            */
            TS_cli.State.cmd_pending= true;

            if( UART_available() )
            {
                data_1= UART_getRX();

                /**
                This converts the ascii number char into binary.
                */
                if( data_1 >= 0x30 )
                {
                    data_1 -= 0x30;
                }

                if((data_1 > TS_ZERO_PAGE) && (data_1 < TSPAGE_COUNT))
                {
                    TS_cli.currentPage= data_1;
                }

                TS_cli.State.cmd_pending = false;
            }

            break;


            case 'Q':
                /**
                send code version
                */
                print(TS_PORT, "speeduino 201708");
                break;


            case 'S':
                /**
                send code version
                */
                //print(TS_PORT, "Speeduino 2017.08");
                print(TS_PORT, "Tuareg V0.2 2020.10");

                //This is required in TS3 due to its stricter timings
                Tuareg.secl = 0;
                break;


            case 'U':

                /**
                received an update config value command
                */
                TS_cli.State.cmd_pending = true;

                /// data order: Offset (MSB, LSB), Value (MSB, x, x, LSB)
                if(UART_available() >= 6)
                {

                    offset= UART_getRX();
                    offset <<= 8;
                    offset |= UART_getRX();

                    value= UART_getRX();
                    value <<= 8;
                    value |= UART_getRX();
                    value <<= 8;
                    value |= UART_getRX();
                    value <<= 8;
                    value |= UART_getRX();

                    #ifdef TS_DEBUG
                    /// TODO (oli#1#): debug action enabled
                    print(DEBUG_PORT, "\r\nU cmd offset, value:");
                    printf_U(DEBUG_PORT, offset, NO_PAD);
                    printf_U(DEBUG_PORT, value, NO_PAD);
                    #endif // TS_DEBUG

                    //try to modify the indicated config page / calibration
                    modify_config(TS_cli.currentPage, offset, value);

                    //ready
                    TS_cli.State.cmd_pending = false;
                }

                break;


            case 'V':
                /**
                send config data in binary
                */
                ts_sendPage();
                break;


            case 'W':
                /**
                receive new config value at 'W'+<offset>+<newbyte>
                */

                #ifdef TS_DEBUG
                /// TODO (oli#1#): debug action enabled
                UART_Tx(DEBUG_PORT, '\r');
                UART_Tx(DEBUG_PORT, '\n');
                UART_Tx(DEBUG_PORT, 'W');
                UART_Tx(DEBUG_PORT, '.');
                UART_Tx(DEBUG_PORT, 'p');
                printf_U(DEBUG_PORT, TS_cli.currentPage, NO_PAD);
                UART_Tx(DEBUG_PORT, '.');
                UART_Tx(DEBUG_PORT, 'a');
                printf_U(DEBUG_PORT, UART_available(), NO_PAD);
                 UART_Tx(DEBUG_PORT, '\r');
                UART_Tx(DEBUG_PORT, '\n');
                #endif // TS_DEBUG


                TS_cli.State.cmd_pending = true;



                switch(TS_cli.currentPage)
                {

                    case CALIBPAGE:
                    case DECODERPAGE:
                    case IGNITIONPAGE:

                        /**
                        16 bit calibration data and 8 bit offset
                        */

                        if(UART_available() >= 3)
                        {
                            /**
                            offset
                            */
                            data_1= UART_getRX();


                            /**
                            value
                            MSB, LSB
                            */
                            data_2= UART_getRX();
                            data_3= UART_getRX();

                            //take the received data to config
                            ts_replaceConfig(data_1, word(data_2, data_3));

                            TS_cli.State.cmd_pending = false;
                        }

                        break;


                    case VEMAPPAGE:
                    case IGNMAPPAGE:
                    case AFRMAPPAGE:

                            /**
                            8 bit calibration data and 16 bit offset
                            */

                            if(UART_available() >= 3)
                            {
                                /**
                                offset
                                LSB, MSB
                                */
                                data_1= UART_getRX();
                                data_2= UART_getRX();

                                /**
                                value
                                */
                                data_3= UART_getRX();

                                //take the received data to config
                                ts_replaceConfig(word(data_2, data_1), data_3);

                                TS_cli.State.cmd_pending = false;
                            }

                            break;

                    case IGNITIONMAP_TPS:

                            /**
                            16 bit offset and 16 bit data
                            O_H O_L D_H D_L
                            */

                            if(UART_available() >= 4)
                            {
                                /**
                                offset
                                MSB, LSB
                                */
                                data_1= UART_getRX();
                                data_2= UART_getRX();

                                /**
                                value
                                MSB, LSB
                                */
                                data_3= UART_getRX();
                                data_4= UART_getRX();


                                if(TS_cli.State.mod_permission == false)
                                {
                                    print(DEBUG_PORT, "\r\n*** config modification rejected ***\r\n");
                                    return;
                                }

                                //write to table (boctok 3D coordinates)
                                modify_3D_table(&ignitionTable_TPS, word(data_1, data_2), word(data_3, data_4));

                                TS_cli.State.cmd_pending = false;
                            }

                            break;



                     case IGNITIONMAP_MAP:

                            /**
                            16 bit offset and 16 bit data
                            O_H O_L D_H D_L
                            */

                            if(UART_available() >= 4)
                            {
                                /**
                                offset
                                MSB, LSB
                                */
                                data_1= UART_getRX();
                                data_2= UART_getRX();

                                /**
                                value
                                MSB, LSB
                                */
                                data_3= UART_getRX();
                                data_4= UART_getRX();


                                if(TS_cli.State.mod_permission == false)
                                {
                                    print(DEBUG_PORT, "\r\n*** config modification rejected ***\r\n");
                                    return;
                                }

                                //write to table (boctok 3D coordinates)
                                modify_3D_table(&ignitionTable_MAP, word(data_1, data_2), word(data_3, data_4));

                                TS_cli.State.cmd_pending = false;
                            }

                            break;




                    default:

                            /**
                            8 bit calibration data and 8 bit offset
                            */

                            if(UART_available() >= 2)
                            {
                                /**
                                offset
                                */
                                data_1= UART_getRX();

                                /**
                                value
                                */
                                data_2= UART_getRX();

                                //take the received data to config
                                ts_replaceConfig(data_1, data_2);

                                TS_cli.State.cmd_pending = false;
                            }

                            break;


                } //switch page

                break; // W cmd


            case 'r':

                /// TODO (oli#2#): needed?

                /**
                New format for the optimized OutputChannels
                */
                TS_cli.State.cmd_pending = true;


                if (UART_available() >= 6)
                {
                    /**
                    byte 1
                    this should be $tsCanId
                    but it is currently not in use
                    */
                    data_1= UART_getRX();

                    /**
                    byte 2
                    cmd
                    */
                    data_2= UART_getRX();

                    //read the command byte
                    if(data_2 == 0x30)
                    {
                        /**
                        Send output channels
                        (command 0x30)
                        */

                        /**
                        byte 3 + 4
                        offset LSB, MSB
                        */
                        data_1= UART_getRX();
                        data_2= UART_getRX();

                        /**
                        byte 5 + 6
                        length LSB, MSB
                        */
                        data_3= UART_getRX();
                        data_4= UART_getRX();

                        //send live values
                        ts_sendValues(word(data_2, data_1), word(data_4, data_3));
                    }

                    TS_cli.State.cmd_pending = false;
                }
                break;

        case '?':

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

            break;

        default:
            break;
  } //switch active cmd


}




/**
This function returns the current values of a fixed group of variables

a detailed view on the logs has revealed that the A command is not in use
by TunerStudio, mostly r (offset=0, length=31)
*/
void ts_sendValues(U32 offset, U32 length)
{
    U32 i;

    /**
    print fake data
    */
    for(i=0; i < length; i++)
    {
        UART_Tx(TS_PORT, 0x00);
    }

}

void ts_sendOutputChannels()
{
    U8 output[TS_OUTPUTCHANNELS_BUFFERSIZE];
    U32 i;

    if(TS_cli.A_cmd_requests == 0)
    {
        Tuareg.secl = 0;
    }

    TS_cli.A_cmd_requests++;


    /*
    secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
    secl             = scalar, U08,  0, "sec",    1.000, 0.000
    */
    output[0] = Tuareg.secl;

    /*
    tuareg           = scalar, U32,  1, "bits",   1.000, 0.000
    tConfigErr       = bits,    U32,    1, [0:0]
    tScheduleErr     = bits,    U32,    1, [1:1]
    tO2SensorErr     = bits,    U32,    1, [2:2]
    tTpsSensorErr    = bits,    U32,    1, [3:3]
    tIatSensorErr    = bits,    U32,    1, [4:4]
    tCltSensorErr    = bits,    U32,    1, [5:5]
    tVbatSensorErr   = bits,    U32,    1, [6:6]
    tKnockSensorErr  = bits,    U32,    1, [7:7]
    tBaroSensorErr   = bits,    U32,    1, [8:8]
    tGearSensorErr   = bits,    U32,    1, [9:9]
    tMapSensorErr    = bits,    U32,    1, [10:10]
    tCiSensorErr     = bits,    U32,    1, [11:11]
    tSpareErr        = bits,    U32,    1, [12:15]

    tCrankingMode    = bits,    U32,    1, [16:16]
    tLimpMode        = bits,    U32,    1, [17:17]
    tDiagMode        = bits,    U32,    1, [18:18]

    tSpareBits       = bits,    U32,    1, [19:31]
    */
    serialize_U32_char((U32) ts_tuareg_bits(), &(output[1]));

    /*
    ignition         = scalar,  U08,    5, "bits",   1.000, 0.000
    ignDefTim        = bits,    U08,    5, [0:0]
    ignCrankingTim   = bits,    U08,    5, [1:1]
    ignRevLimit      = bits,    U08,    5, [2:2]
    ignDynamic       = bits,    U08,    5, [3:3]
    ignColdIdle      = bits,    U08,    5, [4:4]
    ignAdvMap        = bits,    U08,    5, [5:5]
    ignAdvTps        = bits,    U08,    5, [6:6]
    ignSpare         = bits,    U08,    5, [7:7]
    */
    output[5] = (U8) ts_ignition_bits();

    /*
    comm             = scalar, U08,  6, "bits",   1.000, 0.000
    comModPerm       = bits,    U08,    6, [0:0]
    comCalModPerm    = bits,    U08,    6, [1:1]
    comIgnModPerm    = bits,    U08,    6, [2:2]
    comDecModPerm    = bits,    U08,    6, [3:3]
    comBurnPerm      = bits,    U08,    6, [4:4]
    comSpare         = bits,    U08,    6, [5:7]
    */
    output[6] = (U8) ts_comm_bits();

    //rpm              = scalar,   U16,    7, "rpm",    1.000, 0.000
    serialize_U16_U8(Tuareg.process.crank_rpm, &(output[7]));

    //rpmDOT           = scalar,   F32,    9, "rpm/s",  1.000, 0.000
    serialize_float_U8(0.42, &(output[9]));

    //advance          = scalar,   U16,    13, "deg",    1.000, 0.000
    serialize_U16_U8(Tuareg.ignition_controls.ignition_advance_deg, &(output[13]));

    //dwell	        = scalar,   U16,    15, "ms",     0.100, 0.00
    serialize_U16_U8(Tuareg.ignition_controls.dwell_ms_phased, &(output[15]));

    //map              = scalar,   F32,    17, "kpa",    1.000, 0.000
    serialize_float_U8(Tuareg.process.MAP_kPa, &(output[17]));

    //baro             = scalar,   F32,    21, "kpa",      1.000, 0.000
    serialize_float_U8(Tuareg.process.Baro_kPa, &(output[21]));

    //tps              = scalar,   F32,    25, "deg",      1.000, 0.000
    serialize_float_U8(Tuareg.process.TPS_deg, &(output[25]));

    //TPSdot           = scalar,   F32,    29, "deg/s",    10.00, 0.000
    serialize_float_U8(0.42, &(output[29]));

    //iatRaw           = scalar,   F32,    33, "K",    1.000, 0.000
    serialize_float_U8(Tuareg.process.IAT_K, &(output[33]));

    //coolantRaw       = scalar,   F32,    37, "K",    1.000, 0.000
    serialize_float_U8(Tuareg.process.CLT_K, &(output[37]));

    //batteryVoltage   = scalar,   F32,    41, "V",      0.100, 0.000
    serialize_float_U8(Tuareg.process.VBAT_V, &(output[41]));

    //afr              = scalar,   F32,    45, "O2",     0.100, 0.000
    serialize_float_U8(14.5, &(output[45]));

    /*
   batCorrection    = scalar,   U08,      49, "%",      1.000, 0.000
   egoCorrection    = scalar,   U08,      50, "%",      1.000, 0.000
   airCorrection    = scalar,   U08,      51, "%",      1.000, 0.000
   warmupEnrich     = scalar,   U08,      52, "%",      1.000, 0.000
   accelEnrich      = scalar,   U08,      53, "%",      1.000, 0.000
   gammaEnrich      = scalar,   U08,      54, "%",      1.000, 0.000
   veCurr           = scalar,   U08,      55, "%",      1.000, 0.000
   afrTarget        = scalar,   U08,      56, "O2",     0.100, 0.000
   pulseWidth       = scalar,   U08,      57, "ms",     0.1,   0.000
   loopsPerSecond   = scalar,   U16,      58, "loops",  1.000, 0.000
   freeRAM          = scalar,   U16,      60, "bytes",  1.000, 0.000
   boostTarget      = scalar,   U08,      62, "kPa",    2.000, 0.000
   boostDuty        = scalar,   U08,      63, "%",      1.000, 0.000

   spark            = scalar,   U08,      64, "bits",   1.000, 0.000
    launchHard       = bits,    U08,    64, [0:0]
    launchSoft       = bits,    U08,    64, [1:1]
    hardLimitOn      = bits,    U08,    64, [2:2]
    softlimitOn      = bits,    U08,    64, [3:3]
    boostCutSpark    = bits,    U08,    64, [4:4]
    error            = bits,    U08,    64, [5:5]
    idle             = bits,    U08,    64, [6:6]
    sync             = bits,    U08,    64, [7:7]

   flex             = scalar,   U08,    65, "%",      1.000, 0.000
   flexFuelCor      = scalar,   U08,    66, "%",      1.000, 0.000
   flexIgnCor       = scalar,   U08,   67, "deg",    1.000, 0.000
   idleLoad         = scalar,   U08,    68, { bitStringValue( idleUnits , iacAlgorithm  ) },    2.000, 0.000 ; This is a combined variable covering both PWM and stepper IACs. The units used depend on which idle algorithm is chosen
   testoutputs      = scalar,   U08,    69, "bits",   1.000, 0.000
   testenabled       = bits,    U08,	  69, [0:0]
   testactive        = bits,    U08,	  69, [1:1]
   afr2             = scalar,   U08,    70, "O2",     0.100, 0.000
   tpsADC           = scalar,   U08,  71, "ADC",1.000, 0.000

   squirt           = scalar, U08,  72, "bits",   1.000, 0.000
    inj1Status       = bits,    U08,    72, [0:0]
    inj2Status       = bits,    U08,    72, [1:1]
    inj3Status       = bits,    U08,    72, [2:2]
    inj4Status       = bits,    U08,    72, [3:3]
    DFCOOn           = bits,    U08,    72, [4:4]
    boostCutFuel     = bits,    U08,    72, [5:5]
    toothLog1Ready   = bits,    U08,    72, [6:6]
    toothLog2Ready   = bits,    U08,    72, [7:7]

   engine           = scalar, U08,  73, "bits",   1.000, 0.000
    ready            = bits,    U08,    73, [0:0]
    crank            = bits,    U08,    73, [1:1]
    startw           = bits,    U08,    73, [2:2]
    warmup           = bits,    U08,    73, [3:3]
    tpsaccaen        = bits,    U08,    73, [4:4]
    tpsaccden        = bits,    U08,    73, [5:5]
    mapaccaen        = bits,    U08,    73, [6:6]
    mapaccden        = bits,    U08,    73, [7:7]

    errors           = scalar,   U08,    74, "bits",   1.000, 0.000
    errorNum        = bits,     U08,    74, [0:1]
    currentError    = bits,     U08,    74, [2:7]
   */

    /**
    the last bytes are not implemented yet
    */
    for(i=46; i < TS_OUTPUTCHANNELS_BUFFERSIZE; i++)
    {
        output[i]= 0;
    }

    /**
    print output channels
    */
    for(i=0; i < TS_OUTPUTCHANNELS_BUFFERSIZE; i++)
    {
        UART_Tx(TS_PORT, output[i]);
    }

}




/**
in system debug features

keep in mind that DEBUG_PORT might not always be connected while running diagnostic features!

FeatureID is a 16 Bit 0x00 .. 0xFFFF

*/

/// TODO (oli#3#): Implement debug features
/// TODO (oli#3#): Implement binary diag data printout



void ts_debug_features(U32 FeatureID)
{
    U8 u8_data =0;
    U32 data_1 =0;
    U32 data_2 =0;

    switch (FeatureID)
    {
        case 'dD':

            print_decoder_diag(TS_PORT);
            break;

        case 'dd':

            print_decoder_diag_legend(TS_PORT);
            break;

        case 'dT':

            print_tuareg_diag(TS_PORT);
            break;

        case 'dt':

            print_tuareg_diag_legend(TS_PORT);
            break;

        case 'dS':

            print_scheduler_diag(TS_PORT);
            break;

        case 'ds':

            print_scheduler_diag_legend(TS_PORT);
            break;

        case 'dI':

            print_ignhw_diag(TS_PORT);
            break;

        case 'di':

            print_ignhw_diag_legend(TS_PORT);
            break;



        case 'ep':

            /**
            dump eeprom content in binary format
            */

            print(TS_PORT, "EEprom data:\r\n");

            for(data_1=0; data_1 < EEPROM_STORAGE_END; data_1++)
            {
                data_2= eeprom_read_byte(data_1, &u8_data);

                if(data_2 != 0)
                {
                    //read error - terminate
                    print(TS_PORT, "ERROR!");
                    return;
                }

                UART_Tx(TS_PORT, u8_data);
            }

            print(TS_PORT, "Transmission complete.\r\n");

            break;

        case 'Pr':

            /**
            print current process data
            */
            ts_diag_process_data(&(Tuareg.process));

            break;

        case 'Pt':

            /**
            print current process table
            */
            print_process_table(TS_PORT);

            break;

        case 'Ig':

            /**
            print current ignition setup
            */
            ts_diag_ignition_timing(&(Tuareg.ignition_controls));

            break;

        case 'sn':

            //show analog and digital sensor values
            print_sensor_data(TS_PORT);
            break;

        case 'rs':

            //show analog and digital sensor values
            NVIC_SystemReset();
            break;


    default:
      break;
  }
}




/**
ts_sendPage() packs the data within the current page (As set with the 'P' command)
into a buffer and sends it.
Note that some translation of the data is required to lay it out in the way Megasquirt / TunerStudio expect it

TunerStudio expects data as raw values, if you want human readable output - use ts_diagPage()
*/
void ts_sendPage()
{
    volatile void* pnt_configPage =NULL;
    U32 configPage_size =0;
    volatile table3D_t * currentTable =NULL;
    U8 response[MAP_PAGE_SIZE];
    U32 i, l;
/// TODO (oli#8#): implement calibbration data binary printout for verification


    switch (TS_cli.currentPage)
    {

        case VEMAPPAGE:
            currentTable = &fuelTable;
            break;

        case VESETPAGE:
            pnt_configPage= &configPage1;
            configPage_size= VESETPAGE_SIZE;
            break;

        case IGNMAPPAGE:
            currentTable = &ignitionTable_TPS;
            break;

        case IGNSETPAGE:
            pnt_configPage = &configPage2;
            configPage_size= IGNSETPAGE_SIZE;
            break;

        case AFRMAPPAGE:
            currentTable = &afrTable;
            break;

        case AFRSETPAGE:
            pnt_configPage = &configPage3;
            configPage_size= AFRSETPAGE_SIZE;
            break;

        case IACPAGE:
            pnt_configPage = &configPage4;
            configPage_size= IACPAGE_SIZE;
            break;


        case BOOSTVVCPAGE:

            /**
            Need to perform a translation of the values[MAP/TPS][RPM] into the MS expected format
            the size is: (8x8 + 8 + 8) + (8x8 + 8 + 8) = 160
            */

            //Boost table
            for (i = 0; i < 64; i++) { response[i] = boostTable.axisZ[(i / 8)][i % 8]; }
            for (i = 64; i < 72; i++) { response[i] = (U8) (boostTable.axisX[(i - 64)] / TABLE_RPM_MULTIPLIER); }
            for (i = 72; i < 80; i++) { response[i] = (U8) (boostTable.axisY[7 - (i - 72)]); }

            //VVT table
            for (i = 0; i < 64; i++) { response[i + 80] = vvtTable.axisZ[(i / 8)][i % 8]; }
            for (i = 64; i < 72; i++) { response[i + 80] = (U8) (vvtTable.axisX[(i - 64)] / TABLE_RPM_MULTIPLIER); }
            for (i = 72; i < 80; i++) { response[i + 80] = (U8) (vvtTable.axisY[7 - (i - 72)]); }

            //transmit via serial
            for (i = 0; i < BOOSTVVCPAGE_SIZE; i++)
            {
                UART_Tx(TS_PORT, response[i]);
            }

            return;

            break;


        case SEQFUELPAGE:

            /**
            Need to perform a translation of the values[MAP/TPS][RPM] into the MS expected format
            the size is: (6x6 + 6 + 6) * 4 = 192
            */

            //trim1 table
            for (i= 0; i < 36; i++) { response[i] = trim1Table.axisZ[(i / 6)][i % 6]; }
            for (i = 36; i < 42; i++) { response[i] = (U8) (trim1Table.axisX[(i - 36)] / TABLE_RPM_MULTIPLIER); }
            for (i = 42; i < 48; i++) { response[i] = (U8) (trim1Table.axisY[5 - (i - 42)] / TABLE_LOAD_MULTIPLIER); }

            //trim2 table
            for (i = 0; i < 36; i++) { response[i + 48] = trim2Table.axisZ[i / 6][i % 6]; }
            for (i = 36; i < 42; i++) { response[i + 48] = (U8) (trim2Table.axisX[(i - 36)] / TABLE_RPM_MULTIPLIER); }
            for (i = 42; i < 48; i++) { response[i + 48] = (U8) (trim2Table.axisY[5 - (i - 42)] / TABLE_LOAD_MULTIPLIER); }

            //trim3 table
            for (i = 0; i < 36; i++) { response[i + 96] = trim3Table.axisZ[(i / 6)][i % 6]; }
            for (i = 36; i < 42; i++) { response[i + 96] = (U8) (trim3Table.axisX[(i - 36)] / TABLE_RPM_MULTIPLIER); }
            for (i = 42; i < 48; i++) { response[i + 96] = (U8) (trim3Table.axisY[5 - (i - 42)] / TABLE_LOAD_MULTIPLIER); }

            //trim4 table
            for (i = 0; i < 36; i++) { response[i + 144] = trim4Table.axisZ[(i / 6)][i % 6]; }
            for (i = 36; i < 42; i++) { response[i + 144] = (U8) (trim4Table.axisX[(i - 36)] / TABLE_RPM_MULTIPLIER); }
            for (i = 42; i < 48; i++) { response[i + 144] = (U8) (trim4Table.axisY[5 - (i - 42)] / TABLE_LOAD_MULTIPLIER); }


            //transmit via serial
            for (i = 0; i < SEQFUELPAGE_SIZE; i++)
            {
                UART_Tx(TS_PORT, response[i]);
            }

            return;

            break;


        case CANBUSPAGE:
            pnt_configPage = &configPage10;
            configPage_size= CANBUSPAGE_SIZE;
            break;

        case WARMUPPAGE:
            pnt_configPage = &configPage11;
            configPage_size= WARMUPPAGE_SIZE;
            break;

        default:
            print(DEBUG_PORT, "\n Page has not been implemented yet. Change to another page.");

            /**
            lets trick the output logic
            */
            pnt_configPage= &configPage1;
            configPage_size= 0;
            break;
    }

/// TODO (oli#1#): refactor!!!

    if ( (TS_cli.currentPage == VEMAPPAGE) || (TS_cli.currentPage == IGNMAPPAGE) || (TS_cli.currentPage == AFRMAPPAGE) )
    {
        /**
        this is a map
        */


        for(l= 0; l < 256; l++)
        {
            /**
            Z axis
            */
            response[l]= currentTable->axisZ[l / TABLE_3D_ARRAYSIZE][l % TABLE_3D_ARRAYSIZE];

            if(TS_cli.currentPage == IGNMAPPAGE)
            {
                //add offset for TS
                response[l] += TS_IGNITION_ADVANCE_OFFSET;
            }

        }

        for(l= 256; l < 272; l++)
        {
            //RPM Bins for VE table (Need to be divided by 100)
            response[l]= (U8) (currentTable->axisX[(l - 256)] / TABLE_RPM_MULTIPLIER);
        }

        for(l= 272; l < 288; l++)
        {
            //MAP or TPS bins for VE table
            response[l]= (U8) (currentTable->axisY[(l - 272)] / TABLE_LOAD_MULTIPLIER);
        }

        /**
        send via serial
        */
        for (l= 0; l < MAP_PAGE_SIZE; l++)
        {
            UART_Tx(TS_PORT, response[l]);
        }

    }
    else
    {
        /**
        we have a config page to print
        */
        for (l= 0; l < configPage_size; l++)
        {
            /**
            Each byte is simply the location in memory of the
            configPage + the offset + the variable number (x)
            */
            UART_Tx(TS_PORT,  *((U8 *)pnt_configPage + l) );
        }
    }

}



/**
ts_diagPage() packs the data within the current page (As set with the 'P' command)
into a buffer and sends it in human readable form.
*/
void ts_diagPage()
{
    volatile table3D_t * currentTable;
    U32 sendComplete = false;
    U32 x, i;
    U32 value;
    VU8 * currentVar;


    switch(TS_cli.currentPage)
    {
        case VEMAPPAGE:
            currentTable = &fuelTable;
            print(TS_PORT, "\r \n Volumetric Efficiency Map \r \n");
            break;

        case VESETPAGE:

            /**
            Display Values from Config Page 1
            */
            print(TS_PORT, "\r \n Page 1 Config \r \n");

            printf_S(TS_PORT, configPage1.flexBoostLow, NO_PAD);
            printf_U(TS_PORT, configPage1.flexBoostHigh, NO_PAD);
            printf_U(TS_PORT, configPage1.asePct, NO_PAD);
            printf_U(TS_PORT, configPage1.aseCount, NO_PAD);

            print(TS_PORT, "\r\n");

            for (i= 0; i < 10; i++)
            {
                printf_U(TS_PORT, configPage1.wueValues[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage1.crankingPct, NO_PAD);
            printf_U(TS_PORT, configPage1.pinMapping, NO_PAD);
            printf_U(TS_PORT, configPage1.tachoPin, NO_PAD);
            printf_U(TS_PORT, configPage1.tdePct, NO_PAD);
            printf_U(TS_PORT, configPage1.taeColdA, NO_PAD);
            printf_U(TS_PORT, configPage1.tpsThresh, NO_PAD);
            printf_U(TS_PORT, configPage1.taeTime, NO_PAD);
            printf_U(TS_PORT, configPage1.displayType, NO_PAD);
            printf_U(TS_PORT, configPage1.display3, NO_PAD);
            printf_U(TS_PORT, configPage1.displayB1, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage1.reqFuel, NO_PAD);
            printf_U(TS_PORT, configPage1.divider, NO_PAD);
            printf_U(TS_PORT, configPage1.injTiming, NO_PAD);
            printf_U(TS_PORT, configPage1.injOpen, NO_PAD);
            printf_U(TS_PORT, configPage1.inj1Ang, NO_PAD);
            printf_U(TS_PORT, configPage1.inj2Ang, NO_PAD);
            printf_U(TS_PORT, configPage1.inj3Ang, NO_PAD);
            printf_U(TS_PORT, configPage1.inj4Ang, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage1.mapSample, NO_PAD);
            printf_U(TS_PORT, configPage1.cltType1, NO_PAD);
            printf_U(TS_PORT, configPage1.engineType, NO_PAD);
            printf_U(TS_PORT, configPage1.primePulse, NO_PAD);
            printf_U(TS_PORT, configPage1.dutyLim, NO_PAD);
            printf_U(TS_PORT, configPage1.flexFreqLow, NO_PAD);
            printf_U(TS_PORT, configPage1.flexFreqHigh, NO_PAD);
            printf_U(TS_PORT, configPage1.boostMaxDuty, NO_PAD);
            printf_U(TS_PORT, configPage1.tpsMin, NO_PAD);
            printf_U(TS_PORT, configPage1.tpsMax, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_S(TS_PORT, configPage1.mapMin, NO_PAD);
            printf_U(TS_PORT, configPage1.mapMax, NO_PAD);
            printf_U(TS_PORT, configPage1.fpPrime, NO_PAD);
            printf_U(TS_PORT, configPage1.stoich, NO_PAD);
            printf_U(TS_PORT, configPage1.oddfire2, NO_PAD);
            printf_U(TS_PORT, configPage1.oddfire3, NO_PAD);
            printf_U(TS_PORT, configPage1.oddfire4, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage1.flexFuelLow, NO_PAD);
            printf_U(TS_PORT, configPage1.flexFuelHigh, NO_PAD);
            printf_U(TS_PORT, configPage1.flexAdvLow, NO_PAD);
            printf_U(TS_PORT, configPage1.flexAdvHigh, NO_PAD);
            printf_U(TS_PORT, configPage1.iacCLminDuty, NO_PAD);
            printf_U(TS_PORT, configPage1.iacCLmaxDuty, NO_PAD);
            printf_U(TS_PORT, configPage1.boostMinDuty, NO_PAD);

            sendComplete = true;
            break;


        case IGNMAPPAGE:
            currentTable = &ignitionTable_TPS;
            print(TS_PORT, "\r \n Ignition Map \r \n");
            break;

        case IGNSETPAGE:

            /**
            Display Values from Config Page 2
            */
            print(TS_PORT, "\r \n Page 2 Config \r \n");

            printf_U(TS_PORT, configPage2.triggerAngle, NO_PAD);
            printf_U(TS_PORT, configPage2.FixAng, NO_PAD);
            printf_U(TS_PORT, configPage2.CrankAng, NO_PAD);
            printf_U(TS_PORT, configPage2.TrigAngMul, NO_PAD);
            printf_U(TS_PORT, configPage2.TrigEdge, NO_PAD);
            printf_U(TS_PORT, configPage2.TrigEdgeSec, NO_PAD);
            printf_U(TS_PORT, configPage2.sparkDur, NO_PAD);
            printf_U(TS_PORT, configPage2.IdleAdvRPM, NO_PAD);
            printf_U(TS_PORT, configPage2.IdleAdvCLT, NO_PAD);
            printf_U(TS_PORT, configPage2.IdleDelayTime, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage2.StgCycles, NO_PAD);
            printf_U(TS_PORT, configPage2.dwellCont, NO_PAD);
            printf_U(TS_PORT, configPage2.dwellCrank, NO_PAD);
            printf_U(TS_PORT, configPage2.dwellRun, NO_PAD);
            printf_U(TS_PORT, configPage2.triggerTeeth, NO_PAD);
            printf_U(TS_PORT, configPage2.triggerMissingTeeth, NO_PAD);
            printf_U(TS_PORT, configPage2.crankRPM, NO_PAD);
            printf_U(TS_PORT, configPage2.floodClear, NO_PAD);
            printf_U(TS_PORT, configPage2.SoftRevLim, NO_PAD);
            printf_U(TS_PORT, configPage2.SoftLimRetard, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage2.SoftLimMax, NO_PAD);
            printf_U(TS_PORT, configPage2.HardRevLim, NO_PAD);

            for (i= 0; i < 4; i++)
            {
                printf_U(TS_PORT, configPage2.taeBins[i], NO_PAD);
            }

            for (i= 0; i < 4; i++)
            {
                printf_U(TS_PORT, configPage2.taeValues[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            for (i= 0; i < 10; i++)
            {
                printf_U(TS_PORT, configPage2.wueBins[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage2.dwellLimit, NO_PAD);

            for (i= 0; i < 6; i++)
            {
                printf_U(TS_PORT, configPage2.dwellCorrectionValues[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            for (i= 0; i < 6; i++)
            {
                printf_U(TS_PORT, configPage2.iatRetBins[i], NO_PAD);
            }

            for (i= 0; i < 6; i++)
            {
                printf_U(TS_PORT, configPage2.iatRetValues[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage2.dfcoRPM, NO_PAD);
            printf_U(TS_PORT, configPage2.dfcoHyster, NO_PAD);
            printf_U(TS_PORT, configPage2.dfcoTPSThresh, NO_PAD);
            printf_U(TS_PORT, configPage2.ignBypassEnabled, NO_PAD);

            sendComplete = true;
            break;

        case AFRMAPPAGE:
            currentTable = &afrTable;
            print(TS_PORT, "\r \n Air/Fuel Ratio Map \r \n");
            break;

        case AFRSETPAGE:

            /**
            Display Values from Config Page 3
            */
            print(TS_PORT, "\r \n Page 3 Config \r \n");

            printf_U(TS_PORT, configPage3.egoAlgorithm, NO_PAD);
            printf_U(TS_PORT, configPage3.egoKP, NO_PAD);
            printf_U(TS_PORT, configPage3.egoKI, NO_PAD);
            printf_U(TS_PORT, configPage3.egoKD, NO_PAD);
            printf_U(TS_PORT, configPage3.egoTemp, NO_PAD);
            printf_U(TS_PORT, configPage3.egoCount, NO_PAD);
            printf_U(TS_PORT, configPage3.egoDelta, NO_PAD);
            printf_U(TS_PORT, configPage3.egoLimit, NO_PAD);
            printf_U(TS_PORT, configPage3.ego_min, NO_PAD);
            printf_U(TS_PORT, configPage3.ego_max, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage3.ego_sdelay, NO_PAD);
            printf_U(TS_PORT, configPage3.egoRPM, NO_PAD);
            printf_U(TS_PORT, configPage3.egoTPSMax, NO_PAD);
            printf_U(TS_PORT, configPage3.vvtPin, NO_PAD);
            printf_U(TS_PORT, configPage3.boostPin, NO_PAD);

            print(TS_PORT, "\r\n");

            for (i= 0; i < 6; i++)
            {
                printf_U(TS_PORT, configPage3.voltageCorrectionBins[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            for (i= 0; i < 6; i++)
            {
                printf_U(TS_PORT, configPage3.injVoltageCorrectionValues[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            for (i= 0; i < 9; i++)
            {
                printf_U(TS_PORT, configPage3.airDenBins[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            for (i= 0; i < 9; i++)
            {
                printf_U(TS_PORT, configPage3.airDenRates[i], NO_PAD);
            }

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage3.boostFreq, NO_PAD);
            printf_U(TS_PORT, configPage3.vvtFreq, NO_PAD);
            printf_U(TS_PORT, configPage3.idleFreq, NO_PAD);
            printf_U(TS_PORT, configPage3.launchPin, NO_PAD);
            printf_U(TS_PORT, configPage3.lnchSoftLim, NO_PAD);
            printf_U(TS_PORT, configPage3.lnchRetard, NO_PAD);
            printf_U(TS_PORT, configPage3.lnchHardLim, NO_PAD);
            printf_U(TS_PORT, configPage3.lnchFuelAdd, NO_PAD);
            printf_U(TS_PORT, configPage3.idleKP, NO_PAD);
            printf_U(TS_PORT, configPage3.idleKI, NO_PAD);

            print(TS_PORT, "\r\n");

            printf_U(TS_PORT, configPage3.idleKD, NO_PAD);
            printf_U(TS_PORT, configPage3.boostLimit, NO_PAD);
            printf_U(TS_PORT, configPage3.boostKP, NO_PAD);
            printf_U(TS_PORT, configPage3.boostKI, NO_PAD);
            printf_U(TS_PORT, configPage3.boostKD, NO_PAD);
            printf_U(TS_PORT, configPage3.lnchPullRes, NO_PAD);
            printf_U(TS_PORT, configPage3.flatSSoftWin, NO_PAD);
            printf_U(TS_PORT, configPage3.flatSRetard, NO_PAD);
            printf_U(TS_PORT, configPage3.flatSArm, NO_PAD);

            sendComplete = true;
            break;

        case IACPAGE:

            /**
            Display Values from Config Page 4
            */
            print(TS_PORT, "\r \n Page 4 Config \r \n");

            /**
            Display four equally sized arrays at once
            */
            for (i= 4; i; i--)
            {

                switch(i)
                {
                    case 1: currentVar = configPage4.iacBins;
                            break;
                    case 2: currentVar = configPage4.iacOLPWMVal;
                            break;
                    case 3: currentVar = configPage4.iacOLStepVal;
                            break;
                    case 4: currentVar = configPage4.iacCLValues;
                            break;
                    default: break;
                }

                for(x = 10; x; x--)
                {
                    printf_U(TS_PORT, currentVar[10 - x], NO_PAD);
                }

                print(TS_PORT, "\r\n");
            }

            // Three equally sized arrays
            for (i= 3; i; i--)
            {
                switch (i)
                {
                    case 1: currentVar = configPage4.iacCrankBins;
                            break;
                    case 2: currentVar = configPage4.iacCrankDuty;
                            break;
                    case 3: currentVar = configPage4.iacCrankSteps;
                            break;
                    default: break;
                }

                for (x= 4; x; x--)
                {
                    printf_U(TS_PORT, currentVar[4 - x], NO_PAD);
                }

                print(TS_PORT, "\r\n");
            }

            printf_U(TS_PORT, configPage4.iacAlgorithm, NO_PAD);
            printf_U(TS_PORT, configPage4.iacFastTemp, NO_PAD);
            printf_U(TS_PORT, configPage4.iacStepHome, NO_PAD);
            printf_U(TS_PORT, configPage4.iacStepHyster, NO_PAD);
            printf_U(TS_PORT, configPage4.fanInv, NO_PAD);
            printf_U(TS_PORT, configPage4.fanSP, NO_PAD);
            printf_U(TS_PORT, configPage4.fanHyster, NO_PAD);
            printf_U(TS_PORT, configPage4.fanFreq, NO_PAD);

            for (i= 0; i < 4; i++)
            {
                printf_U(TS_PORT, configPage4.fanPWMBins[i], NO_PAD);
            }

            sendComplete = true;
            break;


        case BOOSTVVCPAGE:
                currentTable= &boostTable;
                print(TS_PORT, "\r \n Boost Map \r \n");
                break;


        case SEQFUELPAGE:

                currentTable= &trim1Table;
                print(TS_PORT, "\r \n VVT Map \r \n");

                for (i = 0; i < TABLE3D_DIMENSION; i++)
                {
                    value= (U8) (currentTable->axisY[i]);
                    printf_U(TS_PORT, value, NO_PAD);

                    for (x = 0; x < TABLE3D_DIMENSION; x++)
                    {
                        value= currentTable->axisZ[i][x];
                        printf_U(TS_PORT, value, NO_PAD);
                    }

                    print(TS_PORT, "\r\n");
                }

                sendComplete = true;
                break;

        case CALIBPAGE:

                show_Sensor_Calibration(TS_PORT);
                sendComplete = true;
                break;


        case DECODERPAGE:

                show_Decoder_Config(TS_PORT);
                sendComplete = true;
                break;


        case IGNITIONPAGE:

                show_Ignition_Config(TS_PORT);
                sendComplete = true;
                break;


        case IGNITIONMAP_TPS:
            currentTable = &ignitionTable_TPS;
            print(TS_PORT, "\r \nIgnition Map TPS (in boctok 3D coordinate system)\r\n");
            break;

        case IGNITIONMAP_MAP:
            currentTable = &ignitionTable_MAP;
            print(TS_PORT, "\r \nIgnition Map MAP (in boctok 3D coordinate system)\r\n");
            break;


        default:
            print(TS_PORT, "\n Page has not been implemented yet. Change to another page.");
            sendComplete = true;
            break;
    }

    if(!sendComplete)
    {
            //title already printed
            print_3D_table(TS_PORT, currentTable);

    }

}








void ts_diag_process_data(volatile process_data_t * pImage)
{
    /**
    volatile crank_position_t crank_position;
    volatile crank_position_table_t crank_position_table;
    VU32 crank_T_us;
    VU32 engine_rpm;

    volatile ctrl_strategy_t ctrl_strategy;

    VF32 MAP_Pa;
    VF32 Baro_Pa;
    VF32 TPS_deg;
    VF32 ddt_TPS;
    VF32 IAT_C;
    VF32 CLT_C;
    VF32 VBAT_V;
    */

    print(TS_PORT, "\r\n\r\nprocess data image:\r\n");
/*
    print(TS_PORT, "\r\ncrank position: ");
   printf_crkpos(TS_PORT, pImage->crank_position);


    print(TS_PORT, "\r\ncrank position table: ");

    for(i=0; i< CRK_POSITION_COUNT; i++)
    {
        printf_U(TS_PORT, pImage->crank_position_table.a_deg[i], TYPE_U16, PAD);
    }
*/
    print(TS_PORT, "\r\ncrank rotational period: ");
    printf_U(TS_PORT, pImage->crank_T_us, NO_PAD);

    print(TS_PORT, "rpm: ");
    printf_U(TS_PORT, pImage->crank_rpm, NO_PAD);

    print(TS_PORT, "\r\nstrategy: ");
    printf_U(TS_PORT, pImage->ctrl_strategy, NO_PAD);

    print(TS_PORT, "\r\nMAP (kPa), BARO (kPa), TPS (deg), ddt_TPS, IAT (C), CLT (C), VBAT (V), O2 (AFR), Gear:\r\n");

    printf_F32(TS_PORT, pImage->MAP_kPa);
    printf_F32(TS_PORT, pImage->Baro_kPa);
    printf_F32(TS_PORT, pImage->TPS_deg);
    printf_F32(TS_PORT, pImage->ddt_TPS);
    printf_F32(TS_PORT, pImage->IAT_K - cKelvin_offset);
    printf_F32(TS_PORT, pImage->CLT_K - cKelvin_offset);
    printf_F32(TS_PORT, pImage->VBAT_V);
    printf_F32(TS_PORT, pImage->O2_AFR);
    printf_U(TS_PORT, pImage->Gear, NO_PAD);

}

void ts_diag_ignition_timing(volatile ignition_control_t * pTiming)
{
    /**

    U16 ignition_advance_deg;
    U32 ignition_timing_us;
    crank_position_t ignition_pos;

    crank_position_t dwell_pos_phased;
    engine_phase_t dwell_phase_cyl1;
    engine_phase_t dwell_phase_cyl2;
    U8 dwell_ms_phased;

    crank_position_t dwell_pos_unphased;
    U8 dwell_ms_unphased;

    ignition_logic_state_t state;

    U8 default_timing :1;
        U8 cranking_timing :1;
        U8 rev_limiter :1;
        U8 dynamic :1;
        U8 cold_idle :1;
        U8 advance_map :1;
        U8 advance_tps :1;
        U8 extended_dwell :1;
    */

    print(TS_PORT, "\r\n\r\nignition setup:");

    print(TS_PORT, "\r\nadvance (deg), position, timing (us): ");
    printf_U(TS_PORT, pTiming->ignition_advance_deg, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->ignition_pos);
    printf_U(TS_PORT, pTiming->ignition_timing_us, NO_PAD);

    print(TS_PORT, "\r\ndwell -phased- duration (ms), position, phase cyl #1, phase cyl #2: ");
    printf_U(TS_PORT, pTiming->dwell_ms_phased, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->dwell_pos_phased);
    printf_phase(TS_PORT, pTiming->dwell_phase_cyl1);
    printf_phase(TS_PORT, pTiming->dwell_phase_cyl2);

    print(TS_PORT, "\r\ndefault-cranking-rev_limiter-dyn-cold_idle-a_map-a_tps-ext_dwell: ");

    UART_Tx(TS_PORT, (pTiming->state.default_timing? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.cranking_timing? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.rev_limiter? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.dynamic? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.cold_idle? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.advance_map? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.advance_tps? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.extended_dwell? '1' :'0'));

    print(TS_PORT, "\r\ndwell -unphased- duration (ms), position: ");
    printf_U(TS_PORT, pTiming->dwell_ms_unphased, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->dwell_pos_unphased);

}


/**
a byte sent by TunerStudio shall replace our
current configuration value
*/
void ts_replaceConfig(U32 valueOffset, U32 newValue)
{

    volatile void* pnt_configPage;
    U32 tempOffset;

    if(TS_cli.State.mod_permission == false)
    {
        print(DEBUG_PORT, "\r\n*** config modification rejected ***\r\n");
        return;
    }

    #ifdef TS_DEBUG
    /// TODO (oli#1#): debug action enabled
    UART_Tx(DEBUG_PORT, '\r');
    UART_Tx(DEBUG_PORT, '\n');
    UART_Tx(DEBUG_PORT, 'r');
    printf_U(DEBUG_PORT, valueOffset, NO_PAD);
    UART_Tx(DEBUG_PORT, '.');
    printf_U(DEBUG_PORT, newValue, NO_PAD);
    #endif //TS_DEBUG

    switch(TS_cli.currentPage)
    {
        case VEMAPPAGE:

            if (valueOffset < 256)
            {
                fuelTable.axisZ[(valueOffset / 16)][valueOffset % 16] = newValue;
            }
            else if (valueOffset < 272)
            {
                /**
                X Axis
                The RPM values sent by megasquirt are divided by 100,
                need to multiple it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                fuelTable.axisX[(valueOffset - 256)] = (U16)(newValue * TABLE_RPM_MULTIPLIER);
            }
            else
            {
                /**
                Y Axis
                Need to do a translation to flip the order
                (Due to us using (0,0) in the top left rather than bottom right
                */
                tempOffset = 15 - (valueOffset - 272);
                fuelTable.axisY[tempOffset] = (U16)(newValue) * TABLE_LOAD_MULTIPLIER;
            }

            break;

        case VESETPAGE:

            pnt_configPage = &configPage1;

            /**
            For some reason, TunerStudio is sending offsets greater than the maximum page size.
            I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
            */
            if (valueOffset < PAGE_SIZE)
            {
                *((U8 *)pnt_configPage + (U8)valueOffset) = newValue;
            }
            break;


        case IGNMAPPAGE:

            /**
            perform basic transformation to adapt to boctok 3D coordinate system
            */

            if(valueOffset < TABLE3D_DIMENSION * TABLE3D_DIMENSION)
            {
                // Z-axis
                #ifdef TS_DEBUG
                /// TODO (oli#1#): debug action enabled
                print(DEBUG_PORT, "\r\nIGN Z: ");
                printf_U(DEBUG_PORT, valueOffset, NO_PAD);
                printf_U(DEBUG_PORT, newValue, NO_PAD);
                #endif //TS_DEBUG

                //all values are offset by 40
                if(newValue >= TS_IGNITION_ADVANCE_OFFSET)
                {
                    newValue -= TS_IGNITION_ADVANCE_OFFSET;
                }

            }
            else if(valueOffset < TABLE3D_DIMENSION * TABLE3D_DIMENSION + TABLE3D_DIMENSION )
            {
                // X-axis
                #ifdef TS_DEBUG
                /// TODO (oli#1#): debug action enabled
                print(DEBUG_PORT, "\r\nIGN X: ");
                printf_U(DEBUG_PORT, valueOffset, NO_PAD);
                printf_U(DEBUG_PORT, newValue, NO_PAD);
                #endif //TS_DEBUG

                // the RPM values sent by megasquirt are divided by 100
                newValue *= TABLE_RPM_MULTIPLIER;

            }
            else if(valueOffset < TABLE3D_DIMENSION * TABLE3D_DIMENSION + 2 * TABLE3D_DIMENSION )
            {
                // Y-axis
                #ifdef TS_DEBUG
                /// TODO (oli#1#): debug action enabled
                print(DEBUG_PORT, "\r\nIGN Y: ");
                printf_U(DEBUG_PORT, valueOffset, NO_PAD);
                printf_U(DEBUG_PORT, newValue, NO_PAD);
                #endif //TS_DEBUG

                // the RPM values sent by megasquirt are divided by 2
                newValue *= TABLE_LOAD_MULTIPLIER;
            }
            else
            {
                //invalid offset, do not attempt to modify anything
                #ifdef TS_DEBUG
                /// TODO (oli#1#): debug action enabled
                print(DEBUG_PORT, "received invalid ignition bin: ");
                printf_U(DEBUG_PORT, valueOffset, NO_PAD);
                printf_U(DEBUG_PORT, newValue, NO_PAD);
                #endif //TS_DEBUG

                return;
            }

            //write to table
            modify_3D_table(&ignitionTable_TPS, valueOffset, newValue);

            break;

        case IGNSETPAGE:

            pnt_configPage = &configPage2;

            /**
            For some reason, TunerStudio is sending offsets greater than the maximum page size.
            I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
            */
            if (valueOffset < PAGE_SIZE)
            {
                *((U8 *)pnt_configPage + (U8)valueOffset) = newValue;
            }
            break;

        case AFRMAPPAGE:

            if (valueOffset < 256)
            {
                //New value is part of the afr map
                afrTable.axisZ[15 - (valueOffset / 16)][valueOffset % 16] = newValue;
            }
            else if (valueOffset < 272)
            {
                /**
                X Axis
                The RPM values sent by megasquirt are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                afrTable.axisX[(valueOffset - 256)] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else
            {
                /**
                Y Axis
                Need to do a translation to flip the order
                */
                tempOffset = 15 - (valueOffset - 272);
                afrTable.axisY[tempOffset] = (S16) newValue * TABLE_LOAD_MULTIPLIER;
            }
            break;


        case AFRSETPAGE:
            pnt_configPage = &configPage3;

            /**
            For some reason, TunerStudio is sending offsets greater than the maximum page size.
            I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
            */
            if (valueOffset < PAGE_SIZE)
            {
                *((U8 *)pnt_configPage + (U8)valueOffset) = newValue;
            }
            break;


        case IACPAGE:
            pnt_configPage = &configPage4;

            /**
            For some reason, TunerStudio is sending offsets greater than the maximum page size.
            I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
            */
            if (valueOffset < PAGE_SIZE)
            {
                *((U8 *)pnt_configPage + (U8)valueOffset) = newValue;
            }
            break;

        case BOOSTVVCPAGE:

            /**
            Boost and VVT maps (8x8)
            */
            if (valueOffset < 64)
            {
                //New value is part of the boost map
                boostTable.axisZ[7 - (valueOffset / 8)][valueOffset % 8] = newValue;
            }
            else if (valueOffset < 72)
            {
                /**
                New value is on the X (RPM) axis of the boost table
                The RPM values sent by TunerStudio are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                boostTable.axisX[(valueOffset - 64)] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else if (valueOffset < 80)
            {
                /**
                New value is on the Y (TPS) axis of the boost table
                TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
                */
                boostTable.axisY[(7 - (valueOffset - 72))] = (S16) newValue;
            }
            else if (valueOffset < 144)
            {
                //New value is part of the vvt map
                tempOffset = valueOffset - 80;
                vvtTable.axisZ[7 - (tempOffset / 8)][tempOffset % 8] = newValue;
            }
            else if (valueOffset < 152)
            {
                /**
                New value is on the X (RPM) axis of the vvt table
                The RPM values sent by TunerStudio are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                tempOffset = valueOffset - 144;
                vvtTable.axisX[tempOffset] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else
            {
                /**
                New value is on the Y (Load) axis of the vvt table
                TABLE_LOAD_MULTIPLIER is NOT used for vvt as it is TPS based (0-100)
                */
                tempOffset = valueOffset - 152;
                vvtTable.axisY[(7 - tempOffset)] = (S16) newValue;
            }

            break;

        case SEQFUELPAGE:
        {
            if (valueOffset < 36)
            {
                //Trim1 values
                trim1Table.axisZ[5 - (valueOffset / 6)][valueOffset % 6] = newValue;
            }
            else if (valueOffset < 42)
            {
                /**
                New value is on the X (RPM) axis of the trim1 table.
                The RPM values sent by TunerStudio are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                trim1Table.axisX[(valueOffset - 36)] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else if (valueOffset < 48)
            {
                trim1Table.axisY[(5 - (valueOffset - 42))] = (S16) newValue * TABLE_LOAD_MULTIPLIER;
            }
            else if (valueOffset < 84)
            {
                //New value is part of the trim2 map
                tempOffset = valueOffset - 48; trim2Table.axisZ[5 - (tempOffset / 6)][tempOffset % 6] = newValue;
            }
            else if (valueOffset < 90)
            {
                /**
                New value is on the X (RPM) axis of the table.
                The RPM values sent by TunerStudio are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                tempOffset = valueOffset - 84;
                trim2Table.axisX[tempOffset] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else if (valueOffset < 96)
            {
                //New value is on the Y (Load) axis of the table
                tempOffset = valueOffset - 90;
                trim2Table.axisY[(5 - tempOffset)] = (S16) newValue * TABLE_LOAD_MULTIPLIER;
            }
            else if (valueOffset < 132)
            {
                //New value is part of the trim3 map
                tempOffset = valueOffset - 96; trim3Table.axisZ[5 - (tempOffset / 6)][tempOffset % 6] = newValue;
            }
            else if (valueOffset < 138)
            {
                /**
                New value is on the X (RPM) axis of the table.
                The RPM values sent by TunerStudio are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                tempOffset = valueOffset - 132; trim3Table.axisX[tempOffset] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else if (valueOffset < 144)
            {
                //New value is on the Y (Load) axis of the table
                tempOffset = valueOffset - 138; trim3Table.axisY[(5 - tempOffset)] = (S16) newValue * TABLE_LOAD_MULTIPLIER;
            }
            else if (valueOffset < 180)
            {
                //New value is part of the trim2 map
                tempOffset = valueOffset - 144;
                trim4Table.axisZ[5 - (tempOffset / 6)][tempOffset % 6] = newValue;
            }
            else if (valueOffset < 186)
            {
                /**
                New value is on the X (RPM) axis of the table.
                The RPM values sent by TunerStudio are divided by 100,
                need to multiply it back by 100 to make it correct (TABLE_RPM_MULTIPLIER)
                */
                tempOffset = valueOffset - 180;
                trim4Table.axisX[tempOffset] = (S16) newValue * TABLE_RPM_MULTIPLIER;
            }
            else if (valueOffset < 192)
            {
                //New value is on the Y (Load) axis of the table
                tempOffset = valueOffset - 186;
                trim4Table.axisY[(5 - tempOffset)] = (S16) newValue * TABLE_LOAD_MULTIPLIER;
            }
        }

        break;


        case WARMUPPAGE:
            pnt_configPage = &configPage11;

            /**
            For some reason, TunerStudio is sending offsets greater than the maximum page size.
            I'm not sure if it's their bug or mine, but the fix is to only update the config page if the offset is less than the maximum size
            */
            if (valueOffset < PAGE_SIZE)
            {
                *((U8 *)pnt_configPage + (U8)valueOffset) = newValue;
            }

            break;


        case CALIBPAGE:

            print(DEBUG_PORT, "\r\n*** use U command for calibration modification! ***\r\n");
            break;


        case DECODERPAGE:

            print(DEBUG_PORT, "\r\n*** use U command for decoder config modification! ***\r\n");
            break;


        case IGNITIONPAGE:

            print(DEBUG_PORT, "\r\n*** use U command for ignition config modification! ***\r\n");
            break;


        default:
            break;
  }

}








/**
update a configuration value
*/
void modify_config(U32 Page, U32 Offset, U32 Value)
{
    switch(Page)
    {
    case DECODERPAGE:

        if(TS_cli.State.decoder_mod_permission == false)
        {
            print(DEBUG_PORT, "\r\n*** decoder config modification rejected (permission) ***\r\n");
            return;
        }

        modify_Decoder_Config(Offset, Value);
        break;

    case IGNITIONPAGE:

        if(TS_cli.State.ignition_mod_permission == false)
        {
            print(DEBUG_PORT, "\r\n*** ignition config modification rejected (permission) ***\r\n");
            return;
        }

        modify_Ignition_Config(Offset, Value);
        break;

    case CALIBPAGE:

        if(TS_cli.State.calib_mod_permission == false)
        {
            print(DEBUG_PORT, "\r\n*** calibration modification rejected (permission) ***\r\n");
            return;
        }

        modify_Sensor_Calibration(Offset, Value);
        break;

    default:
        print(DEBUG_PORT, "\r\n*** modification rejected (invalid page) ***\r\n");
        break;

    }

}



/**
prepare OutputChannel "comm" field
*/
BF8 ts_comm_bits()
{
    BF8 commbits =0;

    if(TS_cli.State.burn_permission)
    {
        setBit_BF8(COMMBIT_BURN_PERMISSION, &commbits);
    }

    if(TS_cli.State.mod_permission)
    {
        setBit_BF8(COMMBIT_MOD_PERMISSION, &commbits);
    }

    if(TS_cli.State.calib_mod_permission)
    {
        setBit_BF8(COMMBIT_CALMOD_PERMISSION, &commbits);
    }

    if(TS_cli.State.ignition_mod_permission)
    {
        setBit_BF8(COMMBIT_IGNMOD_PERMISSION, &commbits);
    }

    if(TS_cli.State.decoder_mod_permission)
    {
        setBit_BF8(COMMBIT_DECMOD_PERMISSION, &commbits);
    }

    return commbits;
}


/**
prepare OutputChannel "tuareg" field

*/
BF32 ts_tuareg_bits()
{
    BF32 tuaregbits =0;

    if(Tuareg.Errors.config_load_error)
    {
        setBit_BF32(TBIT_CONFIGLOAD_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.scheduler_error)
    {
        setBit_BF32(TBIT_SCHEDULER_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_O2_error)
    {
        setBit_BF32(TBIT_O2SENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_TPS_error)
    {
        setBit_BF32(TBIT_TPSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_IAT_error)
    {
        setBit_BF32(TBIT_IATSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_CLT_error)
    {
        setBit_BF32(TBIT_CLTSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_VBAT_error)
    {
        setBit_BF32(TBIT_VBATSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_KNOCK_error)
    {
        setBit_BF32(TBIT_KNOCKSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_BARO_error)
    {
        setBit_BF32(TBIT_BAROSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_GEAR_error)
    {
        setBit_BF32(TBIT_GEARSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_MAP_error)
    {
        setBit_BF32(TBIT_MAPSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_CIS_error)
    {
        setBit_BF32(TBIT_CISENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Runmode == TMODE_CRANKING)
    {
        setBit_BF32(TBIT_CRANKING_MODE, &tuaregbits);
    }
    else if(Tuareg.Runmode == TMODE_LIMP)
    {
        setBit_BF32(TBIT_LIMP_MODE, &tuaregbits);
    }
    else if(Tuareg.Runmode == TMODE_DIAG)
    {
        setBit_BF32(TBIT_DIAG_MODE, &tuaregbits);
    }

    return tuaregbits;
}


/**
prepare OutputChannel "ignition" field

*/
VU8 ts_ignition_bits()
{
    BF32 ignitionbits =0;

    if(Tuareg.ignition_controls.state.default_timing)
    {
        setBit_BF32(IGNBIT_DEFAULT_TIMING, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.cranking_timing)
    {
        setBit_BF32(IGNBIT_CRANKING_TIMING, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.rev_limiter)
    {
        setBit_BF32(IGNBIT_REV_LIMITER, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.dynamic)
    {
        setBit_BF32(IGNBIT_DYNAMIC, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.cold_idle)
    {
        setBit_BF32(IGNBIT_COLD_IDLE, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.advance_map)
    {
        setBit_BF32(IGNBIT_ADVANCE_MAP, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.advance_tps)
    {
        setBit_BF32(IGNBIT_ADVANCE_TPS, &ignitionbits);
    }

    return ignitionbits;
}
