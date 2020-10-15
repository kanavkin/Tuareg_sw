/**
TODO
- implement diag buttons
    (ts_commandButtons)
*/

#include "stm32_libs/boctok_types.h"

#include "utils.h"
#include "table.h"

#include "uart.h"
#include "conversion.h"
#include "TunerStudio.h"

#include "config_pages.h"
#include "config.h"
#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "sensors.h"
#include "debug.h"
#include "base_calc.h"
#include "diagnostics.h"



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
    VU32 data_1, data_2, data_3, data_4, data_5;


    // reset cmd pending when timeout occurred
    if((TS_cli.State.cmd_pending == TRUE) && (TS_cli.command_duration == 0))
    {
        TS_cli.State.cmd_pending = FALSE;

        #ifdef TS_DEBUG
        #warning TODO (oli#1#): debug action enabled
        UART_Tx(DEBUG_PORT, '!');
        #endif // TS_DEBUG

    }

    //nothing to do if no new rx data is available
    if( UART_available() == 0)
    {
        return;
    }

    //if we have no currently active command the received byte shall be a command
    if(TS_cli.State.cmd_pending == FALSE)
    {
        TS_cli.currentCommand= UART_getRX();
        TS_cli.command_duration= COMMAND_MAX_DURATION_S;

        #ifdef TS_DEBUG
        #warning TODO (oli#1#): debug action enabled
        UART_Tx(DEBUG_PORT, '*');
        #endif // TS_DEBUG

    }
#warning TODO (oli#4#): export diagnostics


    switch(TS_cli.currentCommand)
    {

        case 'A':
            /**
            send 74 bytes of realtime values actually CAN part is fake
            */
            ts_sendValues(0, SENDVALUE_FAKE_PACKETSIZE);

            break;


        case 'B':
            /**
            Burn current configuration data to eeprom if permission has been given
            */
            if( TS_cli.State.burn_permission == FALSE )
            {
                UART_Send(DEBUG_PORT, "\r\n*** config write rejected ***\r\n");
            }
            else
            {
                #warning TODO (oli#8#): evaluate return value!
                UART_Send(DEBUG_PORT, "\r\n*** writing config to eeprom ***\r\n");
                config_write();
                UART_Send(DEBUG_PORT, "\r\n*** config has been written ***\r\n");
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
            TS_cli.State.cmd_pending = TRUE;

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

                TS_cli.State.cmd_pending = FALSE;
            }
            break;

        case 'E':

            /**
            received modify calibration command
            */
            TS_cli.State.cmd_pending = TRUE;

            //taking 5 bytes of input data

            if(UART_available() >= 5)
            {
                /**
                data order: Offset, Value (MSB, x, x, LSB)
                */
                data_1= UART_getRX();
                data_2= UART_getRX();
                data_3= UART_getRX();
                data_4= UART_getRX();
                data_5= UART_getRX();

                //run the desired feature
                //replaceCalib((U32) data_1, compose_U32(data_2, data_3, data_4, data_5) );
                replaceCalib((U32) data_1, dword(data_2, data_3, data_4, data_5) );

                TS_cli.State.cmd_pending = FALSE;
            }
            break;


        case 'F':
                /**
                send serial protocol version
                */
                UART_Send(TS_PORT, "001");
                break;


        case 'J':

                /**
                user permission management
                */
                TS_cli.State.cmd_pending= TRUE;

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
                        TS_cli.State.mod_permission = TRUE;
                        UART_Send(DEBUG_PORT, "\r\n *** unlocked config modification ***");
                    }
                    else if((data_1 == 'b') && (data_2 == 'r') && (data_3 == 'n') && (data_4 == '!'))
                    {
                        TS_cli.State.burn_permission = TRUE;
                        UART_Send(DEBUG_PORT, "\r\n *** unlocked config burn ***");
                    }
                    else if((data_1 == 'c') && (data_2 == 'a') && (data_3 == 'l') && (data_4 == '#'))
                    {
                        TS_cli.State.calibmod_permission = TRUE;
                        UART_Send(DEBUG_PORT, "\r\n *** unlocked calibration modification ***");
                    }
                    else if((data_1 == 'l') && (data_2 == 'o') && (data_3 == 'c') && (data_4 == 'k'))
                    {
                        TS_cli.State.burn_permission = FALSE;
                        TS_cli.State.mod_permission = FALSE;
                        TS_cli.State.calibmod_permission = FALSE;
                        UART_Send(DEBUG_PORT, "\r\n *** config locked ***");
                    }

                    //ready
                    TS_cli.State.cmd_pending = FALSE;
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
            UART_Send(TS_PORT, "\r\n");
            break;


        case 'P':

            /**
            set the current page
            (A 2nd byte of data is required after the 'P' specifying the new page number)
            */
            TS_cli.State.cmd_pending= TRUE;

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

                TS_cli.State.cmd_pending = FALSE;
            }

            break;


            case 'Q':
                /**
                send code version
                */
                UART_Send(TS_PORT, "speeduino 201708");
                break;


            case 'S':
                /**
                send code version
                */
                UART_Send(TS_PORT, "Speeduino 2017.08");

                //This is required in TS3 due to its stricter timings
                Tuareg.secl = 0;
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
                #warning TODO (oli#1#): debug action enabled
                UART_Tx(DEBUG_PORT, '\r');
                UART_Tx(DEBUG_PORT, '\n');
                UART_Tx(DEBUG_PORT, 'W');
                UART_Tx(DEBUG_PORT, '.');
                UART_Tx(DEBUG_PORT, 'p');
                UART_Print_U(DEBUG_PORT, TS_cli.currentPage, TYPE_U32, NO_PAD);
                UART_Tx(DEBUG_PORT, '.');
                UART_Tx(DEBUG_PORT, 'a');
                UART_Print_U(DEBUG_PORT, UART_available(), TYPE_U32, NO_PAD);
                 UART_Tx(DEBUG_PORT, '\r');
                UART_Tx(DEBUG_PORT, '\n');
                #endif // TS_DEBUG


                TS_cli.State.cmd_pending = TRUE;



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

                            TS_cli.State.cmd_pending = FALSE;
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

                                TS_cli.State.cmd_pending = FALSE;
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


                                if(TS_cli.State.mod_permission == FALSE)
                                {
                                    UART_Send(DEBUG_PORT, "\r\n*** config modification rejected ***\r\n");
                                    return;
                                }

                                //write to table (boctok 3D coordinates)
                                modify_3D_table(&ignitionTable_TPS, word(data_1, data_2), word(data_3, data_4));

                                TS_cli.State.cmd_pending = FALSE;
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


                                if(TS_cli.State.mod_permission == FALSE)
                                {
                                    UART_Send(DEBUG_PORT, "\r\n*** config modification rejected ***\r\n");
                                    return;
                                }

                                //write to table (boctok 3D coordinates)
                                modify_3D_table(&ignitionTable_MAP, word(data_1, data_2), word(data_3, data_4));

                                TS_cli.State.cmd_pending = FALSE;
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

                                TS_cli.State.cmd_pending = FALSE;
                            }

                            break;


                } //switch page

                break; // W cmd


            case 'r':
                /**
                New format for the optimized OutputChannels
                */
                TS_cli.State.cmd_pending = TRUE;


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

                    TS_cli.State.cmd_pending = FALSE;
                }
                break;

        case '?':

            UART_Tx(TS_PORT, '\n');
            UART_Send(TS_PORT, "===Command Help===\n\r");
            UART_Send(TS_PORT, "All commands are single character and are concatenated with their parameters \n\r");
            UART_Send(TS_PORT, "without spaces. Some parameters are binary and cannot be entered through this \n\r");
            UART_Send(TS_PORT, "prompt by conventional means. \n\r");
            UART_Send(TS_PORT, "Syntax:  <command>+<parameter1>+<parameter2>+<parameterN>\n\r");
            UART_Send(TS_PORT, "===List of Commands===\n\r");
            UART_Send(TS_PORT, "A - Displays 31 bytes of Tuareg values in binary (live data)\n\r");
            UART_Send(TS_PORT, "B - Burn current map and configPage values to eeprom\n\r");
            UART_Send(TS_PORT, "C - Test COM port.  Used by Tunerstudio to see whether an ECU is on a given serial \n\r");
            UART_Send(TS_PORT, "    port. Returns a binary number.\n\r");
            UART_Send(TS_PORT, "L - Displays map page (aka table) or configPage values.  Use P to change page (not \n\r");
            UART_Send(TS_PORT, "    every page is a map)\n\r");
            UART_Send(TS_PORT, "J - get permissions <mod#> or <brn!> \n\r");
            UART_Send(TS_PORT, "N - Print new line.\n\r");
            UART_Send(TS_PORT, "P - Set current page.  Syntax:  P+<pageNumber>\n\r");
            UART_Send(TS_PORT, "R - Same as A command\n\r");
            UART_Send(TS_PORT, "S - Display signature number\n\r");
            UART_Send(TS_PORT, "Q - Same as S command\n\r");
            UART_Send(TS_PORT, "V - Display map or configPage values in binary\n\r");
            UART_Send(TS_PORT, "W - Set one byte in map or configPage.  Expects binary parameters. \n\r");
            UART_Send(TS_PORT, "    Syntax:  W+<offset>+<new byte>\n\r");
            UART_Send(TS_PORT, "? - Displays this help page\n\r");

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
    U8 fullStatus[SENDVALUE_BUFFERSIZE];
    U32 i;

    if(TS_cli.A_cmd_requests == 0)
    {
        Tuareg.secl = 0;
    }

    TS_cli.A_cmd_requests++;

    fullStatus[0] = Tuareg.secl; //secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
    fullStatus[1] = Tuareg.squirt; //Squirt Bitfield
    fullStatus[2] = Tuareg.engine; //Engine Status Bitfield
    fullStatus[3] = Tuareg.ignition_timing.dwell_ms *10; //Dwell in ms * 10
    fullStatus[4] = lowByte((U32) Tuareg.process.MAP_kPa / 10); //2 U8s for MAP
    fullStatus[5] = highByte((U32) Tuareg.process.MAP_kPa / 10);
    fullStatus[6] = (U8) (Tuareg.process.IAT_K - cKelvin_offset); //mat
    fullStatus[7] = (U8) (Tuareg.process.CLT_K -cKelvin_offset); //Coolant ADC
    fullStatus[8] = (U8) Tuareg.process.VBAT_V; //Battery voltage correction (%)
    fullStatus[9] = (U8) Tuareg.process.VBAT_V; //battery voltage
    fullStatus[10] = (U8) Tuareg.sensors->asensors[ASENSOR_O2]; //O2
    fullStatus[11] = Tuareg.egoCorrection; //Exhaust gas correction (%)
    fullStatus[12] = Tuareg.iatCorrection; //Air temperature Correction (%)
    fullStatus[13] = Tuareg.wueCorrection; //Warmup enrichment (%)
    fullStatus[14] = lowByte(Tuareg.process.engine_rpm); //rpm HB
    fullStatus[15] = highByte(Tuareg.process.engine_rpm); //rpm LB
    fullStatus[16] = Tuareg.TAEamount; //acceleration enrichment (%)
    fullStatus[17] = Tuareg.corrections; //Total GammaE (%)
    fullStatus[18] = Tuareg.VE; //Current VE 1 (%)
    fullStatus[19] = Tuareg.afrTarget;
    fullStatus[20] = (U8)(Tuareg.PW1 / 100); //Pulsewidth 1 multiplied by 10 in ms. Have to convert from uS to mS.
    fullStatus[21] = (U8) Tuareg.process.ddt_TPS; //TPS DOT
    fullStatus[22] = (U8) Tuareg.ignition_timing.ignition_advance_deg;
    fullStatus[23] = (U8) Tuareg.process.TPS_deg; // TPS (0% to 100%)

    //Need to split the int loopsPerSecond value into 2 bytes
    fullStatus[24] = 0x42;
    fullStatus[25] = 0x01;

    //The following can be used to show the amount of free memory
    //not needed by now
    fullStatus[26] = 0x42; //lowByte(Tuareg.freeRAM);
    fullStatus[27] = 0x42; //highByte(Tuareg.freeRAM);

    fullStatus[28] = 0x42; //boost target not needed
    fullStatus[29] = 0x42; //boost duty not needed
    fullStatus[30] = Tuareg.spark; //Spark related bitfield

    //rpmDOT must be sent as a signed integer
    //fullStatus[31] = lowByte(Tuareg.decoder->crank_deltaT_us);
    //fullStatus[32] = highByte(Tuareg.decoder->crank_deltaT_us);
    fullStatus[31] = 0;
    fullStatus[32] = 0;

    if(length > 31)
    {
        fullStatus[33] = 0x42; //Tuareg.ethanolPct; //Flex sensor value (or 0 if not used)
        fullStatus[34] = 0x42; //Tuareg.flexCorrection; //Flex fuel correction (% above or below 100)
        fullStatus[35] = 0x42; //Tuareg.flexIgnCorrection; //Ignition correction (Increased degrees of advance) for flex fuel
        fullStatus[36] = 0x42; //getNextError();

        fullStatus[37] = 0x42; //Tuareg.idleLoad;
        fullStatus[38] = 0x42; //Tuareg.testOutputs;

        fullStatus[39] = (U8) Tuareg.sensors->asensors[ASENSOR_O2]; //O2
        fullStatus[40] = (U8) Tuareg.process.Baro_kPa; //Barometer value

        /**
        we do not use CAN
        but message bytes 41 to 73 are expected
        to be about CAN
        by now
        */

        fullStatus[41] = (U8) Tuareg.process.TPS_deg;
    }

    /**
    print the requested section
    but avoid memory access violation
    */

    if(length > SENDVALUE_FAKE_PACKETSIZE)
    {
        length= SENDVALUE_FAKE_PACKETSIZE;
    }

    if((offset + length) > SENDVALUE_FAKE_PACKETSIZE)
    {
        offset= SENDVALUE_FAKE_PACKETSIZE - length;
    }


    /**
    i is number of sent bytes
    take care for fake CAN section

    TODO
    once CAN has been removed we can ease this procedure a lot
    */
    for(i=0; i < length; i++)
    {
        if( (i + offset) < 41)
        {
            //live data
            UART_Tx(TS_PORT, fullStatus[i + offset]);
        }
        else if( (i + offset) < 74)
        {
            //this is the fake CAN section
            UART_Tx(TS_PORT, 0);
        }
        else if( (i + offset) == 74 )
        {
            //TPS
            UART_Tx(TS_PORT, fullStatus[41]);
        }
    }
}


/**
in system debug features

keep in mind that DEBUG_PORT might not always be connected while running diagnostic features!

FeatureID is a 16 Bit 0x00 .. 0xFFFF

*/

#warning TODO (oli#3#): Implement debug features
#warning TODO (oli#3#): Implement binary diag data printout

#define DEBUG_DATA_MAXLEN 25

void ts_debug_features(U32 FeatureID)
{
    U8 u8_data =0;
    U32 data_1 =0;
    U32 data_2 =0;

    U32 debug_data[DEBUG_DATA_MAXLEN];

    switch (FeatureID)
    {
        case 'dd':

            print_decoder_diag(TS_PORT);
            break;

        case 'dT':

            print_tuareg_diag(TS_PORT);
            break;

        case 'ep':

            /**
            dump eeprom content in binary format
            */

            UART_Send(TS_PORT, "EEprom data:\r\n");

            for(data_1=0; data_1 < EEPROM_STORAGE_END; data_1++)
            {
                data_2= eeprom_read_byte(data_1, &u8_data);

                if(data_2 != 0)
                {
                    //read error - terminate
                    UART_Send(TS_PORT, "ERROR!");
                    return;
                }

                UART_Tx(TS_PORT, u8_data);
            }

            UART_Send(TS_PORT, "Transmission complete.\r\n");

            break;

        case 'Pr':

            /**
            print current process data
            */
            ts_diag_process_data(&(Tuareg.process));

            break;

        case 'Ig':

            /**
            print current ignition setup
            */
            ts_diag_ignition_timing(&(Tuareg.ignition_timing));

            break;


        case 'sd':

            //print decoder statistics
            UART_Send(TS_PORT, "\r\ndecoder statistics:\r\n");

            //get diag data

            //decoder_export_statistics(debug_data);

            for(data_1=0; data_1 < CRK_POSITION_COUNT; data_1++)
            {
                UART_Print_U(TS_PORT, debug_data[data_1],TYPE_U16, NO_PAD);
            }

            UART_Send(TS_PORT, " rpm: ");

            UART_Print_U(TS_PORT, debug_data[CRK_POSITION_COUNT],TYPE_U16, NO_PAD);


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
#warning TODO (oli#8#): implement calibbration data binary printout for verification


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
            UART_Send(DEBUG_PORT, "\n Page has not been implemented yet. Change to another page.");

            /**
            lets trick the output logic
            */
            pnt_configPage= &configPage1;
            configPage_size= 0;
            break;
    }

#warning TODO (oli#1#): refactor!!!

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
    U32 sendComplete = FALSE;
    U32 x, i;
    U32 value;
    VU8 * currentVar;


    switch(TS_cli.currentPage)
    {
        case VEMAPPAGE:
            currentTable = &fuelTable;
            UART_Send(TS_PORT, "\r \n Volumetric Efficiency Map \r \n");
            break;

        case VESETPAGE:

            /**
            Display Values from Config Page 1
            */
            UART_Send(TS_PORT, "\r \n Page 1 Config \r \n");

            UART_Print_S(TS_PORT, configPage1.flexBoostLow, TYPE_S8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.flexBoostHigh, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.asePct, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.aseCount, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 10; i++)
            {
                UART_Print_U(TS_PORT, configPage1.wueValues[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage1.crankingPct, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.pinMapping, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.tachoPin, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.tdePct, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.taeColdA, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.tpsThresh, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.taeTime, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.displayType, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.display3, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.displayB1, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage1.reqFuel, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.divider, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.injTiming, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.injOpen, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.inj1Ang, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.inj2Ang, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.inj3Ang, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.inj4Ang, TYPE_U16, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage1.mapSample, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.cltType1, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.engineType, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.primePulse, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.dutyLim, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.flexFreqLow, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.flexFreqHigh, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.boostMaxDuty, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.tpsMin, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.tpsMax, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_S(TS_PORT, configPage1.mapMin, TYPE_S8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.mapMax, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.fpPrime, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.stoich, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.oddfire2, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.oddfire3, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.oddfire4, TYPE_U16, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage1.flexFuelLow, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.flexFuelHigh, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.flexAdvLow, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.flexAdvHigh, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.iacCLminDuty, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.iacCLmaxDuty, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage1.boostMinDuty, TYPE_U8, NO_PAD);

            sendComplete = TRUE;
            break;


        case IGNMAPPAGE:
            currentTable = &ignitionTable_TPS;
            UART_Send(TS_PORT, "\r \n Ignition Map \r \n");
            break;

        case IGNSETPAGE:

            /**
            Display Values from Config Page 2
            */
            UART_Send(TS_PORT, "\r \n Page 2 Config \r \n");

            UART_Print_U(TS_PORT, configPage2.triggerAngle, TYPE_U16, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.FixAng, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.CrankAng, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.TrigAngMul, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.TrigEdge, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.TrigEdgeSec, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.sparkDur, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.IdleAdvRPM, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.IdleAdvCLT, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.IdleDelayTime, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage2.StgCycles, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.dwellCont, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.dwellCrank, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.dwellRun, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.triggerTeeth, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.triggerMissingTeeth, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.crankRPM, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.floodClear, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.SoftRevLim, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.SoftLimRetard, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage2.SoftLimMax, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.HardRevLim, TYPE_U8, NO_PAD);

            for (i= 0; i < 4; i++)
            {
                UART_Print_U(TS_PORT, configPage2.taeBins[i], TYPE_U8, NO_PAD);
            }

            for (i= 0; i < 4; i++)
            {
                UART_Print_U(TS_PORT, configPage2.taeValues[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 10; i++)
            {
                UART_Print_U(TS_PORT, configPage2.wueBins[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage2.dwellLimit, TYPE_U8, NO_PAD);

            for (i= 0; i < 6; i++)
            {
                UART_Print_U(TS_PORT, configPage2.dwellCorrectionValues[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 6; i++)
            {
                UART_Print_U(TS_PORT, configPage2.iatRetBins[i], TYPE_U8, NO_PAD);
            }

            for (i= 0; i < 6; i++)
            {
                UART_Print_U(TS_PORT, configPage2.iatRetValues[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage2.dfcoRPM, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.dfcoHyster, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.dfcoTPSThresh, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage2.ignBypassEnabled, TYPE_U8, NO_PAD);

            sendComplete = TRUE;
            break;

        case AFRMAPPAGE:
            currentTable = &afrTable;
            UART_Send(TS_PORT, "\r \n Air/Fuel Ratio Map \r \n");
            break;

        case AFRSETPAGE:

            /**
            Display Values from Config Page 3
            */
            UART_Send(TS_PORT, "\r \n Page 3 Config \r \n");

            UART_Print_U(TS_PORT, configPage3.egoAlgorithm, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoKP, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoKI, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoKD, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoTemp, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoCount, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoDelta, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoLimit, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.ego_min, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.ego_max, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage3.ego_sdelay, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoRPM, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.egoTPSMax, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.vvtPin, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.boostPin, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 6; i++)
            {
                UART_Print_U(TS_PORT, configPage3.voltageCorrectionBins[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 6; i++)
            {
                UART_Print_U(TS_PORT, configPage3.injVoltageCorrectionValues[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 9; i++)
            {
                UART_Print_U(TS_PORT, configPage3.airDenBins[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            for (i= 0; i < 9; i++)
            {
                UART_Print_U(TS_PORT, configPage3.airDenRates[i], TYPE_U8, NO_PAD);
            }

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage3.boostFreq, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.vvtFreq, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.idleFreq, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.launchPin, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.lnchSoftLim, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.lnchRetard, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.lnchHardLim, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.lnchFuelAdd, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.idleKP, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.idleKI, TYPE_U8, NO_PAD);

            UART_Send(TS_PORT, "\r\n");

            UART_Print_U(TS_PORT, configPage3.idleKD, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.boostLimit, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.boostKP, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.boostKI, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.boostKD, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.lnchPullRes, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.flatSSoftWin, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.flatSRetard, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage3.flatSArm, TYPE_U8, NO_PAD);

            sendComplete = TRUE;
            break;

        case IACPAGE:

            /**
            Display Values from Config Page 4
            */
            UART_Send(TS_PORT, "\r \n Page 4 Config \r \n");

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
                    UART_Print_U(TS_PORT, currentVar[10 - x], TYPE_U8, NO_PAD);
                }

                UART_Send(TS_PORT, "\r\n");
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
                    UART_Print_U(TS_PORT, currentVar[4 - x], TYPE_U8, NO_PAD);
                }

                UART_Send(TS_PORT, "\r\n");
            }

            UART_Print_U(TS_PORT, configPage4.iacAlgorithm, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.iacFastTemp, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.iacStepHome, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.iacStepHyster, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.fanInv, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.fanSP, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.fanHyster, TYPE_U8, NO_PAD);
            UART_Print_U(TS_PORT, configPage4.fanFreq, TYPE_U8, NO_PAD);

            for (i= 0; i < 4; i++)
            {
                UART_Print_U(TS_PORT, configPage4.fanPWMBins[i], TYPE_U8, NO_PAD);
            }

            sendComplete = TRUE;
            break;


        case BOOSTVVCPAGE:
                currentTable= &boostTable;
                UART_Send(TS_PORT, "\r \n Boost Map \r \n");
                break;


        case SEQFUELPAGE:

                currentTable= &trim1Table;
                UART_Send(TS_PORT, "\r \n VVT Map \r \n");

                for (i = 0; i < TABLE3D_DIMENSION; i++)
                {
                    value= (U8) (currentTable->axisY[i]);
                    UART_Print_U(TS_PORT, value, TYPE_U8, NO_PAD);

                    for (x = 0; x < TABLE3D_DIMENSION; x++)
                    {
                        value= currentTable->axisZ[i][x];
                        UART_Print_U(TS_PORT, value, TYPE_U8, NO_PAD);
                    }

                    UART_Send(TS_PORT, "\r\n");
                }

                sendComplete = TRUE;
                break;

        case CALIBPAGE:

                ts_diagPage_calib();
                sendComplete = TRUE;
                break;


        case DECODERPAGE:

                ts_diagPage_decoder();
                sendComplete = TRUE;
                break;


        case IGNITIONPAGE:

                ts_diagPage_ignition();
                sendComplete = TRUE;
                break;


        case IGNITIONMAP_TPS:
            currentTable = &ignitionTable_TPS;
            UART_Send(TS_PORT, "\r \nIgnition Map TPS (in boctok 3D coordinate system)\r\n");
            break;

        case IGNITIONMAP_MAP:
            currentTable = &ignitionTable_MAP;
            UART_Send(TS_PORT, "\r \nIgnition Map MAP (in boctok 3D coordinate system)\r\n");
            break;


        default:
            UART_Send(TS_PORT, "\n Page has not been implemented yet. Change to another page.");
            sendComplete = TRUE;
            break;
    }

    if(!sendComplete)
    {
            //title already printed
            print_3D_table(TS_PORT, currentTable);

    }

}


void ts_diagPage_calib()
{
    /**
    Display Values from Config Page 9
    */
    UART_Send(TS_PORT, "\r\n\r\nsensors calibration:\r\n");

    /*
    IAT
    */
    UART_Send(TS_PORT, "\r\nIAT M N: ");
    UART_Print_F32(TS_PORT, configPage9.IAT_calib_M);
    UART_Print_F32(TS_PORT, configPage9.IAT_calib_N);



    /*
    CLT
    */
    UART_Send(TS_PORT, "\r\nCLT M N: ");
    UART_Print_F32(TS_PORT, configPage9.CLT_calib_M);
    UART_Print_F32(TS_PORT, configPage9.CLT_calib_N);


    /*
    TPS
    */
    UART_Send(TS_PORT, "\r\nTPS M N: ");
    UART_Print_F32(TS_PORT, configPage9.TPS_calib_M);
    UART_Print_F32(TS_PORT, configPage9.TPS_calib_N);

    /*
    MAP
    */
    UART_Send(TS_PORT, "\r\nMAP M N: ");
    UART_Print_F32(TS_PORT, configPage9.MAP_calib_M);
    UART_Print_F32(TS_PORT, configPage9.MAP_calib_N);


    /*
    BARO
    */
    UART_Send(TS_PORT, "\r\nBARO M N: ");
    UART_Print_F32(TS_PORT, configPage9.BARO_calib_M);
    UART_Print_F32(TS_PORT, configPage9.BARO_calib_N);


    /*
    O2
    */
    UART_Send(TS_PORT, "\r\nO2 M N: ");
    UART_Print_F32(TS_PORT, configPage9.O2_calib_M);
    UART_Print_F32(TS_PORT, configPage9.O2_calib_N);


    /*
    VBAT
    */
    UART_Send(TS_PORT, "\r\nVBAT M N: ");
    UART_Print_F32(TS_PORT, configPage9.VBAT_calib_M);
    UART_Print_F32(TS_PORT, configPage9.VBAT_calib_N);


    /*
    KNOCK
    */
    UART_Send(TS_PORT, "\r\nKNOCK M N: ");
    UART_Print_F32(TS_PORT, configPage9.KNOCK_calib_M);
    UART_Print_F32(TS_PORT, configPage9.KNOCK_calib_N);

}

void ts_diagPage_decoder()
{
    U32 i;

    /**
    U16 trigger_position_map[POSITION_COUNT];
    S16 decoder_offset_deg;
    U16 decoder_delay_us;
    U8 crank_noise_filter;
    U8 sync_ratio_min_pct;
    U8 sync_ratio_max_pct;
    U8 decoder_timeout_s;
    */

    /**
    Display Values from Config Page 12
    */
    UART_Send(TS_PORT, "\r\n\r\ndecoderpage:\r\n\r\n");

    /*
    trigger_position_map[CRK_POSITION_COUNT]
    */
    UART_Send(TS_PORT, "Trigger position map A1 .. D2: \r\n");

    for(i=0; i< CRK_POSITION_COUNT; i++)
    {
        UART_Print_U(TS_PORT, configPage12.trigger_position_map.a_deg[i], TYPE_U16, PAD);
    }

    //decoder_offset_deg
    UART_Send(TS_PORT, "\r\ndecoder offset (deg): ");
    UART_Print_S(TS_PORT, configPage12.decoder_offset_deg, TYPE_S16, NO_PAD);

    //decoder_delay_us
    UART_Send(TS_PORT, "\r\ndecoder delay (us): ");
    UART_Print_U(TS_PORT, configPage12.decoder_delay_us, TYPE_U16, NO_PAD);

    //crank_noise_filter
    UART_Send(TS_PORT, "\r\ncrank noise filter: ");
    UART_Print_U(TS_PORT, configPage12.crank_noise_filter, TYPE_U8, NO_PAD);

    //sync_ratio_min_pct
    UART_Send(TS_PORT, "\r\nsync ratio min (pct): ");
    UART_Print_U(TS_PORT, configPage12.sync_ratio_min_pct, TYPE_U8, NO_PAD);


    //sync_ratio_max_pct
    UART_Send(TS_PORT, "\r\nsync ratio max (pct): ");
    UART_Print_U(TS_PORT, configPage12.sync_ratio_max_pct, TYPE_U8, NO_PAD);

    //sync_stability_thrs
    UART_Send(TS_PORT, "\r\nsync stability thrs: ");
    UART_Print_U(TS_PORT, configPage12.sync_stability_thrs, TYPE_U8, NO_PAD);

    //decoder_timeout_s
    UART_Send(TS_PORT, "\r\ndecoder timeout (s): ");
    UART_Print_U(TS_PORT, configPage12.decoder_timeout_s, TYPE_U8, NO_PAD);
}

void ts_diagPage_ignition()
{
    /**
    U16 dynamic_min_rpm
    U16 dynamic_dwell_us
    U8 safety_margin_us
    crank_position_t idle_ignition_position
    crank_position_t idle_dwell_position
    U8 idle_advance_deg
    U8 idle_dwell_deg
    */

    /**
    Display Values from Config Page 13
    */
    UART_Send(TS_PORT, "\r\n\r\nignitionpage:\r\n");

    //dynamic_min_rpm
    UART_Send(TS_PORT, "\r\ndynamic min (rpm): ");
    UART_Print_U(TS_PORT, configPage13.dynamic_min_rpm, TYPE_U16, NO_PAD);

    //dynamic_dwell_us
    UART_Send(TS_PORT, "\r\ndynamic dwell (us): ");
    UART_Print_U(TS_PORT, configPage13.dynamic_dwell_us, TYPE_U16, NO_PAD);

    //safety_margin_us
    UART_Send(TS_PORT, "\r\nsafety margin (us): ");
    UART_Print_U(TS_PORT, configPage13.safety_margin_us, TYPE_U8, NO_PAD);

    //idle_ignition_position
    UART_Send(TS_PORT, "\r\nidle ignition position: ");
    UART_Print_U(TS_PORT, configPage13.idle_ignition_position, TYPE_U8, NO_PAD);

    //idle_dwell_position
    UART_Send(TS_PORT, "\r\nidle dwell position: ");
    UART_Print_U(TS_PORT, configPage13.idle_dwell_position, TYPE_U8, NO_PAD);

    //idle_advance_deg
    UART_Send(TS_PORT, "\r\nidle advance (deg): ");
    UART_Print_U(TS_PORT, configPage13.idle_advance_deg, TYPE_U8, NO_PAD);

    //idle_dwell_deg
    UART_Send(TS_PORT, "\r\nidle dwell (deg): ");
    UART_Print_U(TS_PORT, configPage13.idle_dwell_deg, TYPE_U8, NO_PAD);
}

void ts_diag_process_data(volatile process_data_t * pImage)
{
    VU32 i;
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

    UART_Send(TS_PORT, "\r\n\r\nprocess data image:\r\n");

    UART_Send(TS_PORT, "\r\ncrank position: ");
    UART_Print_U(TS_PORT, pImage->crank_position, TYPE_U8, NO_PAD);


    UART_Send(TS_PORT, "\r\ncrank position table: ");

    for(i=0; i< CRK_POSITION_COUNT; i++)
    {
        UART_Print_U(TS_PORT, pImage->crank_position_table.a_deg[i], TYPE_U16, PAD);
    }

    UART_Send(TS_PORT, "\r\ncrank rotational period: ");
    UART_Print_U(TS_PORT, pImage->crank_T_us, TYPE_U32, NO_PAD);

    UART_Send(TS_PORT, "rpm: ");
    UART_Print_U(TS_PORT, pImage->engine_rpm, TYPE_U32, NO_PAD);

    UART_Send(TS_PORT, "\r\nstrategy: ");
    UART_Print_U(TS_PORT, pImage->ctrl_strategy, TYPE_U8, NO_PAD);

    UART_Send(TS_PORT, "\r\nMAP (kPa), BARO (kPa), TPS (deg), ddt_TPS, IAT (C), CLT (C), VBAT (V): ");
    UART_Print_F32(TS_PORT, pImage->MAP_kPa);
    UART_Print_F32(TS_PORT, pImage->Baro_kPa);
    UART_Print_F32(TS_PORT, pImage->TPS_deg);
    UART_Print_F32(TS_PORT, pImage->ddt_TPS);
    UART_Print_F32(TS_PORT, pImage->IAT_K - cKelvin_offset);
    UART_Print_F32(TS_PORT, pImage->CLT_K - cKelvin_offset);
    UART_Print_F32(TS_PORT, pImage->VBAT_V);

}

void ts_diag_ignition_timing(volatile ignition_timing_t * pTiming)
{
    /**
    U16 ignition_advance_deg;
    U16 dwell_deg;
    U32 coil_dwell_timing_us;
    U32 coil_ignition_timing_us;
    crank_position_t coil_dwell_pos;
    crank_position_t coil_ignition_pos;
    */

    UART_Send(TS_PORT, "\r\n\r\nignition setup:\r\n");

    UART_Send(TS_PORT, "\r\nadvance (deg), position, timing (us): ");
    UART_Print_U(TS_PORT, pTiming->ignition_advance_deg, TYPE_U16, NO_PAD);
    UART_Print_U(TS_PORT, pTiming->coil_ignition_pos, TYPE_U32, NO_PAD);
    UART_Print_U(TS_PORT, pTiming->coil_ignition_timing_us, TYPE_U32, NO_PAD);

    UART_Send(TS_PORT, "\r\ndwell angle (deg), duration (ms), position, timing (us): ");
    UART_Print_U(TS_PORT, pTiming->dwell_deg, TYPE_U16, NO_PAD);
    UART_Print_U(TS_PORT, pTiming->dwell_ms, TYPE_U16, NO_PAD);
    UART_Print_U(TS_PORT, pTiming->coil_dwell_pos, TYPE_U32, NO_PAD);
    UART_Print_U(TS_PORT, pTiming->coil_dwell_timing_us, TYPE_U32, NO_PAD);

}







/**
a byte sent by TunerStudio shall replace our
current configuration value
*/
void ts_replaceConfig(U32 valueOffset, U32 newValue)
{

    volatile void* pnt_configPage;
    U32 tempOffset;

    if(TS_cli.State.mod_permission == FALSE)
    {
        UART_Send(DEBUG_PORT, "\r\n*** config modification rejected ***\r\n");
        return;
    }

    #ifdef TS_DEBUG
    #warning TODO (oli#1#): debug action enabled
    UART_Tx(DEBUG_PORT, '\r');
    UART_Tx(DEBUG_PORT, '\n');
    UART_Tx(DEBUG_PORT, 'r');
    UART_Print_U(DEBUG_PORT, valueOffset, TYPE_U32, NO_PAD);
    UART_Tx(DEBUG_PORT, '.');
    UART_Print_U(DEBUG_PORT, newValue, TYPE_U32, NO_PAD);
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
                #warning TODO (oli#1#): debug action enabled
                UART_Send(DEBUG_PORT, "\r\nIGN Z: ");
                UART_Print_U(DEBUG_PORT, valueOffset, TYPE_U32, NO_PAD);
                UART_Print_U(DEBUG_PORT, newValue, TYPE_U32, NO_PAD);
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
                #warning TODO (oli#1#): debug action enabled
                UART_Send(DEBUG_PORT, "\r\nIGN X: ");
                UART_Print_U(DEBUG_PORT, valueOffset, TYPE_U32, NO_PAD);
                UART_Print_U(DEBUG_PORT, newValue, TYPE_U32, NO_PAD);
                #endif //TS_DEBUG

                // the RPM values sent by megasquirt are divided by 100
                newValue *= TABLE_RPM_MULTIPLIER;

            }
            else if(valueOffset < TABLE3D_DIMENSION * TABLE3D_DIMENSION + 2 * TABLE3D_DIMENSION )
            {
                // Y-axis
                #ifdef TS_DEBUG
                #warning TODO (oli#1#): debug action enabled
                UART_Send(DEBUG_PORT, "\r\nIGN Y: ");
                UART_Print_U(DEBUG_PORT, valueOffset, TYPE_U32, NO_PAD);
                UART_Print_U(DEBUG_PORT, newValue, TYPE_U32, NO_PAD);
                #endif //TS_DEBUG

                // the RPM values sent by megasquirt are divided by 2
                newValue *= TABLE_LOAD_MULTIPLIER;
            }
            else
            {
                //invalid offset, do not attempt to modify anything
                #ifdef TS_DEBUG
                #warning TODO (oli#1#): debug action enabled
                UART_Send(DEBUG_PORT, "received invalid ignition bin: ");
                UART_Print_U(DEBUG_PORT, valueOffset, TYPE_U32, NO_PAD);
                UART_Print_U(DEBUG_PORT, newValue, TYPE_U32, NO_PAD);
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

            UART_Send(DEBUG_PORT, "\r\n*** use E command for calibration modification! ***\r\n");
            break;


        case DECODERPAGE:

            /**
            U16 trigger_position_map[CRK_POSITION_COUNT];
            S16 decoder_offset_deg;
            U16 decoder_delay_us;
            U8 crank_noise_filter;
            U8 sync_ratio_min_pct;
            U8 sync_ratio_max_pct;
            U8 sync_stability_thrs;
            U8 decoder_timeout_s;
            */

            switch(valueOffset)
            {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            case 5:
            case 6:
            case 7:

                configPage12.trigger_position_map.a_deg[valueOffset]= (U16) newValue;
                break;

            case 8:

                configPage12.decoder_offset_deg= (S16) newValue;
                break;

            case 9:

                configPage12.decoder_delay_us= (U16) newValue;
                break;

            case 10:

                configPage12.crank_noise_filter= (U8) newValue;
                break;

            case 11:

                configPage12.sync_ratio_min_pct= (U8) newValue;
                break;

            case 12:

                configPage12.sync_ratio_max_pct= (U8) newValue;
                break;

            case 13:

                configPage12.sync_stability_thrs= (U8) newValue;
                break;

            case 14:

                configPage12.decoder_timeout_s= (U8) newValue;
                break;

            default:
                break;

            }

            break;

        case IGNITIONPAGE:

            /**
            U16 dynamic_min_rpm;
            U16 dynamic_dwell_us;
            U8 safety_margin_us;
            crank_position_t idle_ignition_position;
            crank_position_t idle_dwell_position;
            U8 idle_advance_deg;
            U8 idle_dwell_deg;
            */

            switch(valueOffset)
            {
            case 0:

                configPage13.dynamic_min_rpm= (U16) newValue;
                break;

            case 1:

                configPage13.dynamic_dwell_us= (U16) newValue;
                break;

            case 2:

                configPage13.safety_margin_us= (U8) newValue;
                break;

            case 3:

                configPage13.idle_ignition_position= (crank_position_t) newValue;
                break;

            case 4:

                configPage13.idle_dwell_position= (crank_position_t) newValue;
                break;

            case 5:

                configPage13.idle_advance_deg= (U8) newValue;
                break;

            case 6:

                configPage13.idle_dwell_deg= (U8) newValue;
                break;

            default:
                break;

            }




            break;

        default:
            break;
  }
}

/**
replace a calibration value
*/
void replaceCalib(U32 Offset, U32 Value)
{

    if(TS_cli.State.calibmod_permission == FALSE)
    {
        UART_Send(DEBUG_PORT, "\r\n*** calibration modification rejected (permission) ***\r\n");
        return;
    }

    #ifdef TS_DEBUG
    #warning TODO (oli#1#): debug action enabled
    UART_Tx(DEBUG_PORT, '\r');
    UART_Tx(DEBUG_PORT, '\n');
    UART_Tx(DEBUG_PORT, 'E');
    UART_Print_U(DEBUG_PORT, valueOffset, TYPE_U32, NO_PAD);
    UART_Tx(DEBUG_PORT, '.');
    UART_Print_U(DEBUG_PORT, newValue, TYPE_U32, NO_PAD);
    #endif //TS_DEBUG

    /**
    calibration layout:

    float IAT_calib_M;
    float IAT_calib_N;

    float CLT_calib_M;
    float CLT_calib_N;

    float TPS_calib_M;
    float TPS_calib_N;

    float MAP_calib_M;
    float MAP_calib_N;

    float BARO_calib_M;
    float BARO_calib_N;

    float O2_calib_M;
    float O2_calib_N;

    float VBAT_calib_M;
    float VBAT_calib_N;

    float KNOCK_calib_M;
    float KNOCK_calib_N;
    */

    switch(Offset)
    {
    case CALIB_OS_IAT_M:

        configPage9.IAT_calib_M= compose_float(Value);
        break;

    case CALIB_OS_IAT_N:

        configPage9.IAT_calib_N= compose_float(Value);
        break;


    case CALIB_OS_CLT_M:

        configPage9.CLT_calib_M= compose_float(Value);
        break;

    case CALIB_OS_CLT_N:

        configPage9.CLT_calib_N= compose_float(Value);
        break;


    case CALIB_OS_TPS_M:

        configPage9.TPS_calib_M= compose_float(Value);
        break;

    case CALIB_OS_TPS_N:

        configPage9.TPS_calib_N= compose_float(Value);
        break;


    case CALIB_OS_MAP_M:

        configPage9.MAP_calib_M= compose_float(Value);
        break;

    case CALIB_OS_MAP_N:

        configPage9.MAP_calib_N= compose_float(Value);
        break;


    case CALIB_OS_BARO_M:

        configPage9.BARO_calib_M= compose_float(Value);
        break;

    case CALIB_OS_BARO_N:

        configPage9.BARO_calib_N= compose_float(Value);
        break;


    case CALIB_OS_O2_M:

        configPage9.O2_calib_M= compose_float(Value);
        break;

    case CALIB_OS_O2_N:

        configPage9.O2_calib_N= compose_float(Value);
        break;


    case CALIB_OS_VBAT_M:

        configPage9.VBAT_calib_M= compose_float(Value);
        break;

    case CALIB_OS_VBAT_N:

        configPage9.VBAT_calib_N= compose_float(Value);
        break;


    case CALIB_OS_KNOCK_M:

        configPage9.KNOCK_calib_M= compose_float(Value);
        break;

    case CALIB_OS_KNOCK_N:

        configPage9.KNOCK_calib_N= compose_float(Value);
        break;

    default:
        UART_Send(DEBUG_PORT, "\r\n*** calibration modification rejected (invalid offset) ***\r\n");
        break;

    }

}

