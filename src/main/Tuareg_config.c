/*
#include "map.h"
#include "table.h"
#include "ctrlset.h"
#include "eeprom.h"
#include "eeprom_layout.h"
*/

#include "Tuareg.h"

/*
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
*/



/**
Tuareg main configuration page
*/
volatile Tuareg_Setup_t Tuareg_Setup;

volatile U8 * const pTuareg_Setup_data= (volatile U8 *) &Tuareg_Setup;
const U32 cTuareg_Setup_size= sizeof(Tuareg_Setup);

//tach table
volatile table_t TachTable;



/**
*
* loads all Tuareg main config data from eeprom
*
*/
exec_result_t load_Tuareg_Config()
{
    //bring up eeprom
    Eeprom_init();

    //load setup
    ASSERT_EXEC_OK( Eeprom_load_data(EEPROM_TUAREG_CONFIG_BASE, pTuareg_Setup_data, cTuareg_Setup_size) );

    //load tachometer output table
    ASSERT_EXEC_OK( load_TachTable() );

    return EXEC_OK;
}


/**
*
* writes Tuareg config data to eeprom
*
*/
exec_result_t store_Tuareg_Setup()
{
    return Eeprom_update_data(EEPROM_TUAREG_CONFIG_BASE, pTuareg_Setup_data, cTuareg_Setup_size);
}


void show_Tuareg_Setup(USART_TypeDef * Port)
{
    U32 pos, gear;

    print(Port, "\r\n\r\nTuareg Config:");

    /*
    Version
    */
    print(Port, "\r\nVersion: ");
    printf_U(Port, Tuareg_Setup.Version, NO_PAD | NO_TRAIL);

    /*
    trigger_advance_map[CRK_POSITION_COUNT]
    */
    print(Port, "\r\nTrigger advance map\r\n");

    for(pos=0; pos< CRK_POSITION_COUNT; pos++)
    {
        printf_crkpos(Port, pos);
        UART_Tx(Port, ':');
        printf_U(Port, Tuareg_Setup.trigger_advance_map[pos], NO_PAD);
    }

    print(Port, "\r\nDecoder Delay (us): ");
    printf_U(Port, Tuareg_Setup.decoder_delay_us, NO_PAD);

    //max_rpm
    print(Port, "\r\nrev limiter turn on rpm in normal and limp mode: ");
    printf_U(Port, Tuareg_Setup.max_rpm, NO_PAD);
    printf_U(Port, Tuareg_Setup.limp_max_rpm, NO_PAD);

    //overheat protection
    print(Port, "\r\noverheat protection turn on temperature (K): ");
    printf_U(Port, Tuareg_Setup.overheat_thres_K, NO_PAD);

    //standby timeout
    print(Port, "\r\ntime out for standby (s): ");
    printf_U(Port, Tuareg_Setup.standby_timeout_s, NO_PAD);

    //rpm until cranking features are activated
//    print(Port, "\r\ncranking features turn off rpm: ");
//    printf_U(Port, Tuareg_Setup.cranking_end_rpm, NO_PAD);

    //U16 spd_min_rpm
   // print(Port, "\r\nSpeed Density min (rpm):");
    //printf_U(Port, Fueling_Setup.spd_min_rpm, NO_PAD);

    //U16 spd_max_rpm
    print(Port, "\r\nSpeed Density max rpm:");
    printf_U(Port, Tuareg_Setup.spd_max_rpm, NO_PAD);


    /*
    gear_ratio[GEAR_COUNT]
    */
    print(Port, "\r\ngear ratio map\r\n");

    for(gear=GEAR_1; gear < GEAR_COUNT; gear++)
    {
        printf_U(Port, gear, PAD_2 | NO_TRAIL);
        UART_Tx(Port, ':');
        printf_F32(Port, Tuareg_Setup.gear_ratio[gear -1]);
    }


    /*
    filter parameters
    */
    print(Port, "\r\nTPS filter coefficient: ");
    printf_F32(Port, Tuareg_Setup.TPS_alpha);

    print(Port, "\r\nMAP filter coefficient: ");
    printf_F32(Port, Tuareg_Setup.MAP_alpha);


    //flags
    print(Port, "\r\nfeature enabled flags: CrashPol-SidesPol-RunSwOver-SidesOver-CrashOver: ");

    UART_Tx(TS_PORT, (Tuareg_Setup.flags.CrashSensor_trig_high? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.SidestandSensor_trig_high? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.RunSwitch_override? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.Sidestand_override? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.CrashSensor_override? '1' :'0'));
}


/**
replace a decoder configuration value
*/
exec_result_t modify_Tuareg_Setup(U32 Offset, U32 Value)
{
    if(Offset >= cTuareg_Setup_size)
    {
        return EXEC_ERROR;
    }

    *(pTuareg_Setup_data + Offset)= (U8) Value;

    return EXEC_OK;
}


/**
this function implements the TS interface binary config page read command for Decoder Config
*/
void send_Tuareg_Setup(USART_TypeDef * Port)
{
    UART_send_data(Port, pTuareg_Setup_data, cTuareg_Setup_size);
}

/***************************************************************************************************************************************************
*   Tachometer output table - TachTable
*
* x-Axis -> tachometer reading to be displayed in rpm (no offset, no scaling)
* y-Axis -> timer compare value in % (no offset, no scaling)
***************************************************************************************************************************************************/

exec_result_t load_TachTable()
{
    return load_table(&TachTable, EEPROM_TACHTABLE_BASE);
}

exec_result_t store_TachTable()
{
    return store_table(&TachTable, EEPROM_TACHTABLE_BASE);
}


void show_TachTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nTachometer output table:\r\n");

    show_table(TS_PORT, &TachTable);
}


exec_result_t modify_TachTable(U32 Offset, U32 Value)
{
    //modify_table provides offset range check!
    return modify_table(&TachTable, Offset, Value);
}


/**
this function implements the TS interface binary config page read command for TachTable
*/
void send_TachTable(USART_TypeDef * Port)
{
    send_table(Port, &TachTable);
}


/**
returns the timer compare value # that makes the tachometer display the commanded speed
*/
U32 getValue_TachTable(U32 Rpm)
{
    return getValue_table(&TachTable, Rpm);
}

