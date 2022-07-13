

#include "table.h"
#include "eeprom.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


///Tuareg main configuration page
volatile Tuareg_Setup_t Tuareg_Setup;

volatile U8 * const pTuareg_Setup_data= (volatile U8 *) &Tuareg_Setup;
const U32 cTuareg_Setup_size= sizeof(Tuareg_Setup);

//tach table
volatile t2D_t TachTable;



/**
*
* reads Tuareg main config data from eeprom
*
*/
exec_result_t load_Tuareg_Setup()
{
   return Eeprom_load_data(EEPROM_TUAREG_CONFIG_BASE, pTuareg_Setup_data, cTuareg_Setup_size);
}


/**
*
* provides built in defaults if config data from eeprom is not available
*
*/
void load_essential_Tuareg_Setup()
{
    Tuareg_Setup.Version= 0;

    /**
    trigger position map initialization
    */
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_A1]= TUAREG_SETUP_DEFAULT_POSITION_A1_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_A2]= TUAREG_SETUP_DEFAULT_POSITION_A2_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_B1]= TUAREG_SETUP_DEFAULT_POSITION_B1_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_B2]= TUAREG_SETUP_DEFAULT_POSITION_B2_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_C1]= TUAREG_SETUP_DEFAULT_POSITION_C1_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_C2]= TUAREG_SETUP_DEFAULT_POSITION_C2_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_D1]= TUAREG_SETUP_DEFAULT_POSITION_D1_ADVANCE;
    Tuareg_Setup.trigger_advance_map[CRK_POSITION_D2]= TUAREG_SETUP_DEFAULT_POSITION_D2_ADVANCE;

    Tuareg_Setup.decoder_delay_us= TUAREG_SETUP_DEFAULT_DECODER_DELAY;

    Tuareg_Setup.max_rpm= 0;
    Tuareg_Setup.limp_max_rpm= TUAREG_SETUP_DEFAULT_MAX_RPM;

    Tuareg_Setup.overheat_thres_K= 100 + cKelvin_offset;

    Tuareg_Setup.standby_timeout_s= 5;

    Tuareg_Setup.cranking_end_rpm= 800;

    Tuareg_Setup.gear_ratio[GEAR_1]= 0.0;
    Tuareg_Setup.gear_ratio[GEAR_2]= 0.0;
    Tuareg_Setup.gear_ratio[GEAR_3]= 0.0;
    Tuareg_Setup.gear_ratio[GEAR_4]= 0.0;
    Tuareg_Setup.gear_ratio[GEAR_5]= 0.0;
    Tuareg_Setup.gear_ratio[GEAR_NEUTRAL]= 0.0;

    Tuareg_Setup.TPS_alpha= 0.85;
    Tuareg_Setup.MAP_alpha= 0.85;

    Tuareg_Setup.flags.all_flags=0;

}


/**
*
* check Tuareg config data integrity to prevent writing bogus data to storage
*
*/
exec_result_t check_Tuareg_Setup()
{
/*
    VU32 i;


    //VU16 trigger_advance_map[CRK_POSITION_COUNT]
    for(i=0; i < CRK_POSITION_COUNT; i++)
    {
        if(Tuareg_Setup.trigger_advance_map[i] > 360)
        {
            return EXEC_ERROR;
        }
    }

    //U16 decoder_delay_us
    if(Tuareg_Setup.decoder_delay_us > 500)
    {
        return EXEC_ERROR;
    }

    //rev limiter function
    U16 max_rpm;
    U16 limp_max_rpm;

    //overheat protection
    U16 overheat_thres_K;

    //standby timeout
    U8 standby_timeout_s;

    //rpm until cranking features are activated
    U16 cranking_end_rpm;

    //conversion factors for ground speed calculation
    VF32 gear_ratio[GEAR_COUNT];

    //EMA filter factors
    VF32 TPS_alpha;
    VF32 MAP_alpha;

    //all boolean elements
    volatile Tuareg_Setup_flags_t flags;




*/

    return EXEC_ERROR;
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
    print(Port, "\r\ncranking features turn off rpm: ");
    printf_U(Port, Tuareg_Setup.cranking_end_rpm, NO_PAD);

    /*
    gear_ratio[GEAR_COUNT]
    */
    print(Port, "\r\ngear ratio map\r\n");

    for(gear=0; gear< GEAR_COUNT; gear++)
    {
        printf_U(Port, Tuareg_Setup.gear_ratio[gear], PAD_2 | NO_TRAIL);
        UART_Tx(Port, ':');
        printf_F32(Port, Tuareg_Setup.gear_ratio[gear]);
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
    return load_t2D_data(&(TachTable.data), EEPROM_TACHTABLE_BASE);
}

exec_result_t store_TachTable()
{
    return store_t2D_data(&(TachTable.data), EEPROM_TACHTABLE_BASE);
}


void show_TachTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nTachometer output table:\r\n");

    show_t2D_data(TS_PORT, &(TachTable.data));
}


exec_result_t modify_TachTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(TachTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for TachTable
*/
void send_TachTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(TachTable.data));
}


/**
returns the timer compare value # that makes the tachometer display the commanded speed
*/
U32 getValue_TachTable(U32 Rpm)
{
    return getValue_t2D(&TachTable, Rpm);
}

