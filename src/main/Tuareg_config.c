

#include "table.h"
#include "eeprom.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


///the decoder configuration page
volatile Tuareg_Setup_t Tuareg_Setup;

volatile U8 * const pTuareg_Setup_data= (volatile U8 *) &Tuareg_Setup;
const U32 cTuareg_Setup_size= sizeof(Tuareg_Setup);


/**
*
* reads decoder config data from eeprom
*
*/
exec_result_t load_Tuareg_Setup()
{
   return Eeprom_load_data(EEPROM_TUAREG_CONFIG_BASE, pTuareg_Setup_data, cTuareg_Setup_size);
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
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

    Tuareg_Setup.flags.all_flags=0;

}

/**
*
* writes decoder config data to eeprom
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

    //flags
    print(Port, "\r\nfeature enabled flags: CrashSenPol-RunSenPol-SidestandSenPol-HaltOnSidestand: ");

    UART_Tx(TS_PORT, (Tuareg_Setup.flags.CrashSensor_trig_high? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.RunSwitch_trig_high? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.SidestandSensor_trig_high? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Tuareg_Setup.flags.Halt_on_SidestandSensor? '1' :'0'));
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


