

#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
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
    U32 pos;

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


