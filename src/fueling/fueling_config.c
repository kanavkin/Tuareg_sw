#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "base_calc.h"
#include "table.h"
#include "eeprom.h"
#include "eeprom_layout.h"

#include "eeprom_layout.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "fueling_config.h"

///the Fueling Setup Page
volatile Fueling_Setup_t Fueling_Setup;

/*
// Fueling Tables
volatile t3D_t ignAdvTable_TPS;
volatile t3D_t ignAdvTable_MAP;
*/

volatile U8 * const pFueling_Setup_data= (volatile U8 *) &Fueling_Setup;
const U32 cFueling_Setup_size= sizeof(Fueling_Setup);



/**
*
* reads Fueling config (setup and tables) data from eeprom
*
*/
exec_result_t load_Fueling_Config()
{
    /*
   exec_result_t load_result;

   load_result= Eeprom_load_data(EEPROM_Fueling_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);

   ASSERT_EXEC_OK(load_result);

   load_result= load_t3D_data(&(ignAdvTable_TPS.data), EEPROM_Fueling_ADVTPS_BASE);
   ignAdvTable_TPS.mgr.div_X_lookup= 1;
   ignAdvTable_TPS.mgr.div_Y_lookup= 1;

   return load_result;
   */
   return EXEC_ERROR;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Fueling_Config()
{



}


/***************************************************************************************************************************************************
*   Fueling Setup
***************************************************************************************************************************************************/


/**
*
* writes Fueling config data to eeprom
*
*/
exec_result_t store_Fueling_Setup()
{
    //return Eeprom_update_data(EEPROM_Fueling_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);
    return EXEC_ERROR;
}


void show_Fueling_Setup(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling Config:");
/*
    //max_rpm
    print(Port, "\r\nrev limiter (rpm): ");
    printf_U(Port, Fueling_Setup.max_rpm, NO_PAD);


    //dynamic_min_rpm
    print(Port, "\r\ndynamic Fueling function minimum rpm: ");
    printf_U(Port, Fueling_Setup.dynamic_min_rpm, NO_PAD);

    //dynamic_Fueling_base_position
    print(Port, "\r\ndynamic Fueling base position: ");
    printf_crkpos(Port, Fueling_Setup.dynamic_Fueling_base_position);

    //dynamic_dwell_target_us
    print(Port, "\r\ndynamic dwell target (us): ");
    printf_U(Port, Fueling_Setup.dynamic_dwell_target_us, NO_PAD);


    //cold_idle_cutoff_rpm
    print(Port, "\r\ncold idle cutoff (rpm): ");
    printf_U(Port, Fueling_Setup.cold_idle_cutoff_rpm, NO_PAD);

    //cold_idle_cutoff_CLT_K
    print(Port, "\r\ncold idle cutoff CLT (K): ");
    printf_U(Port, Fueling_Setup.cold_idle_cutoff_CLT_K, NO_PAD);

    //cold_idle_Fueling_advance_deg
    print(Port, "\r\ncold idle Fueling advance (deg): ");
    printf_U(Port, Fueling_Setup.cold_idle_Fueling_advance_deg, NO_PAD);

    //cold_idle_dwell_target_us
    print(Port, "\r\ncold idle dwell target (us): ");
    printf_U(Port, Fueling_Setup.cold_idle_dwell_target_us, NO_PAD);


    //cranking_Fueling_position
    print(Port, "\r\ncranking Fueling position: ");
    printf_crkpos(Port, Fueling_Setup.cranking_Fueling_position);

    //cranking_dwell_position
    print(Port, "\r\ncranking dwell position: ");
    printf_crkpos(Port, Fueling_Setup.cranking_dwell_position);


    //coil_setup_t coil_setup
    print(Port, "\r\ncoil setup: ");
    printf_U(Port, Fueling_Setup.coil_setup, NO_PAD);

    //U16 spark_duration_us
    print(Port, "\r\nspark duration (us): ");
    printf_U(Port, Fueling_Setup.spark_duration_us, NO_PAD);
    */
}


/**
replace an Fueling configuration value
*/
exec_result_t modify_Fueling_Setup(U32 Offset, U32 Value)
{
    if(Offset >= cFueling_Setup_size)
    {
        return EXEC_ERROR;
    }

    *(pFueling_Setup_data + Offset)= (U8) Value;

    return EXEC_OK;

}

/**
this function implements the TS interface binary config page read command for Fueling Config
*/
void send_Fueling_Setup(USART_TypeDef * Port)
{
    UART_send_data(Port, pFueling_Setup_data, cFueling_Setup_size);
}



/***************************************************************************************************************************************************
*   Fueling Advance Table (TPS)
***************************************************************************************************************************************************/
/*

exec_result_t store_ignAdvTable_TPS()
{
    return store_t3D_data(&(ignAdvTable_TPS.data), EEPROM_Fueling_ADVTPS_BASE);
}


void show_ignAdvTable_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling Advance Table (TPS):\r\n");

    show_t3D_data(TS_PORT, &(ignAdvTable_TPS.data));
}


exec_result_t modify_ignAdvTable_TPS(U32 Offset, U32 Value)
{
    //modify_t3D_data provides offset range check!
    return modify_t3D_data(&(ignAdvTable_TPS.data), Offset, Value);
}
*/

/**
this function implements the TS interface binary config page read command for ignAdvTable_TPS
*/
/*
void send_ignAdvTable_TPS(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(ignAdvTable_TPS.data));
}


VU32 getValue_ignAdvTable_TPS(VU32 Rpm, VF32 TPS)
{
    return (VU32) getValue_t3D(&ignAdvTable_TPS, divide_VU32(Rpm, ignAdvTable_TPS.mgr.div_X_lookup), TPS);
}

*/
