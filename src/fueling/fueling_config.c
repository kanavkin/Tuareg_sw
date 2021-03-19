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


// Fueling Tables
volatile t3D_t VeTable_TPS;
volatile t3D_t VeTable_MAP;
volatile t3D_t AfrTable_TPS;


volatile U8 * const pFueling_Setup_data= (volatile U8 *) &Fueling_Setup;
const U32 cFueling_Setup_size= sizeof(Fueling_Setup);



/**
*
* reads Fueling config (setup and tables) data from eeprom
*
*/
exec_result_t load_Fueling_Config()
{

   exec_result_t load_result;

  // load_result= Eeprom_load_data(EEPROM_Fueling_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);

//   ASSERT_EXEC_OK(load_result);

   load_result= load_t3D_data(&(VeTable_TPS.data), EEPROM_FUELING_VETPS_BASE);
   VeTable_TPS.mgr.div_X_lookup= 0;
   VeTable_TPS.mgr.div_Y_lookup= 0;

   //   ASSERT_EXEC_OK(load_result);

   load_result= load_t3D_data(&(VeTable_MAP.data), EEPROM_FUELING_VEMAP_BASE);
   VeTable_MAP.mgr.div_X_lookup= 0;
   VeTable_MAP.mgr.div_Y_lookup= 0;

    //   ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(AfrTable_TPS.data), EEPROM_FUELING_AFRTPS_BASE);
   AfrTable_TPS.mgr.div_X_lookup= 0;
   AfrTable_TPS.mgr.div_Y_lookup= 0;



   /*
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
*   Fueling VE Table (TPS) - VeTable_TPS
***************************************************************************************************************************************************/

exec_result_t store_VeTable_TPS()
{
    return store_t3D_data(&(VeTable_TPS.data), EEPROM_FUELING_VETPS_BASE);
}


void show_VeTable_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling VE Table (TPS based):\r\n");

    show_t3D_data(TS_PORT, &(VeTable_TPS.data));
}


exec_result_t modify_VeTable_TPS(U32 Offset, U32 Value)
{
    //modify_t3D_data provides offset range check!
    return modify_t3D_data(&(VeTable_TPS.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for VeTable_TPS
*/
void send_VeTable_TPS(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(VeTable_TPS.data));
}


/**
returns the volumetric efficiency in percent
*/
VF32 getValue_VeTable_TPS(VU32 Rpm, VF32 Tps_deg)
{
    return getValue_t3D(&VeTable_TPS, Rpm, Tps_deg);
}



/***************************************************************************************************************************************************
*   Fueling VE Table (MAP based) - VeTable_MAP
***************************************************************************************************************************************************/

exec_result_t store_VeTable_MAP()
{
    return store_t3D_data(&(VeTable_MAP.data), EEPROM_FUELING_VEMAP_BASE);
}


void show_VeTable_MAP(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling VE Table (MAP based):\r\n");

    show_t3D_data(TS_PORT, &(VeTable_MAP.data));
}


exec_result_t modify_VeTable_MAP(U32 Offset, U32 Value)
{
    //modify_t3D_data provides offset range check!
    return modify_t3D_data(&(VeTable_MAP.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for VeTable_MAP
*/
void send_VeTable_MAP(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(VeTable_MAP.data));
}


/**
returns the volumetric efficiency in percent
*/
VF32 getValue_VeTable_MAP(VU32 Rpm, VF32 Map_kPa)
{
    return getValue_t3D(&VeTable_MAP, Rpm, Map_kPa);
}



/***************************************************************************************************************************************************
*   Fueling AFR target Table (TPS based) - AfrTable_TPS
***************************************************************************************************************************************************/

exec_result_t store_AfrTable_TPS()
{
    return store_t3D_data(&(AfrTable_TPS.data), EEPROM_FUELING_AFRTPS_BASE);
}


void show_AfrTable_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling AFR target Table (TPS based) -  x10\r\n");

    show_t3D_data(TS_PORT, &(AfrTable_TPS.data));
}


exec_result_t modify_AfrTable_TPS(U32 Offset, U32 Value)
{
    //modify_t3D_data provides offset range check!
    return modify_t3D_data(&(AfrTable_TPS.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AfrTable_TPS
*/
void send_AfrTable_TPS(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(AfrTable_TPS.data));
}

/**
returns the target AFR value
(multiplied by 10 for decimal place storage in table)
*/
VF32 getValue_AfrTable_TPS(VU32 Rpm, VF32 Tps_deg)
{
    return (getValue_t3D(&AfrTable_TPS, Rpm, Tps_deg) / 10);
}






