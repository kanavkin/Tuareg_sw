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

volatile t2D_t AccelCompTable;
volatile t2D_t WarmUpCompTable;

volatile t2D_t InjectorTimingTable;

volatile t2D_t CrankingFuelTable;


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

    load_result= Eeprom_load_data(EEPROM_FUELING_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(VeTable_TPS.data), EEPROM_FUELING_VETPS_BASE);
    VeTable_TPS.mgr.div_X_lookup= 0;
    VeTable_TPS.mgr.div_Y_lookup= 0;
    VeTable_TPS.mgr.mul_Res= 0;

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(VeTable_MAP.data), EEPROM_FUELING_VEMAP_BASE);
    VeTable_MAP.mgr.div_X_lookup= 0;
    VeTable_MAP.mgr.div_Y_lookup= 0;
    VeTable_MAP.mgr.mul_Res= 0;

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(AfrTable_TPS.data), EEPROM_FUELING_AFRTPS_BASE);
    AfrTable_TPS.mgr.div_X_lookup= 0;
    AfrTable_TPS.mgr.div_Y_lookup= 0;
    AfrTable_TPS.mgr.mul_Res= 0;

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(AccelCompTable.data), EEPROM_FUELING_ACCELCOMP_BASE);
    AccelCompTable.mgr.div_X_lookup= 0;
    AccelCompTable.mgr.div_Y_lookup= 0;
    AccelCompTable.mgr.mul_Res= 0;

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(WarmUpCompTable.data), EEPROM_FUELING_WARMUPCOMP_BASE);
    WarmUpCompTable.mgr.div_X_lookup= 0;
    WarmUpCompTable.mgr.div_Y_lookup= 0;
    WarmUpCompTable.mgr.mul_Res= 0;

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(InjectorTimingTable.data), EEPROM_FUELING_INJECTORTIMING_BASE);
    InjectorTimingTable.mgr.div_X_lookup= 0;
    InjectorTimingTable.mgr.div_Y_lookup= 0;
    InjectorTimingTable.mgr.mul_Res= 0;

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(CrankingFuelTable.data), EEPROM_FUELING_CRANKINGTABLE_BASE);
    CrankingFuelTable.mgr.div_X_lookup= 0;
    CrankingFuelTable.mgr.div_Y_lookup= 0;
    CrankingFuelTable.mgr.mul_Res= 0;

    return load_result;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Fueling_Config()
{
    /*U16 ve_from_map_min_rpm= 2000;
    U16 ve_from_map_max_rpm= 7000;

    U16 cylinder_volume_ccm= 425;

    U16 injector1_rate_mgps= 4160;
    U16 injector2_rate_mgps= 4160;

    U16 injector_deadtime_us= 500;
    U8 max_injector_duty_cycle_pct= 85;
    */


    Fueling_Setup.Version =0;

    Fueling_Setup.ve_from_map_min_rpm= 0;
    Fueling_Setup.ve_from_map_max_rpm= 0;

    Fueling_Setup.cylinder_volume_ccm= 0;

    Fueling_Setup.injector1_rate_mgps= 0;
    Fueling_Setup.injector2_rate_mgps= 0;

    Fueling_Setup.injector_deadtime_us= 0;
    Fueling_Setup.max_injector_duty_cycle_pct= 0;
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
    return Eeprom_update_data(EEPROM_FUELING_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);
}


void show_Fueling_Setup(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling Config:");

    //U8 Version
    print(Port, "\r\nVersion: ");
    printf_U(Port, Fueling_Setup.Version, NO_PAD);

    //U16 ve_from_map_min_rpm
    print(Port, "\r\nVE from MAP min (rpm):");
    printf_U(Port, Fueling_Setup.ve_from_map_min_rpm, NO_PAD);

    //U16 ve_from_map_max_rpm
    print(Port, "\r\nVE from MAP max (rpm):");
    printf_U(Port, Fueling_Setup.ve_from_map_max_rpm, NO_PAD);

    //U16 cylinder_volume_ccm
    print(Port, "\r\ncylinder volume (ccm):");
    printf_U(Port, Fueling_Setup.cylinder_volume_ccm, NO_PAD);

    //U16 injector1_rate_mgps
    print(Port, "\r\ninjector #1 flow rate (mg/s):");
    printf_U(Port, Fueling_Setup.injector1_rate_mgps, NO_PAD);

    //U16 injector2_rate_mgps
    print(Port, "\r\ninjector #2 flow rate (mg/s):");
    printf_U(Port, Fueling_Setup.injector2_rate_mgps, NO_PAD);

    //U16 injector_deadtime_us
    print(Port, "\r\ninjector dead time (us):");
    printf_U(Port, Fueling_Setup.injector_deadtime_us, NO_PAD);

    //U16 max_injector_duty_cycle_pct
    print(Port, "\r\ninjector max duty cycle (%):");
    printf_U(Port, Fueling_Setup.max_injector_duty_cycle_pct, NO_PAD);
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



/***************************************************************************************************************************************************
*   Fueling acceleration compensation table - AccelCompTable
***************************************************************************************************************************************************/

exec_result_t store_AccelCompTable()
{
    return store_t2D_data(&(AccelCompTable.data), EEPROM_FUELING_ACCELCOMP_BASE);
}


void show_AccelCompTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling acceleration compensation table:\r\n");

    show_t2D_data(TS_PORT, &(AccelCompTable.data));
}


exec_result_t modify_AccelCompTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(AccelCompTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AccelCompTable
*/
void send_AccelCompTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(AccelCompTable.data));
}


/**
returns the acceleration compensation value in percent
*/
VF32 getValue_AccelCompTable(VF32 Ddt_TPS)
{
    return getValue_t2D(&AccelCompTable, Ddt_TPS);
}


/***************************************************************************************************************************************************
*   Fueling Warm up Enrichment compensation table - WarmUpCompTable
***************************************************************************************************************************************************/

exec_result_t store_WarmUpCompTable()
{
    return store_t2D_data(&(WarmUpCompTable.data), EEPROM_FUELING_WARMUPCOMP_BASE);
}


void show_WarmUpCompTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nWarm up Enrichment table (%):\r\n");

    show_t2D_data(TS_PORT, &(WarmUpCompTable.data));
}


exec_result_t modify_WarmUpCompTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(WarmUpCompTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for WarmUpCompTable
*/
void send_WarmUpCompTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(WarmUpCompTable.data));
}


/**
returns the Warm up Enrichment compensation in percent
*/
VF32 getValue_WarmUpCompTable(VF32 CLT_K)
{
    return getValue_t2D(&WarmUpCompTable, CLT_K);
}


/***************************************************************************************************************************************************
*   Injector dead time table - InjectorTimingTable
***************************************************************************************************************************************************/

exec_result_t store_InjectorTimingTable()
{
    return store_t2D_data(&(InjectorTimingTable.data), EEPROM_FUELING_INJECTORTIMING_BASE);
}


void show_InjectorTimingTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nInjector timing table:\r\n");

    show_t2D_data(TS_PORT, &(InjectorTimingTable.data));
}


exec_result_t modify_InjectorTimingTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(InjectorTimingTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for InjectorTimingTable
*/
void send_InjectorTimingTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(InjectorTimingTable.data));
}


/**
returns the injector dead time in 24 us intervals
*/
VF32 getValue_InjectorTimingTable(VF32 Bat_V)
{
    return getValue_t2D(&InjectorTimingTable, Bat_V);
}


/***************************************************************************************************************************************************
*   Cranking base fuel mass table - CrankingFuelTable
***************************************************************************************************************************************************/

exec_result_t store_CrankingFuelTable()
{
    return store_t2D_data(&(CrankingFuelTable.data), EEPROM_FUELING_CRANKINGTABLE_BASE);
}


void show_CrankingFuelTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nWarm up Enrichment table (%):\r\n");

    show_t2D_data(TS_PORT, &(CrankingFuelTable.data));
}


exec_result_t modify_CrankingFuelTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(CrankingFuelTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for CrankingFuelTable
*/
void send_CrankingFuelTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(CrankingFuelTable.data));
}


/**
returns the Cranking base fuel mass in 16 ug increments
*/
VF32 getValue_CrankingFuelTable(VF32 CLT_K)
{
    return getValue_t2D(&CrankingFuelTable, CLT_K);
}
