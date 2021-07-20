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

volatile t2D_t InjectorPhaseTable;

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

    //bring up eeprom
    Eeprom_init();

    load_result= Eeprom_load_data(EEPROM_FUELING_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(VeTable_TPS.data), EEPROM_FUELING_VETPS_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(VeTable_MAP.data), EEPROM_FUELING_VEMAP_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t3D_data(&(AfrTable_TPS.data), EEPROM_FUELING_AFRTPS_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(AccelCompTable.data), EEPROM_FUELING_ACCELCOMP_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(WarmUpCompTable.data), EEPROM_FUELING_WARMUPCOMP_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(InjectorTimingTable.data), EEPROM_FUELING_INJECTORTIMING_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(CrankingFuelTable.data), EEPROM_FUELING_CRANKINGTABLE_BASE);

    ASSERT_EXEC_OK(load_result);

    load_result= load_t2D_data(&(InjectorPhaseTable.data), EEPROM_FUELING_INJECTORPHASE_BASE);

    return load_result;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Fueling_Config()
{
    Fueling_Setup.Version= 0;

    Fueling_Setup.cylinder_volume_ccm= 425;

    Fueling_Setup.injection_reference_pos= CRK_POSITION_A1;

    Fueling_Setup.injector1_rate_mgps= 4000;
    Fueling_Setup.injector2_rate_mgps= 4000;
    Fueling_Setup.max_injector_duty_cycle_pct= 0;

    Fueling_Setup.accel_comp_thres= 10000;
    Fueling_Setup.decel_comp_thres= 10000;
    Fueling_Setup.decel_comp_pct= 0;
    Fueling_Setup.accel_comp_cycles= 0;

    Fueling_Setup.afterstart_comp_pct= 0;
    Fueling_Setup.afterstart_comp_cycles= 0;

    Fueling_Setup.max_fuel_mass_comp_pct= 0;
    Fueling_Setup.ve_from_map_min_rpm= 0;
    Fueling_Setup.ve_from_map_max_rpm= 0;

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


    //U16 cylinder_volume_ccm
    print(Port, "\r\ncylinder volume (ccm):");
    printf_U(Port, Fueling_Setup.cylinder_volume_ccm, NO_PAD);


    //injection_reference_pos
    print(Port, "\r\ndefault injection reference position: ");
    printf_crkpos(Port, Fueling_Setup.injection_reference_pos);


    //U16 injector1_rate_mgps
    print(Port, "\r\ninjector #1 flow rate (mg/s):");
    printf_U(Port, Fueling_Setup.injector1_rate_mgps, NO_PAD);

    //U16 injector2_rate_mgps
    print(Port, "\r\ninjector #2 flow rate (mg/s):");
    printf_U(Port, Fueling_Setup.injector2_rate_mgps, NO_PAD);

    //U8 max_injector_duty_cycle_pct
    print(Port, "\r\nmaximum injector duty cycle (%):");
    printf_U(Port, Fueling_Setup.max_injector_duty_cycle_pct, NO_PAD);


    //F32 accel_comp_thres
    print(Port, "\r\nacceleration compensation turn on TPS rate (deg/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres);

    //F32 decel_comp_thres
    print(Port, "\r\ndeceleration compensation turn on TPS rate (deg/s):");
    printf_F32(Port, Fueling_Setup.decel_comp_thres);

    //U8 decel_comp_pct
    print(Port, "\r\ndeceleration compensation (%):");
    printf_U(Port, Fueling_Setup.decel_comp_pct, NO_PAD);

    //U8 accel_comp_cycles
    print(Port, "\r\nacceleration compensation duration (events):");
    printf_U(Port, Fueling_Setup.accel_comp_cycles, NO_PAD);


    //U8 afterstart_comp_pct
    print(Port, "\r\nafter start enrichment (%):");
    printf_U(Port, Fueling_Setup.afterstart_comp_pct, NO_PAD);

    //U8 afterstart_comp_cycles
    print(Port, "\r\nafter start enrichment duration (events):");
    printf_F32(Port, Fueling_Setup.afterstart_comp_cycles);



    //U8 max_fuel_mass_comp_pct
    print(Port, "\r\nmaximum fuel mass compensation (%):");
    printf_U(Port, Fueling_Setup.max_fuel_mass_comp_pct, NO_PAD);



    //U16 ve_from_map_min_rpm
    print(Port, "\r\nVE from MAP min (rpm):");
    printf_U(Port, Fueling_Setup.ve_from_map_min_rpm, NO_PAD);

    //U16 ve_from_map_max_rpm
    print(Port, "\r\nVE from MAP max (rpm):");
    printf_U(Port, Fueling_Setup.ve_from_map_max_rpm, NO_PAD);


    //dry cranking
    print(Port, "\r\ndry cranking TPS threshold:");
    printf_U(Port, Fueling_Setup.dry_cranking_TPS_thres, NO_PAD);


    //features
    print(Port, "\r\nfeature enabled features: AE-WUE-ASE-seq-dry: ");

    UART_Tx(TS_PORT, (Fueling_Setup.features.load_transient_comp_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Fueling_Setup.features.warmup_comp_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Fueling_Setup.features.afterstart_corr_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Fueling_Setup.features.sequential_mode_enabled? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (Fueling_Setup.features.dry_cranking_enabled? '1' :'0'));

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
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> TPS angle in ° (no offset, no scaling)
* z-Axis -> VE in % (no offset, no scaling)
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
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> MAP in kPa (no offset, no scaling)
* z-Axis -> VE in % (no offset, no scaling)
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
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> TPS angle in ° (no offset, no scaling)
* z-Axis -> target AFR (no offset, table values are multiplied by 10)
***************************************************************************************************************************************************/

const F32 cAccelCompDivider= 10.0;

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
(divided by 10 for decimal place storage in table)
*/
VF32 getValue_AfrTable_TPS(VU32 Rpm, VF32 Tps_deg)
{
    return (getValue_t3D(&AfrTable_TPS, Rpm, Tps_deg) / cAccelCompDivider);
}



/***************************************************************************************************************************************************
*   Fueling acceleration compensation table - AccelCompTable
*
* x-Axis -> TPS change rate in °/s (no offset, no scaling)
* y-Axis -> Acceleration compensation value in % (no offset, no scaling)
***************************************************************************************************************************************************/

const F32 cAccelCompMultiplier= 4.0;


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
    return cAccelCompMultiplier * getValue_t2D(&AccelCompTable, Ddt_TPS);
}


/***************************************************************************************************************************************************
*   Fueling Warm up Enrichment compensation table - WarmUpCompTable
*
* x-Axis -> CLT in K (no offset, no scaling)
* y-Axis -> Warm up compensation value in % (no offset, no scaling)
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
*
* x-Axis -> System Voltage in mV (no offset, no scaling)
* y-Axis -> Injector dead time in us (no offset, table values are in 24 us increments)
***************************************************************************************************************************************************/

const U32 cInjTimingMultiplier= 24;

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
VU32 getValue_InjectorTimingTable(VF32 Bat_V)
{
    return cInjTimingMultiplier * (VU32) getValue_t2D(&InjectorTimingTable, 1000 * Bat_V);
}


/***************************************************************************************************************************************************
*   Cranking base fuel mass table - CrankingFuelTable
*
* x-Axis -> CLT in K (no offset, no scaling)
* y-Axis -> Cranking base fuel amount in ug (no offset, table values are in 512 ug increments)
***************************************************************************************************************************************************/

const U32 cCrkFuelMultiplier= 512;

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
returns the Cranking base fuel mass in 128 ug increments
*/
VU32 getValue_CrankingFuelTable(VF32 CLT_K)
{
    return cCrkFuelMultiplier * (VU32) getValue_t2D(&CrankingFuelTable, CLT_K);
}



/***************************************************************************************************************************************************
*   Injection end target advance relative to Intake valve opening - InjectorPhaseTable
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> Injection end target advance (offset := 128, table values are in 2 deg increments)
***************************************************************************************************************************************************/

const U32 cInjPhaseMultiplier= 2;

exec_result_t store_InjectorPhaseTable()
{
    return store_t2D_data(&(InjectorPhaseTable.data), EEPROM_FUELING_INJECTORPHASE_BASE);
}


void show_InjectorPhaseTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nInjector phase table:\r\n");

    show_t2D_data(TS_PORT, &(InjectorPhaseTable.data));
}


exec_result_t modify_InjectorPhaseTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(InjectorPhaseTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for InjectorPhaseTable
*/
void send_InjectorPhaseTable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(InjectorPhaseTable.data));
}


/**
returns the Injection end target advance in 2 deg interval
*/
VU32 getValue_InjectorPhaseTable(VU32 Rpm)
{
    return cInjPhaseMultiplier * (VU32) getValue_t2D(&InjectorPhaseTable, Rpm);
}







