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
volatile t3D_t AfrTable_MAP;

volatile t2D_t AccelCompTableTPS;
volatile t2D_t AccelCompTableMAP;

volatile t2D_t WarmUpCompTable;

volatile t2D_t InjectorTimingTable;

volatile t2D_t CrankingFuelTable;

volatile t2D_t BAROtable;

volatile t3D_t ChargeTempTable;

volatile U8 * const pFueling_Setup_data= (volatile U8 *) &Fueling_Setup;
const U32 cFueling_Setup_size= sizeof(Fueling_Setup);



/**
* reads Fueling config (setup and tables) data from eeprom
*
*/
exec_result_t load_Fueling_Config()
{
    //bring up eeprom first
    Eeprom_init();

    ASSERT_EXEC_OK( load_Fueling_Setup() );

    ASSERT_EXEC_OK( load_VeTable_TPS() );
    ASSERT_EXEC_OK( load_VeTable_MAP() );

    ASSERT_EXEC_OK( load_AfrTable_MAP() );
    ASSERT_EXEC_OK( load_AfrTable_TPS() );

    ASSERT_EXEC_OK( load_AccelCompTableTPS() );
    ASSERT_EXEC_OK( load_AccelCompTableMAP() );

    ASSERT_EXEC_OK( load_WarmUpCompTable() );

    ASSERT_EXEC_OK( load_InjectorTimingTable() );

    ASSERT_EXEC_OK( load_CrankingFuelTable() );

    ASSERT_EXEC_OK( load_BAROtable() );

    ASSERT_EXEC_OK( load_ChargeTempTable() );

    return EXEC_OK;
}


/***************************************************************************************************************************************************
*   Fueling Setup
***************************************************************************************************************************************************/


exec_result_t load_Fueling_Setup()
{
    return Eeprom_load_data(EEPROM_FUELING_SETUP_BASE, pFueling_Setup_data, cFueling_Setup_size);
}

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


    //F32 accel_comp_thres_MAP_low
    print(Port, "\r\nacceleration compensation turn on MAP rate -low- (kPa/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_MAP_low);

    //F32 accel_comp_thres_MAP_high
    print(Port, "\r\nacceleration compensation turn on MAP rate -high- (kPa/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_MAP_high);

    //F32 accel_comp_thres_TPS_low
    print(Port, "\r\nacceleration compensation turn on TPS rate -low- (deg/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_TPS_low);

    //F32 accel_comp_thres_TPS_high
    print(Port, "\r\nacceleration compensation turn on TPS rate -high- (deg/s):");
    printf_F32(Port, Fueling_Setup.accel_comp_thres_TPS_high);

    //U16 accel_comp_thres_rpm
    print(Port, "\r\nMAP/TPS rate switching threshold (rpm):");
    printf_U(Port, Fueling_Setup.accel_comp_thres_rpm, NO_PAD);

    //F32 decel_comp_thres_TPS
    print(Port, "\r\ndeceleration compensation turn on TPS rate (deg/s):");
    printf_F32(Port, Fueling_Setup.decel_comp_thres_TPS);

    //F32 decel_comp_thres_MAP
    print(Port, "\r\ndeceleration compensation turn on MAP rate (kPa/s):");
    printf_F32(Port, Fueling_Setup.decel_comp_thres_MAP);

    //F32 accel_comp_taper_factor
    print(Port, "\r\nacceleration compensation taper factor (<1!):");
    printf_F32(Port, Fueling_Setup.accel_comp_taper_factor);

    //U16 accel_comp_scaling_thres_rpm
    print(Port, "\r\nacceleration compensation rpm scaling begin (rpm):");
    printf_U(Port, Fueling_Setup.accel_comp_scaling_thres_rpm, NO_PAD);

    //U16 accel_comp_scaling_max_rpm
    print(Port, "\r\nacceleration compensation rpm scaling maximum (rpm):");
    printf_U(Port, Fueling_Setup.accel_comp_scaling_thres_rpm, NO_PAD);

    //U8 cold_accel_pct
    print(Port, "\r\ncold engine acceleration compensation bonus fuel (%):");
    printf_U(Port, Fueling_Setup.cold_accel_pct, NO_PAD);

    //U16 decel_comp_ug
    print(Port, "\r\ndeceleration compensation (ug):");
    printf_U(Port, Fueling_Setup.decel_comp_ug, NO_PAD);

    //U8 accel_comp_cycles
    print(Port, "\r\nacceleration compensation duration (events):");
    printf_U(Port, Fueling_Setup.accel_comp_cycles, NO_PAD);

    //U8 decel_comp_cycles
    print(Port, "\r\ndeceleration compensation duration (events):");
    printf_U(Port, Fueling_Setup.decel_comp_cycles, NO_PAD);

    //U8 accel_comp_taper_thres
    print(Port, "\r\nacceleration compensation taper begin (remaining events):");
    printf_U(Port, Fueling_Setup.accel_comp_taper_thres, NO_PAD);


    //U8 afterstart_comp_pct
    print(Port, "\r\nafter start enrichment (%):");
    printf_U(Port, Fueling_Setup.afterstart_comp_pct, NO_PAD);

    //U8 afterstart_comp_cycles
    print(Port, "\r\nafter start enrichment duration (events):");
    printf_F32(Port, Fueling_Setup.afterstart_comp_cycles);

    //U16 afterstart_thres_K
    print(Port, "\r\nafter start enrichment CLT threshold (K):");
    printf_U(Port, Fueling_Setup.afterstart_thres_K, NO_PAD);


    //U16 spd_min_rpm
    print(Port, "\r\nSpeed Density min (rpm):");
    printf_U(Port, Fueling_Setup.spd_min_rpm, NO_PAD);

    //U16 spd_max_rpm
    print(Port, "\r\nSpeed Density max (rpm):");
    printf_U(Port, Fueling_Setup.spd_max_rpm, NO_PAD);


    //dry cranking
    print(Port, "\r\ndry cranking TPS threshold:");
    printf_U(Port, Fueling_Setup.dry_cranking_TPS_thres, NO_PAD);


    //features
    print(Port, "\r\nfeature enabled features: AE-legacyAE-sequential-WUE-ASE-seq-dry: ");

    UART_Tx(Port, (Fueling_Setup.features.load_transient_comp_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.legacy_AE_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.sequential_mode_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.WUE_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.ASE_enabled? '1' :'0'));
    UART_Tx(Port, '-');
    UART_Tx(Port, (Fueling_Setup.features.dry_cranking_enabled? '1' :'0'));

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
* z-Axis -> VE in % (no offset, scaling * 0,5)
***************************************************************************************************************************************************/

const F32 cVeTableDivider= 2.0;

exec_result_t load_VeTable_TPS()
{
    return load_t3D_data(&(VeTable_TPS.data), EEPROM_FUELING_VETPS_BASE);
}

exec_result_t store_VeTable_TPS()
{
    return store_t3D_data(&(VeTable_TPS.data), EEPROM_FUELING_VETPS_BASE);
}


void show_VeTable_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling VE Table (TPS based):\r\n");

    show_t3D_data(Port, &(VeTable_TPS.data));
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
F32 getValue_VeTable_TPS(U32 Rpm, F32 Tps_deg)
{
    return divide_float( getValue_t3D(&VeTable_TPS, Rpm, Tps_deg) , cVeTableDivider );
}



/***************************************************************************************************************************************************
*   Fueling VE Table (MAP based) - VeTable_MAP
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> MAP in kPa (no offset, no scaling)
* z-Axis -> VE in % (no offset, scaling * 0,5)
***************************************************************************************************************************************************/


exec_result_t load_VeTable_MAP()
{
    return load_t3D_data(&(VeTable_MAP.data), EEPROM_FUELING_VEMAP_BASE);
}

exec_result_t store_VeTable_MAP()
{
    return store_t3D_data(&(VeTable_MAP.data), EEPROM_FUELING_VEMAP_BASE);
}


void show_VeTable_MAP(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling VE Table (MAP based):\r\n");

    show_t3D_data(Port, &(VeTable_MAP.data));
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
F32 getValue_VeTable_MAP(U32 Rpm, F32 Map_kPa)
{
    return divide_float( getValue_t3D(&VeTable_MAP, Rpm, Map_kPa) , cVeTableDivider );
}



/***************************************************************************************************************************************************
*   Fueling VE Table (MAP based) - AfrTable_MAP
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> MAP in kPa (no offset, no scaling)
* z-Axis -> target AFR (no offset, table values are multiplied by 10)
***************************************************************************************************************************************************/

const F32 cAfrDivider= 10.0;


exec_result_t load_AfrTable_MAP()
{
    return load_t3D_data(&(AfrTable_TPS.data), EEPROM_FUELING_AFRTPS_BASE);
}

exec_result_t store_AfrTable_MAP()
{
    return store_t3D_data(&(AfrTable_MAP.data), EEPROM_FUELING_AFRMAP_BASE);
}


void show_AfrTable_MAP(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling AFR Table (MAP based):\r\n");

    show_t3D_data(Port, &(AfrTable_MAP.data));
}


exec_result_t modify_AfrTable_MAP(U32 Offset, U32 Value)
{
    //modify_t3D_data provides offset range check!
    return modify_t3D_data(&(AfrTable_MAP.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AfrTable_MAP
*/
void send_AfrTable_MAP(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(AfrTable_MAP.data));
}


/**
returns the AFR target
*/
F32 getValue_AfrTable_MAP(U32 Rpm, F32 Map_kPa)
{
    return getValue_t3D(&AfrTable_MAP, Rpm, Map_kPa) / cAfrDivider;
}



/***************************************************************************************************************************************************
*   Fueling AFR target Table (TPS based) - AfrTable_TPS
*
* x-Axis -> rpm (no offset, no scaling)
* y-Axis -> TPS angle in ° (no offset, no scaling)
* z-Axis -> target AFR (no offset, table values are multiplied by 10)
***************************************************************************************************************************************************/


exec_result_t load_AfrTable_TPS()
{
    return load_t3D_data(&(AfrTable_MAP.data), EEPROM_FUELING_AFRMAP_BASE);
}

exec_result_t store_AfrTable_TPS()
{
    return store_t3D_data(&(AfrTable_TPS.data), EEPROM_FUELING_AFRTPS_BASE);
}


void show_AfrTable_TPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling AFR target Table (TPS based) -  x10\r\n");

    show_t3D_data(Port, &(AfrTable_TPS.data));
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
F32 getValue_AfrTable_TPS(U32 Rpm, F32 Tps_deg)
{
    return getValue_t3D(&AfrTable_TPS, Rpm, Tps_deg) / cAfrDivider;
}



/***************************************************************************************************************************************************
*   Fueling acceleration compensation table - AccelCompTableTPS
*
* x-Axis -> TPS change rate in °/s (no offset, no scaling)
* y-Axis -> Acceleration compensation value in ug (no offset, no scaling)
***************************************************************************************************************************************************/

exec_result_t load_AccelCompTableTPS()
{
    return load_t2D_data(&(AccelCompTableTPS.data), EEPROM_FUELING_ACCELCOMPTPS_BASE);
}

exec_result_t store_AccelCompTableTPS()
{
    return store_t2D_data(&(AccelCompTableTPS.data), EEPROM_FUELING_ACCELCOMPTPS_BASE);
}


void show_AccelCompTableTPS(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling acceleration compensation table (TPS):\r\n");

    show_t2D_data(Port, &(AccelCompTableTPS.data));
}


exec_result_t modify_AccelCompTableTPS(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(AccelCompTableTPS.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AccelCompTableTPS
*/
void send_AccelCompTableTPS(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(AccelCompTableTPS.data));
}


/**
returns the acceleration compensation value in ug
*/
F32 getValue_AccelCompTableTPS(F32 Ddt_TPS)
{
    return getValue_t2D(&AccelCompTableTPS, Ddt_TPS);
}


/***************************************************************************************************************************************************
*   Fueling acceleration compensation table - AccelCompTableMAP
*
* x-Axis -> TPS change rate in °/s (no offset, no scaling)
* y-Axis -> Acceleration compensation value in ug (no offset, no scaling)
***************************************************************************************************************************************************/

exec_result_t load_AccelCompTableMAP()
{
    return load_t2D_data(&(AccelCompTableMAP.data), EEPROM_FUELING_ACCELCOMPMAP_BASE);
}

exec_result_t store_AccelCompTableMAP()
{
    return store_t2D_data(&(AccelCompTableMAP.data), EEPROM_FUELING_ACCELCOMPMAP_BASE);
}


void show_AccelCompTableMAP(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nFueling acceleration compensation table (MAP):\r\n");

    show_t2D_data(Port, &(AccelCompTableMAP.data));
}


exec_result_t modify_AccelCompTableMAP(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(AccelCompTableMAP.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for AccelCompTableMAP
*/
void send_AccelCompTableMAP(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(AccelCompTableMAP.data));
}


/**
returns the acceleration compensation value in ug
*/
F32 getValue_AccelCompTableMAP(F32 Ddt_MAP)
{
    return getValue_t2D(&AccelCompTableMAP, Ddt_MAP);
}


/***************************************************************************************************************************************************
*   Fueling Warm up Enrichment compensation table - WarmUpCompTable
*
* x-Axis -> CLT in K (no offset, no scaling)
* y-Axis -> Warm up compensation value in % (no offset, scaling * 10)
***************************************************************************************************************************************************/

const F32 cWarmUpDivider= 10.0;

exec_result_t load_WarmUpCompTable()
{
    return load_t2D_data(&(WarmUpCompTable.data), EEPROM_FUELING_WARMUPCOMP_BASE);
}

exec_result_t store_WarmUpCompTable()
{
    return store_t2D_data(&(WarmUpCompTable.data), EEPROM_FUELING_WARMUPCOMP_BASE);
}


void show_WarmUpCompTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nWarm up Enrichment table (%):\r\n");

    show_t2D_data(Port, &(WarmUpCompTable.data));
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
F32 getValue_WarmUpCompTable(F32 CLT_K)
{
    return getValue_t2D(&WarmUpCompTable, CLT_K) / cWarmUpDivider;
}


/***************************************************************************************************************************************************
*   Injector dead time table - InjectorTimingTable
*
* x-Axis -> System Voltage in mV (no offset, no scaling)
* y-Axis -> Injector dead time in us (no offset, table values are in 24 us increments)
***************************************************************************************************************************************************/

exec_result_t load_InjectorTimingTable()
{
    return load_t2D_data(&(InjectorTimingTable.data), EEPROM_FUELING_INJECTORTIMING_BASE);
}

exec_result_t store_InjectorTimingTable()
{
    return store_t2D_data(&(InjectorTimingTable.data), EEPROM_FUELING_INJECTORTIMING_BASE);
}


void show_InjectorTimingTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nInjector timing table:\r\n");

    show_t2D_data(Port, &(InjectorTimingTable.data));
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
returns the injector dead time in its intervals
*/
F32 getValue_InjectorTimingTable(F32 Bat_V)
{
    return getValue_t2D(&InjectorTimingTable, 1000.0 * Bat_V);
}


/***************************************************************************************************************************************************
*   Cranking base fuel mass table - CrankingFuelTable
*
* x-Axis -> CLT in K (no offset, no scaling)
* y-Axis -> Cranking base fuel amount in ug (no offset, table values are in 256 ug increments)
***************************************************************************************************************************************************/

exec_result_t load_CrankingFuelTable()
{
    return load_t2D_data(&(CrankingFuelTable.data), EEPROM_FUELING_CRANKINGTABLE_BASE);
}

exec_result_t store_CrankingFuelTable()
{
    return store_t2D_data(&(CrankingFuelTable.data), EEPROM_FUELING_CRANKINGTABLE_BASE);
}


void show_CrankingFuelTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nCranking fuel table (%):\r\n");

    show_t2D_data(Port, &(CrankingFuelTable.data));
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
returns the Cranking base fuel mass in its increments
*/
F32 getValue_CrankingFuelTable(F32 CLT_K)
{
    return getValue_t2D(&CrankingFuelTable, CLT_K);
}




/***************************************************************************************************************************************************
*   Barometric pressure correction - BAROtable
*
* x-Axis -> Barometric pressure in hPa (no offset, no scaling)
* y-Axis -> correction  in % (offset := 100, no scaling)
***************************************************************************************************************************************************/

const F32 cBAROtableOffset_pct= 100.0;


exec_result_t load_BAROtable()
{
    return load_t2D_data(&(BAROtable.data), EEPROM_FUELING_BARO_BASE);
}

exec_result_t store_BAROtable()
{
    return store_t2D_data(&(BAROtable.data), EEPROM_FUELING_BARO_BASE);
}


void show_BAROtable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nBARO correction table:\r\n");

    show_t2D_data(Port, &(BAROtable.data));
}


exec_result_t modify_BAROtable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t2D_data(&(BAROtable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for BAROtable
*/
void send_BAROtable(USART_TypeDef * Port)
{
    send_t2D_data(Port, &(BAROtable.data));
}


/**
returns the Barometric pressure correction factor in %
*/
F32 getValue_BAROtable(F32 BARO_kPa)
{
    VF32 raw;

    raw= getValue_t2D(&BAROtable, 10.0 * BARO_kPa);
    return raw - cBAROtableOffset_pct;
}



/***************************************************************************************************************************************************
*   charge temperature table - ChargeTempTable
*
* x-Axis -> IAT in K (no offset, no scaling)
* y-Axis -> CLT in K (no offset, no scaling)
* z-Axis -> charge temperature in K (offset := 173,15K, no scaling)
***************************************************************************************************************************************************/

const F32 cChargeTempOffset_K= 173.15;

exec_result_t load_ChargeTempTable()
{
    return load_t3D_data(&(ChargeTempTable.data), EEPROM_FUELING_CHARGETEMP_BASE);
}

exec_result_t store_ChargeTempTable()
{
    return store_t3D_data(&(ChargeTempTable.data), EEPROM_FUELING_CHARGETEMP_BASE);
}


void show_ChargeTempTable(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\ncharge temperature table (K):\r\n");

    show_t3D_data(Port, &(ChargeTempTable.data));
}


exec_result_t modify_ChargeTempTable(U32 Offset, U32 Value)
{
    //modify_t2D_data provides offset range check!
    return modify_t3D_data(&(ChargeTempTable.data), Offset, Value);
}


/**
this function implements the TS interface binary config page read command for ChargeTempTable
*/
void send_ChargeTempTable(USART_TypeDef * Port)
{
    send_t3D_data(Port, &(ChargeTempTable.data));
}


/**
returns the charge temperature in K
*/
F32 getValue_ChargeTempTable(F32 IAT_K, F32 CLT_K)
{
    return cChargeTempOffset_K + getValue_t3D(&ChargeTempTable, IAT_K, CLT_K);
}





