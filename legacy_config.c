

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_tables.h"
#include "legacy_config.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

//DEBUG
#include "uart.h"
#include "conversion.h"



volatile configPage1_t configPage1;
volatile configPage2_t configPage2;
volatile configPage3_t configPage3;
volatile configPage4_t configPage4;
volatile configPage10_t configPage10;
volatile configPage11_t configPage11;



/****************************************************************************************************************************************************
*
* Load configuration data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t load_legacy_config()
{
    U32 offset, x, y, z, i;
    U8 * pnt_configPage;
    U8 eeprom_data;
    eeprom_result_t ee_result;
    exec_result_t result;


    /*************
    * Fuel table *
    *************/

    result= load_3D_table(&fuelTable, EEPROM_CONFIG1_MAP, TABLE_RPM_MULTIPLIER, TABLE_LOAD_MULTIPLIER);

    ASSERT_EXEC_OK(result);


    /****************
    * config page 1 *
    ****************/

    pnt_configPage = (U8 *)&configPage1;

    for(x=EEPROM_CONFIG2_START; x<EEPROM_CONFIG2_END; x++)
    {
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        *(pnt_configPage + (U8) (x - EEPROM_CONFIG2_START)) = eeprom_data;
    }


    /***************************
    * IGNITION CONFIG PAGE (2) *
    ***************************/

    pnt_configPage = (U8 *)&configPage2;

    for(x=EEPROM_CONFIG4_START; x<EEPROM_CONFIG4_END; x++)
    {
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        *(pnt_configPage + (U8) (x - EEPROM_CONFIG4_START)) = eeprom_data;
    }

    /*****************************
    * AFR TARGET CONFIG PAGE (3) *
    *****************************/

    result= load_3D_table(&afrTable, EEPROM_CONFIG5_MAP, TABLE_RPM_MULTIPLIER, TABLE_LOAD_MULTIPLIER);

    ASSERT_EXEC_OK(result);


    pnt_configPage = (U8 *)&configPage3;

    for(x=EEPROM_CONFIG6_START; x<EEPROM_CONFIG6_END; x++)
    {
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        *(pnt_configPage + (U8) (x - EEPROM_CONFIG6_START)) = eeprom_data;
    }

    /******************
    * CONFIG PAGE (4) *
    ******************/

    pnt_configPage = (U8 *)&configPage4;

    //The first 64 bytes can simply be pulled straight from the configTable
    for(x=EEPROM_CONFIG7_START; x<EEPROM_CONFIG7_END; x++)
    {
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        *(pnt_configPage + (U8) (x - EEPROM_CONFIG7_START)) = eeprom_data;
    }

    /****************************
    * Boost and vvt tables load *
    ****************************/

    y = EEPROM_CONFIG8_MAP2;

    for(x=EEPROM_CONFIG8_MAP1; x<EEPROM_CONFIG8_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_MAP1;
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //Read the 8x8 map
        boostTable.axisZ[(offset/8)][offset%8] = eeprom_data;


        offset = y - EEPROM_CONFIG8_MAP2;
        ee_result= eeprom_read_byte(y, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //Read the 8x8 map
        vvtTable.axisZ[(offset/8)][offset%8] = eeprom_data;

        y++;
    }

    //RPM bins
    y = EEPROM_CONFIG8_XBINS2;

    for(x=EEPROM_CONFIG8_XBINS1; x<EEPROM_CONFIG8_YBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_XBINS1;
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //RPM bins are divided by 100 when stored. Multiply them back now
        boostTable.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        offset = y - EEPROM_CONFIG8_XBINS2;
        ee_result= eeprom_read_byte(y, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //RPM bins are divided by 100 when stored. Multiply them back now
        vvtTable.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        y++;
    }

    //TPS/MAP bins
    y = EEPROM_CONFIG8_YBINS2;

    for(x=EEPROM_CONFIG8_YBINS1; x<EEPROM_CONFIG8_XSIZE2; x++)
    {
        offset = x - EEPROM_CONFIG8_YBINS1;
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
        boostTable.axisY[offset] = eeprom_data;

        offset = y - EEPROM_CONFIG8_YBINS2;

        ee_result= eeprom_read_byte(y, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //TABLE_LOAD_MULTIPLIER is NOT used for VVT as it is TPS based (0-100)
        vvtTable.axisY[offset] = eeprom_data;

        y++;
    }

    /************************
    * Fuel trim tables load *
    ************************/

    y = EEPROM_CONFIG9_MAP2;
    z = EEPROM_CONFIG9_MAP3;
    i = EEPROM_CONFIG9_MAP4;

    for(x=EEPROM_CONFIG9_MAP1; x<EEPROM_CONFIG9_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG9_MAP1;
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        trim1Table.axisZ[(offset/6)][offset%6] = eeprom_data;

        offset = y - EEPROM_CONFIG9_MAP2;
        ee_result= eeprom_read_byte(y, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        trim2Table.axisZ[(offset/6)][offset%6] = eeprom_data;

        offset = z - EEPROM_CONFIG9_MAP3;
        ee_result= eeprom_read_byte(z, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        trim3Table.axisZ[(offset/6)][offset%6] = eeprom_data;

        offset = i - EEPROM_CONFIG9_MAP4;
        ee_result= eeprom_read_byte(i, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        trim4Table.axisZ[(offset/6)][offset%6] = eeprom_data;

        y++;
        z++;
        i++;
    }

    //RPM bins
    y = EEPROM_CONFIG9_XBINS2;
    z = EEPROM_CONFIG9_XBINS3;
    i = EEPROM_CONFIG9_XBINS4;

    for(x=EEPROM_CONFIG9_XBINS1; x<EEPROM_CONFIG9_YBINS1; x++)
    {
        offset = x - EEPROM_CONFIG9_XBINS1;
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //RPM bins are divided by 100 when stored. Multiply them back now
        trim1Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        offset = y - EEPROM_CONFIG9_XBINS2;
        ee_result= eeprom_read_byte(y, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //RPM bins are divided by 100 when stored. Multiply them back now
        trim2Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        offset = z - EEPROM_CONFIG9_XBINS3;
        ee_result= eeprom_read_byte(z, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //RPM bins are divided by 100 when stored. Multiply them back now
        trim3Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        offset = i - EEPROM_CONFIG9_XBINS4;
        ee_result= eeprom_read_byte(i, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //RPM bins are divided by 100 when stored. Multiply them back now
        trim4Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        y++;
        z++;
        i++;

    }

    //TPS/MAP bins
    y = EEPROM_CONFIG9_YBINS2;
    z = EEPROM_CONFIG9_YBINS3;
    i = EEPROM_CONFIG9_YBINS4;

    for(x=EEPROM_CONFIG9_YBINS1; x<EEPROM_CONFIG9_XSIZE2; x++)
    {
        offset = x - EEPROM_CONFIG9_YBINS1;
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //Table load is divided by 2 (Allows for MAP up to 511)
        trim1Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;

        offset = y - EEPROM_CONFIG9_YBINS2;
        ee_result= eeprom_read_byte(y, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //Table load is divided by 2 (Allows for MAP up to 511)
        trim2Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;

        offset = z - EEPROM_CONFIG9_YBINS3;
        ee_result= eeprom_read_byte(z, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //Table load is divided by 2 (Allows for MAP up to 511)
        trim3Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;

        offset = i - EEPROM_CONFIG9_YBINS4;
        ee_result= eeprom_read_byte(i, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //Table load is divided by 2 (Allows for MAP up to 511)
        trim4Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;

        y++;
        z++;
        i++;
    }

    /*******************
    * CONFIG PAGE (11) *
    *******************/

    pnt_configPage = (U8 *)&configPage11;

    //All 192 bytes can simply be pulled straight from the configTable
    for(x=EEPROM_CONFIG11_START; x<EEPROM_CONFIG11_END; x++)
    {
        ee_result= eeprom_read_byte(x, &eeprom_data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        *(pnt_configPage + (U8) (x - EEPROM_CONFIG11_START)) = eeprom_data;
    }

    //success
    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* Takes the current configuration (config pages and maps)
* and writes them to EEPROM as per the layout defined in eeprom_layout.h
*
TODO remove map dimensions from eeprom
****************************************************************************************************************************************************/
exec_result_t write_legacy_config()
{

    U16 offset, x, y, z, i;
    U8 * pnt_configPage;
    eeprom_result_t ee_result;
    exec_result_t result;


    /*************
    * Fuel table *
    *************/
    result= write_3D_table(&fuelTable, EEPROM_CONFIG1_MAP, TABLE_RPM_MULTIPLIER, TABLE_LOAD_MULTIPLIER);

    ASSERT_EXEC_OK(result);


    /*******************
    * CONFIG PAGE (1)  *
    *******************/

    pnt_configPage = (U8 *)&configPage1;

    for(x=EEPROM_CONFIG2_START; x<EEPROM_CONFIG2_END; x++)
    {
        ee_result= eeprom_update_byte(x, *(pnt_configPage + (U8)(x - EEPROM_CONFIG2_START)) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }


    /*******************
    * CONFIG PAGE (2)  *
    *******************/

    pnt_configPage = (U8 *)&configPage2;

    for(x=EEPROM_CONFIG4_START; x<EEPROM_CONFIG4_END; x++)
    {
        ee_result= eeprom_update_byte(x, *(pnt_configPage + (U8)(x - EEPROM_CONFIG4_START)) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }


    /*************
    * AFR TABLE  *
    *************/

    ee_result= eeprom_update_byte(EEPROM_CONFIG5_XSIZE, TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_byte(EEPROM_CONFIG5_YSIZE, TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);


    for( x=EEPROM_CONFIG5_MAP; x<EEPROM_CONFIG5_XBINS; x++)
    {
        offset = x - EEPROM_CONFIG5_MAP;

        ee_result= eeprom_update_byte(x, (afrTable.axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE]) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    //RPM bins
    for(x=EEPROM_CONFIG5_XBINS; x<EEPROM_CONFIG5_YBINS; x++)
    {
        offset = x - EEPROM_CONFIG5_XBINS;

        ee_result= eeprom_update_byte(x, (U8) (afrTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    //TPS/MAP bins
    for(x=EEPROM_CONFIG5_YBINS; x<EEPROM_CONFIG6_START; x++)
    {
        offset = x - EEPROM_CONFIG5_YBINS;

        //Table load is divided by 2 (Allows for MAP up to 511)
        ee_result= eeprom_update_byte(x, afrTable.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        ASSERT_CONFIG_SUCESS(ee_result);
    }


    /*******************
    * CONFIG PAGE (2)  *
    *******************/

    pnt_configPage = (U8 *)&configPage3;

    for(x=EEPROM_CONFIG6_START; x<EEPROM_CONFIG6_END; x++)
    {
        ee_result= eeprom_update_byte(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG6_START)) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    /*******************
    * CONFIG PAGE (4)  *
    *******************/

    pnt_configPage = (U8 *)&configPage4;

    for(x=EEPROM_CONFIG7_START; x<EEPROM_CONFIG7_END; x++)
    {
        ee_result= eeprom_update_byte(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG7_START)) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    /****************************
    * Boost and vvt tables load *
    ****************************/

    ee_result= eeprom_update_byte(EEPROM_CONFIG8_XSIZE1, TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_byte(EEPROM_CONFIG8_YSIZE1, TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_byte(EEPROM_CONFIG8_XSIZE2, TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_byte(EEPROM_CONFIG8_YSIZE2, TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);


    y = EEPROM_CONFIG8_MAP2;

    for(x=EEPROM_CONFIG8_MAP1; x<EEPROM_CONFIG8_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_MAP1;

        ee_result= eeprom_update_byte(x, (boostTable.axisZ[(offset/8)][offset%8]) );

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = y - EEPROM_CONFIG8_MAP2;

        ee_result= eeprom_update_byte(y, (vvtTable.axisZ[(offset/8)][offset%8]) );

        ASSERT_CONFIG_SUCESS(ee_result);

        y++;
    }

    //RPM bins
    y = EEPROM_CONFIG8_XBINS2;

    for(x=EEPROM_CONFIG8_XBINS1; x<EEPROM_CONFIG8_YBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_XBINS1;

        ee_result= eeprom_update_byte(x, (U8) (boostTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = y - EEPROM_CONFIG8_XBINS2;

        //RPM bins are divided by 100
        ee_result= eeprom_update_byte(y, (U8) (vvtTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        ASSERT_CONFIG_SUCESS(ee_result);

        y++;
    }

    //TPS/MAP bins
    y=EEPROM_CONFIG8_YBINS2;

    for(x=EEPROM_CONFIG8_YBINS1; x<EEPROM_CONFIG8_XSIZE2; x++)
    {
        offset = x - EEPROM_CONFIG8_YBINS1;

        //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
        ee_result= eeprom_update_byte(x, boostTable.axisY[offset]);

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = y - EEPROM_CONFIG8_YBINS2;

        //TABLE_LOAD_MULTIPLIER is NOT used for VVT as it is TPS based (0-100)
        ee_result= eeprom_update_byte(y, vvtTable.axisY[offset]);

        ASSERT_CONFIG_SUCESS(ee_result);

        y++;
  }

    /*******************
    * Fuel trim tables *
    *******************/

    //Write the boost Table RPM dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_XSIZE1,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table MAP/TPS dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_YSIZE1,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table RPM dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_XSIZE2,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table MAP/TPS dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_YSIZE2,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table RPM dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_XSIZE3,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table MAP/TPS dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_YSIZE3,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table RPM dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_XSIZE4,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    //Write the boost Table MAP/TPS dimension size
    ee_result= eeprom_update_byte(EEPROM_CONFIG9_YSIZE4,TABLE3D_DIMENSION);

    ASSERT_CONFIG_SUCESS(ee_result);

    y = EEPROM_CONFIG9_MAP2;
    z = EEPROM_CONFIG9_MAP3;
    i = EEPROM_CONFIG9_MAP4;

    for(x=EEPROM_CONFIG9_MAP1; x<EEPROM_CONFIG9_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG9_MAP1;
        ee_result= eeprom_update_byte(x, (trim1Table.axisZ[(offset/6)][offset%6]) );

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = y - EEPROM_CONFIG9_MAP2;
        ee_result= eeprom_update_byte(y, trim2Table.axisZ[(offset/6)][offset%6]);

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = z - EEPROM_CONFIG9_MAP3;
        ee_result= eeprom_update_byte(z, trim3Table.axisZ[(offset/6)][offset%6]);

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = i - EEPROM_CONFIG9_MAP4;
        ee_result= eeprom_update_byte(i, trim4Table.axisZ[(offset/6)][offset%6]);

        ASSERT_CONFIG_SUCESS(ee_result);

        y++;
        z++;
        i++;
    }

    //RPM bins
    y = EEPROM_CONFIG9_XBINS2;
    z = EEPROM_CONFIG9_XBINS3;
    i = EEPROM_CONFIG9_XBINS4;

    for(x=EEPROM_CONFIG9_XBINS1; x<EEPROM_CONFIG9_YBINS1; x++)
    {
        offset = x - EEPROM_CONFIG9_XBINS1;

        //RPM bins are divided by 100 and converted to a byte
        ee_result= eeprom_update_byte(x, (U8) (trim1Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = y - EEPROM_CONFIG9_XBINS2;

        //RPM bins are divided by 100 and converted to a byte
        ee_result= eeprom_update_byte(y, (U8) (trim2Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = z - EEPROM_CONFIG9_XBINS3;

        //RPM bins are divided by 100 and converted to a byte
        ee_result= eeprom_update_byte(z, (U8) (trim3Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = i - EEPROM_CONFIG9_XBINS4;

        //RPM bins are divided by 100 and converted to a byte
        ee_result= eeprom_update_byte(i, (U8) (trim4Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        ASSERT_CONFIG_SUCESS(ee_result);

        y++;
        z++;
        i++;
    }

    //TPS/MAP bins
    y=EEPROM_CONFIG9_YBINS2;
    z=EEPROM_CONFIG9_YBINS3;
    i=EEPROM_CONFIG9_YBINS4;

    for(x=EEPROM_CONFIG9_YBINS1; x<EEPROM_CONFIG9_XSIZE2; x++)
    {
        offset = x - EEPROM_CONFIG9_YBINS1;

        //Table load is divided by 2 (Allows for MAP up to 511)
        ee_result= eeprom_update_byte(x, trim1Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = y - EEPROM_CONFIG9_YBINS2;

        //Table load is divided by 2 (Allows for MAP up to 511)
        ee_result=  eeprom_update_byte(y, trim2Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

       ASSERT_CONFIG_SUCESS(ee_result);

        offset = z - EEPROM_CONFIG9_YBINS3;

        //Table load is divided by 2 (Allows for MAP up to 511)
        ee_result= eeprom_update_byte(z, trim3Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        ASSERT_CONFIG_SUCESS(ee_result);

        offset = i - EEPROM_CONFIG9_YBINS4;

        //Table load is divided by 2 (Allows for MAP up to 511)
        ee_result= eeprom_update_byte(i, trim4Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        ASSERT_CONFIG_SUCESS(ee_result);

        y++;
        z++;
        i++;
    }


    /********************
    * CONFIG PAGE (10)  *
    ********************/

    pnt_configPage = (U8 *)&configPage10;

    for(x=EEPROM_CONFIG10_START; x<EEPROM_CONFIG10_END; x++)
    {
        ee_result= eeprom_update_byte(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG10_START)) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }


    /********************
    * CONFIG PAGE (11)  *
    ********************/

    pnt_configPage = (U8 *)&configPage11;

    for(x=EEPROM_CONFIG11_START; x<EEPROM_CONFIG11_END; x++)
    {
        ee_result= eeprom_update_byte(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG11_START)) );

        ASSERT_CONFIG_SUCESS(ee_result);
    }


    return EXEC_OK;
}


/**
    Repoint the 2D table structs to the config pages
    (initialise the 8 table2D structs)


    to be reconfigured!

/// TODO (oli#3#): find out which kind of 2d table will make sense for us

*/
void init_2Dtables()
{
    taeTable.dimension = 4;
    taeTable.axisY = (U16 *) configPage2.taeValues;
    taeTable.axisX = (U16 *)configPage2.taeBins;

    WUETable.dimension = 10;
    WUETable.axisY = (U16 *) configPage1.wueValues;
    WUETable.axisX = (U16 *) configPage2.wueBins;

    crankingEnrichTable.dimension = 4;
    crankingEnrichTable.axisY = (U16 *) configPage11.crankingEnrichValues;
    crankingEnrichTable.axisX = (U16 *) configPage11.crankingEnrichBins;

    dwellVCorrectionTable.dimension = 6;
    dwellVCorrectionTable.axisY = (U16 *) configPage2.dwellCorrectionValues;
    dwellVCorrectionTable.axisX =  (U16 *)configPage3.voltageCorrectionBins;

    injectorVCorrectionTable.dimension = 6;
    injectorVCorrectionTable.axisY = (U16 *) configPage3.injVoltageCorrectionValues;
    injectorVCorrectionTable.axisX = (U16 *) configPage3.voltageCorrectionBins;

    IATDensityCorrectionTable.dimension = 9;
    IATDensityCorrectionTable.axisY = (U16 *) configPage3.airDenRates;
    IATDensityCorrectionTable.axisX = (U16 *) configPage3.airDenBins;

    IATRetardTable.dimension = 6;
    IATRetardTable.axisY = (U16 *) configPage2.iatRetValues;
    IATRetardTable.axisX = (U16 *) configPage2.iatRetBins;

    rotarySplitTable.dimension = 8;
    rotarySplitTable.axisY = (U16 *) configPage11.rotarySplitValues;
    rotarySplitTable.axisX = (U16 *) configPage11.rotarySplitBins;
}
