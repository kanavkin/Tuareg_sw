

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config.h"
#include "eeprom_layout.h"

#include "Tuareg.h"
#include "trigger_wheel_layout.h"

//DEBUG
#include "uart.h"
#include "conversion.h"

/**
these are our config pages
*/
volatile configPage1_t configPage1;
volatile configPage2_t configPage2;
volatile configPage3_t configPage3;
volatile configPage4_t configPage4;
volatile configPage9_t configPage9;
volatile configPage10_t configPage10;
volatile configPage11_t configPage11;
volatile configPage12_t configPage12;
volatile configPage13_t configPage13;



/****************************************************************************************************************************************************
*
* Load configuration data from EEPROM
*
****************************************************************************************************************************************************/
U32 config_load()
{
    U32 offset, x, y, z, i;
    U8 * pnt_configPage;
    U8 eeprom_data;
    U32 eeprom_code;


    /*************
    * Fuel table *
    *************/

    for(x=EEPROM_CONFIG1_MAP; x<EEPROM_CONFIG1_XBINS; x++)
    {
        offset = x - EEPROM_CONFIG1_MAP;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            fuelTable.axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE]= eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }

    for(x=EEPROM_CONFIG1_XBINS; x<EEPROM_CONFIG1_YBINS; x++)
    {
        offset = x - EEPROM_CONFIG1_XBINS;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            fuelTable.axisX[offset]= eeprom_data * TABLE_RPM_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }
    }

    for(x=EEPROM_CONFIG1_YBINS; x<EEPROM_CONFIG2_START; x++)
    {
        offset = x - EEPROM_CONFIG1_YBINS;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            fuelTable.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }
    }

    pnt_configPage = (U8 *)&configPage1;

    for(x=EEPROM_CONFIG2_START; x<EEPROM_CONFIG2_END; x++)
    {
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            *(pnt_configPage + (U8) (x - EEPROM_CONFIG2_START)) = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }

    /***************************
    * IGNITION CONFIG PAGE (2) *
    ***************************/


    //ignition table loaded by table module
    eeprom_code= load_3D_table(&ignitionTable_TPS, EEPROM_CONFIG3_MAP, 100, 2);



    pnt_configPage = (U8 *)&configPage2;

    for(x=EEPROM_CONFIG4_START; x<EEPROM_CONFIG4_END; x++)
    {
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            *(pnt_configPage + (U8) (x - EEPROM_CONFIG4_START)) = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }

    /*****************************
    * AFR TARGET CONFIG PAGE (3) *
    *****************************/

    for(x=EEPROM_CONFIG5_MAP; x<EEPROM_CONFIG5_XBINS; x++)
    {
        offset = x - EEPROM_CONFIG5_MAP;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            afrTable.axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }


    for(x=EEPROM_CONFIG5_XBINS; x<EEPROM_CONFIG5_YBINS; x++)
    {
        offset = x - EEPROM_CONFIG5_XBINS;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            afrTable.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);
        }
        else
        {
            return eeprom_code;
        }
    }


    for(x=EEPROM_CONFIG5_YBINS; x<EEPROM_CONFIG6_START; x++)
    {
        offset = x - EEPROM_CONFIG5_YBINS;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Table load is divided by 2 (Allows for MAP up to 511)
            afrTable.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }
    }

    pnt_configPage = (U8 *)&configPage3;

    for(x=EEPROM_CONFIG6_START; x<EEPROM_CONFIG6_END; x++)
    {
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            *(pnt_configPage + (U8) (x - EEPROM_CONFIG6_START)) = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }

    /******************
    * CONFIG PAGE (4) *
    ******************/

    pnt_configPage = (U8 *)&configPage4;

    //The first 64 bytes can simply be pulled straight from the configTable
    for(x=EEPROM_CONFIG7_START; x<EEPROM_CONFIG7_END; x++)
    {
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            *(pnt_configPage + (U8) (x - EEPROM_CONFIG7_START)) = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }

    /****************************
    * Boost and vvt tables load *
    ****************************/

    y = EEPROM_CONFIG8_MAP2;

    for(x=EEPROM_CONFIG8_MAP1; x<EEPROM_CONFIG8_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_MAP1;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Read the 8x8 map
            boostTable.axisZ[(offset/8)][offset%8] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG8_MAP2;
        eeprom_code= eeprom_read_byte(y, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Read the 8x8 map
            vvtTable.axisZ[(offset/8)][offset%8] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

        y++;
    }

    //RPM bins
    y = EEPROM_CONFIG8_XBINS2;

    for(x=EEPROM_CONFIG8_XBINS1; x<EEPROM_CONFIG8_YBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_XBINS1;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            boostTable.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);
        }
        else
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG8_XBINS2;
        eeprom_code= eeprom_read_byte(y, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            vvtTable.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);
        }
        else
        {
            return eeprom_code;
        }

        y++;
    }

    //TPS/MAP bins
    y = EEPROM_CONFIG8_YBINS2;

    for(x=EEPROM_CONFIG8_YBINS1; x<EEPROM_CONFIG8_XSIZE2; x++)
    {
        offset = x - EEPROM_CONFIG8_YBINS1;
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
            boostTable.axisY[offset] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG8_YBINS2;

        eeprom_code= eeprom_read_byte(y, &eeprom_data);

        if(eeprom_code == 0)
        {
            //TABLE_LOAD_MULTIPLIER is NOT used for VVT as it is TPS based (0-100)
            vvtTable.axisY[offset] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

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
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            trim1Table.axisZ[(offset/6)][offset%6] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG9_MAP2;
        eeprom_code= eeprom_read_byte(y, &eeprom_data);

        if(eeprom_code == 0)
        {
            trim2Table.axisZ[(offset/6)][offset%6] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

        offset = z - EEPROM_CONFIG9_MAP3;
        eeprom_code= eeprom_read_byte(z, &eeprom_data);

        if(eeprom_code == 0)
        {
            trim3Table.axisZ[(offset/6)][offset%6] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

        offset = i - EEPROM_CONFIG9_MAP4;
        eeprom_code= eeprom_read_byte(i, &eeprom_data);

        if(eeprom_code == 0)
        {
            trim4Table.axisZ[(offset/6)][offset%6] = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }

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
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            trim1Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);
        }
        else
        {
            return eeprom_code;
        }


        offset = y - EEPROM_CONFIG9_XBINS2;
        eeprom_code= eeprom_read_byte(y, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            trim2Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);

        }
        else
        {
            return eeprom_code;
        }

        offset = z - EEPROM_CONFIG9_XBINS3;
        eeprom_code= eeprom_read_byte(z, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            trim3Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);
        }
        else
        {
            return eeprom_code;
        }


        offset = i - EEPROM_CONFIG9_XBINS4;
        eeprom_code= eeprom_read_byte(i, &eeprom_data);

        if(eeprom_code == 0)
        {
            //RPM bins are divided by 100 when stored. Multiply them back now
            trim4Table.axisX[offset] = (eeprom_data * TABLE_RPM_MULTIPLIER);
        }
        else
        {
            return eeprom_code;
        }


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
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Table load is divided by 2 (Allows for MAP up to 511)
            trim1Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }


        offset = y - EEPROM_CONFIG9_YBINS2;
        eeprom_code= eeprom_read_byte(y, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Table load is divided by 2 (Allows for MAP up to 511)
            trim2Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }


        offset = z - EEPROM_CONFIG9_YBINS3;
        eeprom_code= eeprom_read_byte(z, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Table load is divided by 2 (Allows for MAP up to 511)
            trim3Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }


        offset = i - EEPROM_CONFIG9_YBINS4;
        eeprom_code= eeprom_read_byte(i, &eeprom_data);

        if(eeprom_code == 0)
        {
            //Table load is divided by 2 (Allows for MAP up to 511)
            trim4Table.axisY[offset] = eeprom_data * TABLE_LOAD_MULTIPLIER;
        }
        else
        {
            return eeprom_code;
        }

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
        eeprom_code= eeprom_read_byte(x, &eeprom_data);

        if(eeprom_code == 0)
        {
            *(pnt_configPage + (U8) (x - EEPROM_CONFIG11_START)) = eeprom_data;
        }
        else
        {
            return eeprom_code;
        }
    }


    /********************
    * Tuareg config     *
    ********************/
    eeprom_code= load_DecoderConfig();

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= load_IgnitionConfig();

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= load_SensorCalibration();

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //success
    return RETURN_OK;

}









/****************************************************************************************************************************************************
*
* Takes the current configuration (config pages and maps)
* and writes them to EEPROM as per the layout defined in eeprom_layout.h
*
TODO remove map dimensions from eeprom
****************************************************************************************************************************************************/
U32 config_write()
{

    U16 offset, x, y, z, i;
    U8 * pnt_configPage;
    U32 eeprom_code;


    /*************
    * Fuel table *
    *************/

    eeprom_code= eeprom_update(EEPROM_CONFIG1_XSIZE, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }

    for(x=EEPROM_CONFIG1_MAP; x<EEPROM_CONFIG1_XBINS; x++)
    {
        offset = x - EEPROM_CONFIG1_MAP;

        eeprom_code= eeprom_update(x, (fuelTable.axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE]));

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    //RPM bins
    for(x=EEPROM_CONFIG1_XBINS; x<EEPROM_CONFIG1_YBINS; x++)
    {
        offset = x - EEPROM_CONFIG1_XBINS;

        eeprom_code= eeprom_update(x, (fuelTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    //TPS/MAP bins
    for(x=EEPROM_CONFIG1_YBINS; x<EEPROM_CONFIG2_START; x++)
    {
        offset = x - EEPROM_CONFIG1_YBINS;

        //Table load is divided by 2 (Allows for MAP up to 511)
        eeprom_code= eeprom_update(x, fuelTable.axisY[offset] / TABLE_LOAD_MULTIPLIER);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }


    /*******************
    * CONFIG PAGE (1)  *
    *******************/

    pnt_configPage = (U8 *)&configPage1;

    for(x=EEPROM_CONFIG2_START; x<EEPROM_CONFIG2_END; x++)
    {
        eeprom_code= eeprom_update(x, *(pnt_configPage + (U8)(x - EEPROM_CONFIG2_START)) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }


    /*****************
    * Ignition table *
    *****************/
/*
    eeprom_code= eeprom_update(EEPROM_CONFIG3_XSIZE, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }

    for(x=EEPROM_CONFIG3_MAP; x<EEPROM_CONFIG3_XBINS; x++)
    {
        offset = x - EEPROM_CONFIG3_MAP;

        eeprom_code= eeprom_update(x, (ignitionTable.axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE]) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    //RPM bins
    for( x=EEPROM_CONFIG3_XBINS; x<EEPROM_CONFIG3_YBINS; x++)
    {
        offset = x - EEPROM_CONFIG3_XBINS;

        eeprom_code= eeprom_update(x, (U8)(ignitionTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    //TPS/MAP bins
    for( x=EEPROM_CONFIG3_YBINS; x<EEPROM_CONFIG4_START; x++)
    {
        offset = x - EEPROM_CONFIG3_YBINS;

        //Table load is divided by 2 (Allows for MAP up to 511)
        eeprom_code= eeprom_update(x, ignitionTable.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }
*/

    //ignition table handled by table module
    eeprom_code= write_3D_table(&ignitionTable_TPS, EEPROM_CONFIG3_MAP, 100, 2);


    /*******************
    * CONFIG PAGE (2)  *
    *******************/

    pnt_configPage = (U8 *)&configPage2;

    for(x=EEPROM_CONFIG4_START; x<EEPROM_CONFIG4_END; x++)
    {
        eeprom_code= eeprom_update(x, *(pnt_configPage + (U8)(x - EEPROM_CONFIG4_START)) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }


    /*************
    * AFR TABLE  *
    *************/

    eeprom_code= eeprom_update(EEPROM_CONFIG5_XSIZE, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }

    eeprom_code= eeprom_update(EEPROM_CONFIG5_YSIZE, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }


    for( x=EEPROM_CONFIG5_MAP; x<EEPROM_CONFIG5_XBINS; x++)
    {
        offset = x - EEPROM_CONFIG5_MAP;

        eeprom_code= eeprom_update(x, (afrTable.axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE]) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    //RPM bins
    for(x=EEPROM_CONFIG5_XBINS; x<EEPROM_CONFIG5_YBINS; x++)
    {
        offset = x - EEPROM_CONFIG5_XBINS;

        eeprom_code= eeprom_update(x, (U8) (afrTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    //TPS/MAP bins
    for(x=EEPROM_CONFIG5_YBINS; x<EEPROM_CONFIG6_START; x++)
    {
        offset = x - EEPROM_CONFIG5_YBINS;

        //Table load is divided by 2 (Allows for MAP up to 511)
        eeprom_code= eeprom_update(x, afrTable.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }


    /*******************
    * CONFIG PAGE (2)  *
    *******************/

    pnt_configPage = (U8 *)&configPage3;

    for(x=EEPROM_CONFIG6_START; x<EEPROM_CONFIG6_END; x++)
    {
        eeprom_code= eeprom_update(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG6_START)) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    /*******************
    * CONFIG PAGE (4)  *
    *******************/

    pnt_configPage = (U8 *)&configPage4;

    for(x=EEPROM_CONFIG7_START; x<EEPROM_CONFIG7_END; x++)
    {
        eeprom_code= eeprom_update(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG7_START)) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }

    /****************************
    * Boost and vvt tables load *
    ****************************/

    eeprom_code= eeprom_update(EEPROM_CONFIG8_XSIZE1, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }

    eeprom_code= eeprom_update(EEPROM_CONFIG8_YSIZE1, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }

    eeprom_code= eeprom_update(EEPROM_CONFIG8_XSIZE2, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }

    eeprom_code= eeprom_update(EEPROM_CONFIG8_YSIZE2, TABLE3D_DIMENSION);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }


    y = EEPROM_CONFIG8_MAP2;

    for(x=EEPROM_CONFIG8_MAP1; x<EEPROM_CONFIG8_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_MAP1;

        eeprom_code= eeprom_update(x, (boostTable.axisZ[(offset/8)][offset%8]) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG8_MAP2;

        eeprom_code= eeprom_update(y, (vvtTable.axisZ[(offset/8)][offset%8]) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        y++;
    }

    //RPM bins
    y = EEPROM_CONFIG8_XBINS2;

    for(x=EEPROM_CONFIG8_XBINS1; x<EEPROM_CONFIG8_YBINS1; x++)
    {
        offset = x - EEPROM_CONFIG8_XBINS1;

        eeprom_code= eeprom_update(x, (U8) (boostTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG8_XBINS2;

        //RPM bins are divided by 100
        eeprom_code= eeprom_update(y, (U8) (vvtTable.axisX[offset]/TABLE_RPM_MULTIPLIER) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        y++;
    }

    //TPS/MAP bins
    y=EEPROM_CONFIG8_YBINS2;

    for(x=EEPROM_CONFIG8_YBINS1; x<EEPROM_CONFIG8_XSIZE2; x++)
    {
        offset = x - EEPROM_CONFIG8_YBINS1;

        //TABLE_LOAD_MULTIPLIER is NOT used for boost as it is TPS based (0-100)
        eeprom_code= eeprom_update(x, boostTable.axisY[offset]);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG8_YBINS2;

        //TABLE_LOAD_MULTIPLIER is NOT used for VVT as it is TPS based (0-100)
        eeprom_code= eeprom_update(y, vvtTable.axisY[offset]);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        y++;
  }

    /*******************
    * Fuel trim tables *
    *******************/

    //Write the boost Table RPM dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_XSIZE1,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table MAP/TPS dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_YSIZE1,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table RPM dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_XSIZE2,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table MAP/TPS dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_YSIZE2,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table RPM dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_XSIZE3,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table MAP/TPS dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_YSIZE3,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table RPM dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_XSIZE4,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    //Write the boost Table MAP/TPS dimension size
    eeprom_code= eeprom_update(EEPROM_CONFIG9_YSIZE4,TABLE3D_DIMENSION);

    if(eeprom_code != 0)
        {
            return eeprom_code;
        }

    y = EEPROM_CONFIG9_MAP2;
    z = EEPROM_CONFIG9_MAP3;
    i = EEPROM_CONFIG9_MAP4;

    for(x=EEPROM_CONFIG9_MAP1; x<EEPROM_CONFIG9_XBINS1; x++)
    {
        offset = x - EEPROM_CONFIG9_MAP1;
        eeprom_code= eeprom_update(x, (trim1Table.axisZ[(offset/6)][offset%6]) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG9_MAP2;
        eeprom_code= eeprom_update(y, trim2Table.axisZ[(offset/6)][offset%6]);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = z - EEPROM_CONFIG9_MAP3;
        eeprom_code= eeprom_update(z, trim3Table.axisZ[(offset/6)][offset%6]);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = i - EEPROM_CONFIG9_MAP4;
        eeprom_code= eeprom_update(i, trim4Table.axisZ[(offset/6)][offset%6]);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

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
        eeprom_code= eeprom_update(x, (U8) (trim1Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG9_XBINS2;

        //RPM bins are divided by 100 and converted to a byte
        eeprom_code= eeprom_update(y, (U8) (trim2Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = z - EEPROM_CONFIG9_XBINS3;

        //RPM bins are divided by 100 and converted to a byte
        eeprom_code= eeprom_update(z, (U8) (trim3Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = i - EEPROM_CONFIG9_XBINS4;

        //RPM bins are divided by 100 and converted to a byte
        eeprom_code= eeprom_update(i, (U8) (trim4Table.axisX[offset]/TABLE_RPM_MULTIPLIER));

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

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
        eeprom_code= eeprom_update(x, trim1Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = y - EEPROM_CONFIG9_YBINS2;

        //Table load is divided by 2 (Allows for MAP up to 511)
        eeprom_code=  eeprom_update(y, trim2Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

       if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = z - EEPROM_CONFIG9_YBINS3;

        //Table load is divided by 2 (Allows for MAP up to 511)
        eeprom_code= eeprom_update(z, trim3Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

        offset = i - EEPROM_CONFIG9_YBINS4;

        //Table load is divided by 2 (Allows for MAP up to 511)
        eeprom_code= eeprom_update(i, trim4Table.axisY[offset]/TABLE_LOAD_MULTIPLIER);

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }

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
        eeprom_code= eeprom_update(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG10_START)) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }


    /********************
    * CONFIG PAGE (11)  *
    ********************/

    pnt_configPage = (U8 *)&configPage11;

    for(x=EEPROM_CONFIG11_START; x<EEPROM_CONFIG11_END; x++)
    {
        eeprom_code= eeprom_update(x, *(pnt_configPage + (U8) (x - EEPROM_CONFIG11_START)) );

        if(eeprom_code != 0)
        {
            return eeprom_code;
        }
    }


    /********************
    * Tuareg config     *
    ********************/
    eeprom_code= write_DecoderConfig();

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= write_IgnitionConfig();

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= write_SensorCalibration();

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure






    /******************************************************
    * THE END                                             *
    *******************************************************/
    return 0;
}



/****************************************************************************************************************************************************
 * This routine is used for doing any data conversions that are required during firmware changes
 * This prevents users getting difference reports in TS when such a data change occurs.
 * It also can be used for setting good values when there are variables that move locations in the ini
 * When a user skips multiple firmware versions at a time, this will roll through the updates 1 at a time
 ****************************************************************************************************************************************************/
U32 migrate_configData()
{
    U8 eeprom_data;
    U32 eeprom_code;


    /**
    check DATA_VERSION
    (Brand new eeprom)
    */
    eeprom_code= eeprom_read_byte(EEPROM_DATA_VERSION, &eeprom_data);

    if(eeprom_code != 0)
    {
        return eeprom_code;
    }
#warning TODO (oli#4#): Add an eeprom layout and config version check (at config load)

    if( (eeprom_data == 0) || (eeprom_data == 255) )
    {
        eeprom_code= eeprom_update(EEPROM_DATA_VERSION, CURRENT_DATA_VERSION);
    }

    return eeprom_code;
}


/**
*
* reads decoder config data from eeprom
*
*/
U32 load_DecoderConfig()
{
    U32 data, x;
    U8 eeprom_data;
    U32 eeprom_code;


    //U16 trigger_position_map[POSITION_COUNT]
    for(x=0; x < CRK_POSITION_COUNT; x++)
    {
        //every config item is 2 bytes in width
        eeprom_code= eeprom_read_bytes(EEPROM_CONFIG12_TRIGGER_POSITION_MAP_START + 2*x, &data, 2);

        if(eeprom_code) return eeprom_code; //exit on eeprom read failure

        configPage12.trigger_position_map.a_deg[x]= (U16) data;
    }


    //S16 decoder_offset_deg
    eeprom_code= eeprom_read_bytes(EEPROM_CONFIG12_DECODER_OFFSET, &data, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.decoder_offset_deg= (S16) data;


    //U16 decoder_delay_us
    eeprom_code= eeprom_read_bytes(EEPROM_CONFIG12_DECODER_DELAY, &data, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.decoder_delay_us= (U16) data;


    //U8 crank_noise_filter
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG12_CRANK_NOISE_FILTER, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.crank_noise_filter= eeprom_data;


    //U8 sync_ratio_min_pct
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG12_SYNC_RATIO_MIN, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.sync_ratio_min_pct= eeprom_data;


    //U8 sync_ratio_max_pct
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG12_SYNC_RATIO_MAX, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.sync_ratio_max_pct= eeprom_data;


    //U8 sync_stability_thrs
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG12_SYNC_STABILITY_THRS, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.sync_stability_thrs= eeprom_data;


    //U8 decoder_timeout_s
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG12_DECODER_TIMEOUT, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage12.decoder_timeout_s= eeprom_data;

    //all done
    return RETURN_OK;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_DecoderConfig()
{

    /**
    config page 12 - crank decoder
    */
    configPage12.trigger_position_map.a_deg[CRK_POSITION_A1]= DEFAULT_CONFIG12_POSITION_A1_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_A2]= DEFAULT_CONFIG12_POSITION_A2_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_B1]= DEFAULT_CONFIG12_POSITION_B1_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_B2]= DEFAULT_CONFIG12_POSITION_B2_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_C1]= DEFAULT_CONFIG12_POSITION_C1_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_C2]= DEFAULT_CONFIG12_POSITION_C2_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_D1]= DEFAULT_CONFIG12_POSITION_D1_ANGLE;
    configPage12.trigger_position_map.a_deg[CRK_POSITION_D2]= DEFAULT_CONFIG12_POSITION_D2_ANGLE;

    configPage12.decoder_offset_deg= DEFAULT_CONFIG12_DECODER_OFFSET;
    configPage12.decoder_delay_us= DEFAULT_CONFIG12_DECODER_DELAY;
    configPage12.crank_noise_filter= DEFAULT_CONFIG12_CRANK_NOISE_FILTER;
    configPage12.sync_ratio_min_pct= DEFAULT_CONFIG12_SYNC_RATIO_MIN;
    configPage12.sync_ratio_max_pct= DEFAULT_CONFIG12_SYNC_RATIO_MAX;
    configPage12.sync_stability_thrs= DEFAULT_CONFIG12_SYNC_STABILITY_THRS;
    configPage12.decoder_timeout_s= DEFAULT_CONFIG12_DECODER_TIMEOUT_S;

}

/**
*
* writes decoder config data to eeprom
*
*/
U32 write_DecoderConfig()
{
    U32 eeprom_code, x;


    //U16 trigger_position_map[CRK_POSITION_COUNT]
    for(x=0; x < CRK_POSITION_COUNT; x++)
    {
        //every config item is 2 bytes in width
        eeprom_code= eeprom_update_bytes(EEPROM_CONFIG12_TRIGGER_POSITION_MAP_START + 2*x, configPage12.trigger_position_map.a_deg[x], 2);

        if(eeprom_code) return eeprom_code; //exit on eeprom read failure
    }


    //S16 decoder_offset_deg
    eeprom_code= eeprom_update_bytes(EEPROM_CONFIG12_DECODER_OFFSET, configPage12.decoder_offset_deg, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //U16 decoder_delay_us
    eeprom_code= eeprom_update_bytes(EEPROM_CONFIG12_DECODER_DELAY, configPage12.decoder_delay_us, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //U8 crank_noise_filter
    eeprom_code= eeprom_update(EEPROM_CONFIG12_CRANK_NOISE_FILTER, configPage12.crank_noise_filter);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //U8 sync_ratio_min_pct
    eeprom_code= eeprom_update(EEPROM_CONFIG12_SYNC_RATIO_MIN, configPage12.sync_ratio_min_pct);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //U8 sync_ratio_max_pct
    eeprom_code= eeprom_update(EEPROM_CONFIG12_SYNC_RATIO_MAX, configPage12.sync_ratio_max_pct);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //U8 sync_stability_thrs
    eeprom_code= eeprom_update(EEPROM_CONFIG12_SYNC_STABILITY_THRS, configPage12.sync_stability_thrs);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //U8 decoder_timeout_s
    eeprom_code= eeprom_update(EEPROM_CONFIG12_DECODER_TIMEOUT, configPage12.decoder_timeout_s);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //all done
    return RETURN_OK;
}


/**
*
* reads ignition config data from eeprom
*
*/
U32 load_IgnitionConfig()
{
    U32 data;
    U8 eeprom_data;
    U32 eeprom_code;


    //U16 dynamic_min_rpm
    eeprom_code= eeprom_read_bytes(EEPROM_CONFIG13_DYNAMIC_MIN, &data, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.dynamic_min_rpm= (U16) data;


    //U16 dynamic_dwell_us
    eeprom_code= eeprom_read_bytes(EEPROM_CONFIG13_DYNAMIC_DWELL, &data, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.dynamic_dwell_us= (U16) data;


    //U8 safety_margin_us
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG13_SAFETY_MARGIN, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.safety_margin_us= eeprom_data;


    //crank_position_t idle_ignition_position
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG13_IDLE_IGN_POS, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.idle_ignition_position= eeprom_data;


    //crank_position_t idle_dwell_position
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG13_IDLE_DWELL_POS, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.idle_dwell_position= eeprom_data;


    //U8 idle_advance_deg
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG13_IDLE_ADVANCE_DEG, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.idle_advance_deg= eeprom_data;


    //U8 idle_dwell_deg
    eeprom_code= eeprom_read_byte(EEPROM_CONFIG13_IDLE_DWELL_DEG, &eeprom_data);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage13.idle_dwell_deg= eeprom_data;


    //all done
    return RETURN_OK;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_IgnitionConfig()
{



}


/**
*
* writes ignition config data to eeprom
*
*/
U32 write_IgnitionConfig()
{
    U32 eeprom_code;

    //U16 dynamic_min_rpm
    eeprom_code= eeprom_update_bytes(EEPROM_CONFIG13_DYNAMIC_MIN, configPage13.dynamic_min_rpm, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //U16 dynamic_dwell_us
    eeprom_code= eeprom_update_bytes(EEPROM_CONFIG13_DYNAMIC_DWELL, configPage13.dynamic_dwell_us, 2);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //U8 safety_margin_us
    eeprom_code= eeprom_update(EEPROM_CONFIG13_SAFETY_MARGIN, configPage13.safety_margin_us);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //crank_position_t idle_ignition_position
    eeprom_code= eeprom_update(EEPROM_CONFIG13_IDLE_IGN_POS, configPage13.idle_ignition_position);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //crank_position_t idle_dwell_position
    eeprom_code= eeprom_update(EEPROM_CONFIG13_IDLE_DWELL_POS, configPage13.idle_dwell_position);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //U8 idle_advance_deg
    eeprom_code= eeprom_update(EEPROM_CONFIG13_IDLE_ADVANCE_DEG, configPage13.idle_advance_deg);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //U8 idle_dwell_deg
    eeprom_code= eeprom_update(EEPROM_CONFIG13_IDLE_DWELL_DEG, configPage13.idle_dwell_deg);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure


    //all done
    return RETURN_OK;
}



/**
*
* reads sensor calibration data from eeprom
*
*/
U32 load_SensorCalibration()
{
    U32 data, eeprom_code;

    /**
    CLT
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_CLT_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.CLT_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_CLT_N, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.CLT_calib_N= compose_float(data);


    /**
    IAT
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_IAT_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.IAT_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_IAT_N, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.IAT_calib_N= compose_float(data);


    /**
    TPS
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_TPS_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.TPS_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_TPS_N, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.TPS_calib_N= compose_float(data);


    /**
    MAP
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_MAP_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.MAP_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_MAP_N, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.MAP_calib_N= compose_float(data);


    /**
    BARO
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_BARO_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.BARO_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_BARO_N, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.BARO_calib_N= compose_float(data);


    /**
    O2
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_O2_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.O2_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_O2_N, &data, 4);

    configPage9.O2_calib_N= compose_float(data);


    /**
    VBAT
    */
    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_VBAT_M, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.VBAT_calib_M= compose_float(data);

    eeprom_code= eeprom_read_bytes(EEPROM_CALIBRATION_VBAT_N, &data, 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    configPage9.VBAT_calib_N= compose_float(data);


    //all done
    return RETURN_OK;
}


/**
*
* writes sensor calibration data to eeprom
*
*/
U32 write_SensorCalibration()
{
    U32 eeprom_code;

    /**
    IAT
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_IAT_M, serialize_float(configPage9.IAT_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_IAT_N, serialize_float(configPage9.IAT_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    /**
    CLT
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_CLT_M, serialize_float(configPage9.CLT_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_CLT_N, serialize_float(configPage9.CLT_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    /**
    TPS
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_TPS_M, serialize_float(configPage9.TPS_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_TPS_N, serialize_float(configPage9.TPS_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    /**
    MAP
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_MAP_M, serialize_float(configPage9.MAP_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_MAP_N, serialize_float(configPage9.MAP_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    /**
    BARO
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_BARO_M, serialize_float(configPage9.BARO_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_BARO_N, serialize_float(configPage9.BARO_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    /**
    O2
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_O2_M, serialize_float(configPage9.O2_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_O2_N, serialize_float(configPage9.O2_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    /**
    VBAT
    */
    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_VBAT_M, serialize_float(configPage9.VBAT_calib_M), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    eeprom_code= eeprom_update_bytes(EEPROM_CALIBRATION_VBAT_N, serialize_float(configPage9.VBAT_calib_N), 4);

    if(eeprom_code) return eeprom_code; //exit on eeprom read failure

    //all done
    return RETURN_OK;
}





/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void config_load_essentials()
{

    // crank decoder
    load_essential_DecoderConfig();

}
