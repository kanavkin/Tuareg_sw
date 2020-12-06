/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/

#include <stdlib.h>


#include "table.h"
#include "legacy_config.h"
#include "Tuareg_config.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include <math.h>

#include "Tuareg.h"
#include "Tuareg_ID.h"
#include "Tuareg_errors.h"

#include "eeprom.h"
#include "eeprom_layout.h"


/**
tables
*/
volatile table3D_t ignitionTable_TPS, ignitionTable_MAP;

volatile table3D_t fuelTable, afrTable;
volatile table3D_t boostTable, vvtTable;
volatile table3D_t trim1Table, trim2Table, trim3Table, trim4Table;
volatile table2D taeTable, WUETable, crankingEnrichTable, dwellVCorrectionTable;
volatile table2D injectorVCorrectionTable, IATDensityCorrectionTable, IATRetardTable, rotarySplitTable;


/**
This function pulls a 1D linear interpolated value from a 2D table
*/
VF32 table2D_getValue(volatile table2D *fromTable, VU32 X)
{
    S32 xMin =0, xMax =0, yMin =0, yMax =0, xMax_index =0, i =0;
    F32 m, y;

    /**
    clip the requested X to fit the interval covered by this table
    (borrowing xM variables)
    */
    xMin = fromTable->axisX[0];
    xMax = fromTable->axisX[ fromTable->dimension -1 ];
    if(X > xMax) { X = xMax; }
    if(X < xMin) { X = xMin; }

    /**
    check if we're still in the same X interval as last time
    */
    xMax = fromTable->axisX[fromTable->last_Xmax_index];
    xMin = fromTable->axisX[fromTable->last_Xmax_index -1];

    if ( (X < xMax) && (X > xMin) )
    {
        //xM already set
        xMax_index = fromTable->last_Xmax_index;
    }
    else
    {
        /**
        Loop from the table end to find a suitable x interval
        */
        for(i = fromTable->dimension-1; i >= 0; i--)
        {
            /**
            quick exit: direct fit
            the requested X value has been found among the X values -> take Y from the defined values

            or

            Last available element:
            looping through the values from high to low has not revealed a suitable X interval
            -> take the minimum defined Y value

            */
            if( (X == fromTable->axisX[i]) || (i == 0) )
            {
                //exit here taking the Y value from table
                return fromTable->axisY[i];
            }

            /**
            interval fit approach:
            as X is not a direct fit it could be between axisX[i] and axisX[i-1]

            axisX[i] > axisX[i-1]

            because of the quick exit for (i==0) above this code is reached only for [1 < i < dimension -1]
            */
            if( (X < fromTable->axisX[i]) && (X > fromTable->axisX[i-1]) )
            {
                //found!
                xMax_index= i;

                //store X value for next time
                fromTable->last_Xmax_index= xMax_index;

                //exit the search loop
                break;
            }
        }
    }

    /**
    xMin/Max indexes found, look up data for calculation
    */
    xMax= fromTable->axisX[xMax_index];
    xMin= fromTable->axisX[xMax_index -1];
    yMax= fromTable->axisY[xMax_index];
    yMin= fromTable->axisY[xMax_index -1];

    /**
    y=  m * X + n
    y= ( (dY/dX) * (X - xMin) * yMin )
    y= yMin + mDx
    */
    m=  (yMax - yMin) / (xMax - xMin);
    y= m * (X - xMin) + yMin;

    return y;
}


/**
This function pulls a value from a 3D table given a target for X and Y coordinates.
It performs a bilinear interpolation
*/
VF32 lookup_3D_table(volatile table3D_t * fromTable, VU32 X, VU32 Y)
{
    F32 A =0, B =0, C =0, D =0;
    U32 xMin =0, xMax =0, yMin =0, yMax =0;
    U32 xMin_index =0, xMax_index =0, yMin_index =0, yMax_index =0;
    U32 i;

    /**
    X handling
    */

    /**
    clip the requested X to fit the interval covered by this table
    (borrowing xM variables)
    */
    xMin = fromTable->axisX[0];
    xMax = fromTable->axisX[ TABLE3D_DIMENSION -1 ];
    if(X > xMax) { X = xMax; }
    if(X < xMin) { X = xMin; }

    /**
    check if we're still in the same X interval as last time
    preset xM with the ones from last request
    */
    xMax = fromTable->axisX[fromTable->last_Xmax_index];
    xMin = fromTable->axisX[fromTable->last_Xmax_index -1];

    /**
    check if we're still in the same X environment as last time
    or if it is in a neighbor cell
    (engine rpm changes slowly between cycles)
    */
    if( (X < xMax) && (X > xMin) )
    {
        //xM already set
        xMax_index = fromTable->last_Xmax_index;
        xMin_index = xMax_index -1;
        //last_xMax_index remains valid

    }
    else if ( ((fromTable->last_Xmax_index + 1) < TABLE3D_DIMENSION ) && (X > xMax) && (X < fromTable->axisX[fromTable->last_Xmax_index +1 ])  )
    {
        //x is in right neighbor interval
        xMax_index= fromTable->last_Xmax_index + 1;
        xMin_index= fromTable->last_Xmax_index;
        xMax= fromTable->axisX[xMax_index];
        xMin= fromTable->axisX[xMin_index];

        //store for next time
        fromTable->last_Xmax_index= xMax_index;
    }
    else if ( (fromTable->last_Xmax_index > 1 ) && (X < xMin) && (X > fromTable->axisX[fromTable->last_Xmax_index -2]) )
    {
        //x is in left neighbor interval
        xMax_index= fromTable->last_Xmax_index -1;
        xMin_index= fromTable->last_Xmin_index -2;
        xMax= fromTable->axisX[xMax_index];
        xMin= fromTable->axisX[xMin_index];

        //store for next time
        fromTable->last_Xmax_index= xMax_index;
    }
    else
    {
        /**
        Loop from the end to find a suitable x interval
        */
        for(i = TABLE3D_DIMENSION -1; i >= 0; i--)
        {
            /**
            If the requested X value has been directly found on the x axis
            but we have to provide a suitable x interval for interpolation
            */
            if ( (X == fromTable->axisX[i]) || (i == 0) )
            {
                if(i == 0)
                {
                    /**
                    first element  can be xMin only
                    interpolation square to the right
                    */
                    xMax_index= i+1;
                    xMin_index= i;
                }
                else
                {
                    /**
                    take i as xMax
                    interpolation square to the left
                    */
                    xMax_index= i;
                    xMin_index= i-1;
                }

                xMax= fromTable->axisX[xMax_index];
                xMin= fromTable->axisX[xMin_index];
                break;
            }

            /**
            Standard scenario
            The requested X value is between axisX[i] and axisX[i-1]
            */
            if ( (X < fromTable->axisX[i]) && (X > fromTable->axisX[i-1]) )
            {
                xMax= fromTable->axisX[i];
                xMin= fromTable->axisX[i-1];
                xMax_index= i;
                xMin_index= i-1;
                fromTable->last_Xmax_index= xMax_index;
                break;
            }
        }
    }

    /**
    Y handling
    */

    /**
    clip the requested Y to fit the interval covered by this table
    (borrowing yM variables)
    */
    yMin = fromTable->axisY[0];
    yMax = fromTable->axisY[ TABLE3D_DIMENSION -1 ];
    if(Y > yMax) { Y = yMax; }
    if(Y < yMin) { Y = yMin; }

    /**
    check if we're still in the same Y interval as last time
    preset xM with the ones from last request
    */
    yMax = fromTable->axisY[fromTable->last_Ymax_index];
    yMin = fromTable->axisY[fromTable->last_Ymax_index -1];

    /**
    check if we're still in the same Y environment as last time
    or if it is in a neighbor cell
    (engine rpm changes slowly between cycles)
    */
    if( (Y < yMax) && (Y > yMin) )
    {
        //yM already set
        yMax_index = fromTable->last_Ymax_index;
        yMin_index = yMax_index -1;
        //last_yMax_index remains valid

    }
    else if ( ((fromTable->last_Ymax_index + 1) < TABLE3D_DIMENSION ) && (Y > yMax) && (Y < fromTable->axisY[fromTable->last_Ymax_index +1 ])  )
    {
        //y is in right neighbor interval
        yMax_index= fromTable->last_Ymax_index + 1;
        yMin_index= fromTable->last_Ymax_index;
        yMax= fromTable->axisY[yMax_index];
        yMin= fromTable->axisY[yMin_index];

        //store for next time
        fromTable->last_Ymax_index= yMax_index;
    }
    else if ( (fromTable->last_Ymax_index > 1 ) && (Y < yMin) && (Y > fromTable->axisY[fromTable->last_Ymax_index -2]) )
    {
        //y is in left neighbor interval
        yMax_index= fromTable->last_Ymax_index -1;
        yMin_index= fromTable->last_Ymin_index -2;
        yMax= fromTable->axisY[yMax_index];
        yMin= fromTable->axisY[yMin_index];

        //store for next time
        fromTable->last_Ymax_index= yMax_index;
    }
    else
    {
        /**
        Loop from the end to find a suitable y interval
        */
        for(i = TABLE3D_DIMENSION -1; i >= 0; i--)
        {
            /**
            If the requested Y value has been directly found on the y axis
            but we have to provide a suitable y interval for interpolation
            */
            if ( (Y == fromTable->axisY[i]) || (i == 0) )
            {
                if(i == 0)
                {
                    /**
                    first element  can be yMin only
                    interpolation square to the right
                    */
                    yMax_index= i+1;
                    yMin_index= i;
                }
                else
                {
                    /**
                    take i as yMax
                    interpolation square to the left
                    */
                    yMax_index= i;
                    yMin_index= i-1;
                }

                yMax= fromTable->axisY[yMax_index];
                yMin= fromTable->axisY[yMin_index];
                break;
            }

            /**
            Standard scenario
            The requested Y value is between axisY[i] and axisY[i-1]
            */
            if ( (Y < fromTable->axisY[i]) && (Y > fromTable->axisY[i-1]) )
            {
                yMax= fromTable->axisY[i];
                yMin= fromTable->axisY[i-1];
                yMax_index= i;
                yMin_index= i-1;
                fromTable->last_Ymax_index= yMax_index;
                break;
            }
        }
    }

    /*************************************************
    At this point we have the 4 corners of the map
    where the interpolated value will fall in

    C(yMax,xMin)  D(yMax,xMax)
    A(yMin,xMin)  B(yMin,xMax)
    ************************************************/
    A= fromTable->axisZ[yMin_index][xMin_index];
    B= fromTable->axisZ[yMin_index][xMax_index];
    C= fromTable->axisZ[yMax_index][xMin_index];
    D= fromTable->axisZ[yMax_index][xMax_index];

    /**
    Check that all values aren't just the same
    (This regularly happens with things like the fuel trim maps)

/// TODO (oli#4#): improve float equality check

    if( (A == B) && (A == C) && (A == D) )
    {
        return A;
    }
    */

    /**
    RESULTS finally
    */
    A *= (xMax - X) * (yMax - Y);
    B *= (X -xMin)  * (yMax - Y);
    C *= (xMax - X) * (Y - yMin);
    D *= (X - xMin) * (Y -yMin);

    return (U32) ( (A + B + C +D) / ((xMax - xMin) * (yMax - yMin)) );
}


/****************************************************************************************************************************************************
*
* Load 3D table data from EEPROM
*
* table dimension is fixed to TABLE3D_DIMENSION
* every item is represented by 1 Byte of eeprom data
* x and y axis are scaled by their scaling factors
****************************************************************************************************************************************************/
exec_result_t load_3D_table(volatile table3D_t * pTarget, VU32 BaseAddress, VU32 Scaling_X, VU32 Scaling_Y)
{
    U32 data;
    U8 eeprom_data;
    eeprom_result_t ee_result;


    U32 offset, address;

    //Z-axis -> U8 from U8
    for(offset=0; offset < (TABLE3D_DIMENSION * TABLE3D_DIMENSION); offset++)
    {
        //resulting eeprom memory address
        address= offset + BaseAddress;

        //read 1 byte from eeprom
        ee_result= eeprom_read_byte(address, &eeprom_data);

        ASSERT_CONFIG_SUCESS(ee_result);

        pTarget->axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE] = eeprom_data;
    }

    //X-axis -> U16 from U8
    for(offset=0; offset < TABLE3D_DIMENSION; offset++)
    {
        address= offset + BaseAddress + (TABLE3D_DIMENSION * TABLE3D_DIMENSION);

        //read 1 byte from eeprom
        ee_result= eeprom_read_bytes(address, &data, 1);

        ASSERT_CONFIG_SUCESS(ee_result);

        //scale data read with Scaling_X
        pTarget->axisX[offset]= (U16) (data * Scaling_X);
    }


    //Y-axis -> U16 from U8
    for(offset=0; offset < TABLE3D_DIMENSION; offset++)
    {
        address= offset + BaseAddress + (TABLE3D_DIMENSION * TABLE3D_DIMENSION) + TABLE3D_DIMENSION;

        //read 1 byte from eeprom
        ee_result= eeprom_read_bytes(address, &data, 1);

        ASSERT_CONFIG_SUCESS(ee_result);

        //scale data read with Scaling_Y
        pTarget->axisY[offset]= (U16) (data * Scaling_Y);
    }

    //all done
    return EXEC_OK;
}


/****************************************************************************************************************************************************
*
* Save 3D table data to EEPROM
*
* table dimension is fixed to TABLE3D_DIMENSION
* every item is represented by 1 Byte of eeprom data
* x and y axis are scaled by their scaling factors
****************************************************************************************************************************************************/
exec_result_t write_3D_table(volatile table3D_t * pTable, VU32 BaseAddress, VU32 Scaling_X, VU32 Scaling_Y)
{
    U32 offset, address;
    eeprom_result_t ee_result;

    //Z-axis -> U8 from U8
    for(offset=0; offset < (TABLE3D_DIMENSION * TABLE3D_DIMENSION); offset++)
    {
        //resulting eeprom memory address
        address= offset + BaseAddress;

        //update 1 byte in eeprom
        ee_result= eeprom_update_byte(address, pTable->axisZ[offset / TABLE_3D_ARRAYSIZE][offset % TABLE_3D_ARRAYSIZE] );

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    //X-axis -> U16 from U8
    for(offset=0; offset < TABLE3D_DIMENSION; offset++)
    {
        address= offset + BaseAddress + (TABLE3D_DIMENSION * TABLE3D_DIMENSION);

        //update 1 byte in eeprom scaled by Scaling_X
        ee_result= eeprom_update_byte(address, pTable->axisX[offset] / Scaling_X);

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    //Y-axis -> U16 from U8
    for(offset=0; offset < TABLE3D_DIMENSION; offset++)
    {
        address= offset + BaseAddress + (TABLE3D_DIMENSION * TABLE3D_DIMENSION) + TABLE3D_DIMENSION;

        //read 1 byte from eeprom scaled by Scaling_Y
        ee_result= eeprom_update_byte(address, pTable->axisY[offset] / Scaling_Y);

        ASSERT_CONFIG_SUCESS(ee_result);
    }

    //all done
    return EXEC_OK;
}




/****************************************************************************************************************************************************
*
* print 3D table in human readable form
*
* assuming that the title has already been printed
* assuming fixed table dimension of TABLE3D_DIMENSION
* layout:
* left column: Y-Axis
* row below: X-Axis
* data layout: z[y][x]
* orientation (order from low to high) of Y-axis: top to bottom
* orientation (order from low to high) of X-axis: left to right
****************************************************************************************************************************************************/
void print_3D_table(USART_TypeDef * pPort, volatile table3D_t * pTable)
{
    U32 row, column;

    // for every row, printing the top row first (highest value) Z[0..15][x]
    for (row= 0; row < TABLE3D_DIMENSION; row++)
    {
        // Y value for this row
        printf_U(pPort, pTable->axisY[TABLE3D_DIMENSION -1 - row], PAD_5);

        //separator
        print(pPort, " . ");

        // Z values of this row, printing from left to right Z[y][0..15]
        for (column = 0; column < TABLE3D_DIMENSION; column++)
        {
            printf_U(pPort, pTable->axisZ[TABLE3D_DIMENSION -1 - row][column], PAD_2);
            print(pPort, "  ");
        }

        print(pPort, "\r\n");
    }

    //separator
    print(pPort, "       ................................................................................................\r\n");
    print(pPort, "       ");

    // X-axis
    for (column = 0; column < TABLE3D_DIMENSION; column++)
    {
        printf_U(pPort, pTable->axisX[column], PAD_5);
    }

    print(pPort, "\r\n");

}


/****************************************************************************************************************************************************
*
* replace one item in 3D table
*
* offsets:
* 0 .. TABLE3D_DIMENSION² -> z-axis (U8)
* TABLE3D_DIMENSION² .. TABLE3D_DIMENSION² + TABLE3D_DIMENSION -> X-axis (U16)
* TABLE3D_DIMENSION² + TABLE3D_DIMENSION .. TABLE3D_DIMENSION² + 2* TABLE3D_DIMENSION  -> Y-axis (U16)
*
*
****************************************************************************************************************************************************/
exec_result_t modify_3D_table(volatile table3D_t * pTable, U32 Offset, U32 Value)
{
    //range check
    if(Offset >= TABLE3D_DIMENSION * TABLE3D_DIMENSION + 2 * TABLE3D_DIMENSION)
    {
        return EXEC_ERROR;
    }

    if(Offset < TABLE3D_DIMENSION * TABLE3D_DIMENSION)
    {
        // Z-axis
        pTable->axisZ[Offset / TABLE3D_DIMENSION][Offset % TABLE3D_DIMENSION] = (U8) Value;
    }
    else if(Offset < TABLE3D_DIMENSION * TABLE3D_DIMENSION + TABLE3D_DIMENSION )
    {
        // X-axis
        pTable->axisX[Offset - TABLE3D_DIMENSION * TABLE3D_DIMENSION] = (U16) Value;
    }
    else
    {
        // Y-axis
        pTable->axisY[Offset - (TABLE3D_DIMENSION * TABLE3D_DIMENSION + TABLE3D_DIMENSION)] = (U16) Value;
    }

    return EXEC_OK;
}



