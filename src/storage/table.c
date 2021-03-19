#include <math.h>

#include "table.h"
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg.h"
#include "Tuareg_ID.h"
#include "Tuareg_errors.h"

#include "eeprom.h"
#include "eeprom_layout.h"


const U32 ct2D_data_size= sizeof(t2D_data_t);
const U32 ct3D_data_size= sizeof(t3D_data_t);


/**
This function pulls a 1D linear interpolated value from a 2D table
*/
VF32 getValue_t2D(volatile t2D_t *fromTable, VU32 X)
{
    S32 xMin =0, xMax =0, yMin =0, yMax =0, xMax_index =0, i =0;
    F32 m, y;

    /**
    clip the requested X to fit the interval covered by this table
    (borrowing xM variables)
    */
    xMin = fromTable->data.axisX[0];
    xMax = fromTable->data.axisX[ T2D_DATA_DIMENSION -1 ];
    if(X > xMax) { X = xMax; }
    if(X < xMin) { X = xMin; }

    /**
    check if we're still in the same X interval as last time
    */
    xMax = fromTable->data.axisX[fromTable->mgr.last_Xmax_index];
    xMin = fromTable->data.axisX[fromTable->mgr.last_Xmax_index -1];

    if ( (X < xMax) && (X > xMin) )
    {
        //xM already set
        xMax_index = fromTable->mgr.last_Xmax_index;
    }
    else
    {
        /**
        Loop from the table end to find a suitable x interval
        */
        for(i = T2D_DATA_DIMENSION -1; i >= 0; i--)
        {
            /**
            quick exit: direct fit
            the requested X value has been found among the X values -> take Y from the defined values

            or

            Last available element:
            looping through the values from high to low has not revealed a suitable X interval
            -> take the minimum defined Y value

            */
            if( (X == fromTable->data.axisX[i]) || (i == 0) )
            {
                //exit here taking the Y value from table
                return fromTable->data.axisY[i];
            }

            /**
            interval fit approach:
            as X is not a direct fit it could be between axisX[i] and axisX[i-1]

            axisX[i] > axisX[i-1]

            because of the quick exit for (i==0) above this code is reached only for [1 < i < dimension -1]
            */
            if( (X < fromTable->data.axisX[i]) && (X > fromTable->data.axisX[i-1]) )
            {
                //found!
                xMax_index= i;

                //store X value for next time
                fromTable->mgr.last_Xmax_index= xMax_index;

                //exit the search loop
                break;
            }
        }
    }

    /**
    xMin/Max indexes found, look up data for calculation
    */
    xMax= fromTable->data.axisX[xMax_index];
    xMin= fromTable->data.axisX[xMax_index -1];
    yMax= fromTable->data.axisY[xMax_index];
    yMin= fromTable->data.axisY[xMax_index -1];

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
VF32 getValue_t3D(volatile t3D_t * fromTable, VU32 X, VU32 Y)
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
    xMin = fromTable->data.axisX[0];
    xMax = fromTable->data.axisX[ T3D_DATA_DIMENSION -1 ];
    if(X > xMax) { X = xMax; }
    if(X < xMin) { X = xMin; }

    /**
    check if we're still in the same X interval as last time
    preset xM with the ones from last request
    */
    xMax = fromTable->data.axisX[fromTable->mgr.last_Xmax_index];
    xMin = fromTable->data.axisX[fromTable->mgr.last_Xmax_index -1];

    /**
    check if we're still in the same X environment as last time
    or if it is in a neighbor cell
    (engine rpm changes slowly between cycles)
    */
    if( (X < xMax) && (X > xMin) )
    {
        //xM already set
        xMax_index = fromTable->mgr.last_Xmax_index;
        xMin_index = xMax_index -1;
        //last_xMax_index remains valid

    }
    else if ( ((fromTable->mgr.last_Xmax_index + 1) < T3D_DATA_DIMENSION ) && (X > xMax) && (X < fromTable->data.axisX[fromTable->mgr.last_Xmax_index +1 ])  )
    {
        //x is in right neighbor interval
        xMax_index= fromTable->mgr.last_Xmax_index + 1;
        xMin_index= fromTable->mgr.last_Xmax_index;
        xMax= fromTable->data.axisX[xMax_index];
        xMin= fromTable->data.axisX[xMin_index];

        //store for next time
        fromTable->mgr.last_Xmax_index= xMax_index;
    }
    else if ( (fromTable->mgr.last_Xmax_index > 1 ) && (X < xMin) && (X > fromTable->data.axisX[fromTable->mgr.last_Xmax_index -2]) )
    {
        //x is in left neighbor interval
        xMax_index= fromTable->mgr.last_Xmax_index -1;
        xMin_index= fromTable->mgr.last_Xmin_index -2;
        xMax= fromTable->data.axisX[xMax_index];
        xMin= fromTable->data.axisX[xMin_index];

        //store for next time
        fromTable->mgr.last_Xmax_index= xMax_index;
    }
    else
    {
        /**
        Loop from the end to find a suitable x interval
        */
        for(i = T3D_DATA_DIMENSION -1; i >= 0; i--)
        {
            /**
            If the requested X value has been directly found on the x axis
            but we have to provide a suitable x interval for interpolation
            */
            if ( (X == fromTable->data.axisX[i]) || (i == 0) )
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

                xMax= fromTable->data.axisX[xMax_index];
                xMin= fromTable->data.axisX[xMin_index];
                break;
            }

            /**
            Standard scenario
            The requested X value is between axisX[i] and axisX[i-1]
            */
            if ( (X < fromTable->data.axisX[i]) && (X > fromTable->data.axisX[i-1]) )
            {
                xMax= fromTable->data.axisX[i];
                xMin= fromTable->data.axisX[i-1];
                xMax_index= i;
                xMin_index= i-1;
                fromTable->mgr.last_Xmax_index= xMax_index;
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
    yMin = fromTable->data.axisY[0];
    yMax = fromTable->data.axisY[ T3D_DATA_DIMENSION -1 ];
    if(Y > yMax) { Y = yMax; }
    if(Y < yMin) { Y = yMin; }

    /**
    check if we're still in the same Y interval as last time
    preset xM with the ones from last request
    */
    yMax = fromTable->data.axisY[fromTable->mgr.last_Ymax_index];
    yMin = fromTable->data.axisY[fromTable->mgr.last_Ymax_index -1];

    /**
    check if we're still in the same Y environment as last time
    or if it is in a neighbor cell
    (engine rpm changes slowly between cycles)
    */
    if( (Y < yMax) && (Y > yMin) )
    {
        //yM already set
        yMax_index = fromTable->mgr.last_Ymax_index;
        yMin_index = yMax_index -1;
        //last_yMax_index remains valid

    }
    else if ( ((fromTable->mgr.last_Ymax_index + 1) < T3D_DATA_DIMENSION ) && (Y > yMax) && (Y < fromTable->data.axisY[fromTable->mgr.last_Ymax_index +1 ])  )
    {
        //y is in right neighbor interval
        yMax_index= fromTable->mgr.last_Ymax_index + 1;
        yMin_index= fromTable->mgr.last_Ymax_index;
        yMax= fromTable->data.axisY[yMax_index];
        yMin= fromTable->data.axisY[yMin_index];

        //store for next time
        fromTable->mgr.last_Ymax_index= yMax_index;
    }
    else if ( (fromTable->mgr.last_Ymax_index > 1 ) && (Y < yMin) && (Y > fromTable->data.axisY[fromTable->mgr.last_Ymax_index -2]) )
    {
        //y is in left neighbor interval
        yMax_index= fromTable->mgr.last_Ymax_index -1;
        yMin_index= fromTable->mgr.last_Ymin_index -2;
        yMax= fromTable->data.axisY[yMax_index];
        yMin= fromTable->data.axisY[yMin_index];

        //store for next time
        fromTable->mgr.last_Ymax_index= yMax_index;
    }
    else
    {
        /**
        Loop from the end to find a suitable y interval
        */
        for(i = T3D_DATA_DIMENSION -1; i >= 0; i--)
        {
            /**
            If the requested Y value has been directly found on the y axis
            but we have to provide a suitable y interval for interpolation
            */
            if ( (Y == fromTable->data.axisY[i]) || (i == 0) )
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

                yMax= fromTable->data.axisY[yMax_index];
                yMin= fromTable->data.axisY[yMin_index];
                break;
            }

            /**
            Standard scenario
            The requested Y value is between axisY[i] and axisY[i-1]
            */
            if ( (Y < fromTable->data.axisY[i]) && (Y > fromTable->data.axisY[i-1]) )
            {
                yMax= fromTable->data.axisY[i];
                yMin= fromTable->data.axisY[i-1];
                yMax_index= i;
                yMin_index= i-1;
                fromTable->mgr.last_Ymax_index= yMax_index;
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
    A= fromTable->data.axisZ[yMin_index][xMin_index];
    B= fromTable->data.axisZ[yMin_index][xMax_index];
    C= fromTable->data.axisZ[yMax_index][xMin_index];
    D= fromTable->data.axisZ[yMax_index][xMax_index];

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

    return (A + B + C +D) / ((xMax - xMin) * (yMax - yMin));
}


/****************************************************************************************************************************************************
*
* Load 2D table data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t load_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress)
{
    exec_result_t load_result;

    volatile U8 * const pData= (volatile U8 *) pTableData;

    load_result= Eeprom_load_data(BaseAddress, pData, ct2D_data_size);

    return load_result;
}


/****************************************************************************************************************************************************
*
* Load 3D table data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t load_t3D_data(volatile t3D_data_t * pTableData, U32 BaseAddress)
{
    exec_result_t load_result;

    volatile U8 * const pData= (volatile U8 *) pTableData;

    load_result= Eeprom_load_data(BaseAddress, pData, ct3D_data_size);

    return load_result;
}


/****************************************************************************************************************************************************
*
* Save 2D table data to EEPROM
*
****************************************************************************************************************************************************/
exec_result_t store_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pTableData;

    return Eeprom_update_data(BaseAddress, pData, ct2D_data_size);
}


/****************************************************************************************************************************************************
*
* Save 3D table data to EEPROM
*
****************************************************************************************************************************************************/
exec_result_t store_t3D_data(volatile t3D_data_t * pTableData, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pTableData;

    return Eeprom_update_data(BaseAddress, pData, ct3D_data_size);
}



/****************************************************************************************************************************************************
*
* print 3D table in human readable form
*
* assuming that the title has already been printed
* assuming fixed table dimension of T3D_DATA_DIMENSION
* layout:
* left column: Y-Axis
* row below: X-Axis
* data layout: z[y][x]
* orientation (order from low to high) of Y-axis: top to bottom
* orientation (order from low to high) of X-axis: left to right
****************************************************************************************************************************************************/
void show_t3D_data(USART_TypeDef * pPort, volatile t3D_data_t * pTableData)
{
    U32 row, column;

    // for every row, printing the top row first (highest value) Z[0..15][x]
    for (row= 0; row < T3D_DATA_DIMENSION; row++)
    {
        // Y value for this row
        printf_U(pPort, pTableData->axisY[T3D_DATA_DIMENSION -1 - row], PAD_5);

        //separator
        print(pPort, " . ");

        // Z values of this row, printing from left to right Z[y][0..15]
        for (column = 0; column < T3D_DATA_DIMENSION; column++)
        {
            printf_U(pPort, pTableData->axisZ[T3D_DATA_DIMENSION -1 - row][column], PAD_3);
            print(pPort, "  ");
        }

        print(pPort, "\r\n");
    }

    //separator
    print(pPort, "       ................................................................................................\r\n");
    print(pPort, "       ");

    // X-axis
    for (column = 0; column < T3D_DATA_DIMENSION; column++)
    {
        printf_U(pPort, pTableData->axisX[column], PAD_5);
    }

    print(pPort, "\r\n");

}


/****************************************************************************************************************************************************
*
* replace one byte in 2D table
*
****************************************************************************************************************************************************/
exec_result_t modify_t2D_data(volatile t2D_data_t * pTableData, U32 Offset, U32 Value)
{
    volatile U8 * const pData= (volatile U8 *) pTableData;

    //range check
    if(Offset >= ct2D_data_size)
    {
        return EXEC_ERROR;
    }

    *(pData + Offset)= (U8) Value;

    return EXEC_OK;
}


/****************************************************************************************************************************************************
*
* replace one byte in 3D table
*
****************************************************************************************************************************************************/
exec_result_t modify_t3D_data(volatile t3D_data_t * pTableData, U32 Offset, U32 Value)
{
    volatile U8 * const pData= (volatile U8 *) pTableData;

    //range check
    if(Offset >= ct3D_data_size)
    {
        return EXEC_ERROR;
    }

    *(pData + Offset)= (U8) Value;

    return EXEC_OK;
}


/****************************************************************************************************************************************************
*
* send 2D table to Tuner Studio
*
* data will be sent "as is" (no scaling, offset, etc ...)
****************************************************************************************************************************************************/
void send_t2D_data(USART_TypeDef * pPort, volatile t2D_data_t * pTable)
{
    volatile U8 * const pData= (volatile U8 *) pTable;

    UART_send_data(pPort, pData, ct2D_data_size);
}


/****************************************************************************************************************************************************
*
* send 3D table to Tuner Studio
*
* data will be sent "as is" (no scaling, offset, etc ...)
****************************************************************************************************************************************************/
void send_t3D_data(USART_TypeDef * pPort, volatile t3D_data_t * pTable)
{
    volatile U8 * const pData= (volatile U8 *) pTable;

    UART_send_data(pPort, pData, ct3D_data_size);
}




