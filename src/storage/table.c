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

#include "debug_port_messages.h"
#include "storage_syslog_locations.h"


/**
some thoughts about table / map storage:

all use cases can be implemented via positive x,y,z axis values
all use cases can be implemented via data scaling

U8 is ok for y/z axis

t2d dimension too large?


*/


const U32 ct2D_data_size= sizeof(t2D_data_t);
const U32 ct3D_data_size= sizeof(t3D_data_t);


const U32 ct2D_data_dimension= T2D_DATA_DIMENSION;
const U32 ct3D_data_dimension= T3D_DATA_DIMENSION;





/**
helper function - checks the integrity of the axes

by now only the monotony is checked
loops through the axis data and checks the monotony
*/
exec_result_t check_t2D_integrity(volatile t2D_t * pTable)
{
    VU32 i;

    /**
    check axis monotony
    */
    for(i=0; i < ct2D_data_dimension -1; i++)
    {
        //axisX
        if(pTable->data.axisX[i] >= pTable->data.axisX[i+1])
        {
            return EXEC_ERROR;
        }

        //axisY
        if(pTable->data.axisY[i] >= pTable->data.axisY[i+1])
        {
            return EXEC_ERROR;
        }
    }

    /**
    check axis data range
    */
    for(i=0; i < ct2D_data_dimension; i++)
    {
        //axisX
        if(pTable->data.axisX[i] < pTable->iParm.X_min_valid)
        {
            return EXEC_ERROR;
        }

        if(pTable->data.axisX[i] > pTable->iParm.X_max_valid)
        {
            return EXEC_ERROR;
        }

        //axisY
        if(pTable->data.axisY[i] < pTable->iParm.Y_min_valid)
        {
            return EXEC_ERROR;
        }

        if(pTable->data.axisY[i] > pTable->iParm.Y_max_valid)
        {
            return EXEC_ERROR;
        }
    }

    return EXEC_OK;
}







/**
This function pulls a 1D linear interpolated value from a 2D table

x axis data order: value ~ index
*/
F32 getValue_t2D(volatile t2D_t *fromTable, U32 X)
{
    U32 xMin =0, xMax =0, yMin =0, yMax =0, i =0;
    F32 m, y;


    /**
    check table range validity
    */
    xMin = fromTable->data.axisX[0];
    xMax = fromTable->data.axisX[ ct2D_data_dimension -1 ];

    //check precondition - x axis data validity
    if(xMin >= xMax)
    {
        Limp(TID_TABLE, STORAGE_LOC_T2D_DATA_RANGE);
        return 0;
    }

    /**
    check if the requested argument is covered by the tables range -> early exit!
    */
    if(X >= xMax)
    {
        return fromTable->data.axisY[ct2D_data_dimension -1];
    }

    if(X <= xMin)
    {
        return fromTable->data.axisY[0];
    }


    /**
    Loop from the table end to find a suitable x interval
    -> the previously executed table range check showed: X < axisX[<last>]
    -> the previously executed table range check showed: X > axisX[0]
    */
    for(i = ct2D_data_dimension -1; i > 0; i--)
    {
        /**
        get the corresponding X interval
        axisX[i] and axisX[i+1] -> xMin and XMax

        By design (X == xMax) has been checked by previous steps
        */
        xMax = fromTable->data.axisX[i];
        xMin = fromTable->data.axisX[i-1];


        //check precondition - x axis interval data steady
        if(xMin >= xMax)
        {
            Limp(TID_TABLE, STORAGE_LOC_T2D_NOT_STEADY);
            return 0;
        }


        //check if xMin is a direct fit
        if(X == xMin)
        {
            //exit here taking the corresponding Y value from table
            return fromTable->data.axisY[i-1];
        }

        /**
        check if the given argument is within the interval xMin .. xMax

        By design (X != xMax) has been assured by previous steps
        By design (X != xMin) has been assured by previous steps
        By design (X < Xmax) has been assured by previous steps
        */
        if(X > xMin)
        {
            /**
            xMin/Max indexes found, look up data for calculation
            */
            yMax= fromTable->data.axisY[i];
            yMin= fromTable->data.axisY[i-1];

            //uSMDS#Req2: xMax shall be != xMin

            /**
            y=  m * X + n
            y= ( (dY/dX) * (X - xMin) * yMin )
            y= yMin + mDx
            */
            m=  ((F32) yMax - (F32) yMin) / ((F32) xMax - (F32) xMin);
            y= m * (X - xMin) + yMin;

            return y;
        }

    }

    //error
    Fatal(TID_TABLE, STORAGE_LOC_T2D_NO_MATCH);
    return 0;

}


/**
This function pulls a value from a 3D table given a target for X and Y coordinates.
It performs a bilinear interpolation
*/
F32 getValue_t3D(volatile t3D_t * fromTable, U32 X, U32 Y)
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
    xMax = fromTable->data.axisX[fromTable->cache.last_Xmax_index];
    xMin = fromTable->data.axisX[fromTable->cache.last_Xmax_index -1];

    /**
    check if we're still in the same X environment as last time
    or if it is in a neighbor cell
    (engine rpm changes slowly between cycles)
    */
    if( (X < xMax) && (X > xMin) )
    {
        //xM already set
        xMax_index = fromTable->cache.last_Xmax_index;
        xMin_index = xMax_index -1;
        //last_xMax_index remains valid

    }
    else if ( ((fromTable->cache.last_Xmax_index + 1) < T3D_DATA_DIMENSION ) && (X > xMax) && (X < fromTable->data.axisX[fromTable->cache.last_Xmax_index +1 ])  )
    {
        //x is in right neighbor interval
        xMax_index= fromTable->cache.last_Xmax_index + 1;
        xMin_index= fromTable->cache.last_Xmax_index;
        xMax= fromTable->data.axisX[xMax_index];
        xMin= fromTable->data.axisX[xMin_index];

        //store for next time
        fromTable->cache.last_Xmax_index= xMax_index;
    }
    else if ( (fromTable->cache.last_Xmax_index > 1 ) && (X < xMin) && (X > fromTable->data.axisX[fromTable->cache.last_Xmax_index -2]) )
    {
        //x is in left neighbor interval
        xMax_index= fromTable->cache.last_Xmax_index -1;
        xMin_index= fromTable->cache.last_Xmin_index -2;
        xMax= fromTable->data.axisX[xMax_index];
        xMin= fromTable->data.axisX[xMin_index];

        //store for next time
        fromTable->cache.last_Xmax_index= xMax_index;
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
                fromTable->cache.last_Xmax_index= xMax_index;
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
    yMax = fromTable->data.axisY[fromTable->cache.last_Ymax_index];
    yMin = fromTable->data.axisY[fromTable->cache.last_Ymax_index -1];

    /**
    check if we're still in the same Y environment as last time
    or if it is in a neighbor cell
    (engine rpm changes slowly between cycles)
    */
    if( (Y < yMax) && (Y > yMin) )
    {
        //yM already set
        yMax_index = fromTable->cache.last_Ymax_index;
        yMin_index = yMax_index -1;
        //last_yMax_index remains valid

    }
    else if ( ((fromTable->cache.last_Ymax_index + 1) < T3D_DATA_DIMENSION ) && (Y > yMax) && (Y < fromTable->data.axisY[fromTable->cache.last_Ymax_index +1 ])  )
    {
        //y is in right neighbor interval
        yMax_index= fromTable->cache.last_Ymax_index + 1;
        yMin_index= fromTable->cache.last_Ymax_index;
        yMax= fromTable->data.axisY[yMax_index];
        yMin= fromTable->data.axisY[yMin_index];

        //store for next time
        fromTable->cache.last_Ymax_index= yMax_index;
    }
    else if ( (fromTable->cache.last_Ymax_index > 1 ) && (Y < yMin) && (Y > fromTable->data.axisY[fromTable->cache.last_Ymax_index -2]) )
    {
        //y is in left neighbor interval
        yMax_index= fromTable->cache.last_Ymax_index -1;
        yMin_index= fromTable->cache.last_Ymin_index -2;
        yMax= fromTable->data.axisY[yMax_index];
        yMin= fromTable->data.axisY[yMin_index];

        //store for next time
        fromTable->cache.last_Ymax_index= yMax_index;
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
                fromTable->cache.last_Ymax_index= yMax_index;
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

/// TODO (oli#7#): improve float equality check

    if( (A == B) && (A == C) && (A == D) )
    {
        return A;
    }
    */

    /**
    RESULTS finally
    */
    A *= ((F32) xMax - (F32) X) * ((F32) yMax - (F32) Y);
    B *= ((F32) X - (F32) xMin)  * ((F32) yMax - (F32) Y);
    C *= ((F32) xMax - (F32) X) * ((F32) Y - (F32) yMin);
    D *= ((F32) X - (F32) xMin) * ((F32) Y - (F32) yMin);

    return (A + B + C +D) / (((F32) xMax - (F32) xMin) * ((F32) yMax - (F32) yMin));
}


/****************************************************************************************************************************************************
*
* Load 2D table data from EEPROM
*
* table data is always packed, we have to be careful with unaligned accesses when using pointers!
****************************************************************************************************************************************************/
exec_result_t load_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress)
{
    exec_result_t load_result;

    volatile U8 * const pData= (volatile U8 *) pTableData;

    load_result= Eeprom_load_data(BaseAddress, pData, ct2D_data_size);

    return load_result;
}

//more comfortable syntax
exec_result_t load_t2D(volatile t2D_t * pTable, U32 BaseAddress)
{
    exec_result_t load_result;

    volatile U8 * const pData= (volatile U8 *) &(pTable->data);

    load_result= Eeprom_load_data(BaseAddress, pData, ct2D_data_size);

    return load_result;
}

/****************************************************************************************************************************************************
*
* Load 3D table data from EEPROM
*
* table data is always packed, we have to be careful with unaligned accesses when using pointers!
****************************************************************************************************************************************************/
exec_result_t load_t3D_data(volatile t3D_data_t * pTableData, U32 BaseAddress)
{
    exec_result_t load_result;

    volatile U8 * const pData= (volatile U8 *) pTableData;

    load_result= Eeprom_load_data(BaseAddress, pData, ct3D_data_size);

    return load_result;
}

//more comfortable syntax
exec_result_t load_t3D(volatile t3D_t * pTable, U32 BaseAddress)
{
    exec_result_t load_result;

    volatile U8 * const pData= (volatile U8 *) &(pTable->data);

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

exec_result_t store_t2D(volatile t2D_t * pTable, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) &(pTable->data);

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

exec_result_t store_t3D(volatile t3D_t * pTable, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) &(pTable->data);

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
* print 2D table in human readable form
*
* assuming that the title has already been printed
* assuming fixed table dimension of T2D_DATA_DIMENSION
* layout:
* row below: X-Axis
* data layout: y[x]
* orientation (order from low to high) of X-axis: left to right
****************************************************************************************************************************************************/
void show_t2D_data(USART_TypeDef * pPort, volatile t2D_data_t * pTableData)
{
    U32 column;

    print(pPort, "Y: ");

    // Y values, printing from left to right y[0..15]
    for (column = 0; column < T2D_DATA_DIMENSION; column++)
    {
        printf_U(pPort, pTableData->axisY[column], PAD_5);
        print(pPort, " | ");
    }

    //separator
    //print(pPort, "\r\n................................................................................................\r\n");

    print(pPort, "\r\nX: ");

    // X-axis
    for (column = 0; column < T2D_DATA_DIMENSION; column++)
    {
        printf_U(pPort, pTableData->axisX[column], PAD_5);
        print(pPort, " | ");
    }

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





