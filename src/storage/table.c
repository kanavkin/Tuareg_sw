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

2D table shall be U16->U16

*/


const U32 ct2D_data_size= sizeof(t2D_data_t);


const U32 cT2D_data_dimension= T2D_DATA_DIMENSION;


const U32 cT2D_MinIndex =0, cT2D_MaxIndex= T2D_DATA_DIMENSION -1;
const U32 cT2D_MinInterval =0, cT2D_MaxInterval= T2D_DATA_DIMENSION -2;



typedef enum { SMALLER, WITHIN, GREATER, COMPARET_SIZE} compare_t;


typedef struct _interpolation_t {

    U32 Interval;
    F32 Arg;
    F32 Res;

} interpolation_t;



/**
2D table interpolation helper function
interpolate the corresponding value to the given argument if
the given argument lies within the commanded interval
*/
compare_t interpolate_t2D(volatile t2D_t * pTable, interpolation_t * pData)
{
    F32 xMin, xMax, yMin, yMax, m, y;

    //post fence check
    if(pData->Interval > cT2D_MaxInterval)
    {
        Fatal(TID_TABLE, STORAGE_LOC_T2D_INTERPOL_ERROR);
        return SMALLER;
    }

    //look up x values
    xMin= pTable->data.axisX[pData->Interval];
    xMax= pTable->data.axisX[pData->Interval +1];


    if( pData->Arg > xMax)
    {
        return GREATER;
    }
    else if( pData->Arg < xMin)
    {
        return SMALLER;
    }
    else
    {
        //look up y values
        yMin= pTable->data.axisY[pData->Interval];
        yMax= pTable->data.axisY[pData->Interval +1];

        /*
        y=  m * X + n
        y= ( (dY/dX) * (X - xMin) + yMin )
        */
        m= divide_float( yMax - yMin,  xMax - xMin );
        y= m * (pData->Arg - xMin) + yMin;

        //update cache
        pTable->last_index= pData->Interval;

        //export result
        pData->Res= y;

        return WITHIN;
    }
}





/**
This function pulls a 1D linear interpolated value from a 2D table

x axis data order: value ~ index

index -> xMin_index := interval; x_max_index := interval +1

Axis:       | 0 | 1 | 2 | 3 | ...
Interval:   ---(0)-(1)-(2)-

*/
F32 getValue_t2D(volatile t2D_t *pTable, F32 X)
{
    U32 Interval;
    compare_t compare;
    interpolation_t iData;


    /**
    check data integrity first of all!
    X axis monotony
    */
    for(Interval= cT2D_MinInterval; Interval < cT2D_MaxInterval; Interval++)
    {
        //assert xMin < xMax
        if(pTable->data.axisX[Interval] >= pTable->data.axisX[Interval +1])
        {
            Fatal(TID_TABLE, STORAGE_LOC_T2D_X_DATA_ERROR);
            return 0.0f;
        }
    }

    //define the starting Interval by cache - with post fence check
    Interval= (pTable->last_index > cT2D_MaxInterval)? cT2D_MaxInterval : pTable->last_index;

    //prepare transfer object
    iData.Arg= X;
    iData.Interval= Interval;

    //check if X is within the starting Interval
    compare= interpolate_t2D(pTable, &iData);


    if( compare == WITHIN )
    {
        return iData.Res;
    }
    else if ( compare == GREATER )
    {

        while(Interval < cT2D_MaxInterval)
        {
            //select right neighbor Interval
            Interval++;

            //prepare transfer object
            iData.Arg= X;
            iData.Interval= Interval;

            compare= interpolate_t2D(pTable, &iData);

            if(compare == WITHIN)
            {
                return iData.Res;
            }
        }

        //X lies outside of the axis range
        pTable->last_index= cT2D_MaxIndex;
        return pTable->data.axisY[cT2D_MaxIndex];

    }
    else if ( compare == SMALLER )
    {

        while(Interval > cT2D_MinInterval)
        {
            //select left neighbor Interval
            Interval--;

            //prepare transfer object
            iData.Arg= X;
            iData.Interval= Interval;

            compare= interpolate_t2D(pTable, &iData);

            if(compare == WITHIN)
            {
                return iData.Res;
            }
        }

        //X lies outside of the axis range
        pTable->last_index= cT2D_MinIndex;
        return pTable->data.axisY[cT2D_MinIndex];

    }
    else
    {
        //error
        Fatal(TID_TABLE, STORAGE_LOC_T2D_LOGIC_ERROR);
        return 0.0f;
    }

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
* send 2D table to Tuner Studio
*
* data will be sent "as is" (no scaling, offset, etc ...)
****************************************************************************************************************************************************/
void send_t2D_data(USART_TypeDef * pPort, volatile t2D_data_t * pTable)
{
    volatile U8 * const pData= (volatile U8 *) pTable;

    UART_send_data(pPort, pData, ct2D_data_size);
}




