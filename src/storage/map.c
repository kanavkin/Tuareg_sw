#include <math.h>

#include "map.h"
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



const U32 cMap_dimension= MAP_DIM;
const U32 cMap_storage_size= sizeof(map_domain_t) + sizeof(map_codomain_t);




/**
This function determines the interpolation interval in the functions domain for the given X coordinate
*/
exec_result_t map_get_domain_x(volatile map_domain_t * pDom, volatile map_cache_t * pCache, volatile map_domain_req_t * pDomReq, F32 X)
{
    U32 xMin, xMax, xMin_index, xMax_index, i;


    /**
    X axis integrity check
    */
    for(i=0; i < cMap_dimension -2; i++)
    {
        xMin = pDom->axisX[i];
        xMax = pDom->axisX[i+1];

        if(xMin >= xMax)
        {
            Fatal(TID_TABLE, STORAGE_LOC_MAP_X_DATA_ERROR);
            return ERROR;
        }
    }


    /**
    clip the requested X if required to fit the interval covered by this table
    */
    xMin = pDom->axisX[0];
    xMax = pDom->axisX[ cMap_dimension -1 ];
    if(X > xMax) { X = xMax; }
    if(X < xMin) { X = xMin; }


    /**
    use the map cache if its data is valid and
    assume that the interpolation interval from the last request is still suitable for the current one
    */
    if( (pCache->last_Xmax_index < cMap_dimension) && (pCache->last_Xmax_index > 0) )
    {
        xMax_index= pCache->last_Xmax_index;
        xMin_index= xMax_index -1;
    }
    else
    {
        //use safe defaults
        xMax_index= 1;
        xMin_index= 0;
    }

    //look up the corresponding interval
    xMax = pDom->axisX[xMax_index];
    xMin = pDom->axisX[xMin_index];


    /**
    check the current interval and the neighbor cells if required
    */
    if( (X < xMax) && (X > xMin) )
    {
        //direct fit

    }
    else if( (X > xMax) && ((xMax_index + 1) < MAP_DIM) &&  (X < pDom->axisX[xMax_index +1])  )
    {
        //x is in right neighbor interval, update index
        xMax_index += 1;
        xMin_index += 1;
    }
    else if( (X < xMin) && (xMax_index > 1 ) &&  (X > pDom->axisX[xMax_index -1]) )
    {
        //x is in left neighbor interval, update index
        xMax_index -= 1;
        xMin_index -= 1;
    }
    else
    {
        /**
        Loop from the end to find a suitable x interval
        i ~ xMax
        */
        for(i = MAP_DIM -1; i >= 0; i--)
        {

            //check if the last (smallest) element has been reached
            if(i == 0)
            {
                xMax_index= 1;
                xMin_index= 0;
                break;
            }

            //check if the requested X value is between axisX[i] and axisX[i-1]
            if ( (X <= pDom->axisX[i]) && (X > pDom->axisX[i-1]) )
            {
                //update index
                xMax_index= i;
                xMin_index= i-1;

                break;
            }
        }
    }


    /**
    interval has been determined, export data
    */

    //export cache
    pCache->last_Xmax_index= xMax_index;

    //export the domain data
    pDomReq->X= X;

    pDomReq->xMax_index= xMax_index;
    pDomReq->xMin_index= xMin_index;

    pDomReq->xMax= pDom->axisX[xMax_index];
    pDomReq->xMin= pDom->axisX[xMin_index];

    //ready
    return SUCCESS;

}


/**
This function determines the interpolation interval in the functions domain for the given Y coordinate
*/
exec_result_t map_get_domain_y(volatile map_domain_t * pDom, volatile map_cache_t * pCache, volatile map_domain_req_t * pDomReq, F32 Y)
{
    U32 yMin, yMax, yMin_index, yMax_index, i;


    /**
    Y axis integrity check
    */
    for(i=0; i < cMap_dimension -2; i++)
    {
        yMin = pDom->axisY[i];
        yMax = pDom->axisY[i+1];

        if(yMin >= yMax)
        {
            Fatal(TID_TABLE, STORAGE_LOC_MAP_Y_DATA_ERROR);
            return ERROR;
        }
    }


    /**
    clip the requested Y if required to fit the interval covered by this table
    */
    yMin = pDom->axisY[0];
    yMax = pDom->axisY[ cMap_dimension -1 ];
    if(Y > yMax) { Y = yMax; }
    if(Y < yMin) { Y = yMin; }


    /**
    use the map cache if its data is valid and
    assume that the interpolation interval from the last request is still suitable for the current one
    */
    if( (pCache->last_Ymax_index < cMap_dimension) && (pCache->last_Ymax_index > 0) )
    {
        yMax_index= pCache->last_Ymax_index;
        yMin_index= yMax_index -1;
    }
    else
    {
        //use safe defaults
        yMax_index= 1;
        yMin_index= 0;
    }

    //look up the corresponding interval
    yMax = pDom->axisY[yMax_index];
    yMin = pDom->axisY[yMin_index];


    /**
    check the current interval and the neighbor cells if required
    */
    if( (Y < yMax) && (Y > yMin) )
    {
        //direct fit

    }
    else if( (Y > yMax) && ((yMax_index + 1) < MAP_DIM) &&  (Y < pDom->axisY[yMax_index +1])  )
    {
        //x is in right neighbor interval, update index
        yMax_index += 1;
        yMin_index += 1;
    }
    else if( (Y < yMin) && (yMax_index > 1 ) &&  (Y > pDom->axisY[yMax_index -1]) )
    {
        //x is in left neighbor interval, update index
        yMax_index -= 1;
        yMin_index -= 1;
    }
    else
    {
        /**
        Loop from the end to find a suitable x interval
        i ~ yMax
        */
        for(i = MAP_DIM -1; i >= 0; i--)
        {

            //check if the last (smallest) element has been reached
            if(i == 0)
            {
                yMax_index= 1;
                yMin_index= 0;
                break;
            }

            //check if the requested Y value is between axisY[i] and axisY[i-1]
            if ( (Y <= pDom->axisY[i]) && (Y > pDom->axisY[i-1]) )
            {
                //update index
                yMax_index= i;
                yMin_index= i-1;

                break;
            }
        }
    }


    /**
    interval has been determined, export data
    */

    //export cache
    pCache->last_Ymax_index= yMax_index;

    //export the domain data
    pDomReq->Y= Y;

    pDomReq->yMax_index= yMax_index;
    pDomReq->yMin_index= yMin_index;

    pDomReq->yMax= pDom->axisY[yMax_index];
    pDomReq->yMin= pDom->axisY[yMin_index];

    //ready
    return SUCCESS;

}


/**
This function performs a bilinear interpolation to the maps co domain
determined by the indicated intervals
*/
exec_result_t map_interpolate(volatile map_domain_req_t * pDomReq, volatile map_codomain_t * pCod, VF32 * pResult)
{
    F32 A, B, C, D, Res, Area;


    /**
    argument integrity check
    */
    if(pDomReq->xMin_index > MAP_DIM)
    {
        Fatal(TID_TABLE, STORAGE_LOC_MAP_INTERPOL_ARG_ERROR);
        return ERROR;
    }

    if(pDomReq->xMax_index > MAP_DIM)
    {
        Fatal(TID_TABLE, STORAGE_LOC_MAP_INTERPOL_ARG_ERROR);
        return ERROR;
    }

    if(pDomReq->yMin_index > MAP_DIM)
    {
        Fatal(TID_TABLE, STORAGE_LOC_MAP_INTERPOL_ARG_ERROR);
        return ERROR;
    }

    if(pDomReq->yMax_index > MAP_DIM)
    {
        Fatal(TID_TABLE, STORAGE_LOC_MAP_INTERPOL_ARG_ERROR);
        return ERROR;
    }

    /**
    The bilinear interpolation area lies within this 4 points

    C(yMax,xMin)  D(yMax,xMax)
    A(yMin,xMin)  B(yMin,xMax)
    */
    A= pCod->axisZ[pDomReq->yMin_index][pDomReq->xMin_index];
    B= pCod->axisZ[pDomReq->yMin_index][pDomReq->xMax_index];
    C= pCod->axisZ[pDomReq->yMax_index][pDomReq->xMin_index];
    D= pCod->axisZ[pDomReq->yMax_index][pDomReq->xMax_index];


    /**
    perform the interpolation
    */
    A *= (pDomReq->xMax - pDomReq->X) * (pDomReq->yMax - pDomReq->Y);
    B *= (pDomReq->X - pDomReq->xMin)  * (pDomReq->yMax - pDomReq->Y);
    C *= (pDomReq->xMax - pDomReq->X) * ( pDomReq->Y - pDomReq->yMin);
    D *= (pDomReq->X - pDomReq->xMin) * (pDomReq->Y - pDomReq->yMin);

    Area= (xMax - xMin) * (yMax - yMin);

    Res= (A + B + C +D) / Area;

    //export result
    *pResult= Res;

    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* get an interpolated value from a map
*
****************************************************************************************************************************************************/
exec_result_t map_get(volatile map_t * pMap, F32 X, F32 Y, VF32 * pResult);
{
    volatile map_domain_req_t DomainRequest;


    //look up X domain
    ASSERT_EXEC_OK( map_get_domain_x(&(pMap->Dom), &(pMap->Cache), &DomainRequest, X) );

    //look up Y domain
    ASSERT_EXEC_OK( map_get_domain_y(&(pMap->Dom), &(pMap->Cache), &DomainRequest, Y) );

    //perform interpolation
    return map_interpolate(&DomainRequest, &(pMap->Cod), pResult);

}


/****************************************************************************************************************************************************
*
* Load 3D table data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t map_load(volatile map_t * pMap, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pMap;

    return Eeprom_load_data(BaseAddress, pMap, cMap_storage_size);
}


/****************************************************************************************************************************************************
*
* Save 3D table data to EEPROM
*
****************************************************************************************************************************************************/
exec_result_t map_store(volatile map_t * pMap, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pMap;

    return Eeprom_update_data(BaseAddress, pData, cMap_storage_size);
}


/****************************************************************************************************************************************************
*
* replace one byte in 3D table
*
****************************************************************************************************************************************************/
exec_result_t map_modify(volatile map_t * pMap, U32 Offset, U32 Value)
{
    volatile U8 * const pData= (volatile U8 *) pMap;

    //range check
    if(Offset >= cMap_storage_size)
    {
        return EXEC_ERROR;
    }

    *(pData + Offset)= (U8) Value;

    return EXEC_OK;
}


/****************************************************************************************************************************************************
*
* print 3D map axes in human readable form
*
* worker function
*
* assuming that the title has already been printed
* assuming fixed table dimension of cMap_dimension
* layout:
* left column: Y-Axis
* row below: X-Axis
* data layout: z[y][x]
* orientation (order from low to high) of Y-axis: top to bottom
* orientation (order from low to high) of X-axis: left to right
****************************************************************************************************************************************************/
void map_show_axes(USART_TypeDef * pPort, volatile map_domain_t * pDom, volatile map_codomain_t * pCod)
{
    U32 row, column;

    // for every row, printing the top row first (highest value) Z[0..15][x]
    for (row= 0; row < cMap_dimension; row++)
    {
        // Y value for this row
        printf_U(pPort, pDom->axisY[cMap_dimension -1 - row], PAD_5);

        //separator
        print(pPort, " . ");

        // Z values of this row, printing from left to right Z[y][0..15]
        for (column = 0; column < cMap_dimension; column++)
        {
            printf_U(pPort, pCod->axisZ[cMap_dimension -1 - row][column], PAD_3);
            print(pPort, "  ");
        }

        print(pPort, "\r\n");
    }

    //separator
    print(pPort, "       ................................................................................................\r\n");
    print(pPort, "       ");

    // X-axis
    for (column = 0; column < cMap_dimension; column++)
    {
        printf_U(pPort, pDom->axisX[column], PAD_5);
    }

    print(pPort, "\r\n");
}


/****************************************************************************************************************************************************
*
* print 3D map in human readable form
*
* user access function
****************************************************************************************************************************************************/
void map_show(USART_TypeDef * pPort, volatile map_t * pMap)
{
    map_show_axes(pPort, pMap->Dom, pMap->Cod);
}



/****************************************************************************************************************************************************
*
* send 3D table to Tuner Studio
*
* data will be sent "as is" (no scaling, offset, etc ...)
****************************************************************************************************************************************************/
void map_send(USART_TypeDef * pPort, volatile map_t * pMap)
{
    volatile U8 * const pData= (volatile U8 *) pMap;

    UART_send_data(pPort, pData, cMap_storage_size);
}





