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


const U32 cMapset_storage_size= sizeof(map_domain_t) + sizeof(map_codomains_t);






exec_result_t mapset_get(volatile mapset_t * pSet, volatile mapset_req_t * pReq);
{
    volatile map_domain_req_t DomainRequest;


    //look up X domain
    ASSERT_EXEC_OK( map_get_domain_x(&(pSet->Dom), &(pMap->Cache), &DomainRequest, pReq->X) );

    //look up Y domain
    ASSERT_EXEC_OK( map_get_domain_y(&(pSet->Dom), &(pMap->Cache), &DomainRequest, pReq->Y) );


    //perform interpolation - ignition advance
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet.Cods.IgnAdv), &(pReq->IgnAdv)) );

    //perform interpolation - volumetric efficiency
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet.Cods.VE), &(pReq->VE)) );

    //perform interpolation - AFR target
    return map_interpolate(&DomainRequest, &(pSet.Cods.AFRtgt), &(pReq->AFRtgt));
}





/****************************************************************************************************************************************************
*
* Load 3D table data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t mapset_load(volatile mapset_t * pSet, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pSet;

    return Eeprom_load_data(BaseAddress, pSet, cMapset_storage_size);
}


/****************************************************************************************************************************************************
*
* Save 3D table data to EEPROM
*
****************************************************************************************************************************************************/
exec_result_t mapset_store(volatile mapset_t * pSet, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pSet;

    return Eeprom_update_data(BaseAddress, pData, cMapset_storage_size);
}


/****************************************************************************************************************************************************
*
* replace one byte in 3D table
*
****************************************************************************************************************************************************/
exec_result_t mapset_modify(volatile map_t * pSet, U32 Offset, U32 Value)
{
    volatile U8 * const pData= (volatile U8 *) pSet;

    //range check
    if(Offset >= cMapset_storage_size)
    {
        return EXEC_ERROR;
    }

    *(pData + Offset)= (U8) Value;

    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* print 3D map in human readable form
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
void show_mapset(USART_TypeDef * pPort, volatile map_t * pMap)
{


    //separator
    print(pPort, "       ................................................................................................\r\n");
    print(pPort, "       ");



    print(pPort, "\r\n");

}



/****************************************************************************************************************************************************
*
* send 3D table to Tuner Studio
*
* data will be sent "as is" (no scaling, offset, etc ...)
****************************************************************************************************************************************************/
void send_mapset(USART_TypeDef * pPort, volatile mapset_t * pSet)
{
    volatile U8 * const pData= (volatile U8 *) pSet;

    UART_send_data(pPort, pData, cMapset_storage_size);
}





