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




const U32 cCtrlSet_storage_size= sizeof(map_domain_t) + MAPSET_COUNT * sizeof(mapset_codomains_t);



/****************************************************************************************************************************************************
*
* load engine control data from the given map set of the control set
*
****************************************************************************************************************************************************/
exec_result_t ctrlset_get(volatile ctrlset_t * pSet, volatile ctrlset_req_t * pReq);
{
    volatile map_domain_req_t DomainRequest;


    //check if a valid map set has been commanded
    VitalAssert(pReq->Set < MAPSET_COUNT, TID_CTRLSET, STORAGE_LOC_CTRLSET_GET_DESIGNATOR_ERROR);


    //look up X domain
    ASSERT_EXEC_OK( map_get_domain_x(&(pSet->Dom), &(pMap->Cache), &DomainRequest, pReq->X) );

    //look up Y domain
    ASSERT_EXEC_OK( map_get_domain_y(&(pSet->Dom), &(pMap->Cache), &DomainRequest, pReq->Y) );


    //perform interpolation - ignition advance
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet.Cods[pReq->Set].IgnAdv), &(pReq->IgnAdv)) );

    //perform interpolation - volumetric efficiency
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet.Cods[pReq->Set].VE), &(pReq->VE)) );

    //perform interpolation - AFR target
    return map_interpolate(&DomainRequest, &(pSet.Cods[pReq->Set].AFRtgt), &(pReq->AFRtgt));
}




/****************************************************************************************************************************************************
*
* Load control set static data from EEPROM
*
****************************************************************************************************************************************************/
exec_result_t ctrlset_load(volatile ctrlset_t * pCtrl, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pCtrl;

    return Eeprom_load_data(BaseAddress, pData, cCtrlSet_storage_size);
}


/****************************************************************************************************************************************************
*
* Save control set static data to EEPROM
*
****************************************************************************************************************************************************/
exec_result_t ctrlset_store(volatile ctrlset_t * pCtrl, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pCtrl;

    return Eeprom_update_data(BaseAddress, pData, cCtrlSet_storage_size);
}


/****************************************************************************************************************************************************
*
* replace one byte in control set
*
****************************************************************************************************************************************************/
exec_result_t ctrlset_modify(volatile ctrlset_t * pCtrl, U32 Offset, U32 Value)
{
    volatile U8 * const pData= (volatile U8 *) pCtrl;

    //range check
    if(Offset >= cCtrlSet_storage_size)
    {
        return EXEC_ERROR;
    }

    *(pData + Offset)= (U8) Value;

    return EXEC_OK;
}



/****************************************************************************************************************************************************
*
* print control set static data in human readable form - user access function
*
****************************************************************************************************************************************************/
void ctrlset_show(USART_TypeDef * pPort, volatile ctrlset_t * pCtrl)
{
    U32 i;

    for(i=0; i < MAPSET_COUNT; i++)
    {

        print(pPort, "\r\nSet: ");
        printf_U(pPort, i, NO_TRAIL);

        print(pPort, "\r\nIgnAdv:\r\n");
        map_show_axes(pPort, pCtrl->Dom, pCtrl.Cods[i]->IgnAdv);

        print(pPort, "\r\nVE:\r\n");
        map_show_axes(pPort, pCtrl->Dom, pCtrl.Cods[i]->VE);

        print(pPort, "\r\nAFR target:\r\n");
        map_show_axes(pPort, pCtrl->Dom, pCtrl.Cods[i]->AFRtgt);

    }
}



/****************************************************************************************************************************************************
*
* send control set static data to Tuner Studio
*
* data will be sent "as is" (no scaling, offset, etc ...)
****************************************************************************************************************************************************/
void ctrlset_send(USART_TypeDef * pPort, volatile ctrlset_t * pCtrl)
{
    volatile U8 * const pData= (volatile U8 *) pCtrl;

    UART_send_data(pPort, pData, cCtrlSet_storage_size);
}
