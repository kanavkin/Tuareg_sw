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




const U32 cCtrlSet_storage_size= sizeof(map_domain_t) + sizeof(mapset_codomains_t);



/****************************************************************************************************************************************************
*
* load engine control data from the given map set of the control set
*
*  Ignition Advance Table
** x-Axis -> rpm (no offset, no scaling)
** y-Axis -> TPS angle in ° (no offset, no scaling)
** z-Axis -> advance angle in ° BTDC (no offset, no scaling)
*
*  Fueling VE Table
** x-Axis -> rpm (no offset, no scaling)
** y-Axis -> TPS angle in ° (no offset, no scaling)
** z-Axis -> VE in % (no offset, table values are multiplied by 2)
*
*  Fueling AFR Table
*
** x-Axis -> rpm (no offset, no scaling)
** y-Axis -> MAP in kPa (no offset, no scaling)
** z-Axis -> target AFR (no offset, table values are multiplied by 10)
*
*
****************************************************************************************************************************************************/
exec_result_t ctrlset_get(volatile ctrlset_t * pSet, volatile ctrlset_req_t * pReq)
{
    volatile map_domain_req_t DomainRequest;

    const F32 cVeDivider= 2.0;
    const F32 cAfrDivider= 10.0;

    F32 ADV_raw, VE_raw, AFR_raw;

    //mark the outputs invalid first
    pReq->valid= false;

    //look up X domain
    ASSERT_EXEC_OK( map_get_domain_x(&(pSet->Dom), &(pSet->Cache), &DomainRequest, pReq->X) );

    //look up Y domain
    ASSERT_EXEC_OK( map_get_domain_y(&(pSet->Dom), &(pSet->Cache), &DomainRequest, pReq->Y) );


    //perform interpolation - ignition advance
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet->Cods.IgnAdv), &ADV_raw) );

    //perform interpolation - volumetric efficiency
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet->Cods.VE), &VE_raw ) );

    //perform interpolation - AFR target
    ASSERT_EXEC_OK( map_interpolate(&DomainRequest, &(pSet->Cods.AFRtgt), &AFR_raw) );


    //export outputs
    /// review: division is safe for constant dividers defined here
    pReq->IgnAdv= ADV_raw;
    pReq->VE= VE_raw / cVeDivider;
    pReq->AFRtgt= AFR_raw / cAfrDivider;
    pReq->valid= true;


    return EXEC_OK;
}




/****************************************************************************************************************************************************
*
* Load control set config data from EEPROM
* reset table cache
*
****************************************************************************************************************************************************/
exec_result_t ctrlset_load(volatile ctrlset_t * pCtrl, U32 BaseAddress)
{
    volatile U8 * const pData= (volatile U8 *) pCtrl;

    //load config data
    ASSERT_EXEC_OK( Eeprom_load_data(BaseAddress, pData, cCtrlSet_storage_size) );

    //reset cache
    pCtrl->Cache.last_Xmax_index= 0;
    pCtrl->Cache.last_Ymax_index= 0;

    return EXEC_OK;
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
    print(pPort, "\r\nIgnAdv:\r\n");

    map_show_axes(pPort, &(pCtrl->Dom), &(pCtrl->Cods.IgnAdv) );

    print(pPort, "\r\nVE:\r\n");
    map_show_axes(pPort, &(pCtrl->Dom), &(pCtrl->Cods.VE) );

    print(pPort, "\r\nAFR target:\r\n");
    map_show_axes(pPort, &(pCtrl->Dom), &(pCtrl->Cods.AFRtgt) );

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
