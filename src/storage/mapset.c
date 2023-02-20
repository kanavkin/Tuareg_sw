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









