#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"


#define DIAG_SHADOW_LEN 50

VU32 Diag_Shadow[DIAG_SHADOW_LEN];
volatile bool shadow_occupied= false;


/******************************************************************************************************
diag shadow handling
*******************************************************************************************************/
exec_result_t copy_diag_data(VU32 * pSource, U32 Length)
{
    U32 cnt;

    //check preconditions
    if(shadow_occupied == true) return EXEC_ERROR;

    //prevent post fence error
    Assert(Length < DIAG_SHADOW_LEN, 0, 0);

    //mark shadow memory as occupied
    shadow_occupied= true;

    //copy diagnostic data to shadow
    for(cnt=0; cnt < Length; cnt++)
    {
        Diag_Shadow[cnt]= pSource[cnt];
    }

    return EXEC_OK;
}



VU32 get_diag_data(VU32 Index)
{
    //prevent post fence error
    Assert(Index < DIAG_SHADOW_LEN, 0, 0);

    return Diag_Shadow[Index];
}




 void release_diag_shadow()
 {
    shadow_occupied= false;
 }



