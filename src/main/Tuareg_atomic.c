#include <Tuareg_platform.h>
#include <Tuareg.h>



/******************************************************************************************************************************
Atomic section
******************************************************************************************************************************/

const U32 cTuareg_Atomic_max_depth= 64;

void Atomic_Begin()
{
    __disable_irq();

    //check, if max depth has been reached
    if(Tuareg.Atomic_depth < cTuareg_Atomic_max_depth)
    {
        //count new depth
        Tuareg.Atomic_depth++;

        /**
        add debug options
        - save system timestamp
        */
    }
    else
    {
        Fatal(TID_TUAREG_ATOMIC, TUAREG_LOC_ATOMIC_BEGIN_MAX_DEPTH);
    }
}


void Atomic_End()
{
    //check, if an Atomic Section has been entered lately
    if(Tuareg.Atomic_depth > 0)
    {
        //count new depth
        Tuareg.Atomic_depth -= 1;

        //check, if IRQs can be enabled
        if(Tuareg.Atomic_depth == 0)
        {
            //this was the last Atomic Section
            __enable_irq();
        }
    }
    else
    {
        Fatal(TID_TUAREG_ATOMIC, TUAREG_LOC_ATOMIC_END_ERROR);
    }
}






























