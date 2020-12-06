/**
*/
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include "Tuareg_types.h"
#include "process_table.h"

/**
absolute process advance angles for the crank positions at the current engine speed
*/
volatile process_advance_t ProcessTable[PROCESS_TABLE_LENGTH];


/**
calculates the absolute process advance angles based on the information provided in pCrankTable
*/
void update_process_table(volatile crank_position_table_t * pCrankTable)
{
    U32 crank_pos, base_advance_deg;

    //fill table according to crank angle data
    for(crank_pos= 0; crank_pos < CRK_POSITION_COUNT; crank_pos++)
    {
        base_advance_deg= pCrankTable->crank_angle_deg[crank_pos];

        //compression stroke directly before the reference cTDC
        ProcessTable[crank_pos]= 360 - base_advance_deg ;

        //exhaust stroke
        ProcessTable[crank_pos + CRK_POSITION_COUNT]= 720 - base_advance_deg;

        //previous compression stroke
        ProcessTable[crank_pos + 2* CRK_POSITION_COUNT]= 1080 - base_advance_deg;
    }
}


/*
helper function to find the process position corresponding to the process table index provided

in case of error the state of pTarget will be unchanged
*/
exec_result_t get_position_from_index(VU32 Index, volatile process_position_t * pTarget)
{

    if(Index < CRK_POSITION_COUNT)
    {
        //compression stroke directly before the reference cTDC
        pTarget->crank_pos= Index;
        pTarget->phase= PHASE_CYL1_COMP;

        return EXEC_OK;
    }
    else if(Index < 2* CRK_POSITION_COUNT)
    {
        //exhaust stroke
        pTarget->crank_pos= (Index - CRK_POSITION_COUNT);
        pTarget->phase= PHASE_CYL1_EX;

        return EXEC_OK;
    }
    else if(Index < 3* CRK_POSITION_COUNT)
    {
        //compression stroke from previous cycle
        pTarget->crank_pos= (Index - 2* CRK_POSITION_COUNT);
        pTarget->phase= PHASE_CYL1_COMP;

        return EXEC_OK;
    }

    //invalid index supplied
    return EXEC_ERROR;
}


/*
helper function to find the process table index related to the process position provided
in the range from compression stroke to exhaust stroke
(far compression stroke would be ambiguous)
*/
exec_result_t get_index_from_position(volatile process_position_t * pPosition, volatile U32 * pTargetIndex)
{
    U32 index;

    if((pPosition->crank_pos >= CRK_POSITION_COUNT) || (pPosition->phase >= PHASE_UNDEFINED) )
    {
        //invalid input data
        * pTargetIndex= 0;

        return EXEC_ERROR;
    }

    index= pPosition->crank_pos;

    if(pPosition->phase == PHASE_CYL1_EX)
    {
        index += CRK_POSITION_COUNT;
    }

    *pTargetIndex= index;

    return EXEC_OK;
}


/**
find the first position that comes directly BEFORE OR AT the desired absolute process advance angle (relative to compression TDC)
so that an actor, triggering on this position can provide a proper timing by adding a delay

in case of error the state of pTarget will be unchanged
*/
exec_result_t find_process_position_before(volatile process_advance_t Reference_PA, volatile process_position_t * pTarget)
{
    U32 index;
    exec_result_t result;
    process_advance_t base_PA;

    /*
    loop through the positions
    */
    for(index= 0; index < PROCESS_TABLE_LENGTH; index++)
    {
        base_PA= ProcessTable[index];

        if(base_PA >= Reference_PA)
        {
            //bingo!
            result= get_position_from_index(index, pTarget);

            ASSERT_EXEC_OK(result);

            //add base angle data
            pTarget->base_PA= base_PA;

            return EXEC_OK;
        }
    }

    return EXEC_ERROR;
}


/**
find the first position that comes directly AFTER the desired absolute process advance angle (relative to compression TDC)
so that an actor, triggering directly (with no delay) on this position, executes after the completion of the process, terminating at Reference_PA

in case of error the state of pTarget will be unchanged
*/
exec_result_t find_process_position_after(VU32 Reference_PA, volatile process_position_t * pTarget)
{
    U32 index;
    exec_result_t result;

    /*
    check if the first position in table is already before Reference_PA
    (because then the following position would be "now" (undefined in terms of advance)
    */
    if(ProcessTable[0] >= Reference_PA)
    {
        return EXEC_ERROR;
    }

    for(index= 1; index < PROCESS_TABLE_LENGTH; index++)
    {
        result= get_position_from_index(index -1, pTarget);

        ASSERT_EXEC_OK(result);

        //add base angle data
        pTarget->base_PA= ProcessTable[index -1];

        return EXEC_OK;
    }

    return EXEC_ERROR;
}


/**
returns the absolute process advance angle at the provided position
*/
exec_result_t get_process_advance(volatile process_position_t * pPosition)
{
    exec_result_t result;
    U32 index;

    result= get_index_from_position(pPosition, &index);

    ASSERT_EXEC_OK(result);

    pPosition->base_PA= ProcessTable[index];

    return EXEC_OK;
}


void print_process_table(USART_TypeDef * Port)
{
    U32 index;

    print(Port, "\r\n\r\n*** Process Table *** Index:Advance (deg)\r\n");

    for(index=0; index < PROCESS_TABLE_LENGTH; index++)
    {
        printf_U(Port, index, PAD_2 | NO_TRAIL);
        UART_Tx(Port, ':');
        printf_U(Port, ProcessTable[index], PAD_4);

        if(index == CRK_POSITION_COUNT -1 )
        {
            print(Port, "(COMP)\r\n");
        }
        else if(index == 2* CRK_POSITION_COUNT -1 )
        {
            print(Port, "(EX)\r\n");
        }
        else if(index == 3* CRK_POSITION_COUNT -1 )
        {
            print(Port, "(far COMP)\r\n");
        }

    }
}

