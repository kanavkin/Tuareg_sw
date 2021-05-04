/**
*/
#include "base_calc.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include "Tuareg_types.h"
#include "process_table.h"

#include "Tuareg_config.h"

#include "Tuareg.h"
#include "Tuareg_decoder.h"

/**
absolute process advance angles for the crank positions at the current engine speed
*/
volatile process_advance_t ProcessTable[PROCESS_TABLE_LENGTH];
volatile bool ProcessTable_valid= false;


/**
calculates the absolute process advance angles based on the information provided in pCrankTable

The process table shall tolerate updates without a valid crank period figure. This happens while the first crank
rotations after decoder getting sync (and the crank speed calculation has not yet catch up.
*/
void update_process_table(VU32 Crank_period_us)
{
    U32 pos, index, advance_deg;
    U32 delay_deg =0;

    //check if a valid crank period has been commanded
    if(Crank_period_us > 0)
    {
        /// TODO (oli#1#): add vr delay range check?
        //calculate VR introduced delay
        delay_deg= calc_rot_angle_deg(Tuareg_Setup.decoder_delay_us, Crank_period_us);
    }

    //fill table according to crank angle data
    for(pos= 0; pos < CRK_POSITION_COUNT; pos++)
    {
        /// TODO (oli#1#): check if this access is correct (packed structure)
        advance_deg= Tuareg_Setup.trigger_advance_map[pos];

        /*
        Position 0 is our TDC reference position
        * Its base advance angle (the one defined by trigger wheel geometric) is the smallest among all trigger positions.
        * The delay, introduced by the VR interface circuit, makes the effective advance angle even smaller
        * When the VR delay is greater than our TDC reference positions base advance angle, it appears that the reference position gets shifted to a previous cycle
        * with an advance angle ~360° -> becoming the position with the largest advance angle -> "reference inversion"
        *
        * Further, the TDC reference position shall not leave its phase
        *
        */
        if(pos == 0)
        {
            //compression stroke directly before the reference cTDC
            ProcessTable[0]= subtract_VU32(advance_deg, delay_deg);

            //exhaust stroke
            ProcessTable[CRK_POSITION_COUNT]= 360 + subtract_VU32(advance_deg, delay_deg);

            //previous compression stroke
            ProcessTable[2* CRK_POSITION_COUNT]= 720 + subtract_VU32(advance_deg, delay_deg);
        }
        else
        {
            //compression stroke directly before the reference cTDC
            ProcessTable[pos]= subtract_VU32(advance_deg, delay_deg);

            //exhaust stroke
            ProcessTable[pos + CRK_POSITION_COUNT]= 360 + advance_deg - delay_deg;

            //previous compression stroke
            ProcessTable[pos + 2* CRK_POSITION_COUNT]= 720 + advance_deg - delay_deg;
        }
    }

    //check process table
    for(index= 0; index < (PROCESS_TABLE_LENGTH -1); index++)
    {
        if(ProcessTable[index] >= ProcessTable[index +1])
        {
            ProcessTable_valid= false;

           /* print(DEBUG_PORT, "\r\nError Process Table inconsistent at index ");
            printf_U(DEBUG_PORT, index, NO_PAD | NO_TRAIL);
            */

            return;
        }

    }

    //all done
    ProcessTable_valid= true;

}


/*
helper function to find the process position corresponding to the process table index provided

in case of error the state of pTarget will be unchanged
*/
exec_result_t get_position_from_index(VU32 Index, volatile process_position_t * pTarget)
{
    if(!ProcessTable_valid)
    {
        return EXEC_ERROR;
    }

    if(Index < CRK_POSITION_COUNT)
    {
        //compression stroke directly before the reference cTDC
        pTarget->crank_pos= Index;
        pTarget->phase= PHASE_CYL1_COMP;
        pTarget->previous_cycle= false;

        return EXEC_OK;
    }
    else if(Index < 2* CRK_POSITION_COUNT)
    {
        //exhaust stroke
        pTarget->crank_pos= (Index - CRK_POSITION_COUNT);
        pTarget->phase= PHASE_CYL1_EX;
        pTarget->previous_cycle= false;

        return EXEC_OK;
    }
    else if(Index < 3* CRK_POSITION_COUNT)
    {
        //compression stroke from previous cycle
        pTarget->crank_pos= (Index - 2* CRK_POSITION_COUNT);
        pTarget->phase= PHASE_CYL1_COMP;
        pTarget->previous_cycle= true;

        return EXEC_OK;
    }

    //invalid index supplied
    return EXEC_ERROR;
}


/*
helper function to find the process table index related to the process position provided
in the range from compression stroke to exhaust stroke

(to reference the far compression stroke "previous_cycle" must be set)
(far exhaust stroke look up not implemented)
*/
exec_result_t get_index_from_position(volatile process_position_t * pPosition, volatile U32 * pTargetIndex)
{
    U32 index;

    if((pPosition->crank_pos >= CRK_POSITION_COUNT) || (pPosition->phase >= PHASE_UNDEFINED) || ((pPosition->phase == PHASE_CYL1_EX) && (pPosition->previous_cycle == true)))
    {
        //invalid input data
        * pTargetIndex= 0;

        return EXEC_ERROR;
    }

    index= pPosition->crank_pos;

    if(pPosition->phase == PHASE_CYL1_EX)
    {
        //phase := exhaust
        index += CRK_POSITION_COUNT;
    }
    else if(pPosition->previous_cycle == true)
    {
        //phase := far compression
        index += 2* CRK_POSITION_COUNT;
    }

    *pTargetIndex= index;

    return EXEC_OK;
}


/**
find the first position that comes directly BEFORE OR AT the desired absolute process advance angle (relative to compression TDC)
so that an actor, triggering on this position can provide a proper timing by adding a delay

in case of error the state of pTarget will be unchanged
*/
exec_result_t find_process_position_before(volatile process_advance_t Reference_PA, volatile process_position_t * pTarget, VU32 Buffer_deg)
{
    U32 index;
    exec_result_t result;
    process_advance_t base_PA;

    if(!ProcessTable_valid)
    {
        return EXEC_ERROR;
    }

    /*
    loop through the positions
    */
    for(index= 0; index < PROCESS_TABLE_LENGTH; index++)
    {
        base_PA= ProcessTable[index];

        if(base_PA >= Reference_PA + Buffer_deg)
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

    if(!ProcessTable_valid)
    {
        return EXEC_ERROR;
    }

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

    if(!ProcessTable_valid)
    {
        return EXEC_ERROR;
    }

    result= get_index_from_position(pPosition, &index);

    ASSERT_EXEC_OK(result);

    //post fence double check
    if(index >= PROCESS_TABLE_LENGTH)
    {
        return EXEC_ERROR;
    }

    pPosition->base_PA= ProcessTable[index];

    return EXEC_OK;
}


void print_process_table(USART_TypeDef * Port)
{
    U32 index;

    if(!ProcessTable_valid)
    {
        print(Port, "\r\n\r\n*** Process Table (!INVALID!) ***\r\n");
    }
    else
    {
        print(Port, "\r\n\r\n*** Process Table ***\r\n");
    }



    for(index=0; index < PROCESS_TABLE_LENGTH; index++)
    {
        printf_U(Port, index, PAD_2);
        UART_Tx(Port, '(');
        printf_crkpos(TS_PORT, (index % CRK_POSITION_COUNT));
        print(Port, "): ");

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

#define PROCESS_TABLE_FANCY_RES 2

/*

-|+++|-------------------------|+++++++++++++++++++|-----------------------------------------|++++|-----------------------------------------|++++|
*/
void print_process_table_fancy()
{
    U32 phase, pos, index;
    bool print_key= false;
    char fill_space;
    U32 dist[CRK_POSITION_COUNT];
    U32 x_count;
    U32 x_pos[CRK_POSITION_COUNT];

    if(!ProcessTable_valid)
    {
        print(TS_PORT, "\r\n\r\n*** Process Table (!INVALID!) ***\r\n");
    }
    else
    {
        print(TS_PORT, "\r\n\r\n*** Process Table ***\r\n");
    }

    //distance until first position
    dist[0]= ProcessTable[0] / PROCESS_TABLE_FANCY_RES;

    //distance until the other positions
    for(pos=1; pos < CRK_POSITION_COUNT; pos++)
    {
        dist[pos]= (ProcessTable[pos] - ProcessTable[pos -1]) / PROCESS_TABLE_FANCY_RES;
    }

    //get the x coordinate of each position
    x_count= 0;

    for(pos=0; pos < CRK_POSITION_COUNT; pos++)
    {
        x_count= x_count + dist[pos] +1;
        x_pos[pos]= x_count;
    }


    UART_TS_PORT_NEXT_LINE();

    for(pos=0; pos < CRK_POSITION_COUNT; pos++)
    {
        UART_Tx(TS_PORT, '(');
        printf_crkpos(TS_PORT, pos);
        UART_Tx(TS_PORT, ')');

        UART_Tx_n(TS_PORT, ' ', 6);
    }

    UART_TS_PORT_NEXT_LINE();

    for(index=0; index < PROCESS_TABLE_LENGTH; index++)
    {
        UART_Tx(TS_PORT, '[');
        printf_U(TS_PORT, index, PAD_2 | NO_TRAIL);
        UART_Tx(TS_PORT, ']');
        UART_Tx(TS_PORT, ' ');
        printf_U(TS_PORT, ProcessTable[index], PAD_4);

        if(index == CRK_POSITION_COUNT -1 )
        {
            print(TS_PORT, "(COMP)\r\n");
        }
        else if(index == 2* CRK_POSITION_COUNT -1 )
        {
            print(TS_PORT, "(EX)\r\n");
        }
        else if(index == 3* CRK_POSITION_COUNT -1 )
        {
            print(TS_PORT, "(far COMP)\r\n");
        }

    }

    UART_TS_PORT_NEXT_LINE();

    for(phase=0; phase < 3; phase++)
    {
        UART_TS_PORT_reset_char_count();

        //process advance numbers - top
        for(pos=0; pos < CRK_POSITION_COUNT; pos++)
        {
            x_count= UART_TS_PORT_get_char_count();

            //print space until position
            UART_Tx_n(TS_PORT, ' ', subtract_VU32(x_pos[pos], x_count +1) );

            UART_Tx(TS_PORT, '[');
            printf_U(TS_PORT, phase * CRK_POSITION_COUNT + pos, NO_PAD | NO_TRAIL);
            UART_Tx(TS_PORT, ']');
        }

        UART_TS_PORT_NEXT_LINE();

        //lines
        for(pos=0; pos < CRK_POSITION_COUNT; pos++)
        {
            fill_space= print_key? '+': '-';

            //print '-' until position
            UART_Tx_n(TS_PORT, fill_space, dist[pos]);

            //print '|' at position
            UART_Tx(TS_PORT, '|');

            print_key= !print_key;
        }












        print(TS_PORT, "\r\n\r\n\r\n");

    }
}

