#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"
#include "conversion.h"
#include "uart.h"

VU32 scheduler_diag[SCHEDIAG_COUNT];
VU32 ignhw_diag[IGNHWDIAG_COUNT];
VU32 tuareg_diag[TDIAG_COUNT];
VU32 decoder_diag[DDIAG_COUNT];


/******************************************************************************************************
scheduler diag
*******************************************************************************************************/
void scheduler_diag_log_event(scheduler_diag_t event)
{
    if(event < SCHEDIAG_COUNT)
    {
        scheduler_diag[event] += 1;
    }
}


void print_scheduler_diag(USART_TypeDef * Port)
{
    U32 cnt;

    UART_Send(Port, "\r\nscheduler diag: \r\n");

    for(cnt=0; cnt < SCHEDIAG_COUNT; cnt++)
    {
        UART_Print_U(Port, scheduler_diag[cnt], TYPE_U32, PAD);

        if((cnt != 0) && ((cnt % 5) == 0))
        {
            UART_Send(Port, "\r\n");
        }

    }
}


void print_scheduler_diag_legend(USART_TypeDef * Port)
{
    UART_Send(Port, "\r\nDELAY_CLIPPED, DELAY_ENLARGED, ICH1_SET, ICH1_CURRC_SET, ICH1_NEXTC_PRELOAD_SET,");
    UART_Send(Port, "\r\nICH1_NEXTC_UPDATE_SET, ICH2_SET, FCH1_SET, FCH2_SET, ICH1_TRIG,");
    UART_Send(Port, "\r\nICH2_TRIG, FCH1_TRIG, FCH2_TRIG, ICH1_RESET, ICH2_RESET,");
    UART_Send(Port, "\r\nFCH1_RESET, FCH2_RESET, ICH1_RETRIGD, ICH2_RETRIGD, FCH1_RETRIGD,");
    UART_Send(Port, "\r\nFCH2_RETRIGD");
}




/******************************************************************************************************
ignition hardware diag
*******************************************************************************************************/
void ignhw_diag_log_event(ignhw_diag_t event)
{
    if(event < IGNHWDIAG_COUNT)
    {
        ignhw_diag[event] += 1;
    }
}


void print_ignhw_diag(USART_TypeDef * Port)
{
    U32 cnt;

    UART_Send(Port, "\r\nignition hardware diag: \r\n");

    for(cnt=0; cnt < IGNHWDIAG_COUNT; cnt++)
    {
        UART_Print_U(Port, ignhw_diag[cnt], TYPE_U32, PAD);

        if((cnt != 0) && ((cnt % 5) == 0))
        {
            UART_Send(Port, "\r\n");
        }

    }
}


void print_ignhw_diag_legend(USART_TypeDef * Port)
{
    UART_Send(TS_PORT, "\r\nIGNHWDIAG_SWIER3_TRIGGERED");
}



/******************************************************************************************************
Tuareg main diag
*******************************************************************************************************/
void tuareg_diag_log_event(tuareg_diag_t event)
{
    if(event < TDIAG_COUNT)
    {
        tuareg_diag[event] += 1;
    }
}


void tuareg_diag_log_parameter(tuareg_diag_t Parameter, U32 Value)
{
    if(Parameter < TDIAG_COUNT)
    {
        tuareg_diag[Parameter]= Value;
    }
}


void print_tuareg_diag(USART_TypeDef * Port)
{
    U32 cnt;

    UART_Send(Port, "\r\nTuareg diag: \r\n");

    for(cnt=0; cnt < TDIAG_COUNT; cnt++)
    {
        UART_Print_U(Port, tuareg_diag[cnt], TYPE_U32, PAD);

        if((cnt != 0) && ((cnt % 5) == 0))
        {
            UART_Send(Port, "\r\n");
        }

    }
}


void print_tuareg_diag_legend(USART_TypeDef * Port)
{
    UART_Send(TS_PORT, "\r\nTDIAG_DECODER_IRQ, TDIAG_DECODER_UPDATE, TDIAG_DECODER_TIMEOUT, TDIAG_DECODER_PASSIVE, TDIAG_IGNITION_IRQ,");
    UART_Send(TS_PORT, "\r\nTDIAG_TRIG_IGN_CALLS, TDIAG_TRIG_COIL_DWELL, TDIAG_TRIG_COIL_IGN, TDIAG_PROCESSDATA_CALLS, TDIAG_IGNITIONCALC_CALLS,");
    UART_Send(TS_PORT, "\r\nTDIAG_TSTUDIO_CALLS, TDIAG_MODECTRL, TDIAG_KILL_SIDESTAND, TDIAG_KILL_RUNSWITCH, TDIAG_ENTER_INIT,");
    UART_Send(TS_PORT, "\r\nTDIAG_ENTER_HALT, TDIAG_RUNNING_HALT_TR, TDIAG_STB_HALT_TR, TDIAG_INIT_HALT_TR, TDIAG_ENTER_RUNNING,");
    UART_Send(TS_PORT, "\r\nTDIAG_CRANKING_RUNNING_TR, TDIAG_HALT_RUNNING_TR, TDIAG_ENTER_STB, TDIAG_RUNNING_STB_TR, TDIAG_CRANKING_STB_TR,");
    UART_Send(TS_PORT, "\r\nTDIAG_HALT_STB_TR, TDIAG_ENTER_CRANKING, TDIAG_ENTER_MTEST, TDIAG_INVALID_RUNMODE");
}


/******************************************************************************************************
decoder diag
*******************************************************************************************************/
void decoder_diag_log_event(decoder_diag_t Event)
{
    if(Event < DDIAG_COUNT)
    {
        decoder_diag[Event] += 1;
    }
}


void decoder_diag_log_parameter(decoder_diag_t Parameter, U32 Value)
{
    if(Parameter < DDIAG_COUNT)
    {
        decoder_diag[Parameter]= Value;
    }
}


void print_decoder_diag(USART_TypeDef * Port)
{
    U32 cnt, column = 1;

    UART_Send(Port, "\r\nDecoder diag: \r\n");

    for(cnt=0; cnt < DDIAG_COUNT; cnt++)
    {
        UART_Print_U(Port, decoder_diag[cnt], TYPE_U32, PAD);

        if(column == DIAG_PRINT_LEGEND_COLUMNS)
        {
            UART_Send(Port, "\r\n");
            column = 1;
        }
        else
        {
            column++;
        }
    }
}

void print_decoder_diag_legend(USART_TypeDef * Port)
{
    UART_Send(TS_PORT, "\r\nDDIAG_HW_EXTI0_CALLS, DDIAG_CRANKHANDLER_CALLS, DDIAG_CRANKPOS_INIT, DDIAG_CRANKPOS_SYNC, DDIAG_CRANKPOS_ASYNC,");
    UART_Send(TS_PORT, "\r\nDDIAG_CRANKPOS_ASYNC_KEY, DDIAG_CRANKPOS_ASYNC_GAP, DDIAG_ASYNC_SYNC_TR, DDIAG_SYNC_ASYNC_TR, DDIAG_HW_SWIER2_CALLS,");
    UART_Send(TS_PORT, "\r\nDDIAG_TRIGGER_IRQ_SYNC, DDIAG_HW_TIM9_CC1_CALLS, DDIAG_TIMER_COMPARE_EVENTS, DDIAG_HW_TIM9_UE_CALLS, DDIAG_TIMER_UPDATE_EVENTS,");
    UART_Send(TS_PORT, "\r\nDDIAG_TIMEOUT_EVENTS, DDIAG_SYNCCHECK_CALLS, DDIAG_SYNCCHECK_SUCCESS, DDIAG_SYNCCHECK_FAILED, DDIAG_CRANKTABLE_CALLS,");
    UART_Send(TS_PORT, "\r\nDDIAG_ROTSPEED_CALLS, DDIAG_HW_EXTI1_CALLS,");
}
