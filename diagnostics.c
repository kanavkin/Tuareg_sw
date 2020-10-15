#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "diagnostics.h"
#include "conversion.h"
#include "uart.h"

VU32 scheduler_diag[SCHEDIAG_COUNT];
VU32 ignhw_diag[IGNHWDIAG_COUNT];
VU32 tuareg_diag[TDIAG_COUNT];
VU32 decoder_diag[DDIAG_COUNT];



void scheduler_diag_log_event(scheduler_diag_t event)
{
    if(event < SCHEDIAG_COUNT)
    {
        scheduler_diag[event] += 1;
    }
}

void ignhw_diag_log_event(ignhw_diag_t event)
{
    if(event < IGNHWDIAG_COUNT)
    {
        ignhw_diag[event] += 1;
    }
}

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

    UART_Send(Port, "\r\nDELAY_CLIPPED SCHEDIAG_DELAY_ENLARGED ICH1_SET ICH1_CURRCYC_SET ICH1_NEXTC_PRELOAD_SET ICH1_NEXTC_UPDATE_SET ");
    UART_Send(Port, "\r\nICH2_SET FCH1_SET FCH2_SET FCH1_TRIG FCH2_TRIG");
    UART_Send(Port, "\r\nICH1_RESET ICH2_RESET FCH1_RESET FCH2_RESET");
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

    UART_Send(Port, "\r\nSWIRQ3_TRIGGERED");

}

void print_tuareg_diag(USART_TypeDef * Port)
{
    U32 cnt;

    UART_Send(Port, "\r\ntuareg diag: \r\n");

    for(cnt=0; cnt < SCHEDIAG_COUNT; cnt++)
    {
        UART_Print_U(Port, scheduler_diag[cnt], TYPE_U32, PAD);

        if((cnt % 9) == 0)
        {
            UART_Send(Port, "\r\n");
        }

    }

    UART_Send(TS_PORT, "\r\nDECODER_IRQ, DECODER_AGE, DECODER_TIMEOUT, DECODER_PASSIVE, IGNITION_IRQ, MAINLOOP_ENTRY, MODECTRL, INIT_HALT_TR, RUNNING_HALT_TR, RUNNING_STB_TR");
    UART_Send(TS_PORT, "\r\nSTB_RUNNING_TR, STB_HALT_TR, HALT_RUNNING_TR, HALT_STB_TR, ENTER_INIT, ENTER_HALT, ENTER_RUNNING, ENTER_STB, TSTUDIO_CALLS, TRIG_IGN_CALLS");
    UART_Send(TS_PORT, "\r\nTRIG_COIL_DWELL, TRIG_COIL_IGN, KILL_SIDESTAND, KILL_RUNSWITCH TDIAG_INVALID_RUNMODE");
}

void print_decoder_diag(USART_TypeDef * Port)
{
    UART_Send(Port, "\r\ndecoder diag print out not yet implemented");


    /*
    //print decoder diag
            UART_Send(TS_PORT, "\r\ndecoder:\r\n");

            //get diag data
            decoder_export_diag(debug_data);

            data_2 =0;

            for(data_1=0; data_1 < DDIAG_COUNT; data_1++)
            {
                UART_Print_U(TS_PORT, debug_data[data_1],TYPE_U32, PAD);

                if(data_2 == 9)
                {
                    UART_Send(TS_PORT, "\r\n");
                    data_2= 0;
                }
                else
                {
                    data_2++;
                }
            }
                                           //0          0          0          0          0          0          0          0          0          0
            UART_Send(TS_PORT, "\r\n ASYN->SYN  SYN->ASYN  CRKHDLR_C TR_IRQ_SYN TR_IRQ_DEL   CRP_INIT   CRP_SYNC   CRP_A_KEY CRP_A_GAP  TIMEOUT_E");
            UART_Send(TS_PORT, "\r\n TMR_UPD_E   SYNCHK_C  SYNCHK_RLX  CRKTBL_C ROTSPEED_C   CISHDL_C   CRP_PHAS     CRP_UND PHAS->UND");
            UART_Send(TS_PORT, "\r\n");


    */
}
