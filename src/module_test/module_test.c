#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "module_test.h"
//#include "debug.h"
#include "scheduler.h"
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"
#include "ignition_hw.h"
#include "diagnostics.h"


#define TUAREG_MODULE_TEST_SCHEDULER


/**
variables for module test
*/

VU32 mtest_scheduler_delay = MODULE_TEST_SCHEDULER_DELAY_MIN;
VU32 mtest_scheduler_state =0;
VU32 mtest_scheduler_tests_started =0;
VU32 mtest_scheduler_tests_finished =0;


void moduletest_main_action()
{





}

void moduletest_irq2_action()
{





}

void moduletest_irq3_action()
{





}

void moduletest_moduleinit_action()
{





}

void moduletest_initmsg_action()
{

                print(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** WARNING This is Tuareg module test unit, no functional release *** \r \n");

                #ifdef TUAREG_MODULE_TEST_SCHEDULER
                print(DEBUG_PORT, "\r\n*** Scheduler test ***");
                #endif // TUAREG_MODULE_TEST
}


void moduletest_scheduler_main_action()
{
    /*
    if(mtest_scheduler_state == 0)
    {
        mtest_scheduler_state= 0xFF;

        dwt_set_begin();

        //ready for test
        set_ignition_ch1(ACTOR_POWERED);
        scheduler_set_channel(SCHEDULER_CH_IGN1, ACTOR_UNPOWERED, mtest_scheduler_delay, 0);

        mtest_scheduler_tests_started++;

        //increase delay
        if((mtest_scheduler_delay + MODULE_TEST_SCHEDULER_DELAY_INCREMENT) < MODULE_TEST_SCHEDULER_DELAY_MAX)
        {
            mtest_scheduler_delay += MODULE_TEST_SCHEDULER_DELAY_INCREMENT;
        }
    }
    */
}

void moduletest_scheduler_irq3_action()
{
    /*
    //scheduler cycle end
    dwt_set_end();

    mtest_scheduler_tests_finished++;

    //print result
    print(DEBUG_PORT, "\r\nscheduler tests started, finished, target delay (us): ");
    printf_U(DEBUG_PORT, mtest_scheduler_tests_started, NO_PAD);
    printf_U(DEBUG_PORT, mtest_scheduler_tests_finished, NO_PAD);
    printf_U(DEBUG_PORT, mtest_scheduler_delay, NO_PAD);

    print_dwt_delay();

    print_scheduler_diag(DEBUG_PORT);

    mtest_scheduler_state =0;
*/



}
