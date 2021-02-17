#ifndef MODULE_TEST_H_INCLUDED
#define MODULE_TEST_H_INCLUDED


//#define TUAREG_MODULE_TEST
//#define TUAREG_MODULE_TEST_SCHEDULER

#define MODULE_TEST_SCHEDULER_DELAY_MIN 5
#define MODULE_TEST_SCHEDULER_DELAY_MAX 500000
#define MODULE_TEST_SCHEDULER_DELAY_INCREMENT 200


void moduletest_main_action();
void moduletest_irq2_action();
void moduletest_irq3_action();

void moduletest_scheduler_main_action();
void moduletest_scheduler_irq3_action();


#endif // MODULE_TEST_H_INCLUDED
