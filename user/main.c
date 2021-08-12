
#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "directer_time.h"
#include "task.h"
#include "define.h"
#include "Tm1638.h"

int main (void)
{
	set_value_kalman_filter();
	Set_All_GPIO_Quasi_Mode;
	enable_timer_3();
	enable_timer_read_adc();
	PinGND2 = 0;
	PinGND1 = 0;
	P01 = 0;
	init_tm1638();
	InitialUART0_Timer1(9600);
	while(true)
	{
		task_1();
		task_test_pwm();
		task_reset_mineral();
		task_show_led_light();
	}
	return 0;
}
