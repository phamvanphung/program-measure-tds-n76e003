#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "directer_time.h"
#include "task.h"

extern unsigned int time_task_1;
extern unsigned int time_manage_read_tsd ;
extern unsigned int time_resolve_current ;
extern unsigned int time_general_program;
extern unsigned int time_wash_valve ;
extern unsigned int time_test_pwm;
extern unsigned int time_control_power_pwm ;
extern unsigned int time_error_over_load_current;
extern unsigned int time_count_time_mineral;
extern unsigned int time_show_led_light;
extern unsigned int time_show_led_tsd;
extern unsigned int time_reset_mineral;
void Timer3_ISR (void) interrupt 16 
{
	clr_TF3;
	all_task_timer_step();
}


void Timer0_ISR (void) interrupt 1
{
	clr_TF0;
	TH0 = 0xCB;
	TL0 = 0xEA;
	task_read_all_adc();
}


void enable_timer_3(void)
{
	// select clock for timer	: Ftimer = Fsys/128
	set_T3PS2;
	set_T3PS1;
	set_T3PS0;
	
	// set Priority is level 1
	//set_PT3;
	//clr_PT3H;
	
	// set value counter reload timer
	RH3 = RELOAD_TIMER3_500ms / 256;
	RL3 = RELOAD_TIMER3_500ms % 256;
	
	// enable interrup and timer 3. 
	clr_TF3;
	set_ET3;   //enable Timer3 interrupts
	set_EA;    //enable interrupts
	set_TR3;   //Timer3 run
}

void enable_timer_read_adc(void)		// timer 0 use read ADC
{
	TIMER0_MODE1_ENABLE;
	clr_T0M;
	// set Priority is level 2
	//set_PT0H;
	//clr_PT0;
	// set value counter reload timer	. value	 = 65536 - ((Fsys/12)*0.00002). timer interrup after 25us.
	TH0 = 0xCB;
	TL0 = 0xEA;
	
	// enable interrup and timer 0. 
	clr_TF0;
	set_ET0;   //enable Timer0 interrupts
	set_EA;    //enable interrupts
	set_TR0;   //Timer0 run
	
}
/*
	one task can have many step. task control time by valuable task_time. And every task only task_time call task_time_"number".
	task_time help manage step. Change new step to reload task_time.
	one task have paramater is Step and task_time.
	
example task: 
task control 2 led:
step 1: on led on pin 1 in 3s
step 2: off led 1. blink led on pin 2 with F = 2hz in 5 hz
step 3: all led on in 5s.
step 4: goto step 1.
*/



void all_task_timer_step(void)
{
		time_task_1 = time_task_1 + 1;
		time_manage_read_tsd = time_manage_read_tsd + 1;
		time_resolve_current = time_resolve_current + 1;
		time_general_program = time_general_program+ 1;
		time_wash_valve =  time_wash_valve  + 1;
		time_test_pwm = time_test_pwm + 1;
		time_control_power_pwm = time_control_power_pwm + 1;
		time_error_over_load_current = time_error_over_load_current + 1;
		time_count_time_mineral = time_count_time_mineral + 1;
		time_show_led_light = time_show_led_light + 1;
		time_show_led_tsd = time_show_led_tsd + 1;
		time_reset_mineral = time_reset_mineral + 1;
}

