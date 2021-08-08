#include <stdio.h>
#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "directer_time.h"
#include "task.h"
#include "define.h"
#include "simplekalman.h"

bit FLAT_TSD_1 = false;
bit FLAT_TSD_2 = false;
int time_task_1 = 0;
int time_manage_read_tsd = 0;
int time_resolve_current = 0;
int time_general_program = 0;

char count_tsd_1 = 0;
char count_tsd_2 = 0;

uint16_t array_current_kalman_adc[10];
uint16_t array_volt_out_kalman_adc[10];
uint32_t array_tsd_1_adc_kalman[10];
uint32_t array_tsd_2_adc_kalman[10];
_kalman current;
_kalman volt_out;
_kalman tsd_1_volt;
_kalman tsd_2_volt;
_kalman tsd_1_kalman;
_kalman tsd_2_kalman;

int adc_avr_current = 0;

/*
example task: 
task control 2 led:
step 1: on led on pin 1 in 3s
step 2: off led 1. blink led on pin 2 with F = 2hz in 5 hz
step 3: all led on in 5s. after goto step 1
*/
void task_1(void)
{
	static char step_task_1 = 1;
	static int time_blink = 0;
	switch (step_task_1)
	{
		case 1:
		{
			Led1 = 1;
			Led2 = 0;
			if( time_task_1 >= 6)
			{
				step_task_1 = 2;
				time_task_1	= 0;
				Led1 = 0;
			}
			break;
		}
		case 2:
		{
			if(time_task_1 != time_blink)
			{
				Led2 = ~Led2;
				time_blink = time_task_1;
			}
			if(time_task_1 >= 10)
			{
				step_task_1 = 3;
				time_task_1	= 0;
			}
			break;
		}
		case 3:
		{
			Led1 = 1;
			Led2 = 1;
			if(time_task_1 >= 10)
			{
				step_task_1 = 1;
				time_task_1	= 0;
			}
			break;
		}
	}
}

void task_read_all_adc(void)
{
	static char count_current ;
  int current_moment_adc ;
	static char count_volt_pwm ;
  int volt_moment_pwm ;
	int adc_tsd_1_volt ;
	int adc_tsd_1 = 0;
  int adc_tsd_1_volt_kalman;
	int adc_tsd_1_kalman ;
	int adc_tsd_2_volt ;
	int adc_tsd_2 = 0;
  int adc_tsd_2_volt_kalman ;
	int adc_tsd_2_kalman ;
	uint32_t temp = 0;
	// read current fisrt
	Enable_ADC_current;
	clr_ADCF;
	set_ADCS;
	while(ADCF == 0);
	current_moment_adc = ADCRH << 4| ADCRL;
	Disable_ADC;
	array_current_kalman_adc[count_current] = update_measure_kalman(&current,(float) current_moment_adc);
	if(array_current_kalman_adc[count_current] >= ADC_CURRENT_OVER_LOAD)		// check_short_circuit
	{
		// OFF POWER- Anounce ERROR
	}
	count_current = count_current + 1;
	if(count_current >= 10) count_current = 0;
	// read volt to control pwm second
	Enable_ADC_Vout_PWM;
	clr_ADCF;
	set_ADCS;
	while(ADCF == 0);
	volt_moment_pwm = ADCRH << 4| ADCRL;
	Disable_ADC;
	array_volt_out_kalman_adc[count_volt_pwm] = update_measure_kalman(&volt_out,(float) volt_moment_pwm);
	count_volt_pwm = count_volt_pwm + 1;
	if(count_volt_pwm >= 10) count_volt_pwm = 0;
	// read TSD third
	if(FLAT_TSD_1 == true)
	{
		TSD_1 = on;
		Enable_ADC_vTSD_1;
		clr_ADCF;
		set_ADCS;
		while(ADCF == 0);
		adc_tsd_1_volt = ADCRH << 4| ADCRL;
		Disable_ADC;
		adc_tsd_1_volt_kalman = update_measure_kalman(&tsd_1_volt,(float)adc_tsd_1_volt);
		Enable_ADC_TSD_1;
		clr_ADCF;
		set_ADCS;
		while(ADCF == 0);
		adc_tsd_1 = ADCRH << 4| ADCRL;
		Disable_ADC;
		adc_tsd_1_kalman = update_measure_kalman(&tsd_1_kalman,(float)adc_tsd_1);
		temp = adc_tsd_1_kalman * R1_TSD;
		array_tsd_1_adc_kalman[count_tsd_1] =  temp / (adc_tsd_1_volt_kalman - adc_tsd_1_kalman);
		TSD_1 = off;
		FLAT_TSD_1 = false;
		count_tsd_1 = count_tsd_1 + 1;
		
	}
	if(FLAT_TSD_2 == true)
	{
		TSD_2 = on;
		Enable_ADC_vTSD_2;
		clr_ADCF;
		set_ADCS;
		while(ADCF == 0);
		adc_tsd_2_volt = ADCRH << 4| ADCRL;
		Disable_ADC;
		adc_tsd_2_volt_kalman = update_measure_kalman(&tsd_2_volt,(float)adc_tsd_2_volt);
		Enable_ADC_TSD_2;
		clr_ADCF;
		set_ADCS;
		while(ADCF == 0);
		adc_tsd_2 = ADCRH << 4| ADCRL;
		Disable_ADC;
		adc_tsd_2_kalman = update_measure_kalman(&tsd_2_kalman,(float)adc_tsd_2);
		temp = adc_tsd_2_kalman * R1_TSD;
		array_tsd_2_adc_kalman[count_tsd_2] =  temp / (adc_tsd_2_volt_kalman - adc_tsd_2_kalman);
		TSD_2 = off;
		FLAT_TSD_2 = false;
		count_tsd_2 = count_tsd_2 + 1;
	}
	
}

void task_init_all_gpio(void)
{
	Enable_FET;
	Enable_TRAN;
	Enable_STB;
	Enable_CLK;
	Enable_Data_Out;
	Enable_TSD_1;
	Enable_TSD_2;
	
	FET = off;
	TRAN = off;
	TSD_1 = off;
	TSD_2 = off;
}

void task_manage_read_tsd(void)
{
	static char step_manage_read_tsd = 1;
	switch(step_manage_read_tsd)
	{
		case 1:
		{
			step_manage_read_tsd = 2;
			time_manage_read_tsd = 0;
			FLAT_TSD_1 = true;
			break;
		}
		case 2:
		{
			if(count_tsd_1 > 9)
			{
				// caculator TSD 1 to show LED 
				
				// reset count_tsd_1
				count_tsd_1 = 0;
			}
			if(time_manage_read_tsd >= 360)
			{
				step_manage_read_tsd = 3;
				time_manage_read_tsd = 0;
				FLAT_TSD_2 = true;
			}
			break;
		}
		case 3:
		{
			if(count_tsd_2 > 9)
			{
				// caculator TSD 1 to show LED 
				
				// reset count_tsd_2
				count_tsd_2 = 0;
			}
			if(time_manage_read_tsd >= 360)
			{
				step_manage_read_tsd = 1;
				time_manage_read_tsd = 0;
			}
			break;
		}
	}
}

void task_resolve_current(void)
{
	static int pre_time_resolve_current = 0;
	static char j;
	if( time_resolve_current == (pre_time_resolve_current + 2))
	{
		// caculator curruent
		int sum_current = 0;
		for(j = 0; j< 10;j++)
		{
			sum_current = sum_current + array_current_kalman_adc[j];
			adc_avr_current = sum_current / 10;
		}
		pre_time_resolve_current = time_resolve_current;
	}
}

void task_general_program(void)
{
	static char step_general_program = 1;
	switch(step_general_program)
	{
		case 1:
		{
			// power on 90% PWM
			// wait motor run
			if(adc_avr_current > ADC_CURRENT_NO_LOAD)
			{
				step_general_program = 2;
				time_general_program = 0;
			}
			break;
		}
		case 2:
		{
			// check curent if motor no run : step_general_program = 1; 
			if(adc_avr_current <= ADC_CURRENT_NO_LOAD)
			{
				step_general_program = 1;
				time_general_program = 0;
			}
			// wait 90 minue
			if(time_general_program >= 7200)
			{
				time_general_program = 0;
				step_general_program = 3;
				// check current to find error 
				// power off PWM
			}
			break;
		}
		case 3:
		{
			// set flat error
			
			// wait 30 minue
			if(time_general_program >= 3600)
			{
				time_general_program = 0;
				step_general_program = 4;
				// clr flat error
				
				// power on 90% PWM
			}
			break;
		}
		case 4:
		{
			// check curent if motor no run : step_general_program = 1; 
			if(adc_avr_current <= ADC_CURRENT_NO_LOAD)
			{
				step_general_program = 1;
				time_general_program = 0;
			}
			// wait 90 minue
			if(time_general_program >= 7200)
			{
				time_general_program = 0;
				step_general_program = 5;
				// check current to find error 
				// power off PWM
			}
			break;
		}
		case 5:
		{
			// set flat error
			// wait 30 minue
			if(time_general_program >= 3600)
			{
				time_general_program = 0;
				step_general_program = 6;
				// clr flat error
				// power on 90% PWM
			}
			break;
		}
		case 6:
		{
			// check curent if motor no run : step_general_program = 1; 
			if(adc_avr_current <= ADC_CURRENT_NO_LOAD)
			{
				step_general_program = 1;
				time_general_program = 0;
			}
			// wait 90 minue
			if(time_general_program >= 7200)
			{
				time_general_program = 0;
				step_general_program = 7;
				// check current to find error 
				// power off PWM
				// ALWAYS OFF - ANNOUNCE ERROR
			}
			break;
		}
		
	}
}

void set_value_kalman_filter(void)
{
	current.measure_e = 1;
	current.estimate_e = 1;
	current.q = 0.001;

	volt_out.measure_e = 1;
	volt_out.estimate_e = 1;
	volt_out.q = 0.001;
	
	tsd_1_volt.measure_e = 1;
	tsd_1_volt.estimate_e = 1;
	tsd_1_volt.q = 0.001;
	
	tsd_2_volt.measure_e = 1;
	tsd_2_volt.estimate_e = 1;
	tsd_2_volt.q = 0.001;
	
	tsd_1_kalman.measure_e = 1;
	tsd_1_kalman.estimate_e = 1;
	tsd_1_kalman.q = 0.001;
	
	tsd_2_kalman.measure_e = 1;
	tsd_2_kalman.estimate_e = 1;
	tsd_2_kalman.q = 0.001;
}


