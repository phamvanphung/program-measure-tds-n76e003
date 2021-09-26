#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "directer_time.h"
#include "task.h"
#include "define.h"
#include "simplekalman.h"
#include "Tm1638.h"

bit FLAT_TSD_1 = false;
bit FLAT_TSD_2 = false;
bit FLAT_POWER = true;
bit FLAT_MOTOR = true;
bit FLAT_ERROR = false;
bit mod	= MOD_FIRST;
bit mod_wash = MOD_WASH_TWO;		
bit first_read_tsd = false;
bit FLAT_reset_led_light = false;
unsigned int time_task_1 = 0;
unsigned int time_manage_read_tsd = 0;
unsigned int time_resolve_current = 0;
unsigned int time_general_program = 0;
unsigned int time_wash_valve = 0;
unsigned int time_test_pwm = 0;
unsigned int time_control_power_pwm = 0 ;
unsigned int time_error_over_load_current = 0;
unsigned int time_count_time_mineral = 0;
unsigned int time_mineral_1_10p = 0;
unsigned int time_mineral_2_10p = 0;
unsigned int time_mineral_3_10p = 0;
unsigned int time_show_led_light = 0;
unsigned int time_show_led_tsd = 0;
unsigned int time_reset_mineral = 0;

unsigned char count_tsd_1 = 0;
unsigned char count_tsd_2 = 0;
unsigned char count_mineral_reset = 0;


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

unsigned int adc_avr_current = 0;
unsigned int adc_max_current = 0;
unsigned char number_error = 0;

float value_tsd_1 = 0;
float value_tsd_2 = 0;



/*
example task: 
task control 2 led:
step 1: on led on pin 1 in 3s
step 2: off led 1. blink led on pin 2 with F = 2hz in 5 hz
step 3: all led on in 5s. after goto step 1
*/
void task_1(void)
{
	static unsigned char step_task_1 = 1;
	static unsigned int time_blink = 0;
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
	static unsigned char count_current ;
  unsigned int current_moment_adc =0 ;
	static unsigned char count_volt_pwm =0;
  unsigned int volt_moment_pwm =0;
	unsigned int adc_tsd_1_volt =0;
	unsigned int adc_tsd_1 = 0;
  unsigned int adc_tsd_1_volt_kalman =0;
	unsigned int adc_tsd_1_kalman ;
	unsigned int adc_tsd_2_volt =0 ;
	unsigned int adc_tsd_2 = 0;
  unsigned int adc_tsd_2_volt_kalman =0;
	unsigned int adc_tsd_2_kalman =0;
	uint32_t temp = 0;
	// read current fisrt
	Enable_ADC_current;
	clr_ADCF;
	set_ADCS;
	while(ADCF == 0);
	current_moment_adc = ADCRH << 4| ADCRL;
	Disable_ADC;
	array_current_kalman_adc[count_current] = update_measure_kalman(&current,(float) current_moment_adc);
	if(array_current_kalman_adc[count_current] >= ADC_CURRENT_SHORT_CIRCUIT)		// check_short_circuit
	{
		// ALWAYS OFF - ANNOUNCE ERROR
		 task_off_water_purifier_endless();
	}
	count_current = count_current + 1;
	if(count_current >= 10) count_current = 0;
	// read volt to control pwm 
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
	Enable_TRAN;
	Enable_STB;
	Enable_CLK;
	Enable_Data_Out;
	Enable_TSD_1;
	Enable_TSD_2;
	
	TRAN = off;
	TSD_1 = off;
	TSD_2 = off;
}

void task_manage_read_tsd(void)
{
	static unsigned char step_manage_read_tsd = 1;
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
				value_tsd_1 = caculate_tsd(array_tsd_1_adc_kalman[10]);
				// reset count_tsd_1
				count_tsd_1 = 0;
			}
			if(time_manage_read_tsd >= 360)					// between 2 time is 3 minnue
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
				value_tsd_1 = caculate_tsd(array_tsd_2_adc_kalman[10]);
				first_read_tsd  = false;
				// reset count_tsd_2
				count_tsd_2 = 0;
			}
			if(time_manage_read_tsd >= 2)
			{
				step_manage_read_tsd = 1;
				time_manage_read_tsd = 0;
			}
			break;
		}
	}
}

float caculate_tsd(uint32_t array[])
{
	static float r2_f = 0;
	static float tsd = 0;
	static uint8_t c = 0;
	for(c = 0;c < 10; c++)
	{
		r2_f = r2_f + array[c];
	}
	r2_f = r2_f / 10.0;
	tsd = (1.0 / r2_f)*(paramater_A / paramater_d);
	return tsd;
}

void task_resolve_current(void)
{
	static unsigned char j;
	if( time_resolve_current >= 2)
	{
		// caculator curruent
		unsigned int sum_current = 0;
		for(j = 0; j< 10;j++)
		{
			sum_current = sum_current + array_current_kalman_adc[j];
			adc_avr_current = sum_current / 10;
		}
		if(adc_avr_current >= adc_max_current) 
		{
			adc_max_current = adc_avr_current;
		}
		time_resolve_current = 0;
	}
}

/*****************************************
number_error : 1 - don't have water input
number_error : 2 - Vane output broken
number_error : 3 - Vane was narrow
number_error : 4 - over load
number_error : 5 - short circuit
******************************************/
void task_check_error_current()
{
	
	if((adc_max_current >= ADC_CURRENT_NO_LOAD )&&(adc_max_current < ADC_CURRENT_HAVE_WATER))
	{
		number_error = 1;
	}
	else if((adc_max_current >= ADC_CURRENT_HAVE_WATER)&&(adc_max_current < ADC_CURRENT_NARROW_VANE	))
	{
		number_error = 2;
	}
	else if((adc_max_current >= ADC_CURRENT_NARROW_VANE	)&&(adc_max_current < ADC_CURRENT_OVER_LOAD))
	{
		number_error = 3;
	}
}

void task_error_over_load_current()
{
	static unsigned char step_error_over_load_current = 1;
	switch (step_error_over_load_current)
	{
		case 1:
		{
			if((time_error_over_load_current >= 4)&&(adc_avr_current >= ADC_CURRENT_OVER_LOAD))
			{
				step_error_over_load_current = 2;
				time_error_over_load_current = 0;
			}
			break;
		}
		case 2:
		{
			
			if(time_error_over_load_current >= TIME_OVER_LOAD)
			{
				if(adc_avr_current >= ADC_CURRENT_OVER_LOAD)
				{
					step_error_over_load_current = 3;
					time_error_over_load_current = 0;
				}
				else
				{
					step_error_over_load_current = 1;
					time_error_over_load_current = 0;
				}
			}
			break;
			
		}
		case 3:
		{
			if(time_error_over_load_current >= TIME_OVER_LOAD)
			{
				if(adc_avr_current >= ADC_CURRENT_OVER_LOAD)
				{
					step_error_over_load_current = 4;
					time_error_over_load_current = 0;
				}
				else
				{
					step_error_over_load_current = 1;
					time_error_over_load_current = 0;
				}
			}
			break;
		}
		case 4:
		{
			if(time_error_over_load_current >= TIME_OVER_LOAD)
			{
				if(adc_avr_current >= ADC_CURRENT_OVER_LOAD)
				{
				// ALWAYS OFF - ANNOUNCE ERROR
				//	task_off_water_purifier_endless();
				}
				else
				{
					step_error_over_load_current = 1;
					time_error_over_load_current = 0;
				}
			}
			break;
		}
	}
}
void task_general_program(void)
{
	static unsigned char step_general_program = 1;
	switch(step_general_program)
	{
		case 1:
		{
			// power on flat
			FLAT_POWER = on;
			FLAT_ERROR = false;
			// wait motor run
			if(adc_avr_current > ADC_CURRENT_NO_LOAD)
			{
				step_general_program = 2;
				time_general_program = 0;
				FLAT_MOTOR = on;
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
			if(time_general_program >= TIME_PROTECT_ON)
			{
				time_general_program = 0;
				step_general_program = 3;
				// set flat error
				FLAT_ERROR = true;
				// check current to find error 
				task_check_error_current();
				// power off PWM
				FLAT_POWER = off;
				FLAT_MOTOR = off;
			}
			break;
		}
		case 3:
		{
			
			// wait 30 minue
			if(time_general_program >= TIME_PROTECT_OFF)
			{
				time_general_program = 0;
				step_general_program = 4;
				// clr flat error
				FLAT_ERROR = false;
				// power on flat
				FLAT_MOTOR = on;
				FLAT_POWER = on;
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
			if(time_general_program >= TIME_PROTECT_ON)
			{
				time_general_program = 0;
				step_general_program = 5;
				// set flat error
				FLAT_ERROR = true;
				// check current to find error 
				task_check_error_current();
				// power off PWM
				FLAT_MOTOR = off;
				FLAT_POWER = off;
			}
			break;
		}
		case 5:
		{
			// wait 30 minue
			if(time_general_program >= TIME_PROTECT_OFF)
			{
				time_general_program = 0;
				step_general_program = 6;
				// clr flat error
				FLAT_ERROR = false;
				// power on flat
				FLAT_MOTOR = on;
				FLAT_POWER = on;
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
			if(time_general_program >= TIME_PROTECT_ON)
			{
				time_general_program = 0;
				step_general_program = 7;
				// set flat error
				FLAT_ERROR = true;
				// check current to find error
				task_check_error_current();
				// power off PWM
				FLAT_MOTOR = off;
				FLAT_POWER = off;
				// ALWAYS OFF - ANNOUNCE ERROR
				task_off_water_purifier_endless();
			}
			break;
		}
		
	}
}
/*
	mod_wash : 0 - run mod_wash_one
	mod_wash : 1 - run mod_wash_two
*/
void task_wash_valve(void)
{
	static unsigned char step_wash_valve = 1;
	static unsigned int time_motor_runned = 0;
	switch (step_wash_valve)
	{
		case 1:
		{
			if(FLAT_MOTOR == on) 
			{
				step_wash_valve = 2;
				time_wash_valve = time_motor_runned ; 
				TRAN = off;
			}
			break;
		}
		case 2:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0 ;
			}
			time_motor_runned = time_wash_valve;
			if(time_wash_valve >= TIME_WAIT_TO_WASH)
			{
				time_motor_runned = 0;
				if(mod_wash == MOD_WASH_ONE)
				{
					step_wash_valve = 31; 		// on valve 10s seconcd times
					time_wash_valve = 0;
					TRAN = on;
				}
				else if(mod_wash == MOD_WASH_TWO)
				{
					step_wash_valve = 32;			// on valve 10s fourth times
					time_wash_valve = 0;
					TRAN = on;
				}
			}
			break;
		}
		case 31:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if(time_wash_valve >= 20)
			{
				step_wash_valve = 41;
				time_wash_valve = 0;
				TRAN = off;
			}
			break;
		}
		case 32:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if(time_wash_valve >= 20)
			{
				step_wash_valve = 42;
				time_wash_valve = 0;
				TRAN = off;
			}
			break;
		}
		case 41:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if((time_wash_valve >= 150)||(adc_avr_current >= ADC_CURRENT_WASH))
			{
				step_wash_valve = 51;
				time_wash_valve = 0;
				TRAN = on;
			}
			break;
		}
		case 42:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if((time_wash_valve >= 150)||(adc_avr_current >= ADC_CURRENT_WASH))
			{
				step_wash_valve = 52;
				time_wash_valve = 0;
				TRAN = on;
			}
			break;
		}
		case 51:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if(time_wash_valve >= 20)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
				TRAN = off;
			}
			break;
		}
		case 52:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if(time_wash_valve >= 20)
			{
				step_wash_valve = 62;
				time_wash_valve = 0;
				TRAN = off;
			}
			break;
		}
		case 62:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if((time_wash_valve >= 150)||(adc_avr_current >= ADC_CURRENT_WASH))
			{
				step_wash_valve = 72;
				time_wash_valve = 0;
				TRAN = on;
			}
			break;
		}
		case 72:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if(time_wash_valve >= 20)
			{
				step_wash_valve = 82;
				time_wash_valve = 0;
				TRAN = off;
			}
			break;
		}
		case 82:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if((time_wash_valve >= 150)||(adc_avr_current >= ADC_CURRENT_WASH))
			{
				step_wash_valve = 92;
				time_wash_valve = 0;
				TRAN = on;
			}
			break;
		}
		case 92:
		{
			if(FLAT_MOTOR == off)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
			}
			if(time_wash_valve >= 20)
			{
				step_wash_valve = 1;
				time_wash_valve = 0;
				TRAN = off;
			}
			break;
		}
	}
}

/**********************************************************************
	PWM frequency = Fpwm/((PWMPH,PWMPL) + 1) < Fpwm = Fsys/PWM_CLOCK_DIV> 
								= (16MHz/8)/(0x7CF + 1)
								= 1KHz (1ms)
***********************************************************************/
void task_test_pwm(void)
{
	static unsigned char step_test_pwm = 1;
	switch(step_test_pwm)
	{
		case 1:
		{
			PWM_CLOCK_FSYS;
			POWER_ON;
			PWM_IMDEPENDENT_MODE;
			PWM_CLOCK_DIV_8;
			PWMPH = 0x00;
			PWMPL = 0x64;	
			PWM2H = 0x00;						
			PWM2L = 0x5A;
			set_LOAD;
			set_PWMRUN;
			time_test_pwm = 0;
			step_test_pwm = 2;
			break;
		}
		case 2:
		{
			if(time_test_pwm >= 6)
			{
				PWM2H = 0x00;						
				PWM2L = 0x00;
				set_LOAD;
				time_test_pwm = 0;
				step_test_pwm = 3;
			}
			break;
		}
		case 3:
		{
			if(time_test_pwm >= 4)
			{
				PWM2H = 0x00;						
				PWM2L = 0x32;
				set_LOAD;
				time_test_pwm = 0;
				step_test_pwm = 4;
			}
			break;
		}
		case 4:
		{
			if(time_test_pwm >= 4)
			{
				POWER_OFF;
				time_test_pwm = 0;
				step_test_pwm = 5;
			}
			break;
		}
		case 5:
		{
			if(time_test_pwm >= 2)
			{
				POWER_ON;
				PWM2H = 0x00;						
				PWM2L = 0x60;
				set_LOAD;
				time_test_pwm = 0;
				step_test_pwm = 2;
			}
			
		}
		break;
	}
}

void task_init_pwm(void)
{
	PWM_CLOCK_FSYS;
	PWM_IMDEPENDENT_MODE;
	POWER_OFF;
	PWM_CLOCK_DIV_8;
	PWMPH = 0x00;
	PWMPL = 0x64;
	PWMH = 0x00;						
	PWML = 0x46;
	set_LOAD;
	set_PWMRUN;
}


void task_control_power_pwm(void)
{
	static unsigned char count = 0;
	static unsigned int adc_fb_volt = 0;
	static uint32_t DEC_PWM = 0;
	static unsigned int pre_time = 0;
	if(pre_time != time_control_power_pwm)
	{
		for(count = 0; count<10;count++)
		{
			adc_fb_volt = adc_fb_volt + array_volt_out_kalman_adc[count];
		}
		adc_fb_volt = adc_fb_volt / 10;
		if((adc_fb_volt >= 0)&&(adc_fb_volt < ADC_POWER_25V))
		{
			DEC_PWM = 98;
			PWMH = 0x00;
			PWML = DEC_PWM;
			set_LOAD;
		}
		else if((adc_fb_volt >= ADC_POWER_25V)&&(adc_fb_volt < 4095))
		{
			DEC_PWM =  DEC_PWM * ADC_POWER_24V / adc_fb_volt;
			PWMH = 0x00;
			PWML = DEC_PWM;
			set_LOAD;
		}
		if(FLAT_POWER == on)
		{
			POWER_ON;
		}
		else
		{
			POWER_OFF;
		}
		pre_time = time_control_power_pwm;
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

void power_off()
{
	PWM2_P10_OUTPUT_DISABLE;
	P10 = off;
}

void power_on()
{
	PWM2_P10_OUTPUT_ENABLE;
}

void task_off_water_purifier_endless()
{
	while(1)
	{
		// OFF POWER- Anounce ERROR
		POWER_OFF;
		// Disable unsigned interup
		clr_EA;
		// ANOUNCE LED - SHOW ERROR
		FLAT_ERROR = true;
		show_led_tsd();
	}
}

void task_count_time_mineral()
{
	if(time_count_time_mineral >= 1200)
	{
		if(FLAT_MOTOR == on)
		{
			time_mineral_1_10p = time_mineral_1_10p + 1;
			time_mineral_2_10p = time_mineral_2_10p + 1;
			time_mineral_3_10p = time_mineral_3_10p + 1;
		}
		time_count_time_mineral = 0;
	}
}

void task_show_led_tsd()
{
	if(time_show_led_tsd > TIME_SHOW_LED_7)
	{
		if(first_read_tsd == true)
		{
			show_led_tsd_first_time();
		}
		else
		{
			show_led_tsd();
		}
		time_show_led_tsd = 0;
	}
	
}

void task_show_led_light()
{
	
	if(time_show_led_light > TIME_SHOW_LED_LIGHT)
	{
		if(first_read_tsd == true)
		{
			show_led_tsd_first_time();
		}
		else
		{
			if(FLAT_reset_led_light == true)
			{
				show_led_light_reset();
				test_tm1638_led(0x06);
			}
			else
			{
				show_led_light();
				test_tm1638_led(0x76);
			}
		}
		time_show_led_light = 0;
	}
}

void task_reset_mineral()
{
	static uint8_t step_reset_mineral = 1;
	switch (step_reset_mineral)
	{
		case 1:
		{
			if((readButtons()&0x01) == 1)
			{
				step_reset_mineral = 2;
				time_reset_mineral = 0;
				count_mineral_reset = count_mineral_reset + 1;
				if(count_mineral_reset > 3) count_mineral_reset = 0;
				if(count_mineral_reset > 0) FLAT_reset_led_light = true;
				else FLAT_reset_led_light = false;
			}
			else
			{
				if(time_reset_mineral > 40)
				{
					count_mineral_reset = 0;
				}
			}
			break;
		}
		case 2:
		{
			if((readButtons()&0x01)  == 0)
			{
				step_reset_mineral = 3;
				time_reset_mineral = 0;
			}
			break;
		}
		case 3:
		{
			if((readButtons()&0x01) == 1)
			{
				step_reset_mineral = 4;
				time_reset_mineral = 0;
			}
			if(time_reset_mineral > 40)
			{
				count_mineral_reset = 0;
				step_reset_mineral = 1;
				time_reset_mineral = 0;
				FLAT_reset_led_light = false;
			}
			break;
		}
		case 4:
		{
			if((readButtons()&0x01) == 1)
			{
				if(time_reset_mineral > 6)
				{
					if(count_mineral_reset == 1) time_mineral_1_10p = 0;
					else if(count_mineral_reset == 2) time_mineral_2_10p = 0;
					else if(count_mineral_reset == 3) time_mineral_3_10p = 0;
					count_mineral_reset = 0;
					step_reset_mineral = 5;
					FLAT_reset_led_light = false;
				}
			}
			else 
			{
				step_reset_mineral = 3;
				time_reset_mineral = 0;
				count_mineral_reset = count_mineral_reset + 1;
				if(count_mineral_reset > 3) count_mineral_reset = 0;
				if(count_mineral_reset > 0 ) FLAT_reset_led_light = true;
				else FLAT_reset_led_light = false;
			}
			break;
		}
		case 5:
		{
			if((readButtons()&0x01) == 0)
			{
				step_reset_mineral = 1;
				time_reset_mineral = 0;
			}
		}
	}
}

void PinInterrupt_ISR (void) interrupt 7
{
	if(PIF == 0x20)
	{
		mod = ~mod;
	}
	
}
void set_up_interrup_pin()
{
	// pin 1.5 interup
	P15_Input_Mode;
	set_P1S_5;
	Enable_INT_Port1;
	Enable_BIT5_FallEdge_Trig;
	set_EPI;
	set_EA;
}




