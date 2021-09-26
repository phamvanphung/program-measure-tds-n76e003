#include "N76E003.h"
#include "Common.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "define.h"
#include "Tm1638.h"
#include "Delay.h"

extern float value_tsd_1 ;
extern float value_tsd_2 ;
extern bit FLAT_ERROR ;
extern bit FLAT_MOTOR ;
extern bit mod;
extern bit mod_wash;
extern unsigned int time_mineral_1_10p ;
extern unsigned int time_mineral_2_10p ;
extern unsigned int time_mineral_3_10p ;
extern unsigned char count_mineral_reset;

uint8_t	 array_number[10] = {0x3F,0x01,0x02};
uint8_t value_light = 0x00;


void shiftOut_phung(char value,bit bitFist)
{
	static char w = 0;
	CLK = LOW;
	for (w = 0; w < 8; w++)  
	{
		if (bitFist == LSB) 
		{
			DIO = value & 1;
			value >>= 1;
		}
		else 
		{	
			DIO = value & 128;
			value <<= 1;
		}	
		CLK = HIGH;
		CLK = LOW;		
	}
}

uint8_t shift_in(bit bitfirst)
{
	uint8_t value = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) 
	{
		CLK = HIGH;
		if (bitfirst== LSB)
		{
			value |= get_pin(DIO) << i;
		}
		else
		{
			value |= get_pin(DIO) << (7 - i);
		}
		CLK = LOW;
	}
	return value;
}

int get_pin(char pin)
{
	if(pin == 1)	return 1;
	else return 0;
}

void send_command (char value )
{
	STB = LOW;
  shiftOut_phung(value,LSB);
  STB =	HIGH;
}


void reset_TM1638 (void)
{
	static char w1 = 0;
	send_command(0x40);   // auto increment, nomal mode
	STB = LOW;    					// bat dau truyen lenh
  shiftOut_phung(0xc0,LSB);   // 0xC) = 0b1100000000. 
  for(w1 = 0; w1 < 16; w1++)
	{
    shiftOut_phung( 0x00,LSB);  // Ghi d? li?u 00 và các ô d?a ch?. có 16 ô d?a ch?
	}
	STB = HIGH;
}

void init_tm1638()
{
	STB = HIGH;
	DIO = LOW;
	CLK = HIGH;
	send_command(0x8F);		// set display control 
	reset_TM1638();
	send_command(0x40);		// set  data instruction
}
/*

*/
void show_led_tsd()
{
	static int value_tsd_1_int = 0;
	static int value_tsd_2_int = 0;
	if(FLAT_ERROR == true)
	{
		//show_error();
	}
	else
	{
		if(mod == MOD_FIRST)
		{
			if(value_tsd_1_int > 999)
			{
				mod_wash = MOD_WASH_TWO;
				STB = LOW;
				shiftOut_phung(0xc0,LSB);
				shiftOut_phung(0x76,LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(0x76,LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(0x76,LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			else 
			{
				mod_wash = MOD_WASH_ONE;
				value_tsd_1_int = value_tsd_1;
				STB = LOW;
				shiftOut_phung(0xc0,LSB);
				shiftOut_phung(array_number[value_tsd_1_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_1_int%100/10],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_1_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			if(value_tsd_2 < 20)
			{
				value_tsd_2 = value_tsd_2 * 10;
				value_tsd_2_int = value_tsd_2;
				STB = LOW;
				shiftOut_phung(0xc6,LSB);
				shiftOut_phung(array_number[value_tsd_2_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%100/10]|0x80,LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			else
			{
				value_tsd_2_int = value_tsd_2;
				STB = LOW;
				shiftOut_phung(0xc6,LSB);
				shiftOut_phung(array_number[value_tsd_2_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%100/10],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			if((value_tsd_1_int >=TSD_INPUT_HIGH)&&(value_tsd_1_int < 10000))
			{
				value_light = value_light | (1<<LIGHT_TSD1);
			}
			else if((value_tsd_1_int >=TSD_INPUT_LOW)&&(value_tsd_1_int < TSD_INPUT_HIGH))
			{
				value_light = value_light ^(1<<LIGHT_TSD1);
			}
			else if((value_tsd_1_int >=0)&&(value_tsd_1_int < TSD_INPUT_LOW))
			{
				value_light = value_light & ~(1<<LIGHT_TSD1);
			}
			if((value_tsd_2_int /10 >=TSD_OUTPUT_RO_HIGH)&&(value_tsd_2_int/10 < 10000))
			{
				value_light = value_light | (1<<LIGHT_TSD2);
			}
			else if((value_tsd_2_int/10 >=TSD_OUTPUT_RO_LOW)&&(value_tsd_2_int/10 < TSD_OUTPUT_RO_HIGH))
			{
				value_light = value_light ^ (1<<LIGHT_TSD2);
			}
			else if((value_tsd_2_int/10 >=0)&&(value_tsd_2_int/10 < TSD_OUTPUT_RO_LOW))
			{
				value_light = value_light & ~(1<<LIGHT_TSD2);
			}
		}
		else
		{
			mod_wash = MOD_WASH_TWO;
			if(value_tsd_1 < 20)
			{
				value_tsd_1 = value_tsd_1 * 10;
				value_tsd_1_int = value_tsd_1;
				STB = LOW;
				shiftOut_phung(0xc0,LSB);
				shiftOut_phung(array_number[value_tsd_1_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_1_int%100/10]|0x80,LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_1_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			else
			{
				value_tsd_1_int = value_tsd_1;
				STB = LOW;
				shiftOut_phung(0xc0,LSB);
				shiftOut_phung(array_number[value_tsd_1_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_1_int%100/10],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_1_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			if(value_tsd_2 < 20)
			{
				value_tsd_2 = value_tsd_2 * 10;
				value_tsd_2_int = value_tsd_2;
				STB = LOW;
				shiftOut_phung(0xc6,LSB);
				shiftOut_phung(array_number[value_tsd_2_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%100/10]|0x80,LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			else
			{
				value_tsd_2_int = value_tsd_2;
				STB = LOW;
				shiftOut_phung(0xc6,LSB);
				shiftOut_phung(array_number[value_tsd_2_int/100],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%100/10],LSB);
				shiftOut_phung(0x00,LSB);
				shiftOut_phung(array_number[value_tsd_2_int%10],LSB);
				shiftOut_phung(0x00,LSB);
				STB = HIGH;
			}
			if((value_tsd_1_int/10 >=TSD_OUTPUT_RO_HIGH)&&(value_tsd_1_int/10 < 10000))
			{
				value_light = value_light | (1<<LIGHT_TSD1);
			}
			else if((value_tsd_1_int/10 >=TSD_OUTPUT_RO_LOW)&&(value_tsd_1_int/10 < TSD_OUTPUT_RO_HIGH))
			{
				value_light = value_light ^ (1<<LIGHT_TSD1);
			}
			else if((value_tsd_1_int/10 >=0)&&(value_tsd_1_int/10 < TSD_OUTPUT_RO_LOW))
			{
				value_light = value_light & ~(1<<LIGHT_TSD1);
			}
			if((value_tsd_2_int >=0)&&(value_tsd_2_int < TSD_MINERAL_LOW))
			{
				value_light = value_light | (1<<LIGHT_TSD2);
			}
			else if((value_tsd_2_int >=TSD_MINERAL_LOW)&&(value_tsd_2_int < TSD_MINERAL_HIGH))
			{
				value_light = value_light ^ (1<<LIGHT_TSD2);
			}
			else if((value_tsd_2_int >= TSD_MINERAL_HIGH)&&(value_tsd_2_int < 10000))
			{
				value_light = value_light & ~(1<<LIGHT_TSD2);
			}
		}
	}
}


void show_led_light()
{
	if((time_mineral_1_10p >= 0)&&(time_mineral_1_10p < TIME_NEAR_TIME))
	{
		value_light = value_light & ~(1<<LIGHT_FILTER_1);
	}
	else if((time_mineral_1_10p >= TIME_NEAR_TIME)&&(time_mineral_1_10p < TIME_END_TIME	))
	{
		value_light = value_light ^ (1<<LIGHT_FILTER_1);
	}
	else if((time_mineral_1_10p >= TIME_END_TIME)&&(time_mineral_1_10p < 65535))
	{
		value_light = value_light | (1<<LIGHT_FILTER_1);
	}
	
	if((time_mineral_2_10p >= 0)&&(time_mineral_2_10p < TIME_NEAR_TIME))
	{
		value_light = value_light & ~(1<<LIGHT_FILTER_2);
	}
	else if((time_mineral_2_10p >= TIME_NEAR_TIME)&&(time_mineral_2_10p < TIME_END_TIME	))
	{
		value_light = value_light ^ (1 << LIGHT_FILTER_2);
	}
	else if((time_mineral_2_10p >= TIME_END_TIME) && (time_mineral_2_10p < 65535))
	{
		value_light = value_light | (1 << LIGHT_FILTER_2);
	}
	
	if((time_mineral_3_10p >= 0)&&(time_mineral_3_10p < TIME_NEAR_TIME))
	{
		value_light = value_light & ~(1<<LIGHT_FILTER_3);
	}
	else if((time_mineral_3_10p >= TIME_NEAR_TIME)&&(time_mineral_3_10p < TIME_END_TIME	))
	{
		value_light = value_light ^ (1<<LIGHT_FILTER_3);
	}
	else if((time_mineral_3_10p >= TIME_END_TIME)&&(time_mineral_3_10p < 65535))
	{
		value_light = value_light | (1<<LIGHT_FILTER_3);
	}
	
	STB = LOW;
	shiftOut_phung(0xCC,LSB);
	shiftOut_phung(value_light,LSB);
	STB = HIGH;
}

void show_led_tsd_first_time()
{
	STB = LOW;
	shiftOut_phung(0xC0,LSB);
	shiftOut_phung(0x40,LSB);
	shiftOut_phung(0x00,LSB);
	shiftOut_phung(0x40,LSB);
	shiftOut_phung(0x00,LSB);
	shiftOut_phung(0x40,LSB);
	shiftOut_phung(0x00,LSB);	
	shiftOut_phung(0x40,LSB);
	shiftOut_phung(0x00,LSB);
	shiftOut_phung(0x40,LSB);
	shiftOut_phung(0x00,LSB);
	shiftOut_phung(0x40,LSB);
	shiftOut_phung(0x00,LSB);
	shiftOut_phung(0x00,LSB);
	STB = HIGH;
}

void test_tm1638_led(char value)
{
	STB = LOW;
	shiftOut_phung(0xC0,LSB);
	shiftOut_phung(value,LSB);
	STB = HIGH;
}


uint8_t readButtons(void)
{
 uint8_t buttons = 0;
 STB = LOW;
 shiftOut_phung(0x42,LSB);
 Enable_Data_In;  
 buttons = shift_in(LSB);
 STB = HIGH;
 Enable_Data_Out;
 shiftOut_phung(0x40,LSB);
 return buttons;
}

void show_led_light_reset(void)
{
	static unsigned char L = 0;
	if(count_mineral_reset == 1)
	{
			STB = LOW;
			shiftOut_phung(0xCC,LSB);
			L = L ^ (1<<LIGHT_FILTER_1) ;
			shiftOut_phung(L& 0x02 ,LSB);
			STB = HIGH;
	}
	else if(count_mineral_reset == 2)
	{
		STB = LOW;
		shiftOut_phung(0xCC,LSB);
		L = L ^ (1<<LIGHT_FILTER_2) ;
		shiftOut_phung(L& 0x04 ,LSB);
		STB = HIGH;
	}
	else if(count_mineral_reset == 3)
	{
		STB = LOW;
		shiftOut_phung(0xCC,LSB);
		L = L ^(1<< LIGHT_FILTER_3);
		shiftOut_phung(L & 0x08,LSB);
		STB = HIGH;
	}
}








