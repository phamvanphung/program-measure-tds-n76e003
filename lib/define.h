
#define Enable_ADC_current			Enable_ADC_AIN0
#define Enable_ADC_Vout_PWM			Enable_ADC_AIN1
#define Enable_ADC_vTSD_1				Enable_ADC_AIN2
#define Enable_ADC_vTSD_2				Enable_ADC_AIN3
#define Enable_ADC_TSD_1				Enable_ADC_AIN4
#define Enable_ADC_TSD_2				Enable_ADC_AIN5
#define	Enable_ADC_Vin					Enable_ADC_AIN6

#define on		1
#define	off		0
#define true	1
#define false 0
#define LSB 	1
#define MSB		0
#define HIGH	1
#define LOW		0
#define MOD_FIRST 0
#define MOD_SEC		1
#define MOD_WASH_ONE 0
#define MOD_WASH_TWO 1


#define	Enable_TRAN			P01_PushPull_Mode
#define	Enable_STB			P02_PushPull_Mode
#define	Enable_CLK			P03_PushPull_Mode
#define	Enable_Data_In	P15_Input_Mode
#define	Enable_Data_Out	P15_PushPull_Mode
#define	Enable_TSD_1		P04_PushPull_Mode
#define	Enable_TSD_2		P05_PushPull_Mode


// GPIO - ON/OFF
#define	TRAN	P01
#define	TSD_1	P04
#define	TSD_2	P05
#define CLK		P03
#define DIO		P15
#define	STB		P02

#define R1_TSD	510

#define ADC_CURRENT_NO_LOAD  123
#define ADC_CURRENT_OVER_LOAD  1200
#define ADC_CURRENT_WASH 	567
#define ADC_CURRENT_SHORT_CIRCUIT 4095

#define ADC_CURRENT_HAVE_WATER	340
#define ADC_CURRENT_NARROW_VANE		990


#define POWER_OFF		power_off()
#define POWER_ON		power_on()

#define ADC_POWER_24V		234
#define ADC_POWER_25V		345
#define PWMH	PWM2H
#define PWML	PWM2L

#define TIME_PROTECT_ON		7200
#define TIME_PROTECT_OFF	3600
#define TIME_WAIT_TO_WASH	2400
#define TIME_OVER_LOAD		30
#define TIME_NEAR_TIME		42000
#define TIME_END_TIME			60000
#define TIME_SHOW_LED_7		3
#define TIME_SHOW_LED_LIGHT	0

#define LIGHT_TSD1			0
#define LIGHT_TSD2			4
#define LIGHT_FILTER_1	1
#define LIGHT_FILTER_2	2
#define LIGHT_FILTER_3	3

#define TSD_INPUT_LOW		100
#define TSD_INPUT_HIGH	500
#define TSD_OUTPUT_RO_LOW	10
#define TSD_OUTPUT_RO_HIGH	30
#define TSD_MINERAL_LOW		40
#define TSD_MINERAL_HIGH	70






