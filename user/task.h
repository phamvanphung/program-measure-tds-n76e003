#define Led1	P12
#define Led2	P13
#define PinGND1	P11
#define PinGND2	P14

void task_1(void);
void task_read_all_adc(void);
void task_init_all_gpio(void);
void task_manage_read_tsd(void);
void task_resolve_current(void);
void task_check_error_current();
void task_error_over_load_current();
void task_general_program(void);
void task_wash_valve(void);
void task_test_pwm(void);
void task_init_pwm(void);
void task_control_power_pwm(void);
void set_value_kalman_filter(void);
void power_off();
void power_on();
void task_off_water_purifier_endless();
void task_show_led_light();
void task_show_led_tsd();
void task_count_time_mineral();
void task_reset_mineral();
float caculate_tsd(uint32_t array[]);
void PinInterrupt_ISR (void);
void set_up_interrup_pin();
