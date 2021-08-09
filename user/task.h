#define Led1	P12
#define Led2	P13
#define PinGND1	P11
#define PinGND2	P14

void task_1(void);

void task_read_all_adc(void);
void task_init_all_gpio(void);
void task_manage_read_tsd(void);
void task_resolve_current(void);
void task_general_program(void);
void task_wash_valve(void);
void task_test_pwm(void);
void set_value_kalman_filter(void);
void motor_off();
void motor_on();
