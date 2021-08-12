
void shiftOut_phung (char value,bit bitFist);
void send_command (char value );
void reset_TM1638 (void);
void init_tm1638();
void show_led_tsd();
void show_led_light();
void show_led_tsd_first_time();
void test_tm1638_led(char value);
int get_pin(char pin);
uint8_t readButtons(void);
uint8_t shift_in(bit bitfirst);
void show_led_light_reset(void);