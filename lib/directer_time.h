
#define RELOAD_TIMER3_500ms 		65536-62500			// 0.5 / 16000000/128


void enable_timer_3(void);
void enable_timer_read_adc(void);
void Timer3_ISR (void);
void Timer0_ISR (void);
void all_task_timer_step(void);