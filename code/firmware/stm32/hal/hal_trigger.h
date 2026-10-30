/* GPL-3.0-only. RM0090 TIM3 PWM2 preparation for board_trig_start(). */
#ifndef HAL_TRIGGER_H
#define HAL_TRIGGER_H
#include "stm32f405.h"
/* Stopped timer, PWM2 idle high; caller connects AF, arms TIM2, then CEN. */
void hal_trigger_prime(hal_tim_t *,uint16_t psc,uint16_t arr,uint16_t ccr);
#endif
