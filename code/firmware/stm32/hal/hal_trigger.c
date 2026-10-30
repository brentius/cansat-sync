/* GPL-3.0-only. Prime a complete first active-low pulse after capture arming. */
#include "hal_trigger.h"
void hal_trigger_prime(hal_tim_t *r,uint16_t psc,uint16_t arr,uint16_t ccr)
{
    r->cr1=0; r->psc=psc; r->arr=arr; r->ccr[2]=ccr;
    r->ccer=0; r->ccmr2=0; /* frozen before entering PWM recomputes OCREF */
    r->egr=1; r->sr=0; r->cnt=arr;
    r->ccmr2=(7u<<4)|HAL_BIT(3); r->ccer=HAL_BIT(8);
}
