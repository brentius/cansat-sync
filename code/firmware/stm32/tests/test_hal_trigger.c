/* GPL-3.0-only. Timer model: first physical low pulse must have a falling edge.
 * CS_SOURCES: hal/hal_trigger.c hal/hal_logic.c */
#include "cs_test.h"
#include "../hal/hal_trigger.h"
#include "../hal/hal_logic.h"
CS_TEST(first_pulse_and_capture_alignment)
{
    hal_tim_t r={0}; uint16_t psc,arr,ccr,pulse; uint32_t period,i;
    uint32_t falls[3]={0},rises[3]={0}; unsigned nf=0,nr=0; bool previous;
    CS_CHECK(hal_trig_values(200000,100,&psc,&arr,&ccr,&period,&pulse));
    hal_trigger_prime(&r,psc,arr,ccr);
    CS_CHECK(!(r.cr1&1u)); CS_CHECK_EQ((r.ccmr2>>4)&7u,7);
    /* RM0090 PWM2 active high when CNT >= CCR; externally idle high. */
    previous=r.cnt>=r.ccr[2]; CS_CHECK(previous);
    for(i=1;i<=100050;i++) {
        bool level;
        if(++r.cnt>r.arr) r.cnt=0;
        level=r.cnt>=r.ccr[2];
        if(previous && !level && nf<3) falls[nf++]=i*4u;
        if(!previous && level && nr<3) rises[nr++]=i*4u;
        previous=level;
    }
    CS_CHECK(nf>=2 && nr>=2);
    CS_CHECK_EQ(falls[0],4); CS_CHECK_EQ(rises[0]-falls[0],100);
    CS_CHECK_EQ(falls[1]-falls[0],200000);
}
CS_MAIN(first_pulse_and_capture_alignment)
