/* GPL-3.0-only. Queue wrap, DMA loss and timer quantisation tests.
 * CS_SOURCES: hal/hal_logic.c */
#include "cs_test.h"
#include "../hal/hal_logic.h"
CS_TEST(queue_wrap_and_capacity)
{
    uint32_t mem[3],v=0,i; hal_ring_t q;
    hal_ring_init(&q,mem,sizeof(uint32_t),3);
    q.head=UINT32_MAX-1u; q.tail=q.head;
    for(i=0;i<3;i++) CS_CHECK(hal_ring_push(&q,&i));
    CS_CHECK(!hal_ring_push(&q,&i));
    for(i=0;i<3;i++) { CS_CHECK(hal_ring_pop(&q,&v)); CS_CHECK_EQ(v,i); }
    CS_CHECK(!hal_ring_pop(&q,&v));
}
CS_TEST(dma_wrap_and_loss)
{
    uint32_t consumed=UINT32_MAX-10u,lost=0;
    CS_CHECK_EQ(hal_dma_available(10,&consumed,16,&lost),16);
    CS_CHECK_EQ(lost,5); CS_CHECK_EQ(consumed,UINT32_MAX-5u);
    consumed=10; CS_CHECK_EQ(hal_dma_available(10,&consumed,16,&lost),0);
}
CS_TEST(discard_old_trigger_run_before_rearm)
{
    uint32_t memory[3],tick=0,old=1200261,new_tick=1310261; hal_ring_t q;
    hal_ring_init(&q,memory,sizeof(uint32_t),3);
    q.head=UINT32_MAX; q.tail=UINT32_MAX;
    CS_CHECK(hal_ring_push(&q,&old)); CS_CHECK(hal_ring_push(&q,&old));
    CS_CHECK_EQ(hal_ring_discard(&q),2);
    CS_CHECK(!hal_ring_pop(&q,&tick)); CS_CHECK_EQ(hal_ring_discard(&q),0);
    CS_CHECK(hal_ring_push(&q,&new_tick));
    CS_CHECK(hal_ring_pop(&q,&tick)); CS_CHECK_EQ(tick,new_tick);
    CS_CHECK(!hal_ring_pop(&q,&tick));
}
CS_TEST(trigger_resolution)
{
    uint16_t p,a,c,pulse; uint32_t period;
    CS_CHECK(hal_trig_values(200000,100,&p,&a,&c,&period,&pulse));
    CS_CHECK_EQ(p,335); CS_CHECK_EQ(a,49999); CS_CHECK_EQ(c,25);
    CS_CHECK_EQ(period,200000); CS_CHECK_EQ(pulse,100);
    CS_CHECK(!hal_trig_values(0,10,&p,&a,&c,&period,&pulse));
    CS_CHECK(!hal_trig_values(10,10,&p,&a,&c,&period,&pulse));
    CS_CHECK(!hal_trig_values(UINT32_MAX,10,&p,&a,&c,&period,&pulse));
    CS_CHECK(!hal_trig_values(1000000,1,&p,&a,&c,&period,&pulse));
}
CS_TEST(uart_deadline_does_not_overflow_target_size_t)
{
    /* 512 bytes * 10 bits/byte / baud, then 20000 us allowance. */
    CS_CHECK_EQ(hal_uart_transfer_budget(512,9600),553333);
    CS_CHECK_EQ(hal_uart_transfer_budget(512,115200),64444);
    CS_CHECK_EQ(hal_uart_transfer_budget(512,230400),42222);
    CS_CHECK_EQ(hal_uart_transfer_budget(430,9600),467916);
}
CS_TEST(factory_housekeeping_and_faults)
{
    uint16_t mv; int8_t t;
    hal_adc_convert(2048,1150,1500,1500,1000,1300,&mv,&t);
    CS_CHECK_NEAR(mv,2048.0*3300.0*147.0/(4095.0*47.0),1.0); CS_CHECK_EQ(t,70);
    hal_adc_convert(2048,1000,1500,1500,1000,1300,&mv,&t); CS_CHECK_EQ(t,30);
    hal_adc_convert(2048,1300,1500,1500,1000,1300,&mv,&t); CS_CHECK_EQ(t,110);
    hal_adc_convert(2048,1150,0,1500,1000,1300,&mv,&t); CS_CHECK_EQ(mv,0); CS_CHECK_EQ(t,INT8_MIN);
    hal_adc_convert(2048,943,1500,UINT16_MAX,UINT16_MAX,UINT16_MAX,&mv,&t);
    CS_CHECK(mv>5000 && mv<5300); CS_CHECK(t>=24 && t<=26);
}
CS_MAIN(queue_wrap_and_capacity,dma_wrap_and_loss,trigger_resolution,factory_housekeeping_and_faults,
        uart_deadline_does_not_overflow_target_size_t,discard_old_trigger_run_before_rearm)
