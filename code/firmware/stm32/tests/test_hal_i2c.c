/* GPL-3.0-only. Real I2C state machine against a host register model.
 * CS_SOURCES: hal/hal_i2c.c */
#include "cs_test.h"
#include "../hal/hal_i2c.h"
hal_gpio_t hal_host_gpiob;
hal_tim_t hal_host_tim2;
static unsigned mode_calls;
void hal_gpio_mode(hal_gpio_t *p,unsigned pin,unsigned mode,unsigned af,bool od,unsigned pull)
{ (void)p; (void)pin; (void)mode; (void)af; (void)od; (void)pull; mode_calls++; }
static void event(hal_i2c_t *b,uint32_t status,uint32_t dr)
{ b->regs->sr1=status; b->regs->dr=dr; hal_i2c_irq(b); }
static void to_receive(hal_i2c_t *b,hal_i2c_regs_t *r,size_t nr)
{
    uint8_t cmd=0xa0; uint32_t before;
    CS_CHECK(hal_i2c_start(b,0x76,&cmd,1,nr));
    before=r->cr1; CS_CHECK(before&HAL_BIT(8));
    event(b,1,0); CS_CHECK_EQ(r->dr,0xec);
    event(b,2,0); event(b,HAL_BIT(7),0); CS_CHECK_EQ(r->dr,0xa0);
    CS_CHECK(!(r->cr2&HAL_BIT(10)));
    event(b,4,0); CS_CHECK(r->cr1&HAL_BIT(8));
    event(b,1,0); CS_CHECK_EQ(r->dr,0xed);
    event(b,2,0);
}
CS_TEST(receive_one_two_and_three)
{
    size_t nr;
    for(nr=1;nr<=3;nr++) {
        hal_i2c_regs_t regs={0}; hal_i2c_t b; uint32_t errors=0; uint8_t out[3]={0};
        hal_host_tim2.cnt=100; hal_i2c_init(&b,&regs,6,7,100000,&errors);
        CS_CHECK_EQ(regs.ccr,210); to_receive(&b,&regs,nr);
        if(nr==1) {
            CS_CHECK(!(regs.cr1&HAL_BIT(10))); CS_CHECK(regs.cr1&HAL_BIT(9));
            event(&b,HAL_BIT(6),0x51);
        } else {
            CS_CHECK(!(regs.cr2&HAL_BIT(10)));
            if(nr==2) CS_CHECK(regs.cr1&HAL_BIT(11));
            else { event(&b,4,0x51); CS_CHECK(!(regs.cr1&HAL_BIT(10))); }
            event(&b,4,0x52); CS_CHECK(regs.cr1&HAL_BIT(9));
        }
        CS_CHECK_EQ(hal_i2c_result(&b,out,nr),0); /* STOP still in progress */
        regs.cr1&=~HAL_BIT(9); regs.sr2=0;
        hal_i2c_poll(&b,200); CS_CHECK_EQ(hal_i2c_result(&b,out,nr),1);
        CS_CHECK_EQ(hal_i2c_result(&b,out,nr),0); CS_CHECK_EQ(errors,0);
        CS_CHECK_EQ(out[0],nr==2?0x52:0x51); CS_CHECK_EQ(b.pos,nr);
    }
}
CS_TEST(nack_timeout_and_recovery)
{
    hal_i2c_regs_t regs={0}; hal_i2c_t b; uint32_t errors=0,now; uint8_t cmd=0x48;
    hal_host_tim2.cnt=0; hal_host_gpiob.idr=HAL_BIT(6)|HAL_BIT(7); mode_calls=0;
    hal_i2c_init(&b,&regs,6,7,400000,&errors);
    CS_CHECK_EQ(regs.ccr,HAL_BIT(15)|35u);
    CS_CHECK(!hal_i2c_start(&b,0x76,&cmd,5,0));
    CS_CHECK(!hal_i2c_start(&b,0x76,&cmd,1,17));
    CS_CHECK(hal_i2c_start(&b,0x76,&cmd,1,0));
    event(&b,HAL_BIT(10),0); /* address/data NACK */
    CS_CHECK_EQ(errors,1); CS_CHECK_EQ(hal_i2c_result(&b,0,0),-1);
    CS_CHECK(!hal_i2c_start(&b,0x76,&cmd,1,0));
    for(now=0;now<200;now+=5) hal_i2c_poll(&b,now);
    regs.sr2=0; CS_CHECK(mode_calls>=4);
    CS_CHECK(hal_i2c_start(&b,0x76,&cmd,1,0));
    hal_i2c_poll(&b,4000); CS_CHECK_EQ(errors,2); CS_CHECK_EQ(hal_i2c_result(&b,0,0),-1);
    /* SCL externally held low: recovery remains bounded and returns control. */
    hal_host_gpiob.idr=0;
    for(now=4000;now<7000;now+=5) hal_i2c_poll(&b,now);
    regs.sr2=2;
    CS_CHECK(!hal_i2c_start(&b,0x76,&cmd,1,0)); CS_CHECK_EQ(errors,3);
}
CS_MAIN(receive_one_two_and_three,nack_timeout_and_recovery)
