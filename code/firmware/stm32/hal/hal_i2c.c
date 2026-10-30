/* GPL-3.0-only. ST RM0090 single/two/three-byte receiver sequences.
 * IRQs never wait for a flag. Stuck bus: staged nine clocks then STOP.
 * Checked against ST's I2C driver; own implementation, not copied code. */
#include "hal_i2c.h"
enum { I_IDLE,I_START_W,I_ADDR_W,I_TX,I_START_R,I_ADDR_R,I_RX,I_LAST,I_STOP,I_RECOVER };
static void i2c_config(hal_i2c_t *b)
{
    hal_i2c_regs_t *r=b->regs;
    r->cr1=HAL_BIT(15); r->cr1=0;
    r->cr2=42; r->oar1=HAL_BIT(14);
    if(b->hz==400000) { r->ccr=HAL_BIT(15)|35u; r->trise=13; }
    else { r->ccr=210; r->trise=43; }
    r->cr1=1;
}
void hal_i2c_init(hal_i2c_t *b,hal_i2c_regs_t *regs,unsigned scl,unsigned sda,uint32_t hz,uint32_t *errors)
{
    *b=(hal_i2c_t){0}; b->regs=regs; b->scl=(uint8_t)scl; b->sda=(uint8_t)sda;
    b->hz=hz; b->errors=errors; i2c_config(b);
}
static void i2c_fault(hal_i2c_t *b,uint32_t now,bool was_active)
{
    b->regs->cr2=42; b->regs->cr1=0;
    if(was_active) b->completion=-1;
    (*b->errors)++; b->state=I_RECOVER; b->recovery=0; b->recovery_tick=now;
    hal_gpio_write(HAL_GPIOB,b->scl,true); hal_gpio_write(HAL_GPIOB,b->sda,true);
    hal_gpio_mode(HAL_GPIOB,b->scl,1,0,true,0);
    hal_gpio_mode(HAL_GPIOB,b->sda,1,0,true,0);
}
bool hal_i2c_start(void *ctx,uint8_t addr,const uint8_t *tx,size_t nt,size_t nr)
{
    hal_i2c_t *b=ctx; size_t i; uint32_t key;
    if(!tx || !nt || nt>sizeof b->tx || nr>sizeof b->rx || addr>127) return false;
    key=hal_irq_save();
    if(b->state!=I_IDLE || b->completion) { hal_irq_restore(key); return false; }
    if(b->regs->sr2&2u) {
        i2c_fault(b,HAL_TIM2->cnt,false); hal_irq_restore(key); return false;
    }
    for(i=0;i<nt;i++) b->tx[i]=tx[i];
    b->addr=addr; b->nt=(uint8_t)nt; b->nr=(uint8_t)nr; b->pos=0;
    b->deadline=HAL_TIM2->cnt+3000u; b->state=I_START_W;
    b->regs->cr2=42u|HAL_BIT(8)|HAL_BIT(9)|HAL_BIT(10);
    b->regs->cr1=1u|HAL_BIT(8)|HAL_BIT(10); /* START, ACK, POS cleared */
    hal_irq_restore(key); return true;
}
int hal_i2c_result(void *ctx,uint8_t *out,size_t n)
{
    hal_i2c_t *b=ctx; size_t i; int done; uint32_t key=hal_irq_save();
    done=b->completion;
    if(done) {
        if(done>0 && out && n!=b->nr) done=-1;
        if(done>0 && out) for(i=0;i<n;i++) out[i]=b->rx[i];
        b->completion=0;
    }
    hal_irq_restore(key); return done;
}
void hal_i2c_irq(hal_i2c_t *b)
{
    hal_i2c_regs_t *r=b->regs; uint32_t sr=r->sr1;
    if(b->state==I_IDLE || b->state==I_RECOVER) return;
    if(sr&0x0f00u) { i2c_fault(b,HAL_TIM2->cnt,true); return; }
    switch(b->state) {
    case I_START_W:
        if(sr&1u) { r->dr=(uint32_t)b->addr<<1; b->state=I_ADDR_W; }
        break;
    case I_ADDR_W:
        if(sr&2u) { (void)r->sr2; b->state=I_TX; }
        break;
    case I_TX:
        if(b->pos<b->nt && (sr&HAL_BIT(7))) {
            r->dr=b->tx[b->pos++];
            if(b->pos==b->nt) r->cr2&=~HAL_BIT(10); /* wait BTF, not TXE */
        } else if(b->pos==b->nt && (sr&4u)) {
            if(b->nr) { r->cr1|=HAL_BIT(8); b->state=I_START_R; }
            else { r->cr1|=HAL_BIT(9); b->state=I_STOP; r->cr2=42; }
        }
        break;
    case I_START_R:
        if(sr&1u) { r->dr=((uint32_t)b->addr<<1)|1u; b->state=I_ADDR_R; }
        break;
    case I_ADDR_R:
        if(sr&2u) {
            uint32_t key=hal_irq_save();
            b->pos=0;
            if(b->nr==1) {
                r->cr1&=~HAL_BIT(10); (void)r->sr2; r->cr1|=HAL_BIT(9);
                r->cr2|=HAL_BIT(10); b->state=I_RX;
            } else if(b->nr==2) {
                r->cr1|=HAL_BIT(11); r->cr1&=~HAL_BIT(10);
                (void)r->sr2; b->state=I_LAST;
            } else {
                r->cr1|=HAL_BIT(10); (void)r->sr2;
                if(b->nr>3) r->cr2|=HAL_BIT(10);
                b->state=I_RX;
            }
            hal_irq_restore(key);
        }
        break;
    case I_RX:
        if(b->nr-b->pos==1 && (sr&HAL_BIT(6))) {
            b->rx[b->pos++]=(uint8_t)r->dr; b->state=I_STOP; r->cr2=42;
        } else if(b->nr-b->pos>3 && (sr&HAL_BIT(6))) {
            b->rx[b->pos++]=(uint8_t)r->dr;
            if(b->nr-b->pos==3) r->cr2&=~HAL_BIT(10);
        } else if(b->nr-b->pos==3 && (sr&4u)) {
            r->cr1&=~HAL_BIT(10); b->rx[b->pos++]=(uint8_t)r->dr; b->state=I_LAST;
        }
        break;
    case I_LAST:
        if(sr&4u) {
            uint32_t key=hal_irq_save(); r->cr1|=HAL_BIT(9);
            b->rx[b->pos++]=(uint8_t)r->dr; b->rx[b->pos++]=(uint8_t)r->dr;
            r->cr2=42; b->state=I_STOP; hal_irq_restore(key);
        }
        break;
    default: break;
    }
}
void hal_i2c_poll(hal_i2c_t *b,uint32_t now)
{
    uint32_t key=hal_irq_save();
    if(b->state==I_RECOVER) {
        if(drv_due(now,b->recovery_tick)) {
            unsigned phase=b->recovery;
            if(phase<18) {
                bool high=(phase&1u)!=0;
                hal_gpio_write(HAL_GPIOB,b->scl,high);
                /* Clock stretching/stuck-low never turns into an infinite wait. */
                if(high && !hal_gpio_read(HAL_GPIOB,b->scl)) {
                    if(!drv_due(now,b->recovery_tick+1000u)) { hal_irq_restore(key); return; }
                    b->recovery=21;
                }
            } else if(phase==18) {
                hal_gpio_write(HAL_GPIOB,b->scl,false); hal_gpio_write(HAL_GPIOB,b->sda,false);
            } else if(phase==19) hal_gpio_write(HAL_GPIOB,b->scl,true);
            else if(phase==20) hal_gpio_write(HAL_GPIOB,b->sda,true);
            else {
                hal_gpio_write(HAL_GPIOB,b->sda,true); hal_gpio_write(HAL_GPIOB,b->scl,true);
                hal_gpio_mode(HAL_GPIOB,b->scl,2,4,true,0);
                hal_gpio_mode(HAL_GPIOB,b->sda,2,4,true,0);
                i2c_config(b); b->state=I_IDLE;
            }
            b->recovery++; b->recovery_tick=now+5u;
        }
    } else if(b->state==I_STOP && !(b->regs->cr1&HAL_BIT(9)) && !(b->regs->sr2&2u)) {
        b->state=I_IDLE; b->completion=1;
    } else if(b->state!=I_IDLE && drv_due(now,b->deadline)) i2c_fault(b,now,true);
    hal_irq_restore(key);
}
