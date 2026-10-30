/* GPL-3.0-only. Register-level MAIN STM32F405 implementation of board.h.
 * Candidate firmware, not hardware qualified. See README.md and SOURCES.md.
 * Only TIM2 ISR reads BMI data: queued bytes and tick belong to one edge. */
#include "../board.h"
#include "../../hal/stm32f405.h"
#include "../../hal/hal_logic.h"
#include "../../hal/hal_i2c.h"
#include "../../hal/hal_uart.h"
#include "../../hal/hal_backup.h"
#include "../../hal/hal_trigger.h"
#include "../../drivers/drv_sensors.h"
#include "../../drivers/drv_devices.h"

static board_stats_t stats;
static uint8_t reset_cause,epoch;
static volatile uint32_t wraps;
static bool pi_power,pi_uart,triggers;
static drv_bmi_t bmi;
static drv_baro_t baro;
static drv_temp_t temp;
static drv_flash_t flash;
static drv_radio_t radio;
static hal_i2c_t i2c_baro,i2c_temp;
static hal_uart_t uart_pi,uart_gnss;
typedef drv_bmi_raw_sample_t imu_raw_t;
static imu_raw_t gyro_buf[64],accel_buf[64];
static board_baro_raw_t baro_buf[16];
static board_temp_raw_t temp_buf[8];
static uint32_t pps_buf[8],trig_buf[16];
static hal_ring_t gyro_q,accel_q,baro_q,temp_q,pps_q,trig_q;
static uint32_t gate_start,gate_budget,gate_elapsed;
static bool gate_active;
static uint32_t adc_next,adc_started;
static uint16_t adc_values[3],battery_mv;
static int8_t mcu_temp=INT8_MIN;
static unsigned adc_channel;
static bool adc_active;

static void output(hal_gpio_t *p,unsigned pin,bool on,bool od)
{ hal_gpio_write(p,pin,on); hal_gpio_mode(p,pin,1,0,od,0); }
static bool spi_wait(hal_spi_t *r,uint32_t mask,uint32_t expected,uint32_t start)
{
    while((r->sr&mask)!=expected) {
        if((uint32_t)(HAL_TIM2->cnt-start)>=150u) return false;
    }
    return true;
}
static bool spi_transfer(void *ctx,drv_spi_device_t dev,const uint8_t *tx,uint8_t *rx,size_t n)
{
    hal_spi_t *r=(dev==DRV_ACCEL || dev==DRV_GYRO)?HAL_SPI1:HAL_SPI2;
    hal_gpio_t *port=dev==DRV_FLASH?HAL_GPIOB:HAL_GPIOC;
    unsigned cs=dev==DRV_ACCEL?4u:(dev==DRV_GYRO?5u:(dev==DRV_RADIO?6u:12u));
    unsigned br=dev==DRV_FLASH?0u:(dev==DRV_RADIO?4u:3u);
    uint32_t start=HAL_TIM2->cnt; size_t i; bool ok=true;
    (void)ctx;
    if(!tx || !n || n>260 || (dev!=DRV_FLASH && n>18)) return false;
    /* SPI1 5.25 MHz, local flash SPI2 21 MHz, offboard radio 1.3125 MHz.
     * Device rates are changed only with all chip selects deasserted. */
    r->cr1=0; r->cr1=HAL_BIT(9)|HAL_BIT(8)|HAL_BIT(6)|HAL_BIT(2)|(br<<3);
    hal_gpio_write(port,cs,false);
    for(i=0;i<n;i++) {
        uint8_t value;
        if(!spi_wait(r,2,2,start)) { ok=false; break; }
        *(volatile uint8_t *)&r->dr=tx[i];
        if(!spi_wait(r,1,1,start)) { ok=false; break; }
        value=*(volatile uint8_t *)&r->dr;
        if(rx) rx[i]=value;
    }
    if(ok) ok=spi_wait(r,HAL_BIT(7),0,start);
    hal_gpio_write(port,cs,true);
    if(!ok || (r->sr&(HAL_BIT(5)|HAL_BIT(6)))) {
        volatile uint32_t discard=r->dr; discard=r->sr; (void)discard;
        r->cr1=0;
        if(r==HAL_SPI1) { HAL_RCC(0x24)|=HAL_BIT(12); HAL_RCC(0x24)&=~HAL_BIT(12); }
        else { HAL_RCC(0x20)|=HAL_BIT(14); HAL_RCC(0x20)&=~HAL_BIT(14); }
        stats.spi_errors++; return false;
    }
    return true;
}
static bool radio_busy_pin(void *ctx) { (void)ctx; return hal_gpio_read(HAL_GPIOC,8); }
static void radio_reset_pin(void *ctx,bool asserted)
{ (void)ctx; hal_gpio_write(HAL_GPIOC,7,!asserted); }
static void radio_txen_pin(void *ctx,bool on)
{ (void)ctx; hal_gpio_write(HAL_GPIOD,2,false); hal_gpio_write(HAL_GPIOC,12,on); }

static bool excitation_is_on(void *ctx)
{ (void)ctx; return gate_active && !(HAL_TIM8->sr&HAL_BIT(3)); }
static uint32_t excitation(void *ctx,bool on,uint32_t budget)
{
    uint32_t key=hal_irq_save(); (void)ctx;
    if(!on) {
        if(gate_active) gate_elapsed=(HAL_TIM8->sr&HAL_BIT(3))?gate_budget:HAL_TIM2->cnt-gate_start;
        HAL_TIM8->bdtr=HAL_BIT(10); HAL_TIM8->cr1=0;
        output(HAL_GPIOB,1,true,true); gate_active=false;
    } else if(budget && budget<65534u) {
        /* PB1 AF3 = TIM8_CH3N (DS8626 Table 9). Complementary output starts
         * LOW; compare makes it HIGH. OC inactive-on-match holds that state
         * even after the one-shot counter stops. No ISR/poll is needed to
         * release the PMOS gate. GPIO remains open-drain in AF mode. */
        HAL_TIM8->cr1=0; HAL_TIM8->bdtr=HAL_BIT(10);
        HAL_TIM8->cr2=HAL_BIT(13); HAL_TIM8->ccer=HAL_BIT(10); /* CC3NE */
        HAL_TIM8->psc=167; HAL_TIM8->arr=budget+1u; HAL_TIM8->ccr[2]=budget;
        HAL_TIM8->ccmr2=5u<<4; /* force OC3REF active, N output low */
        HAL_TIM8->egr=1; HAL_TIM8->sr=0; HAL_TIM8->cnt=0;
        hal_gpio_mode(HAL_GPIOB,1,2,3,true,0);
        gate_start=HAL_TIM2->cnt; gate_budget=budget; gate_elapsed=0; gate_active=true;
        HAL_TIM8->ccmr2=2u<<4; /* inactive on compare, NOT repetitive PWM */
        HAL_TIM8->cr1=HAL_BIT(3)|1u; /* OPM */
        /* Start counting while MOE=0/OIS3N=1 still holds the gate off.
         * Enabling output afterwards makes on-time <= budget, even including
         * the register-write interval before CEN. */
        HAL_TIM8->bdtr=HAL_BIT(15)|HAL_BIT(10);
    }
    hal_irq_restore(key); return gate_elapsed;
}

static void adc_poll(uint32_t now)
{
    static const uint8_t channels[3]={10,16,17};
    if(adc_active) {
        if(HAL_REG(0x40012000u)&2u) {
            adc_values[adc_channel]=(uint16_t)HAL_REG(0x4001204cu);
            adc_active=false; adc_channel++;
            if(adc_channel==3) {
                /* DS8626 Tables 70/73: per-die factory ADC calibration.
                 * Invalid/erased values use documented typical constants.
                 * Divider tolerance and absolute temperature need bench QA. */
                hal_adc_convert(adc_values[0],adc_values[1],adc_values[2],
                    *(const uint16_t *)0x1fff7a2au,*(const uint16_t *)0x1fff7a2cu,
                    *(const uint16_t *)0x1fff7a2eu,&battery_mv,&mcu_temp);
                adc_channel=0; adc_next=now+10000u;
            }
        } else if((uint32_t)(now-adc_started)>1000u) {
            adc_active=false; adc_channel=0; battery_mv=0; mcu_temp=INT8_MIN; adc_next=now+10000u;
        }
    }
    if(!adc_active && drv_due(now,adc_next)) {
        HAL_REG(0x40012034u)=channels[adc_channel];
        HAL_REG(0x40012008u)|=HAL_BIT(30); adc_started=now; adc_active=true;
    }
}
void board_init(void)
{
    uint32_t csr=HAL_RCC(0x74); drv_spi_t spi={0,spi_transfer};
    __asm__ volatile("cpsid i":::"memory");
    HAL_RCC(0x30)|=15u;
    (void)HAL_RCC(0x30);
    output(HAL_GPIOC,2,false,false); output(HAL_GPIOB,9,true,false);
    output(HAL_GPIOB,0,false,false); output(HAL_GPIOB,1,true,true);
    output(HAL_GPIOC,4,true,false); output(HAL_GPIOC,5,true,false);
    output(HAL_GPIOC,6,true,false); output(HAL_GPIOB,12,true,false);
    output(HAL_GPIOC,7,true,false); output(HAL_GPIOC,12,false,false);
    output(HAL_GPIOD,2,false,false); output(HAL_GPIOB,5,true,true);
    output(HAL_GPIOC,3,false,false); output(HAL_GPIOA,4,false,false);
    output(HAL_GPIOB,8,false,false);
    if(!hal_clock_init()) {
        hal_gpio_write(HAL_GPIOA,4,true);
        /* No invented fallback baud/timestamps when crystal fails. */
        for(;;) __asm__ volatile("wfi");
    }
    HAL_RCC(0x30)|=HAL_BIT(18)|HAL_BIT(20)|HAL_BIT(21)|HAL_BIT(22);
    HAL_RCC(0x40)|=HAL_BIT(0)|HAL_BIT(1)|HAL_BIT(2)|HAL_BIT(14)|HAL_BIT(18)|HAL_BIT(21)|HAL_BIT(22)|HAL_BIT(28);
    HAL_RCC(0x44)|=HAL_BIT(1)|HAL_BIT(4)|HAL_BIT(8)|HAL_BIT(12);
    HAL_PWR(0)|=HAL_BIT(8); HAL_PWR(4)|=HAL_BIT(9);
    if(!hal_backup_epoch(&HAL_RCC(0),&HAL_PWR(0),&HAL_REG(0x40002850u),&epoch)) {
        /* Refuse to run with a fabricated/repeated boot epoch. */
        hal_gpio_write(HAL_GPIOA,4,true);
        for(;;) __asm__ volatile("wfi");
    }
    if(csr&HAL_BIT(27)) reset_cause|=BOARD_RESET_POWER;
    if(csr&HAL_BIT(26)) reset_cause|=BOARD_RESET_PIN;
    if(csr&(HAL_BIT(29)|HAL_BIT(30))) reset_cause|=BOARD_RESET_WATCHDOG;
    if(csr&HAL_BIT(28)) reset_cause|=BOARD_RESET_SOFTWARE;
    if(csr&HAL_BIT(25)) reset_cause|=BOARD_RESET_BROWNOUT;
    HAL_RCC(0x74)|=HAL_BIT(24);
    hal_ring_init(&gyro_q,gyro_buf,sizeof gyro_buf[0],64);
    hal_ring_init(&accel_q,accel_buf,sizeof accel_buf[0],64);
    hal_ring_init(&baro_q,baro_buf,sizeof baro_buf[0],16);
    hal_ring_init(&temp_q,temp_buf,sizeof temp_buf[0],8);
    hal_ring_init(&pps_q,pps_buf,sizeof pps_buf[0],8);
    hal_ring_init(&trig_q,trig_buf,sizeof trig_buf[0],16);
    hal_gpio_mode(HAL_GPIOA,0,2,1,false,0); hal_gpio_mode(HAL_GPIOA,1,2,1,false,0);
    hal_gpio_mode(HAL_GPIOA,2,2,1,false,0); hal_gpio_mode(HAL_GPIOA,3,2,1,false,0);
    hal_gpio_mode(HAL_GPIOA,5,2,5,false,0); hal_gpio_mode(HAL_GPIOA,6,2,5,false,0);
    hal_gpio_mode(HAL_GPIOA,7,2,5,false,0);
    hal_gpio_mode(HAL_GPIOB,13,2,5,false,0); hal_gpio_mode(HAL_GPIOB,14,2,5,false,0);
    hal_gpio_mode(HAL_GPIOB,15,2,5,false,0);
    hal_gpio_mode(HAL_GPIOB,6,2,4,true,0); hal_gpio_mode(HAL_GPIOB,7,2,4,true,0);
    hal_gpio_mode(HAL_GPIOB,10,2,4,true,0); hal_gpio_mode(HAL_GPIOB,11,2,4,true,0);
    hal_gpio_mode(HAL_GPIOA,8,0,0,false,0);
    hal_gpio_mode(HAL_GPIOC,1,0,0,false,0); hal_gpio_mode(HAL_GPIOC,13,0,0,false,0);
    hal_gpio_mode(HAL_GPIOC,8,0,0,false,0); hal_gpio_mode(HAL_GPIOC,9,0,0,false,0);
    hal_gpio_mode(HAL_GPIOA,9,2,7,false,0); hal_gpio_mode(HAL_GPIOA,10,2,7,false,1);
    hal_gpio_mode(HAL_GPIOC,10,2,7,false,0); hal_gpio_mode(HAL_GPIOC,11,2,7,false,1);
    hal_gpio_mode(HAL_GPIOC,0,3,0,false,0);
    HAL_TIM2->cr1=0; HAL_TIM2->psc=83; HAL_TIM2->arr=UINT32_MAX;
    HAL_TIM2->ccmr1=0x0101; HAL_TIM2->ccmr2=0x0101;
    HAL_TIM2->ccer=0x3111; /* CH4 falling, CH1/2/3 rising */
    HAL_TIM2->egr=1; HAL_TIM2->sr=0; HAL_TIM2->dier=0x1f; HAL_TIM2->cr1=1;
    hal_nvic(28,1);
    HAL_TIM4->psc=83; HAL_TIM4->arr=249; HAL_TIM4->ccr[2]=125;
    HAL_TIM4->ccmr2=6u<<4; HAL_TIM4->egr=1;
    hal_gpio_mode(HAL_GPIOB,8,2,2,false,0);
    hal_i2c_init(&i2c_baro,HAL_I2C1,6,7,100000,&stats.i2c_errors);
    hal_i2c_init(&i2c_temp,HAL_I2C2,10,11,400000,&stats.i2c_errors);
    hal_nvic(31,2); hal_nvic(32,2); hal_nvic(33,2); hal_nvic(34,2);
    hal_uart_init(&uart_pi,true,&stats.pi_rx_overrun);
    hal_uart_init(&uart_gnss,false,&stats.gnss_rx_overrun);
    drv_bmi_init(&bmi,spi,board_tick());
    drv_baro_init(&baro,(drv_i2c_t){&i2c_baro,hal_i2c_start,hal_i2c_result},board_tick());
    drv_temp_init(&temp,(drv_i2c_t){&i2c_temp,hal_i2c_start,hal_i2c_result},
                  (drv_excitation_t){0,excitation,excitation_is_on});
    drv_flash_init(&flash,spi);
    drv_radio_init(&radio,spi,(drv_radio_io_t){0,radio_busy_pin,radio_reset_pin,radio_txen_pin},board_tick());
    HAL_REG(0x40012304u)=HAL_BIT(23)|HAL_BIT(16); /* TSVREFE, ADC /4 */
    HAL_REG(0x4001200cu)=7u|(7u<<18)|(7u<<21);
    HAL_REG(0x4001202cu)=0; HAL_REG(0x40012008u)=1; adc_next=board_tick()+100u;
    /* IWDG nominal 1 s at 32 kHz; LSI tolerance requires bench timing. */
    HAL_REG(0x40003000u)=0x5555; HAL_REG(0x40003004u)=3; HAL_REG(0x40003008u)=999;
    HAL_REG(0x40003000u)=0xcccc; HAL_REG(0x40003000u)=0xaaaa;
    __asm__ volatile("cpsie i":::"memory");
}
void board_poll(void)
{
    board_baro_raw_t br; board_temp_raw_t tr; uint32_t now=board_tick();
    hal_i2c_poll(&i2c_baro,now); hal_i2c_poll(&i2c_temp,now);
    hal_uart_poll(&uart_pi,now); hal_uart_poll(&uart_gnss,now);
    drv_bmi_poll(&bmi,now);
    if(drv_baro_poll(&baro,now,&br) && !hal_ring_push(&baro_q,&br)) stats.baro_dropped++;
    if(drv_temp_poll(&temp,now,&tr) && !hal_ring_push(&temp_q,&tr)) stats.temp_dropped++;
    drv_flash_poll(&flash,now); drv_radio_poll(&radio,now); adc_poll(now);
    stats.radio_errors=radio.errors; stats.flash_errors=flash.errors;
}
void board_watchdog_kick(void) { HAL_REG(0x40003000u)=0xaaaa; }
void board_idle(void)
{
    /* Acquisition bus phases and excitation must advance faster than 1 ms.
     * Sleep only when interrupts/ticks can meet the next driver deadlines. */
    if(i2c_baro.state || i2c_temp.state || temp.state ||
       (radio.transmitting && radio.tx_stage<5)) return;
    __asm__ volatile("wfi":::"memory");
}
void board_reboot(void)
{
    (void)excitation(0,false,0); board_trig_stop(true); board_pi_uart_enable(false); board_pi_power(false);
    __asm__ volatile("dsb":::"memory"); HAL_SCB(0x0c)=0x05fa0004;
    for(;;) __asm__ volatile("wfi");
}
uint8_t board_reset_cause(void) { return reset_cause; }
uint8_t board_epoch(void) { return epoch; }
uint32_t board_tick(void) { return HAL_TIM2->cnt; }
uint64_t board_time_us(void)
{
    uint32_t key=hal_irq_save(),hi=wraps,lo=HAL_TIM2->cnt;
    if((HAL_TIM2->sr&1u) && lo<0x80000000u) hi++;
    hal_irq_restore(key); return (uint64_t)hi<<32|lo;
}
uint16_t board_tick_hi(void) { return (uint16_t)(board_time_us()>>32); }
size_t board_persist_size(void) { return 4096; }
void board_persist_read(void *dst,size_t offset,size_t n)
{
    size_t i; uint8_t *out=dst;
    if(!out || offset>4096 || n>4096-offset) return;
    for(i=0;i<n;i++) out[i]=*(volatile uint8_t *)(uintptr_t)(0x40024000u+offset+i);
}
void board_persist_write(const void *src,size_t offset,size_t n)
{
    size_t i; const uint8_t *in=src;
    if(!in || offset>4096 || n>4096-offset) return;
    for(i=0;i<n;i++) *(volatile uint8_t *)(uintptr_t)(0x40024000u+offset+i)=in[i];
    __asm__ volatile("dsb":::"memory");
}
static bool pop(hal_ring_t *q,void *s)
{ uint32_t key=hal_irq_save(); bool ok=hal_ring_pop(q,s); hal_irq_restore(key); return ok; }
void board_imu_info(board_imu_info_t *i) { drv_bmi_info(&bmi,i); }
bool board_gyro_pop(board_imu_sample_t *s)
{ imu_raw_t r; if(!s || !pop(&gyro_q,&r)) return false; drv_bmi_decode(false,r.raw,r.tick,s); return true; }
bool board_accel_pop(board_imu_sample_t *s)
{ imu_raw_t r; if(!s || !pop(&accel_q,&r)) return false; drv_bmi_decode(true,r.raw,r.tick,s); return true; }
bool board_baro_prom(uint16_t out[8])
{ unsigned i; if(!out || !baro.ready) return false; for(i=0;i<8;i++) out[i]=baro.prom[i]; return true; }
bool board_baro_ok(void) { return baro.ok; }
bool board_baro_pop(board_baro_raw_t *s) { return pop(&baro_q,s); }
bool board_temp_configure(const board_temp_cfg_t *cfg) { return drv_temp_configure(&temp,cfg,board_tick()); }
bool board_temp_ok(void) { return temp.ok; }
bool board_temp_pop(board_temp_raw_t *s) { return pop(&temp_q,s); }
size_t board_gnss_read(uint8_t *b,size_t n) { return hal_uart_read(&uart_gnss,b,n); }
bool board_gnss_write(const uint8_t *b,size_t n) { return hal_uart_write(&uart_gnss,b,n,board_tick()); }
bool board_gnss_tx_idle(void) { return hal_uart_idle(&uart_gnss); }
void board_gnss_set_baud(uint32_t b) { hal_uart_baud(&uart_gnss,b); }
void board_gnss_reset(bool asserted) { hal_gpio_write(HAL_GPIOB,5,!asserted); }
bool board_pps_pop(uint32_t *tick) { return pop(&pps_q,tick); }
void board_pi_power(bool on)
{
    if(!on) { board_trig_stop(true); board_pi_uart_enable(false); }
    hal_gpio_write(HAL_GPIOC,2,on); pi_power=on;
}
void board_pi_uart_enable(bool on)
{
    bool desired=on && pi_power;
    if(pi_uart==desired) return;
    /* Isolate first; never deliver stale boot bytes or resume an old DMA
     * command when a power-cycled Pi is connected again. */
    hal_gpio_write(HAL_GPIOB,9,true);
    hal_uart_discard(&uart_pi,true);
    pi_uart=desired; hal_gpio_write(HAL_GPIOB,9,!pi_uart);
    /* Establish camera idle HIGH while Pi boots, before camera arming.
     * Shutdown/isolation still forces LOW. */
    if(!triggers || !desired) board_trig_stop(!desired);
}
bool board_pi_powered(void) { return pi_power; }
bool board_pgood_5v(void) { return hal_gpio_read(HAL_GPIOC,1); }
bool board_pgood_3v3(void) { return hal_gpio_read(HAL_GPIOC,13); }
size_t board_pi_read(uint8_t *b,size_t n) { return pi_uart?hal_uart_read(&uart_pi,b,n):0; }
bool board_pi_write(const uint8_t *b,size_t n) { return pi_uart && hal_uart_write(&uart_pi,b,n,board_tick()); }
bool board_pi_tx_idle(void) { return hal_uart_idle(&uart_pi); }
bool board_trig_start(uint32_t period,uint16_t pulse,uint32_t *actual_period,uint16_t *actual_pulse)
{
    uint16_t psc,arr,ccr; uint32_t key;
    board_trig_stop(true);
    if(!pi_power || !pi_uart || !hal_trig_values(period,pulse,&psc,&arr,&ccr,actual_period,actual_pulse)) return false;
    key=hal_irq_save();
    hal_trigger_prime(HAL_TIM3,psc,arr,ccr);
    /* Prime PWM2 in the HIGH phase before connecting its output. The next
     * timer tick rolls over to CNT=0 and creates the first falling edge only
     * after TIM2 capture is armed. No unreported initial low pulse. */
    output(HAL_GPIOB,0,true,false);
    hal_gpio_mode(HAL_GPIOB,0,2,2,false,0);
    HAL_TIM2->sr=~(HAL_BIT(4)|HAL_BIT(12)); triggers=true;
    HAL_TIM3->cr1=HAL_BIT(7)|1u; hal_irq_restore(key); return true;
}
void board_trig_stop(bool drive_low)
{
    uint32_t key=hal_irq_save(); triggers=false; HAL_TIM3->cr1=0; HAL_TIM3->ccer=0;
    /* Events contain only a tick; keeping an old run would let app relabel
     * it with the next run's period/pulse and FIRST flag. Stop/reconfigure is
     * therefore an explicit counted discontinuity, including pending HW edges.
     * OF is a lower-bound count: it can represent more than one lost edge. */
    stats.trig_dropped+=hal_ring_discard(&trig_q);
    if(HAL_TIM2->sr&HAL_BIT(4)) stats.trig_dropped++;
    if(HAL_TIM2->sr&HAL_BIT(12)) stats.trig_dropped++;
    output(HAL_GPIOB,0,!drive_low,false);
    HAL_TIM2->sr=~(HAL_BIT(4)|HAL_BIT(12));
    hal_irq_restore(key);
}
bool board_trig_running(void) { return triggers; }
bool board_trig_pop(uint32_t *tick) { return pop(&trig_q,tick); }
bool board_radio_configure(const board_radio_cfg_t *cfg) { return drv_radio_configure(&radio,cfg,board_tick()); }
bool board_radio_ok(void) { return radio.ok; }
bool board_radio_send(const uint8_t *p,size_t n) { return drv_radio_send(&radio,p,n,board_tick()); }
bool board_radio_busy(void) { return radio.transmitting || (radio.configured && !radio.ok); }
uint32_t board_radio_tx_count(void) { return radio.tx_count; }
bool board_flash_ok(void) { return flash.ok; }
uint32_t board_flash_size(void) { return flash.ok?0x1000000u:0; }
bool board_flash_busy(void) { return flash.busy; }
bool board_flash_read(uint32_t a,uint8_t *p,size_t n) { return drv_flash_read(&flash,a,p,n); }
bool board_flash_program(uint32_t a,const uint8_t *p,size_t n) { return drv_flash_program(&flash,a,p,n,board_tick()); }
bool board_flash_erase_sector(uint32_t a) { return drv_flash_erase(&flash,a,board_tick()); }
uint16_t board_battery_mv(void) { return battery_mv; }
int8_t board_mcu_temp_c(void) { return mcu_temp; }
void board_led(int led,bool on)
{ if(led==BOARD_LED_STATUS) hal_gpio_write(HAL_GPIOC,3,on); else if(led==BOARD_LED_FAULT) hal_gpio_write(HAL_GPIOA,4,on); }
void board_buzzer(bool on)
{ HAL_TIM4->ccer=on?HAL_BIT(8):0; HAL_TIM4->cr1=on?1:0; }
const board_stats_t *board_stats(void) { return &stats; }

void TIM2_IRQHandler(void)
{
    uint32_t sr=HAL_TIM2->sr; unsigned ch;
    if(sr&1u) { wraps++; HAL_TIM2->sr=~1u; }
    for(ch=0;ch<4;ch++) if(sr&HAL_BIT(ch+1u)) {
        uint32_t tick=HAL_TIM2->ccr[ch]; /* read clears this channel's CCxIF */
        uint32_t *dropped=ch==0?&stats.pps_dropped:(ch==1?&stats.gyro_dropped:
            (ch==2?&stats.accel_dropped:&stats.trig_dropped));
        if((HAL_TIM2->sr|sr)&HAL_BIT(ch+9u)) {
            /* One or more edges were overwritten. Do NOT read latest BMI data
             * and label it as the older capture. Drop conservatively. */
            (*dropped)++; HAL_TIM2->sr=~HAL_BIT(ch+9u); continue;
        }
        if(ch==1 || ch==2) {
            imu_raw_t sample;
            /* Keep margin to finish before the next DRDY. Reject implausibly
             * late edges even after a peripheral overcapture flag fault. */
            if((uint32_t)(HAL_TIM2->cnt-tick)>=(ch==1?950u:575u)) { (*dropped)++; continue; }
            if(drv_bmi_capture(&bmi,ch==2,tick,false,&sample,dropped)) {
                /* A new DRDY during the burst makes the association ambiguous.
                 * Reject this sample, leaving the new capture for the next ISR.
                 * Per-axis shadowing alone does not guarantee a whole XYZ frame
                 * when a register update falls inside the SPI transaction. */
                if((HAL_TIM2->sr&(HAL_BIT(ch+1u)|HAL_BIT(ch+9u))) ||
                   (uint32_t)(HAL_TIM2->cnt-tick)>=(ch==1?1000u:625u)) {
                    (*dropped)++; continue;
                }
                if(!hal_ring_push(ch==1?&gyro_q:&accel_q,&sample)) (*dropped)++;
            }
        } else if(ch==0) { if(!hal_ring_push(&pps_q,&tick)) (*dropped)++; }
        else if(triggers && !hal_ring_push(&trig_q,&tick)) (*dropped)++;
    }
}
void I2C1_EV_IRQHandler(void) { hal_i2c_irq(&i2c_baro); }
void I2C1_ER_IRQHandler(void) { hal_i2c_irq(&i2c_baro); }
void I2C2_EV_IRQHandler(void) { hal_i2c_irq(&i2c_temp); }
void I2C2_ER_IRQHandler(void) { hal_i2c_irq(&i2c_temp); }
void DMA1_Stream1_IRQHandler(void) { hal_uart_rx_irq(&uart_gnss); }
void DMA2_Stream2_IRQHandler(void) { hal_uart_rx_irq(&uart_pi); }
void USART1_IRQHandler(void) { hal_uart_irq(&uart_pi); }
void USART3_IRQHandler(void) { hal_uart_irq(&uart_gnss); }
void SysTick_Handler(void) { /* periodic wake-up; time remains TIM2-based */ }
