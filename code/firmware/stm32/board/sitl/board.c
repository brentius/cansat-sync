/* GPL-3.0-only. board.h simulation, interfaces section 4. Timings are
 * [assumed] synthetic: NOR program 700 us, erase 45 ms; sensor scheduling
 * 1 kHz gyro/1600 Hz accel; UART 8N1 at stated baud. Not a target HAL. */
#include "board_sitl.h"
#include "cs_ms5611.h"
#include "cs_lora.h"
#include "../../hal/hal_logic.h"
#include <string.h>
#include <math.h>
#define QCAP 128u
#define RXCAP 8192u
#define DEFINE_QUEUE(name,type,drop) \
static type name##_q[QCAP]; static size_t name##_head,name##_count; \
static bool name##_push(type x) { if(name##_count==QCAP) { stats.drop++; return false; } \
name##_q[(name##_head+name##_count)%QCAP]=x; name##_count++; return true; } \
bool board_##name##_pop(type *x) { if(!name##_count) return false; *x=name##_q[name##_head]; \
name##_head=(name##_head+1u)%QCAP; name##_count--; return true; }
static board_stats_t stats;
DEFINE_QUEUE(gyro,board_imu_sample_t,gyro_dropped)
DEFINE_QUEUE(accel,board_imu_sample_t,accel_dropped)
DEFINE_QUEUE(baro,board_baro_raw_t,baro_dropped)
DEFINE_QUEUE(temp,board_temp_raw_t,temp_dropped)
static uint32_t pps_q[QCAP],trig_q[QCAP];
static size_t pps_head,pps_count,trig_head,trig_count;
static uint8_t flash_mem[BOARD_SITL_FLASH_MAX],persist[1024];
static uint32_t flash_size;
static uint64_t now,next_g,next_a,next_b,next_t,next_trigger,flash_until,pi_until,gnss_until,radio_until,last_kick;
static bool initialized,powered,uart,trig,flash_pending,radio_pending,pi_pending,gnss_pending;
static uint8_t epoch,reset_cause=BOARD_RESET_POWER,baro_cycle;
static uint8_t pi_rx[RXCAP],gnss_rx[RXCAP],pi_tx[BOARD_PI_TX_MAX],gnss_tx[512],radio_tx[255];
static size_t pi_head,pi_count,gnss_head,gnss_count,pi_len,gnss_len,radio_len;
static uint32_t radio_count,trig_period;
static uint16_t trig_pulse;
static board_temp_cfg_t temp_cfg;
static board_radio_cfg_t radio_cfg;
static board_sitl_inputs_t inputs;
static board_sitl_stats_t simstats;
static board_sitl_sink_t output;
static void *output_ctx;
static uint32_t poll_reads,poll_ops;
static const uint16_t calibration[8]={0,40127,36924,23317,23282,33464,28312,0};
static void defaults(void)
{
    memset(&inputs,0,sizeof inputs); inputs.gyro=inputs.accel=inputs.baro=inputs.temp=true;
    inputs.gnss=inputs.radio=inputs.flash=inputs.pgood5=inputs.pgood3=true;
    inputs.accel_v[2]= -9.80665f; inputs.pressure_pa=101325.0; inputs.battery_mv=8000;
    inputs.temp_excitation_us=6256;
}
void board_sitl_cold_start(uint32_t size)
{
    flash_size=size>=4096u && size<=sizeof flash_mem && size%4096u==0?size:sizeof flash_mem;
    memset(flash_mem,0xff,sizeof flash_mem); memset(persist,0,sizeof persist);
    memset(&simstats,0,sizeof simstats); defaults(); initialized=true; epoch=0; reset_cause=BOARD_RESET_POWER;
    output=NULL; output_ctx=NULL; flash_until=0; flash_pending=false;
}
void board_sitl_warm_reset(uint8_t cause)
{
    /* NOR continues its timed operation while MCU restarts. Preserve memory
     * and remaining busy duration; committed byte contents model acceptance. */
    flash_until=flash_until>now?flash_until-now:0; reset_cause=cause;
}
void board_init(void)
{
    if(!initialized) board_sitl_cold_start(BOARD_SITL_FLASH_MAX);
    epoch++; now=0; next_g=1000; next_a=625; next_b=10000; next_t=0; next_trigger=0;
    gyro_head=gyro_count=accel_head=accel_count=baro_head=baro_count=temp_head=temp_count=0;
    pps_head=pps_count=trig_head=trig_count=pi_head=pi_count=gnss_head=gnss_count=0;
    memset(&stats,0,sizeof stats); powered=uart=trig=radio_pending=pi_pending=gnss_pending=false;
    radio_count=0; baro_cycle=0; temp_cfg.period_us=0; last_kick=0;
    simstats.uart_enabled=false; simstats.trigger_low=true; simstats.buzzer=false;
    pi_until=gnss_until=radio_until=0;
}
uint8_t board_reset_cause(void) { return reset_cause; }
uint32_t board_tick(void) { return (uint32_t)now; }
uint64_t board_time_us(void) { return now; }
uint16_t board_tick_hi(void) { return (uint16_t)(now>>32); }
uint8_t board_epoch(void) { return epoch; }
size_t board_persist_size(void) { return sizeof persist; }
void board_persist_read(void *dst,size_t offset,size_t n)
{ if(offset<=sizeof persist && n<=sizeof persist-offset) memcpy(dst,persist+offset,n); }
void board_persist_write(const void *src,size_t offset,size_t n)
{ if(offset<=sizeof persist && n<=sizeof persist-offset) memcpy(persist+offset,src,n); }
void board_sitl_corrupt_persist(size_t off,uint8_t value) { if(off<sizeof persist) persist[off]=value; }
void board_watchdog_kick(void)
{
    uint64_t gap=now-last_kick; if(gap>simstats.watchdog_max_gap_us) simstats.watchdog_max_gap_us=gap;
    last_kick=now; simstats.watchdog_kicks++;
}
void board_sitl_advance(uint32_t us) { now+=us; }
void board_sitl_set_time(uint64_t us)
{ now=us; next_g=us+1000u; next_a=us+625u; next_b=us+10000u; next_t=us+temp_cfg.period_us; last_kick=us; }
void board_idle(void) { board_sitl_advance(500); }
void board_reboot(void) { board_sitl_warm_reset(BOARD_RESET_SOFTWARE); board_init(); }
board_sitl_inputs_t *board_sitl_inputs(void) { return &inputs; }
const board_sitl_stats_t *board_sitl_stats(void) { return &simstats; }
const board_stats_t *board_stats(void) { return &stats; }
void board_sitl_set_sink(board_sitl_sink_t sink,void *ctx) { output=sink; output_ctx=ctx; }
static void emit(int channel,const uint8_t *p,size_t n) { if(output) output(output_ctx,channel,p,n); }
void board_poll(void)
{
    if(poll_reads>simstats.max_reads_per_poll) simstats.max_reads_per_poll=poll_reads;
    if(poll_ops>simstats.max_flash_ops_per_poll) simstats.max_flash_ops_per_poll=poll_ops;
    poll_reads=poll_ops=0;
    if(pi_pending && now>=pi_until) { emit(BOARD_SITL_LINK,pi_tx,pi_len); pi_pending=false; }
    if(gnss_pending && now>=gnss_until) { emit(BOARD_SITL_GNSS_TX,gnss_tx,gnss_len); gnss_pending=false; }
    if(radio_pending && now>=radio_until) { emit(BOARD_SITL_RADIO,radio_tx,radio_len); radio_count++; radio_pending=false; }
    if(flash_pending && now>=flash_until) flash_pending=false;
    while(next_g<=now) {
        if(inputs.gyro) { board_imu_sample_t s={(uint32_t)next_g,{0,0,0},inputs.gyro_saturated?BOARD_IMU_SATURATED:0};
            memcpy(s.v,inputs.gyro_v,sizeof s.v); (void)gyro_push(s); }
        next_g+=1000;
    }
    while(next_a<=now) {
        if(inputs.accel) { board_imu_sample_t s={(uint32_t)next_a,{0,0,0},inputs.accel_saturated?BOARD_IMU_SATURATED:0};
            memcpy(s.v,inputs.accel_v,sizeof s.v); (void)accel_push(s); }
        next_a+=625;
    }
    while(next_b<=now) {
        if(inputs.baro) {
            bool temperature=(baro_cycle++%10u)==0;
            /* Datasheet linear inverse at fixed D2, for a synthetic pressure.
             * C compensation still runs normally in the application. */
            double dt=8569150.0-33464.0*256.0;
            double off=36924.0*65536.0+23282.0*dt/128.0;
            double sens=40127.0*32768.0+23317.0*dt/256.0;
            double d1=(inputs.pressure_pa*32768.0+off)*2097152.0/sens;
            board_baro_raw_t r={(uint32_t)next_b,temperature?8569150u:(uint32_t)(d1+.5),temperature}; (void)baro_push(r);
        }
        next_b+=10000;
    }
    while(temp_cfg.period_us && next_t<=now) {
        if(inputs.temp) { board_temp_raw_t r={(uint32_t)(next_t-2000u),inputs.temp_open?32767:10560,26400,2048,4096,inputs.temp_excitation_us}; (void)temp_push(r); }
        next_t+=temp_cfg.period_us;
    }
    while(trig && next_trigger<=now) {
        if(trig_count<QCAP) { trig_q[(trig_head+trig_count)%QCAP]=(uint32_t)next_trigger; trig_count++; }
        else stats.trig_dropped++;
        next_trigger+=trig_period;
    }
}
void board_imu_info(board_imu_info_t *i)
{ board_imu_info_t x={inputs.gyro,inputs.accel,1000,1600,34.906585f,235.3596f}; *i=x; }
bool board_baro_prom(uint16_t p[8])
{ if(!inputs.baro) return false; memcpy(p,calibration,sizeof calibration); p[7]=cs_ms5611_crc4(p); return true; }
bool board_baro_ok(void) { return inputs.baro; }
bool board_temp_configure(const board_temp_cfg_t *cfg)
{
    if(!cfg || (cfg->period_us && (cfg->sps!=860 || cfg->fs_mv_ain0!=2048 || cfg->fs_mv_ain1!=4096))) return false;
    temp_cfg=*cfg; next_t=now+cfg->period_us; return true;
}
bool board_temp_ok(void) { return inputs.temp; }
static bool rx_push(uint8_t *buf,size_t head,size_t *count,const uint8_t *p,size_t n)
{
    if(n>RXCAP-*count) return false;
    for(size_t i=0;i<n;i++) buf[(head+*count+i)%RXCAP]=p[i];
    *count+=n; return true;
}
static size_t rx_read(uint8_t *ring,size_t *head,size_t *count,uint8_t *p,size_t n)
{
    if(n>*count) n=*count;
    for(size_t i=0;i<n;i++) p[i]=ring[(*head+i)%RXCAP];
    *head=(*head+n)%RXCAP; *count-=n; return n;
}
bool board_sitl_pi_receive(const uint8_t *p,size_t n)
{ if(!powered || !uart) return false; bool ok=rx_push(pi_rx,pi_head,&pi_count,p,n); if(!ok) stats.pi_rx_overrun++; return ok; }
bool board_sitl_gnss_receive(const uint8_t *p,size_t n)
{ if(!inputs.gnss) return false; bool ok=rx_push(gnss_rx,gnss_head,&gnss_count,p,n); if(!ok) stats.gnss_rx_overrun++; return ok; }
size_t board_pi_read(uint8_t *p,size_t n) { return rx_read(pi_rx,&pi_head,&pi_count,p,n); }
size_t board_gnss_read(uint8_t *p,size_t n) { return rx_read(gnss_rx,&gnss_head,&gnss_count,p,n); }
bool board_pi_tx_idle(void) { return !pi_pending; }
bool board_pi_write(const uint8_t *p,size_t n)
{
    if(pi_pending || n>sizeof pi_tx || !uart || !powered) return false;
    memcpy(pi_tx,p,n); pi_len=n; pi_until=now+(n*10000000u+230399u)/230400u; pi_pending=true; return true;
}
bool board_gnss_tx_idle(void) { return !gnss_pending; }
bool board_gnss_write(const uint8_t *p,size_t n)
{
    if(gnss_pending || n>sizeof gnss_tx || !inputs.gnss || !simstats.baud) return false;
    memcpy(gnss_tx,p,n); gnss_len=n; gnss_until=now+(n*10000000u+simstats.baud-1u)/simstats.baud;
    gnss_pending=true; simstats.gnss_writes++; return true;
}
void board_gnss_set_baud(uint32_t baud) { simstats.baud=baud; }
void board_gnss_reset(bool asserted) { (void)asserted; }
void board_pi_power(bool on)
{
    if(on!=powered && uart) simstats.power_order_errors++;
    if(powered && !on) { pi_pending=false; pi_count=0; }
    powered=on;
}
void board_pi_uart_enable(bool on)
{ if(on && !powered) simstats.power_order_errors++; uart=on; simstats.uart_enabled=on; }
bool board_pi_powered(void) { return powered; }
bool board_pgood_5v(void) { return inputs.pgood5; }
bool board_pgood_3v3(void) { return inputs.pgood3; }
bool board_sitl_pps(uint32_t tick)
{ if(pps_count==QCAP) { stats.pps_dropped++; return false; } pps_q[(pps_head+pps_count)%QCAP]=tick; pps_count++; return true; }
bool board_pps_pop(uint32_t *tick)
{ if(!pps_count) return false; *tick=pps_q[pps_head]; pps_head=(pps_head+1u)%QCAP; pps_count--; return true; }
bool board_trig_pop(uint32_t *tick)
{ if(!trig_count) return false; *tick=trig_q[trig_head]; trig_head=(trig_head+1u)%QCAP; trig_count--; return true; }
bool board_trig_start(uint32_t period,uint16_t pulse,uint32_t *actual_period,uint16_t *actual_pulse)
{
    uint16_t psc,arr,ccr;
    board_trig_stop(true); /* discard/count captures belonging to the old run */
    if(!powered || !uart || !hal_trig_values(period,pulse,&psc,&arr,&ccr,actual_period,actual_pulse)) return false;
    (void)arr; (void)ccr;
    trig_period=*actual_period; trig_pulse=*actual_pulse;
    /* Target primes CNT=ARR in PWM2's HIGH phase, arms capture, then CEN.
     * Its first falling edge is one configured timer quantum later. */
    next_trigger=now+((uint32_t)psc+1u)/84u;
    trig=true; simstats.trigger_low=false; simstats.trigger_starts++; return true;
}
void board_trig_stop(bool low)
{
    trig=false; simstats.trigger_low=low;
    stats.trig_dropped+=(uint32_t)trig_count; trig_head=trig_count=0;
}
bool board_trig_running(void) { return trig; }
bool board_radio_configure(const board_radio_cfg_t *cfg) { radio_cfg=*cfg; return inputs.radio; }
bool board_radio_ok(void) { return inputs.radio; }
bool board_radio_busy(void) { return radio_pending; }
bool board_radio_send(const uint8_t *p,size_t n)
{
    if(!inputs.radio || radio_pending || n>sizeof radio_tx) return false;
    cs_lora_config_t c={radio_cfg.sf,radio_cfg.cr,radio_cfg.preamble,(double)radio_cfg.bw_hz,radio_cfg.crc_on,radio_cfg.implicit_header,radio_cfg.ldro?1:0};
    double air=cs_lora_time_on_air(n,&c); if(!isfinite(air)) return false;
    memcpy(radio_tx,p,n); radio_len=n; radio_until=now+(uint64_t)(air*1000000.0); radio_pending=true; return true;
}
uint32_t board_radio_tx_count(void) { return radio_count; }
bool board_flash_ok(void) { return inputs.flash; }
uint32_t board_flash_size(void) { return flash_size; }
bool board_flash_busy(void) { return flash_pending && now<flash_until; }
bool board_flash_read(uint32_t addr,uint8_t *p,size_t n)
{
    simstats.flash_reads++; poll_reads++;
    if(!inputs.flash || board_flash_busy() || n>256u || addr>flash_size || n>flash_size-addr) return false;
    memcpy(p,flash_mem+addr,n);
    /* [assumed] 21 MHz SPI + transaction overhead, capped below 200 us/page.
     * Charge CPU/transfer time so omitting idle during mount still advances
     * simulated peripherals. This is not measured STM32 timing. */
    now+=20u+(uint64_t)(n+4u)*8u/21u; return true;
}
bool board_flash_program(uint32_t addr,const uint8_t *p,size_t n)
{
    if(!inputs.flash || board_flash_busy() || !n || n>256u || addr>=flash_size || n>flash_size-addr || addr/256u!=(addr+(uint32_t)n-1u)/256u) return false;
    for(size_t i=0;i<n;i++) if((flash_mem[addr+i]&p[i])!=p[i]) { simstats.flash_rule_errors++; return false; }
    for(size_t i=0;i<n;i++) flash_mem[addr+i]&=p[i];
    flash_until=now+700u; flash_pending=true; simstats.flash_programs++; poll_ops++; return true;
}
bool board_flash_erase_sector(uint32_t addr)
{
    if(!inputs.flash || board_flash_busy() || addr%4096u || addr>=flash_size) return false;
    memset(flash_mem+addr,0xff,4096); flash_until=now+45000u; flash_pending=true; simstats.flash_erases++; poll_ops++; return true;
}
const uint8_t *board_sitl_flash_data(void) { return flash_mem; }
uint16_t board_battery_mv(void) { return inputs.battery_mv; }
int8_t board_mcu_temp_c(void) { return 30; }
void board_led(int led,bool on) { if(led>=0 && led<2) simstats.leds[led]=on; }
void board_buzzer(bool on) { simstats.buzzer=on; }
