/* GPL-3.0-only. Fake buses exercise real acquisition state machines.
 * CS_SOURCES: drivers/drv_sensors.c */
#include "cs_test.h"
#include "../drivers/drv_sensors.h"
#include "../app/config.h"
typedef struct {
    uint32_t now, on_at, limit, elapsed;
    uint8_t regs[2][128], reply[16], command, pointer;
    unsigned calls, reads, convs, failures;
    size_t nr;
    bool pending, on, fail, ready;
} fake_t;
static bool spi(void *ctx,drv_spi_device_t dev,const uint8_t *tx,uint8_t *rx,size_t n)
{
    fake_t *f=ctx; size_t i,skip=dev==DRV_ACCEL?2u:1u; unsigned a=dev==DRV_ACCEL?0u:1u;
    f->calls++; if(f->fail) return false;
    if(rx) memset(rx,0,n);
    if(tx[0]&128u) {
        if(rx) for(i=skip;i<n;i++) {
            unsigned reg=(unsigned)(tx[0]&127u)+(unsigned)(i-skip);
            rx[i]=f->regs[a][reg];
            /* BMI088 section 5.5.6: gyro BW bit 7 always reads as one. */
            if(dev==DRV_GYRO && reg==0x10) rx[i]|=0x80;
        }
    }
    else if(n==2) f->regs[a][tx[0]]=tx[1];
    return true;
}
static bool start(void *ctx,uint8_t addr,const uint8_t *tx,size_t nt,size_t nr)
{
    fake_t *f=ctx; if(f->pending) return false;
    f->pending=true; f->nr=nr; f->calls++; f->command=tx[0];
    memset(f->reply,0,sizeof f->reply);
    if(addr==0x76) {
        if((tx[0]&0xf0u)==0xa0u) { f->reply[0]=0x12; f->reply[1]=(uint8_t)(0x30+((tx[0]-0xa0)/2)); }
        if(tx[0]==0 && nr==3) { f->reply[0]=0x65; f->reply[1]=0x43; f->reply[2]=0x21; f->reads++; }
        if((tx[0]&0xe0u)==0x40u) f->convs++;
    } else {
        if(nt==3) { f->pointer=(uint8_t)((tx[1]>>4)&7u); }
        if(tx[0]==1 && nr==2) f->reply[0]=f->ready?0x80:0;
        if(tx[0]==0 && nr==2) { f->reply[0]=f->pointer==4?0x12:0x56; f->reply[1]=0x34; }
    }
    return true;
}
static int result(void *ctx,uint8_t *rx,size_t nr)
{
    fake_t *f=ctx; if(!f->pending) return 0;
    f->pending=false; if(f->fail) { f->fail=false; f->failures++; return -1; }
    if(nr!=f->nr) return -1;
    if(rx) memcpy(rx,f->reply,nr);
    return 1;
}
static uint32_t gate(void *ctx,bool on,uint32_t budget)
{
    fake_t *f=ctx;
    if(on) { f->on=true; f->on_at=f->now; f->limit=budget; f->elapsed=0; }
    else if(f->on) { f->elapsed=f->now-f->on_at; f->on=false; }
    return f->elapsed;
}
static bool is_on(void *ctx) { return ((fake_t *)ctx)->on; }
CS_TEST(bmi_init_and_decode)
{
    fake_t f={0}; drv_bmi_t d; board_imu_info_t info; board_imu_sample_t sample; uint8_t raw[6];
    f.regs[0][0]=0x1e; f.regs[1][0]=0x0f;
    drv_bmi_init(&d,(drv_spi_t){&f,spi},0);
    drv_bmi_info(&d,&info); CS_CHECK(!info.gyro_ok && !info.accel_ok);
    for(f.now=0;f.now<300000;f.now+=1000) drv_bmi_poll(&d,f.now);
    drv_bmi_info(&d,&info); CS_CHECK(info.gyro_ok && info.accel_ok);
    CS_CHECK_EQ(f.regs[0][0x40],0xac); CS_CHECK_EQ(f.regs[1][0x10],2);
    f.regs[0][0x12]=0xff; f.regs[0][0x13]=0x7f;
    f.regs[0][0x14]=0; f.regs[0][0x15]=0x80;
    CS_CHECK(drv_bmi_raw(&d,true,raw)); drv_bmi_decode(true,raw,42,&sample);
    CS_CHECK_NEAR(sample.v[0],24.0*9.80665,0.01); CS_CHECK_NEAR(sample.v[1],-24.0*9.80665,0.01);
    CS_CHECK(sample.flags&BOARD_IMU_SATURATED); CS_CHECK_EQ(sample.tick,42);
    f.fail=true; CS_CHECK(!drv_bmi_raw(&d,false,raw)); CS_CHECK(d.errors>0);
}
CS_TEST(bmi_wrong_id_is_fault)
{
    fake_t f={0}; drv_bmi_t d; drv_bmi_init(&d,(drv_spi_t){&f,spi},UINT32_MAX-10000u);
    for(f.now=0;f.now<200000;f.now+=1000) drv_bmi_poll(&d,f.now);
    CS_CHECK(!d.accel_ok && !d.gyro_ok); CS_CHECK(d.errors>0);
}
CS_TEST(baro_prom_and_interleaving)
{
    fake_t f={0}; drv_baro_t d; board_baro_raw_t s; unsigned p=0,t=0;
    drv_baro_init(&d,(drv_i2c_t){&f,start,result},0);
    for(f.now=0;f.now<300000;f.now+=100) if(drv_baro_poll(&d,f.now,&s)) {
        CS_CHECK_EQ(s.adc,0x654321); CS_CHECK((uint32_t)(f.now-s.tick)>=4500);
        if(s.is_temperature) t++; else p++;
    }
    CS_CHECK(d.ok && d.ready); CS_CHECK_EQ(d.prom[7],0x1237); CS_CHECK(p>=15 && t>=2);
    f.fail=true;
    for(;f.now<500000;f.now+=100) (void)drv_baro_poll(&d,f.now,&s);
    CS_CHECK(d.errors>0); CS_CHECK(d.ok);
}
CS_TEST(ads_registers_duty_and_faults)
{
    fake_t f={0}; drv_temp_t d; board_temp_raw_t s; uint16_t word; unsigned count=0;
    board_temp_cfg_t cfg={100000,1500,860,2048,4096};
    CS_CHECK(drv_ads_config_word(2048,860,0,&word)); CS_CHECK_EQ(word,0xc5e3);
    CS_CHECK(!drv_ads_config_word(2000,860,0,&word)); CS_CHECK(!drv_ads_config_word(2048,861,0,&word));
    drv_temp_init(&d,(drv_i2c_t){&f,start,result},(drv_excitation_t){&f,gate,is_on});
    cfg.sps=128; CS_CHECK(!drv_temp_configure(&d,&cfg,0)); cfg.sps=860;
    cfg.sps=8; cfg.period_us=10000000;
    CS_CHECK(!drv_temp_configure(&d,&cfg,0)); /* 16-bit hard cutoff cannot hold this budget */
    cfg.sps=860; cfg.period_us=100000;
    CS_CHECK(drv_temp_configure(&d,&cfg,0)); f.ready=true;
    for(f.now=0;f.now<300000;f.now+=25) if(drv_temp_poll(&d,f.now,&s)) {
        count++; CS_CHECK_EQ(s.code_ain0,0x1234); CS_CHECK_EQ(s.code_ain1,0x5634);
        CS_CHECK(s.excitation_us<=6000); CS_CHECK(!f.on);
    }
    CS_CHECK_EQ(count,3); CS_CHECK(d.ok);
    f.ready=false;
    for(;f.now<410000;f.now+=25) (void)drv_temp_poll(&d,f.now,&s);
    CS_CHECK(!f.on); CS_CHECK(d.errors>0);
    cfg.period_us=0; CS_CHECK(drv_temp_configure(&d,&cfg,f.now)); CS_CHECK(!f.on);
}
CS_TEST(baro_nominal_conversion_midpoint)
{
    fake_t f={0}; drv_baro_t d; board_baro_raw_t out;
    drv_baro_init(&d,(drv_i2c_t){&f,start,result},0);
    /* Model completion of the OSR4096 command at 12345 us. TE specifies
     * 8.22 ms typical conversion, so midpoint is 16455 us, not max/2. */
    d.state=5; d.deadline=0; f.pending=true; f.nr=0;
    CS_CHECK(!drv_baro_poll(&d,12345,&out));
    CS_CHECK_EQ(d.conversion_tick,16455);
}
CS_TEST(default_app_temperature_contract)
{
    fake_t f={0}; drv_temp_t d; board_temp_raw_t out;
    board_temp_cfg_t cfg={APP_TEMP_PERIOD_US,APP_TEMP_SETTLE_US,APP_TEMP_SPS,2048,4096};
    unsigned count=0;
    drv_temp_init(&d,(drv_i2c_t){&f,start,result},(drv_excitation_t){&f,gate,is_on});
    CS_CHECK(drv_temp_configure(&d,&cfg,0)); CS_CHECK_EQ(d.budget,6256);
    CS_CHECK((uint64_t)d.budget*100<=(uint64_t)cfg.period_us*6);
    CS_CHECK_EQ(cfg.settle_us,2300); CS_CHECK_EQ(cfg.period_us,125000);
    f.ready=true;
    for(f.now=0;f.now<1000000;f.now+=25) if(drv_temp_poll(&d,f.now,&out)) {
        count++; CS_CHECK(out.excitation_us<=d.budget);
    }
    CS_CHECK_EQ(count,8);
    /* Main-loop stall: fake the independent hardware compare releasing gate. */
    (void)drv_temp_poll(&d,f.now,&out); CS_CHECK(f.on);
    f.elapsed=d.budget; f.on=false; f.now+=20000;
    CS_CHECK(!drv_temp_poll(&d,f.now,&out)); CS_CHECK(!d.ok);
    CS_CHECK(!f.on); CS_CHECK(d.errors>0);
    /* A subsequent scheduled sample recovers; no stuck discard state. */
    for(;f.now<1200000;f.now+=25) if(drv_temp_poll(&d,f.now,&out)) count++;
    CS_CHECK(count>8);
}
CS_TEST(captured_bytes_survive_delayed_drain)
{
    fake_t f={0}; drv_bmi_t d; drv_bmi_raw_sample_t a,b; uint32_t dropped=0; unsigned calls;
    drv_bmi_init(&d,(drv_spi_t){&f,spi},0); d.accel_ok=true; d.gyro_ok=true;
    f.regs[1][2]=11;
    CS_CHECK(drv_bmi_capture(&d,false,1000,false,&a,&dropped));
    f.regs[1][2]=22;
    CS_CHECK(drv_bmi_capture(&d,false,2000,false,&b,&dropped));
    CS_CHECK_EQ(a.tick,1000); CS_CHECK_EQ(a.raw[0],11);
    CS_CHECK_EQ(b.tick,2000); CS_CHECK_EQ(b.raw[0],22);
    calls=f.calls;
    CS_CHECK(!drv_bmi_capture(&d,false,3000,true,&b,&dropped));
    CS_CHECK_EQ(f.calls,calls); CS_CHECK_EQ(dropped,1);
    f.fail=true; CS_CHECK(!drv_bmi_capture(&d,true,4000,false,&b,&dropped));
    CS_CHECK_EQ(dropped,2);
}
CS_MAIN(bmi_init_and_decode,bmi_wrong_id_is_fault,baro_prom_and_interleaving,ads_registers_duty_and_faults,
        default_app_temperature_contract,captured_bytes_survive_delayed_drain,baro_nominal_conversion_midpoint)
