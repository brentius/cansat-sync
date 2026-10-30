/* GPL-3.0-only. Datasheet-based acquisition state machines; see SOURCES.md. */
#include "drv_sensors.h"

static bool bmi_read(drv_bmi_t *d,bool accel,uint8_t reg,uint8_t *out,size_t n)
{
    uint8_t tx[8]={0},rx[8]={0}; size_t i,skip=accel?2u:1u;
    if(n+skip>sizeof tx) return false;
    tx[0]=(uint8_t)(reg|0x80u);
    if(!d->bus.transfer(d->bus.ctx,accel?DRV_ACCEL:DRV_GYRO,tx,rx,n+skip)) return false;
    for(i=0;i<n;i++) out[i]=rx[i+skip];
    return true;
}
static bool bmi_write(drv_bmi_t *d,bool accel,uint8_t reg,uint8_t value)
{
    uint8_t tx[2]={reg,value};
    return d->bus.transfer(d->bus.ctx,accel?DRV_ACCEL:DRV_GYRO,tx,0,2);
}
typedef struct { bool accel; uint8_t reg,value; uint32_t wait; } bmi_setting_t;
static const bmi_setting_t bmi_settings[]={
    {true,0x7c,0x00,5000}, {true,0x7d,0x04,50000},
    {true,0x40,0xac,1000}, {true,0x41,0x03,1000},
    {true,0x53,0x0a,1000}, {true,0x58,0x04,1000},
    {false,0x0f,0x00,1000}, {false,0x10,0x02,1000},
    {false,0x11,0x00,30000}, {false,0x15,0x80,1000},
    {false,0x16,0x01,1000}, {false,0x18,0x01,1000}
};
#define BMI_SETTINGS (sizeof bmi_settings/sizeof bmi_settings[0])
void drv_bmi_init(drv_bmi_t *d,drv_spi_t bus,uint32_t now)
{ *d=(drv_bmi_t){0}; d->bus=bus; d->deadline=now+10000u; }
void drv_bmi_poll(drv_bmi_t *d,uint32_t now)
{
    bool ok=true; uint8_t v=0;
    if(d->gyro_ok && d->accel_ok) return;
    if(!drv_due(now,d->deadline)) return;
    if(d->step==0) ok=bmi_read(d,true,0,&v,1); /* first CS selects accel SPI */
    else if(d->step==1) ok=bmi_write(d,true,0x7e,0xb6);
    else if(d->step==2) ok=bmi_write(d,false,0x14,0xb6);
    else if(d->step==3) ok=bmi_read(d,true,0,&v,1); /* reset restores I2C */
    else if(d->step==4) ok=bmi_read(d,true,0,&v,1) && v==0x1e;
    else if(d->step==5) ok=bmi_read(d,false,0,&v,1) && v==0x0f;
    else if(d->step<6u+BMI_SETTINGS) {
        const bmi_setting_t *s=&bmi_settings[d->step-6u];
        ok=bmi_write(d,s->accel,s->reg,s->value); d->deadline=now+s->wait;
    } else if(d->step<6u+2u*BMI_SETTINGS) {
        const bmi_setting_t *s=&bmi_settings[d->step-6u-BMI_SETTINGS];
        /* Bosch 5.5.6: GYRO_BANDWIDTH bit 7 is read-only one. */
        uint8_t mask=(!s->accel && s->reg==0x10)?0x7f:0xff;
        ok=bmi_read(d,s->accel,s->reg,&v,1) && (v&mask)==s->value;
    } else { d->gyro_ok=true; d->accel_ok=true; return; }
    if(!ok) {
        d->errors++; d->step=0; d->deadline=now+1000000u;
    } else {
        if(d->step<6u || d->step>=6u+BMI_SETTINGS)
            d->deadline=now+(d->step==2?30000u:2000u);
        d->step++;
    }
}
bool drv_bmi_raw(drv_bmi_t *d,bool accel,uint8_t raw[6])
{
    if(!raw || !(accel?d->accel_ok:d->gyro_ok)) return false;
    if(!bmi_read(d,accel,accel?0x12:0x02,raw,6)) { d->errors++; return false; }
    return true;
}
bool drv_bmi_capture(drv_bmi_t *d,bool accel,uint32_t tick,bool overcapture,
                     drv_bmi_raw_sample_t *out,uint32_t *dropped)
{
    if(overcapture) { (*dropped)++; return false; }
    if(!(accel?d->accel_ok:d->gyro_ok)) return false;
    out->tick=tick;
    if(!drv_bmi_raw(d,accel,out->raw)) { (*dropped)++; return false; }
    return true;
}
void drv_bmi_decode(bool accel,const uint8_t raw[6],uint32_t tick,board_imu_sample_t *s)
{
    unsigned i; float full=accel?24.0f*9.80665f:2000.0f*0.017453292519943295f;
    s->tick=tick; s->flags=0;
    for(i=0;i<3;i++) {
        int32_t v=(int32_t)((uint16_t)raw[2*i]|(uint16_t)((uint16_t)raw[2*i+1]<<8));
        if(v>=32768) v-=65536;
        s->v[i]=(float)v*(full/32768.0f);
        if(v>=32760 || v<=-32760) s->flags|=BOARD_IMU_SATURATED;
    }
}
void drv_bmi_info(const drv_bmi_t *d,board_imu_info_t *i)
{
    if(!i) return;
    *i=(board_imu_info_t){d->gyro_ok,d->accel_ok,1000.0f,1600.0f,
        2000.0f*0.017453292519943295f,24.0f*9.80665f};
}

void drv_baro_init(drv_baro_t *d,drv_i2c_t bus,uint32_t now)
{ *d=(drv_baro_t){0}; d->bus=bus; d->deadline=now+5000u; }
static void baro_fault(drv_baro_t *d,uint32_t now)
{ d->errors++; d->ok=false; d->ready=false; d->index=0; d->state=0; d->deadline=now+100000u; }
bool drv_baro_poll(drv_baro_t *d,uint32_t now,board_baro_raw_t *out)
{
    uint8_t tx=0,rx[3]; int result;
    if(!drv_due(now,d->deadline)) return false;
    switch(d->state) {
    case 0:
        tx=0x1e;
        if(d->bus.start(d->bus.ctx,0x76,&tx,1,0)) d->state=1;
        break;
    case 1:
        result=d->bus.result(d->bus.ctx,0,0);
        if(result<0) baro_fault(d,now);
        else if(result>0) { d->state=2; d->deadline=now+3000u; }
        break;
    case 2:
        tx=(uint8_t)(0xa0u+2u*d->index);
        if(d->bus.start(d->bus.ctx,0x76,&tx,1,2)) d->state=3;
        break;
    case 3:
        result=d->bus.result(d->bus.ctx,rx,2);
        if(result<0) baro_fault(d,now);
        else if(result>0) {
            d->prom[d->index++]=(uint16_t)((uint16_t)rx[0]<<8|rx[1]);
            if(d->index==8) { d->ready=true; d->ok=true; d->temperature=true; d->state=4; }
            else d->state=2;
        }
        break;
    case 4:
        tx=d->temperature?0x58:0x48;
        if(d->bus.start(d->bus.ctx,0x76,&tx,1,0)) d->state=5;
        break;
    case 5:
        result=d->bus.result(d->bus.ctx,0,0);
        if(result<0) baro_fault(d,now);
        else if(result>0) {
            /* OSR4096 maximum conversion 9.04 ms, conservative 10 ms wait.
             * Mid-time uses the typical 8.22 ms, not the maximum/poll delay.
             * Actual conversion length and command-completion poll latency
             * remain timestamp uncertainty until measured on hardware. */
            d->conversion_tick=now+4110u; d->deadline=now+10000u; d->state=6;
        }
        break;
    case 6:
        if(d->bus.start(d->bus.ctx,0x76,&tx,1,3)) d->state=7;
        break;
    case 7:
        result=d->bus.result(d->bus.ctx,rx,3);
        if(result<0) baro_fault(d,now);
        else if(result>0) {
            uint32_t raw=(uint32_t)rx[0]<<16|(uint32_t)rx[1]<<8|rx[2];
            if(raw==0 || raw==0xffffffu) { baro_fault(d,now); break; }
            *out=(board_baro_raw_t){d->conversion_tick,raw,d->temperature};
            if(d->temperature) { d->temperature=false; d->pressure_count=0; }
            else if(++d->pressure_count==9) d->temperature=true;
            d->state=4; return true;
        }
        break;
    default: baro_fault(d,now); break;
    }
    return false;
}

bool drv_ads_config_word(uint16_t fs,uint16_t sps,unsigned channel,uint16_t *out)
{
    static const uint16_t scales[]={6144,4096,2048,1024,512,256};
    static const uint16_t rates[]={8,16,32,64,128,250,475,860};
    unsigned a,b;
    if(!out || channel>1) return false;
    for(a=0;a<6 && scales[a]!=fs;a++) { }
    for(b=0;b<8 && rates[b]!=sps;b++) { }
    if(a==6 || b==8) return false;
    *out=(uint16_t)(0x8103u|((4u+channel)<<12)|(a<<9)|(b<<5));
    return true;
}
void drv_temp_init(drv_temp_t *d,drv_i2c_t bus,drv_excitation_t gate)
{ *d=(drv_temp_t){0}; d->bus=bus; d->gate=gate; }
static void temp_off(drv_temp_t *d,uint32_t now,bool fault)
{
    uint32_t elapsed=d->gate.excite(d->gate.ctx,false,0);
    /* Enforce off-time from ACTUAL excitation, including a stalled loop. */
    uint32_t interval=(uint32_t)(((uint64_t)elapsed*100u+5u)/6u);
    if(interval>0 && !drv_due(d->next,now+interval)) d->next=now+interval;
    d->state=0;
    if(fault) { d->ok=false; d->errors++; }
}
bool drv_temp_configure(drv_temp_t *d,const board_temp_cfg_t *cfg,uint32_t now)
{
    uint16_t word; uint32_t conv,budget;
    if(!cfg) return false;
    if(cfg->period_us==0) {
        bool pending=d->state==2 || d->state==4 || d->state==6 || d->state==7;
        d->cfg=*cfg; temp_off(d,now,false); if(pending) d->state=7; return true;
    }
    if(d->state) return false;
    if(cfg->period_us>INT32_MAX || cfg->settle_us>100000u ||
       !drv_ads_config_word(cfg->fs_mv_ain0,cfg->sps,0,&word) ||
       !drv_ads_config_word(cfg->fs_mv_ain1,cfg->sps,1,&word)) return false;
    /* [assumed] <=500 us/bus phase at 400 kHz, 12% conversion margin,
     * 25 us device wake-up (TI). Hardware cutoff is the reserved ceiling. */
    conv=(1120000u+cfg->sps-1u)/cfg->sps+25u;
    budget=cfg->settle_us+2u*(conv+500u)+300u;
    /* TIM8 cutoff uses 1 us ticks and a 16-bit ARR (budget+1). */
    if(budget>=65534u || (uint64_t)budget*100u>(uint64_t)cfg->period_us*6u) return false;
    temp_off(d,now,false); d->cfg=*cfg; d->budget=budget;
    if(drv_due(now,d->next)) d->next=now;
    return true;
}
bool drv_temp_poll(drv_temp_t *d,uint32_t now,board_temp_raw_t *out)
{
    uint8_t tx[3]={0},rx[2]; uint16_t word; int result;
    if(d->state && d->state<7 && (drv_due(now,d->cutoff) || !d->gate.is_on(d->gate.ctx))) {
        bool pending=d->state==2 || d->state==4 || d->state==6;
        temp_off(d,now,true);
        /* Drain the outstanding transaction before starting another sample. */
        if(pending) d->state=7;
    }
    switch(d->state) {
    case 0:
        if(!d->cfg.period_us || !drv_due(now,d->next)) break;
        (void)d->gate.excite(d->gate.ctx,true,d->budget);
        d->cutoff=now+d->budget; d->next=now+d->cfg.period_us;
        d->deadline=now+d->cfg.settle_us; d->channel=0; d->state=1;
        break;
    case 1:
        if(!drv_due(now,d->deadline)) break;
        (void)drv_ads_config_word(d->channel?d->cfg.fs_mv_ain1:d->cfg.fs_mv_ain0,
                                  d->cfg.sps,d->channel,&word);
        tx[0]=1; tx[1]=(uint8_t)(word>>8); tx[2]=(uint8_t)word;
        if(d->bus.start(d->bus.ctx,0x48,tx,3,0)) d->state=2;
        break;
    case 2:
        result=d->bus.result(d->bus.ctx,0,0);
        if(result<0) temp_off(d,now,true);
        else if(result>0) {
            uint32_t conv=(1000000u+d->cfg.sps-1u)/d->cfg.sps+25u;
            if(!d->channel) d->conv_tick=now+conv/2u;
            d->deadline=now+conv; d->state=3;
        }
        break;
    case 3:
        if(!drv_due(now,d->deadline)) break;
        tx[0]=1;
        if(d->bus.start(d->bus.ctx,0x48,tx,1,2)) d->state=4;
        break;
    case 4:
        result=d->bus.result(d->bus.ctx,rx,2);
        if(result<0) temp_off(d,now,true);
        else if(result>0) {
            if(rx[0]&0x80u) d->state=5;
            else { d->deadline=now+100u; d->state=3; }
        }
        break;
    case 5:
        if(d->bus.start(d->bus.ctx,0x48,tx,1,2)) d->state=6;
        break;
    case 6:
        result=d->bus.result(d->bus.ctx,rx,2);
        if(result<0) temp_off(d,now,true);
        else if(result>0) {
            int32_t code=(int32_t)((uint16_t)rx[0]<<8|rx[1]);
            if(code>=32768) code-=65536;
            d->codes[d->channel]=(int16_t)code;
            if(d->channel==0) { d->channel=1; d->deadline=now; d->state=1; }
            else {
                uint32_t elapsed=d->gate.excite(d->gate.ctx,false,0);
                *out=(board_temp_raw_t){d->conv_tick,d->codes[0],d->codes[1],
                    d->cfg.fs_mv_ain0,d->cfg.fs_mv_ain1,elapsed};
                temp_off(d,now,false); d->ok=true; return true;
            }
        }
        break;
    case 7:
        /* A completion can have any RX length. Bus result supports discard. */
        result=d->bus.result(d->bus.ctx,0,0);
        if(result!=0) d->state=0;
        break;
    default: temp_off(d,now,true); break;
    }
    return false;
}
