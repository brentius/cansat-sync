/* GPL-3.0-only. Backup-SRAM persistence, interfaces section 4. Explicit LE v1
 * record, no struct padding; CRC16 covers 158 bytes, magic committed last.
 * Mission elapsed time excludes reset downtime (no retained wall clock API).
 * Duty history is kept as an age histogram (bytes 112-143): restoring each
 * bucket at its newest possible time never expires airtime early, without
 * reserving the whole hour's use for another hour after the reset. A record
 * without buckets keeps the former single current-time reservation. */
#include "app.h"
#include "cs_crc16.h"
static void put(uint8_t *p,uint64_t x,size_t n)
{ for(size_t i=0;i<n;i++) p[i]=(uint8_t)(x>>(8u*i)); }
static uint64_t get(const uint8_t *p,size_t n)
{ uint64_t x=0; for(size_t i=0;i<n;i++) x|=(uint64_t)p[i]<<(8u*i); return x; }
static void put_double(uint8_t *p,double x)
{ uint64_t u; memcpy(&u,&x,8); put(p,u,8); }
static double get_double(const uint8_t *p)
{ uint64_t u=get(p,8); double x; memcpy(&x,&u,8); return x; }
static int32_t get_i32(const uint8_t *p)
{ uint32_t x=(uint32_t)get(p,4); return x<=INT32_MAX?(int32_t)x:-1-(int32_t)(UINT32_MAX-x); }
#define DUTY_BUCKETS 8u
#define DUTY_AT 112u
#define DUTY_MAX_MS ((uint64_t)(APP_RADIO_DUTY*APP_RADIO_WINDOW_S*1000.0)+1u)
/* Bucket k holds airtime aged [k,k+1)*window/8, whole ms rounded up. Call
 * after cs_duty_used(now) has expired old entries. */
static void duty_save(uint8_t *p,const cs_duty_t *d,double now)
{
    double sum[DUTY_BUCKETS]={0},width=d->window_s/DUTY_BUCKETS;
    for(size_t i=0;i<d->count;i++) {
        size_t j=(d->head+i)%CS_DUTY_CAP; double age=now-d->time[j];
        size_t k=age>0?(size_t)(age/width):0u;
        sum[k<DUTY_BUCKETS?k:DUTY_BUCKETS-1u]+=d->airtime[j];
    }
    for(size_t k=0;k<DUTY_BUCKETS;k++) put(p+DUTY_AT+4u*k,sum[k]>0?(uint64_t)(sum[k]*1000.0)+1u:0u,4);
}
static void duty_restore(const uint8_t *p,cs_duty_t *d,double saved,double used)
{
    double width=d->window_s/DUTY_BUCKETS,total=0;
    for(size_t k=DUTY_BUCKETS;k--;) {
        double t=saved-(double)k*width,airtime=(double)get(p+DUTY_AT+4u*k,4)/1000.0,room=cs_duty_remaining(d,t);
        if(airtime>room) airtime=room;
        if(airtime>0 && cs_duty_record(d,t,airtime)) total+=airtime;
    }
    if(used>total+1e-9) {
        double room=cs_duty_remaining(d,saved),rest=used-total;
        (void)cs_duty_record(d,saved,rest<room?rest:room);
    }
}
void app_persist_save(app_t *a)
{
    uint8_t p[APP_PERSIST_BYTES];
    if(board_persist_size()<sizeof p) return;
    memset(p,0,sizeof p); put(p,UINT32_C(0x31505041),4); p[4]=1; p[5]=(uint8_t)a->fsm.state;
    p[6]=a->origin_valid?1u:0u; p[7]=(uint8_t)a->pi.cmd_seq;
    put_double(p+8,app_mission_time(a)); put_double(p+16,a->state_time);
    put_double(p+24,a->fsm.baseline_pa); put_double(p+32,a->fsm.max_altitude);
    put_double(p+40,a->fsm.t_launch); put_double(p+48,a->fsm.t_descent); put_double(p+56,a->fsm.t_landed);
    put(p+64,(uint32_t)a->origin_lat,4); put(p+68,(uint32_t)a->origin_lon,4); put(p+72,(uint32_t)a->origin_h,4);
    put(p+76,a->telemetry_seq,2); put(p+80,a->log_ready?a->log.protected_start:a->saved_protected,4);
    put_double(p+88,cs_duty_used(&a->downlink.limiter,app_mission_time(a)));
    duty_save(p,&a->downlink.limiter,app_mission_time(a));
    p[96]=a->shutdown_requested?1u:0u;
    /* Do not turn an invalid warm-reset PAD assumption into proof that old
     * flight flash may be erased on a second reset before/after mounting. */
    p[97]=a->persist_invalid?1u:0u;
    bool checkpoint=a->log_ready && a->log.protected && a->log.sequence>0 && a->fsm.state!=CS_PAD && !a->launch_pending;
    put(p+100,checkpoint?a->log.writer:a->saved_writer,4);
    put(p+104,checkpoint?a->log.sequence:a->saved_sequence,4);
    p[108]=(checkpoint || (!a->log_ready && a->saved_log_valid))?1u:0u;
    put(p+158,cs_crc16(p,158,UINT16_MAX),2);
    /* Invalidate first; an interrupted write fails validation. */
    uint32_t zero=0; board_persist_write(&zero,0,4);
    board_persist_write(p+4,4,sizeof p-4); board_persist_write(p,0,4);
}
bool app_persist_restore(app_t *a)
{
    uint8_t p[APP_PERSIST_BYTES];
    if(board_reset_cause()&BOARD_RESET_POWER) return false;
    a->persist_invalid=true;
    if(board_persist_size()<sizeof p) return false;
    board_persist_read(p,0,sizeof p);
    if(get(p,4)!=UINT32_C(0x31505041) || p[4]!=1 || p[5]>CS_LANDED || p[6]>1 || p[96]>1 || p[97]!=0 || p[108]>1 ||
       get(p+158,2)!=cs_crc16(p,158,UINT16_MAX)) return false;
    double saved=get_double(p+8),entry=get_double(p+16),used=get_double(p+88);
    if(!isfinite(saved) || saved<0 || !isfinite(entry) || entry>saved ||
       !isfinite(used) || used<0 || used>APP_RADIO_DUTY*APP_RADIO_WINDOW_S) return false;
    int32_t lat=get_i32(p+64),lon=get_i32(p+68);
    if(p[6] && (lat< -900000000 || lat>900000000 || lon< -1800000000 || lon>1800000000)) return false;
    for(size_t k=0;k<DUTY_BUCKETS;k++) if(get(p+DUTY_AT+4u*k,4)>DUTY_MAX_MS) return false;
    if(!cs_fsm_restore(&a->fsm,(cs_flight_state_t)p[5],get_double(p+24),get_double(p+32),
                       get_double(p+40),get_double(p+48),get_double(p+56))) return false;
    a->mission_offset=saved-(double)a->now_us/1000000.0; a->state_time=entry;
    a->origin_valid=p[6]!=0; a->origin_lat=lat; a->origin_lon=lon; a->origin_h=get_i32(p+72);
    a->telemetry_seq=(uint16_t)get(p+76,2); a->pi.cmd_seq=p[7];
    /* Store pointer before incremental mount completes. */
    a->saved_protected=(uint32_t)get(p+80,4);
    a->saved_writer=(uint32_t)get(p+100,4); a->saved_sequence=(uint32_t)get(p+104,4);
    a->saved_log_valid=p[108]!=0;
    a->shutdown_requested=p[96]!=0;
    duty_restore(p,&a->downlink.limiter,saved,used);
    a->persist_invalid=false; a->restored=true; return true;
}
