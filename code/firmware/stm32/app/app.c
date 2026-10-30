/* GPL-3.0-only. Implements flight-software-interfaces section 4.
 * Capture ticks remain in wire payloads. Log envelope time is ingestion time,
 * extended since boot, so asynchronously drained queues never reverse it.
 * No task waits for Pi/GNSS/flash/radio. One flash mount page per loop. */
#include "app.h"
#include "cs_ms5611.h"
#include "cs_geo.h"
static bool flash_busy(void *ctx) { (void)ctx; return board_flash_busy(); }
static bool flash_read(void *ctx,uint32_t addr,uint8_t *p,size_t n)
{ (void)ctx; bool ok=board_flash_read(addr,p,n); board_watchdog_kick(); return ok; }
static bool flash_program(void *ctx,uint32_t addr,const uint8_t *p,size_t n)
{ (void)ctx; return board_flash_program(addr,p,n); }
static bool flash_erase(void *ctx,uint32_t addr)
{ (void)ctx; return board_flash_erase_sector(addr); }
double app_mission_time(const app_t *a) { return a->mission_offset+(double)a->now_us/1000000.0; }
static bool fresh(const app_t *a,uint64_t t,uint64_t max_age) { return a->now_us-t<=max_age; }
static uint64_t capture_time(const app_t *a,uint32_t tick)
{
    /* Use the low word of the SAME extended-time snapshot. A flash read may
     * have advanced board_tick since now_us was sampled; mixing them would
     * introduce read-latency jitter and incorrectly coalesce valid D1s. */
    uint32_t age=(uint32_t)((uint32_t)a->now_us-tick);
    return age<=a->now_us?a->now_us-age:0;
}
static void put(uint8_t *p,uint32_t x,size_t n)
{ for(size_t i=0;i<n;i++) p[i]=(uint8_t)(x>>(8u*i)); }
static void log_drain(app_t *a)
{
    if(!a->log_ready || a->launch_pending) return;
    while(a->log_count) {
        app_log_record_t *r=&a->log_queue[a->log_head];
        if((size_t)a->log.used+12u+r->len>CS_LOG_PAGE_SIZE && a->log.pending_valid) return;
        if(!cs_log_append(&a->log,r->type,r->tick,r->payload,r->len) && !a->log.stopped) return;
        a->log_head=(a->log_head+1u)%APP_LOG_QUEUE_CAP; a->log_count--;
    }
}
static void log_record(app_t *a,uint8_t id,const uint8_t *p,size_t n,bool force)
{
    if(!a->log_ready && !a->log_resuming) { a->log_unavailable++; return; }
    /* All raw temperature readings and phase/command events are retained.
     * High-rate streams are sampled independently at 0.1 Hz after landing. */
    if(a->fsm.state==CS_LANDED && !force && id!=APP_LOG_TEMP &&
       a->last_log_us[id] && a->now_us-a->last_log_us[id]<APP_LANDED_LOG_US) {
        a->log_suppressed++; return;
    }
    log_drain(a);
    if(a->log_count==APP_LOG_QUEUE_CAP || n>APP_LOG_PAYLOAD_MAX) { a->log_queue_dropped++; return; }
    app_log_record_t *r=&a->log_queue[(a->log_head+a->log_count)%APP_LOG_QUEUE_CAP];
    r->tick=a->now_us; r->type=id; r->len=(uint8_t)n; memcpy(r->payload,p,n);
    a->log_count++; a->last_log_us[id]=a->now_us; log_drain(a);
}
static void send_payload(app_t *a,uint8_t id,const uint8_t *p,size_t n,bool force)
{
    uint8_t wire[CS_LINK_MAX_WIRE];
    if(a->uart_enabled && board_pi_powered()) {
        size_t len=cs_link_encode(&a->tx,wire,sizeof wire,id,p,n);
        if(len) (void)cs_txqueue_push(&a->txqueue,wire,len,id==CS_LINK_IMU_ID);
    } else a->tx_suppressed++;
    log_record(a,id,p,n,force);
}
#define SEND(a,name,ID,msg,force) do { uint8_t app_payload_[CS_LINK_MAX_PAYLOAD]; \
    size_t app_n_=cs_link_pack_##name(app_payload_,sizeof app_payload_,msg); \
    if(app_n_) send_payload(a,ID,app_payload_,app_n_,force); } while(0)
static void hello(app_t *a)
{
    cs_link_hello_t m={CS_PROTO_VERSION,board_epoch(),APP_FW_VERSION,board_reset_cause(),CS_HELLO_REQUIRE_EPOCH_ECHO,
                       board_tick_hi(),board_tick(),BOARD_TICK_HZ};
    SEND(a,hello,CS_LINK_HELLO_ID,&m,true);
}
static bool fix_usable(const app_t *a)
{ return a->gnss_valid && fresh(a,a->gnss_rx_us,APP_GNSS_STALE_US) &&
         a->gnss.fix_type>=3 && a->gnss.fix_type<=4 && (a->gnss.fix_flags&1u) &&
         a->gnss.lat_deg>= -900000000 && a->gnss.lat_deg<=900000000 &&
         a->gnss.lon_deg>= -1800000000 && a->gnss.lon_deg<=1800000000 && a->gnss.h_msl_m!=INT32_MIN &&
         !(a->gnss.link_flags&CS_GNSS_STALE); }
void app_build_status(app_t *a,cs_link_status_t *s)
{
    board_imu_info_t info; board_imu_info(&info); memset(s,0,sizeof *s);
    s->tick=board_tick(); s->tick_hi=board_tick_hi(); s->epoch=board_epoch(); s->state=(uint8_t)a->fsm.state;
    /* Negative entry times after reset are represented modulo the new TIM2. */
    double age_us=(app_mission_time(a)-a->state_time)*1000000.0;
    uint64_t age=age_us>0?(uint64_t)age_us:0;
    s->tick_state=s->tick-(uint32_t)age;
    s->health=(uint16_t)((board_pgood_5v()?CS_HEALTH_5V:0u)|(board_pgood_3v3()?CS_HEALTH_3V3:0u)|
        (board_pi_powered()?CS_HEALTH_PI_POWER:0u)|(a->uart_enabled?CS_HEALTH_UART:0u)|
        (info.gyro_ok && info.accel_ok && a->gyro_seen && a->accel_seen &&
          (uint32_t)(board_tick()-a->last_gyro)<APP_ACCEL_STALE_US &&
          (uint32_t)(board_tick()-a->last_accel)<APP_ACCEL_STALE_US?CS_HEALTH_IMU:0u)|
        (a->pressure_valid && fresh(a,a->pressure_us,APP_SENSOR_STALE_US) && board_baro_ok()?CS_HEALTH_BARO:0u)|
        (a->gnss_valid && fresh(a,a->gnss_rx_us,APP_GNSS_STALE_US)?CS_HEALTH_GNSS:0u)|
        (fix_usable(a)?CS_HEALTH_FIX:0u)|(a->have_pps && (uint32_t)(board_tick()-a->last_pps)<1500000u?CS_HEALTH_PPS:0u)|
        (board_radio_ok()?CS_HEALTH_RADIO:0u)|(board_flash_ok() && a->log_ready && !a->log.stopped?CS_HEALTH_FLASH:0u)|
        (a->temp_valid && fresh(a,a->temp_us,APP_TEMP_STALE_US) && board_temp_ok()?CS_HEALTH_TEMP:0u)|
        (board_trig_running()?CS_HEALTH_TRIGGERS:0u)|(a->origin_valid?CS_HEALTH_ORIGIN:0u)|
        (isfinite(a->fsm.baseline_pa)?CS_HEALTH_BASELINE:0u)|(a->pi_alive?CS_HEALTH_PI_ALIVE:0u));
    s->battery_v=board_battery_mv();
    s->baseline_pa=cs_link_scale_unsigned(a->fsm.baseline_pa,.01,32);
    s->origin_lat_deg=a->origin_valid?a->origin_lat:INT32_MIN;
    s->origin_lon_deg=a->origin_valid?a->origin_lon:INT32_MIN;
    s->origin_h_msl_m=a->origin_valid?a->origin_h:INT32_MIN;
    s->altitude_m=(int16_t)cs_link_scale_signed(a->fsm.altitude,.1,16);
    s->temperature_c=a->temp_valid && fresh(a,a->temp_us,APP_TEMP_STALE_US)?a->temperature_c:INT16_MIN;
    s->telemetry_seq=a->telemetry_seq; s->rx_crc=(uint16_t)a->rx.stats.crc;
    s->rx_lost=(uint16_t)a->rx.stats.lost; s->tx_dropped=(uint16_t)a->txqueue.tx_dropped;
    s->pi_age_s=a->pi.seen_rx?(uint8_t)cs_link_scale_unsigned((double)(uint32_t)((uint32_t)(a->now_us/1000u)-a->pi.last_rx_ms)/1000.0,.1,8):UINT8_MAX;
    s->mcu_temp_c=board_mcu_temp_c();
}
void app_build_packet(app_t *a,cs_telemetry_t *p)
{
    cs_telemetry_init(p); p->seq=a->telemetry_seq; p->t_ms=(uint32_t)(a->now_us/1000u); p->state=(uint8_t)a->fsm.state;
    p->pressure_pa=cs_telemetry_u16(a->pressure_valid && fresh(a,a->pressure_us,APP_SENSOR_STALE_US)?a->pressure_pa:(double)NAN,1,50000);
    p->temperature_c=a->temp_valid && fresh(a,a->temp_us,APP_TEMP_STALE_US)?a->temperature_c:INT16_MIN;
    p->altitude_m=cs_telemetry_i16(a->fsm.altitude,.1);
    p->battery_v=board_battery_mv()?cs_telemetry_u8((double)board_battery_mv()/(1000.0*APP_BATTERY_CELLS),.01,2.5):UINT8_MAX;
    if(fix_usable(a) && a->origin_valid) {
        double enu[3];
        if(cs_geo_enu_from_link(a->gnss.lat_deg,a->gnss.lon_deg,a->gnss.h_msl_m,
                              a->origin_lat,a->origin_lon,a->origin_h,enu)) {
            p->gps_e=cs_telemetry_i16(enu[0],.1); p->gps_n=cs_telemetry_i16(enu[1],.1);
            p->gps_u=cs_telemetry_i16(enu[2],.1); p->flags|=CS_FLAG_GPS_VALID;
        }
    }
    const cs_link_nav_t *nav=a->nav_valid && a->nav_epoch_ready && a->pi_alive && fresh(a,a->nav_rx_us,2000000u)?&a->nav:NULL;
    cs_apply_nav(p,nav,2.0,board_tick());
}
static void status(app_t *a)
{ app_build_status(a,&a->status); SEND(a,status,CS_LINK_STATUS_ID,&a->status,true); a->last_status_us=a->now_us; }
static void transition(app_t *a,cs_flight_state_t previous)
{
    a->state_time=app_mission_time(a);
    if(a->fsm.state==CS_ASCENT) a->launch_pending=true;
    uint8_t event[8]={0}; event[0]=(uint8_t)previous; event[1]=(uint8_t)a->fsm.state;
    event[2]=(uint8_t)a->fsm.reason; put(event+4,board_tick(),4);
    /* Persist launch state before any operation that might be interrupted. */
    app_persist_save(a);
    if(a->fsm.state==CS_ASCENT) {
        cs_link_cmd_t cmd={board_tick(),(uint8_t)(++a->pi.cmd_seq),CS_CMD_PREPARE,0};
        SEND(a,cmd,CS_LINK_CMD_ID,&cmd,true); app_persist_save(a);
    }
    /* Event is retained even while launch protection waits for busy flash. */
    if(a->launch_pending && a->log_ready) {
        if(cs_log_mark_launch_recent(&a->log,APP_PRELAUNCH_RETAIN_US)) { a->launch_pending=false; app_persist_save(a); }
    }
    memcpy(a->pending_event,event,sizeof event); a->event_pending=true;
    if(a->log_ready && !a->launch_pending) { log_record(a,APP_LOG_EVENT,event,sizeof event,true); a->event_pending=false; }
    status(a);
}
static void fsm_update(app_t *a,double pressure,uint64_t tick_us)
{
    if(tick_us<a->last_fsm_us || (a->last_fsm_us && tick_us-a->last_fsm_us<10000u)) { a->fsm_coalesced++; return; }
    a->last_fsm_us=tick_us;
    cs_flight_state_t old=a->fsm.state;
    double acc=a->accel_valid && fresh(a,a->accel_us,APP_ACCEL_STALE_US)?a->accel_magnitude:(double)NAN;
    (void)cs_fsm_update(&a->fsm,a->mission_offset+(double)tick_us/1000000.0,acc,pressure);
    if(a->fsm.state!=old) transition(a,old);
}
static void imu_snapshot(app_t *a)
{
    if(a->snapshot_pending) {
        cs_link_imu_t m;
        if(cs_imu_snapshot(&a->imu,a->snapshot_tick,&m)) {
            if(a->first_imu_wire && a->uart_enabled && board_pi_powered()) {
                m.flags|=CS_IMU_RESTART; a->first_imu_wire=false;
            }
            SEND(a,imu,CS_LINK_IMU_ID,&m,false); a->snapshot_pending=false;
        }
    }
}
static void map_axes(const float matrix[3][3],const float v[3],float out[3])
{ for(unsigned i=0;i<3;i++) out[i]=matrix[i][0]*v[0]+matrix[i][1]*v[1]+matrix[i][2]*v[2]; }
static void drain_imu(app_t *a)
{
    board_imu_sample_t g,b; bool hg=board_gyro_pop(&g),ha=board_accel_pop(&b);
    board_imu_info_t info; board_imu_info(&info);
    while(hg || ha) {
        float v[3]; uint8_t flags=0;
        if(hg && (!ha || (int32_t)(g.tick-b.tick)<=0)) {
            map_axes(app_gyro_to_body,g.v,v);
            if(g.flags&BOARD_IMU_SATURATED) flags|=CS_IMU_GYRO_SATURATED;
            if(a->gyro_seen && (uint32_t)(g.tick-a->last_gyro)>APP_IMU_GYRO_GAP_US) flags|=CS_IMU_GYRO_MISSED;
            if(!info.gyro_ok) flags|=CS_IMU_GYRO_FAULT;
            if(!info.accel_ok) flags|=CS_IMU_ACCEL_FAULT;
            (void)cs_imu_add_gyro(&a->imu,g.tick,v,flags); a->last_gyro=g.tick; a->gyro_seen=true;
            if(++a->gyro_mod==APP_IMU_SNAPSHOT_GYROS) {
                a->gyro_mod=0; a->snapshot_tick=g.tick; a->snapshot_pending=true;
            }
            hg=board_gyro_pop(&g);
        } else {
            map_axes(app_accel_to_body,b.v,v);
            if(b.flags&BOARD_IMU_SATURATED) flags|=CS_IMU_ACCEL_SATURATED;
            if(a->accel_seen && (uint32_t)(b.tick-a->last_accel)>APP_IMU_ACCEL_GAP_US) flags|=CS_IMU_ACCEL_MISSED;
            if(!info.accel_ok) flags|=CS_IMU_ACCEL_FAULT;
            (void)cs_imu_add_accel(&a->imu,b.tick,v,flags); a->last_accel=b.tick; a->accel_seen=true;
            a->accel_magnitude=sqrt((double)v[0]*(double)v[0]+(double)v[1]*(double)v[1]+(double)v[2]*(double)v[2]);
            a->accel_us=capture_time(a,b.tick); a->accel_valid=isfinite(a->accel_magnitude);
            ha=board_accel_pop(&b);
        }
        imu_snapshot(a);
    }
}
static void baro(app_t *a)
{
    board_baro_raw_t r;
    if(!a->prom_valid && board_baro_prom(a->prom)) a->prom_valid=cs_ms5611_prom_valid(a->prom);
    while(board_baro_pop(&r)) {
        if(r.is_temperature) { a->d2=r.adc; continue; }
        cs_ms5611_result_t value;
        if(!a->prom_valid || !cs_ms5611_compensate(a->prom,r.adc,a->d2,true,&value)) continue;
        a->pressure_pa=(double)value.pressure_cpa/100.0; a->pressure_valid=true;
        a->pressure_us=capture_time(a,r.tick); a->baro_temperature=value.temperature_centi_c;
        fsm_update(a,a->pressure_pa,a->pressure_us);
        if(!a->baro_count) a->baro_tick_base=r.tick;
        if(a->baro_count<255u) { a->baro_pressure_sum+=value.pressure_cpa; a->baro_tick_sum+=(uint32_t)(r.tick-a->baro_tick_base); a->baro_count++; }
    }
    if(a->now_us-a->last_baro_us>=APP_BARO_INTERVAL_US) {
        cs_link_baro_t m={board_tick(),UINT32_MAX,INT16_MIN,0,0};
        if(a->baro_count) {
            m.tick=a->baro_tick_base+(uint32_t)(a->baro_tick_sum/a->baro_count);
            m.pressure_pa=(uint32_t)((a->baro_pressure_sum+a->baro_count/2u)/a->baro_count);
            m.temperature_c=(int16_t)a->baro_temperature; m.n_samples=(uint8_t)a->baro_count; m.flags=1;
        }
        SEND(a,baro,CS_LINK_BARO_ID,&m,false); a->baro_count=0; a->baro_pressure_sum=0; a->baro_tick_sum=0; a->last_baro_us=a->now_us;
    }
    if(a->now_us-a->last_fsm_us>=10000u && (!a->pressure_valid || !fresh(a,a->pressure_us,APP_SENSOR_STALE_US)))
        fsm_update(a,(double)NAN,a->now_us);
}
static unsigned pga(uint16_t fs)
{ switch(fs) { case 6144:return 0; case 4096:return 1; case 2048:return 2; case 1024:return 3; case 512:return 4; case 256:return 5; default:return 8; } }
static void temperature(app_t *a)
{
    board_temp_raw_t r;
    while(board_temp_pop(&r)) {
        cs_ntc_result_t result;
        cs_ntc_fault_t fault=cs_ntc_convert(&a->ntc,r.code_ain0,pga(r.fs_mv_ain0),r.code_ain1,pga(r.fs_mv_ain1),&result);
        if(UINT64_MAX-a->temp_on_us<r.excitation_us) a->temp_on_us=UINT64_MAX;
        else a->temp_on_us+=r.excitation_us;
        /* Board owns excitation sequencing; preflight schedule bounded at 6%.
         * A prefix ending at the immediate first pulse cannot have 6% duty:
         * its off-time is still ahead. Permit one bounded pulse of startup
         * credit, then enforce 6% sustained usage. Independent hardware cutoff
         * and the driver's reserved off-time still bound every actual cycle.
         * Integer split avoids overflow while computing floor(elapsed*6/100). */
        uint64_t elapsed=a->now_us>=a->temp_start_us?a->now_us-a->temp_start_us:0;
        uint64_t allowance=APP_TEMP_MAX_ON_US+(elapsed/100u)*6u+(elapsed%100u)*6u/100u;
        if(r.excitation_us>APP_TEMP_MAX_ON_US || a->temp_on_us>allowance) {
            board_temp_cfg_t off={0,0,APP_TEMP_SPS,2048,4096}; (void)board_temp_configure(&off);
            a->temp_disabled=true; fault=CS_NTC_CONFIG;
        }
        a->temperature_c=fault==CS_NTC_OK?result.temperature_centi_c:INT16_MIN;
        a->temp_valid=fault==CS_NTC_OK && !a->temp_disabled; a->temp_us=capture_time(a,r.tick);
        if(!a->temp_valid) a->temp_faults++;
        uint8_t p[20]={0}; put(p,r.tick,4); put(p+4,(uint16_t)r.code_ain0,2); put(p+6,(uint16_t)r.code_ain1,2);
        put(p+8,r.fs_mv_ain0,2); put(p+10,r.fs_mv_ain1,2); put(p+12,(uint16_t)a->temperature_c,2);
        p[14]=(uint8_t)fault; put(p+16,r.excitation_us,4); log_record(a,APP_LOG_TEMP,p,sizeof p,true);
    }
}
static void gnss_config(app_t *a)
{
    uint32_t now=board_tick();
    if(a->gnss_config_phase==0 && board_gnss_tx_idle()) {
        uint8_t p[CS_UBX_MAX_PAYLOAD+8]; size_t n=cs_ubx_flight_config(p,sizeof p);
        if(n && board_gnss_write(p,n)) {
            a->gnss_config_attempts++; a->gnss_config_tick=now; a->gnss_config_phase=1; a->gnss_ack=false;
        }
    } else if(a->gnss_config_phase==1 && board_gnss_tx_idle() &&
              (a->gnss_ack || (uint32_t)(now-a->gnss_config_tick)>=APP_GNSS_ACK_US)) {
        /* ACK can be sent at either baud. Once TX is drained, listen at the
         * requested rate even if the ACK was missed. NAV-PVT proves success. */
        board_gnss_set_baud(115200); a->gnss_baud=115200;
        a->gnss_config_tick=now; a->gnss_config_phase=2;
    } else if(a->gnss_config_phase==2 && (uint32_t)(now-a->gnss_config_tick)>=APP_GNSS_SETTLE_US) {
        a->gnss_config_phase=3; a->gnss_config_tick=now;
    } else if(a->gnss_config_phase==3 &&
              (!a->gnss_valid || !fresh(a,a->gnss_rx_us,APP_GNSS_RETRY_US)) &&
              (uint32_t)(now-a->gnss_config_tick)>=APP_GNSS_RETRY_US) {
        /* Retry both plausible receiver rates after a reset or lost command. */
        a->gnss_baud=(a->gnss_config_attempts%2u)?115200u:9600u;
        board_gnss_set_baud(a->gnss_baud); a->gnss_config_phase=0;
    }
}
static void gnss(app_t *a)
{
    uint32_t tick;
    while(board_pps_pop(&tick)) {
        cs_link_pps_t m={tick,UINT32_MAX,0,(uint16_t)(++a->pps_count),0};
        if(a->have_pps) {
            uint32_t dt=tick-a->last_pps;
            if(dt>500000u && dt<1500000u) m.interval=dt;
        }
        /* Only label time-of-week if a recent PVT identifies this edge within
         * a quarter-second. Otherwise explicit NA beats a guessed second. */
        if(a->have_itow && (a->gnss.link_flags&CS_GNSS_TICK_FROM_PPS) && fresh(a,a->gnss_rx_us,1500000u) && a->have_pps && m.interval) {
            uint32_t pvt_edge=a->gnss.tick-(a->gnss.itow_ms%1000u)*1000u;
            uint32_t delta=tick-pvt_edge;
            if(delta>750000u && delta<1250000u) {
                m.itow_ms=(a->gnss.itow_ms-a->gnss.itow_ms%1000u+1000u)%604800000u; m.flags=1;
            }
        }
        a->previous_pps=a->last_pps; a->last_pps=tick; a->have_pps=true;
        SEND(a,pps,CS_LINK_PPS_ID,&m,false);
    }
    uint8_t p[256]; size_t n;
    while((n=board_gnss_read(p,sizeof p))!=0) {
        for(size_t i=0;i<n;i++) if(cs_ubx_feed(&a->ubx,p[i])) {
            cs_ubx_frame_t *f=&a->ubx.frame;
            if(f->cls==5 && f->id==1 && f->len==2 && f->payload[0]==6 && f->payload[1]==0x8a) a->gnss_ack=true;
            cs_link_gnss_t m;
            if(cs_ubx_nav_pvt(f,board_tick(),0,&m)) {
                uint32_t forward_ms=(m.itow_ms+604800000u-a->last_itow)%604800000u;
                bool repeated=a->have_itow && (forward_ms==0 || forward_ms>302400000u);
                if(a->have_pps && (uint32_t)(board_tick()-a->last_pps)<1200000u) {
                    uint32_t age=board_tick()-(a->last_pps+(m.itow_ms%1000u)*1000u);
                    if(age<500000u) { m.tick=a->last_pps+(m.itow_ms%1000u)*1000u; m.link_flags|=CS_GNSS_TICK_FROM_PPS; }
                }
                if(repeated) m.link_flags|=CS_GNSS_STALE;
                a->gnss=m; a->gnss_valid=true; a->gnss_rx_us=a->now_us;
                a->last_itow=m.itow_ms; a->have_itow=true;
                if(a->fsm.state==CS_PAD && fix_usable(a) && m.h_acc_m<=APP_ORIGIN_MAX_HACC_MM) {
                    a->origin_valid=true; a->origin_lat=m.lat_deg; a->origin_lon=m.lon_deg; a->origin_h=m.h_msl_m;
                }
                SEND(a,gnss,CS_LINK_GNSS_ID,&m,false);
            }
        }
    }
    gnss_config(a);
}
static void receive(void *context,uint8_t id,uint8_t seq,const uint8_t *p,size_t n)
{
    app_t *a=context; (void)seq;
    if(id<CS_LINK_PI_STATUS_ID || id>CS_LINK_TRIG_CFG_ID) return;
    a->rx_valid=true; log_record(a,id,p,n,false);
    if(id==CS_LINK_NAV_ID) {
        if(a->nav_epoch_ready && cs_link_unpack_nav(&a->nav,p,n)) { a->nav_rx_us=a->now_us; a->nav_valid=true; }
    } else if(id==CS_LINK_ACK_ID) {
        if(cs_link_unpack_ack(&a->ack,p,n)) a->ack_valid=true;
    } else if(id==CS_LINK_PI_STATUS_ID) {
        cs_link_pi_status_t m;
        if(cs_link_unpack_pi_status(&m,p,n)) {
            uint8_t r[12]; put(r,board_tick(),4); put(r+4,m.tick_echo,4);
            put(r+8,m.echo_age_ms,2); r[10]=m.state; r[11]=m.proto; log_record(a,APP_LOG_ROUNDTRIP,r,sizeof r,false);
            a->nav_epoch_ready=m.proto==CS_PROTO_VERSION && (m.flags&CS_PIFLAG_EPOCH_ECHO) &&
                               (uint8_t)(m.flags>>CS_PIFLAG_EPOCH_SHIFT)==board_epoch();
            if(!a->nav_epoch_ready) a->nav_valid=false;
        }
    } else {
        cs_link_trig_cfg_t m;
        if(cs_link_unpack_trig_cfg(&m,p,n)) {
            uint32_t period=(uint32_t)m.period_ms*1000u;
            if(!m.enable) { board_trig_stop(!board_pi_powered() || a->pi.phase!=CS_PI_RUNNING); a->trigger_first=false; }
            else if(a->pi.phase==CS_PI_RUNNING && period>=APP_TRIG_MIN_PERIOD_US && period<=APP_TRIG_MAX_PERIOD_US &&
                    m.pulse_us>0 && m.pulse_us<=APP_TRIG_MAX_PULSE_US && (uint32_t)m.pulse_us<period) {
                /* Compare accepted requests, not rounded actual timer values:
                 * an identical refresh must preserve phase and queued captures. */
                if(!board_trig_running() || a->trigger_requested_period!=period || a->trigger_requested_pulse!=m.pulse_us) {
                    if(board_trig_start(period,m.pulse_us,&a->trigger_period,&a->trigger_pulse)) {
                        a->trigger_requested_period=period; a->trigger_requested_pulse=m.pulse_us;
                        a->trigger_first=true; a->frame=0;
                    }
                    else a->trigger_rejected++;
                }
            } else a->trigger_rejected++;
        }
    }
}
static void pi(app_t *a)
{
    uint8_t p[256]; size_t n;
    a->rx_valid=false; a->ack_valid=false;
    /* Expire trust before decoding a returning frame. An old Nav must not
     * refresh Pi-alive and thereby revive its own stale epoch gate. */
    if(!a->uart_enabled || !a->pi.seen_rx ||
       (uint32_t)((uint32_t)(a->now_us/1000u)-a->pi.last_rx_ms)>=a->pi.cfg.alive_ms) {
        a->nav_epoch_ready=false; a->nav_valid=false;
    }
    while((n=board_pi_read(p,sizeof p))!=0) cs_link_decoder_feed(&a->rx,p,n,receive,a);
    uint16_t bat=board_battery_mv();
    if(bat && bat<APP_LOW_BATTERY_MV) { if(!a->low_battery_us) a->low_battery_us=a->now_us; }
    else a->low_battery_us=0;
    bool ground=a->fsm.state==CS_PAD || a->fsm.state==CS_LANDED;
    if((a->fsm.state==CS_LANDED && app_mission_time(a)-a->fsm.t_landed>=APP_SHUTDOWN_LANDED_S) ||
       (ground && a->low_battery_us && a->now_us-a->low_battery_us>=APP_LOW_BATTERY_HOLD_US)) a->shutdown_requested=true;
    cs_pimgr_input_t in={(uint32_t)(a->now_us/1000u),a->fsm.state,board_pgood_5v(),a->rx_valid,a->shutdown_requested,a->ack_valid?&a->ack:NULL};
    uint16_t old_seq=a->pi.cmd_seq;
    cs_pimgr_output_t out=cs_pimgr_poll(&a->pi,&in);
    if(out.stop_triggers) { board_trig_stop(true); a->trigger_first=false; }
    /* Disable buffer BEFORE power transitions; enable only AFTER power up. */
    if(out.oe_high || board_pi_powered()!=out.pwr_en) board_pi_uart_enable(false);
    board_pi_power(out.pwr_en); board_pi_uart_enable(!out.oe_high);
    bool was_enabled=a->uart_enabled;
    a->uart_enabled=!out.oe_high; a->pi_alive=out.alive;
    if(!a->uart_enabled) {
        uint32_t dropped=a->txqueue.tx_dropped;
        a->tx_suppressed+=(uint32_t)a->txqueue.count;
        cs_txqueue_init(&a->txqueue); a->txqueue.tx_dropped=dropped;
    }
    if(!out.pwr_en || !a->uart_enabled || !a->pi_alive) { a->nav_valid=false; a->nav_epoch_ready=false; }
    if(!out.pwr_en) cs_link_decoder_reset_sequence(&a->rx);
    if(out.send_hello || (!was_enabled && a->uart_enabled)) hello(a);
    if(out.send_cmd) { out.cmd.tick=board_tick(); SEND(a,cmd,CS_LINK_CMD_ID,&out.cmd,true); }
    if(a->pi.cmd_seq!=old_seq) app_persist_save(a);
    uint32_t tick;
    while(board_trig_pop(&tick)) {
        cs_link_trig_t m={a->frame++,tick,a->trigger_period,a->trigger_pulse,a->trigger_first?1u:0u};
        a->trigger_first=false;
        SEND(a,trig,CS_LINK_TRIG_ID,&m,false);
    }
    if(board_pi_tx_idle()) {
        size_t len; const uint8_t *wire=cs_txqueue_peek(&a->txqueue,&len);
        if(wire && board_pi_write(wire,len)) (void)cs_txqueue_consume(&a->txqueue,len);
    }
}
static void logger(app_t *a)
{
    if(!a->log_started && board_flash_ok() && !board_flash_busy()) {
        cs_flash_t f={NULL,board_flash_size(),flash_busy,flash_read,flash_program,flash_erase};
        if(a->log_resuming) {
            a->log_started=cs_log_resume_begin(&a->log,&f,board_epoch(),a->saved_writer,a->saved_sequence,a->saved_protected);
            if(!a->log_started) { a->log_resuming=false; a->resume_failed=true; a->saved_log_valid=false; }
        }
        if(!a->log_started) a->log_started=cs_log_mount_begin(&a->log,&f,board_epoch());
    }
    if(!a->log_started) return;
    if(!a->log_ready) {
        /* Bounded startup work: <=1 page read/pass; flash_read kicks watchdog.
         * Sensors/radio/Pi run every pass throughout the scan. */
        if(a->log.mounting) {
            if(a->log_resuming) (void)cs_log_resume_step(&a->log);
            else (void)cs_log_mount_step(&a->log);
        }
        if(!a->log.mounted) {
            if(a->log_resuming && !a->log.mounting) {
                /* Corrupt/stale checkpoint: full conservative scan next pass,
                 * preserve buffered events, never erase to guess a writer. */
                a->log_resuming=false; a->resume_failed=true; a->saved_log_valid=false; a->log_started=false;
            }
            return;
        }
        a->log_resuming=false;
        a->log_ready=true;
        if(a->fsm.state==CS_PAD && !a->persist_invalid) {
            cs_log_allow_prelaunch_ring(&a->log); a->log_ring_allowed=true;
        } else if(a->saved_protected!=CS_LOG_NO_PROTECTION) {
            (void)cs_log_restore_protection(&a->log,a->saved_protected);
        } else if(a->fsm.state!=CS_PAD && !a->log.protected) {
            /* Early launch before mount finished: protect existing+new data. */
            a->launch_pending=true;
        }
    }
    if(a->launch_pending) {
        if(a->log.protected || cs_log_mark_launch_recent(&a->log,APP_PRELAUNCH_RETAIN_US)) {
            a->launch_pending=false; app_persist_save(a);
        } else return; /* No appends/polls while protection waits for busy. */
    }
    if(a->event_pending) { log_record(a,APP_LOG_EVENT,a->pending_event,sizeof a->pending_event,true); a->event_pending=false; }
    log_drain(a);
    if(a->now_us-a->last_flush_us>=APP_LOG_FLUSH_US) {
        if(cs_log_flush(&a->log)) a->last_flush_us=a->now_us;
    }
    cs_log_poll(&a->log);
}
bool app_init(app_t *a)
{
    if(!a) return false;
    memset(a,0,sizeof *a); a->first_imu_wire=true; a->now_us=board_time_us(); a->saved_protected=CS_LOG_NO_PROTECTION;
    a->temperature_c=INT16_MIN; a->pressure_pa=a->accel_magnitude=(double)NAN;
    a->ntc=cs_ntc_default_config(); a->temp_start_us=a->now_us;
    cs_imu_init(&a->imu,board_epoch()); cs_txqueue_init(&a->txqueue);
    cs_link_encoder_init(&a->tx); cs_link_decoder_init(&a->rx,false); cs_ubx_init(&a->ubx);
    cs_lora_config_t lc=cs_lora_default_config();
    if(!cs_fsm_init(&a->fsm,NULL) || !cs_pimgr_init(&a->pi,NULL,(uint32_t)(a->now_us/1000u),0) ||
       !cs_downlink_init(&a->downlink,cs_lora_time_on_air(CS_TELEMETRY_SIZE,&lc),APP_RADIO_DUTY,
           APP_RADIO_WINDOW_S,APP_RADIO_FLIGHT_INTERVAL_S,APP_RADIO_FLIGHT_RESERVE_S,APP_RADIO_GROUND_INTERVAL_S)) return false;
    (void)app_persist_restore(a);
    a->log_resuming=a->restored && a->saved_log_valid && a->fsm.state!=CS_PAD && a->saved_protected!=CS_LOG_NO_PROTECTION;
    /* Invalid backup after a warm reset cannot safely discard past airtime.
     * Reserve the complete window, failing closed until it expires. */
    if(a->persist_invalid) (void)cs_duty_record(&a->downlink.limiter,app_mission_time(a),APP_RADIO_DUTY*APP_RADIO_WINDOW_S);
    board_temp_cfg_t tc={APP_TEMP_PERIOD_US,APP_TEMP_SETTLE_US,APP_TEMP_SPS,2048,4096};
    if((double)(tc.settle_us+2u*((1120000u+tc.sps-1u)/tc.sps+25u+500u)+300u)>.06*(double)tc.period_us || !board_temp_configure(&tc)) a->temp_disabled=true;
    board_radio_cfg_t rc={APP_RADIO_FREQUENCY_HZ,(uint8_t)lc.sf,(uint32_t)lc.bw_hz,(uint8_t)lc.coding_rate,
        (uint16_t)lc.preamble,lc.crc,lc.implicit_header,false,APP_RADIO_POWER_DBM,APP_RADIO_TCXO_MV};
    (void)board_radio_configure(&rc);
    board_gnss_set_baud(9600); a->gnss_baud=9600; board_gnss_reset(false);
    board_trig_stop(true); hello(a); a->initialized=true;
    return true;
}
void app_poll(app_t *a)
{
    if(!a || !a->initialized) return;
    board_poll(); a->now_us=board_time_us();
    /* Protect first; launch marking never does SPI reads. */
    logger(a); drain_imu(a); baro(a); temperature(a); gnss(a); pi(a);
    if(a->now_us-a->last_status_us>=APP_STATUS_INTERVAL_US) status(a);
    if(board_radio_ok() && !board_radio_busy() && cs_downlink_due(&a->downlink,app_mission_time(a),a->fsm.state)) {
        a->telemetry_seq++; app_build_packet(a,&a->packet);
        uint8_t p[CS_TELEMETRY_SIZE]; (void)cs_telemetry_encode(p,sizeof p,&a->packet);
        /* Persist committed budget before TX; a rejected send still consumes
         * budget conservatively, never refunds an uncertain RF attempt. */
        app_persist_save(a);
        if(!board_radio_send(p,sizeof p)) a->radio_failed++;
        log_record(a,APP_LOG_RADIO,p,sizeof p,true);
    }
    if(a->now_us-a->last_persist_us>=APP_PERSIST_INTERVAL_US) { app_persist_save(a); a->last_persist_us=a->now_us; }
    if(a->fsm.state==CS_LANDED) {
        bool on=(a->now_us%1000000u)<100000u; board_buzzer(on); board_led(BOARD_LED_STATUS,on);
    } else { board_buzzer(false); board_led(BOARD_LED_STATUS,(a->now_us%1000000u)<500000u); }
    board_led(BOARD_LED_FAULT,!(a->status.health&CS_HEALTH_BARO) || a->temp_disabled || a->log.stopped);
    board_watchdog_kick();
}
void app_run_once(app_t *a)
{
    app_poll(a);
    /* Boot/resume scan is already bounded to one SPI page per pass. Avoid
     * adding an interrupt-period sleep to every page; sensors/watchdog still
     * run each pass. SITL charges explicit time to the page read itself. */
    if(!a->log.mounting) board_idle();
}
