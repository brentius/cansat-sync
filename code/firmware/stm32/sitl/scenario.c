/* GPL-3.0-only. Purely synthetic 120 s PAD, 5 s ascent to 300 m, 30 s
 * parachute descent, then recovery. Fake GNSS/Pi produce actual wire frames. */
#include "scenario.h"
#include "cs_atmos.h"
static void put(uint8_t *p,uint32_t x,unsigned n)
{ for(unsigned i=0;i<n;i++) p[i]=(uint8_t)(x>>(8u*i)); }
static void pi_send(app_scenario_t *s,uint8_t id,const uint8_t *p,size_t n)
{ uint8_t wire[CS_LINK_MAX_WIRE]; size_t len=cs_link_encode(&s->encoder,wire,sizeof wire,id,p,n); (void)board_sitl_pi_receive(wire,len); }
static void received(void *ctx,uint8_t id,uint8_t seq,const uint8_t *p,size_t n)
{
    app_scenario_t *s=ctx; (void)seq; s->frames[id]++;
    if(id==CS_LINK_HELLO_ID) cs_link_encoder_init(&s->encoder);
    if(id==CS_LINK_IMU_ID) {
        cs_link_imu_t m; if(cs_link_unpack_imu(&m,p,n)) {
            if(m.flags&CS_IMU_RESTART) s->imu_restart++;
            if(m.flags&(CS_IMU_GYRO_MISSED|CS_IMU_ACCEL_MISSED)) s->imu_missed++;
            if(m.flags&(CS_IMU_GYRO_FAULT|CS_IMU_ACCEL_FAULT)) s->imu_faults++;
        }
    } else if(id==CS_LINK_STATUS_ID) {
        cs_link_status_t m; if(cs_link_unpack_status(&m,p,n)) s->state_mask|=1u<<m.state;
    } else if(id==CS_LINK_CMD_ID) {
        cs_link_cmd_t m; if(cs_link_unpack_cmd(&m,p,n)) {
            if(m.cmd==CS_CMD_PREPARE) s->prepare++;
            if(m.cmd==CS_CMD_SHUTDOWN) {
                s->shutdown++; s->shutdown_seen=true; s->last_shutdown_seq=m.cmd_seq;
                if(s->ack_shutdown && s->pi_enabled) {
                    cs_link_ack_t ack={m.cmd_seq,m.cmd,CS_ACK_DONE}; uint8_t b[4];
                    (void)cs_link_pack_ack(b,sizeof b,&ack); pi_send(s,CS_LINK_ACK_ID,b,sizeof b);
                }
            }
        }
    }
}
void app_scenario_init(app_scenario_t *s)
{
    memset(s,0,sizeof *s); s->receiver_baud=9600; s->pi_enabled=s->gnss_enabled=s->ack_shutdown=true;
    cs_link_encoder_init(&s->encoder); cs_link_decoder_init(&s->decoder,false);
}
void app_scenario_sink(void *ctx,int channel,const uint8_t *p,size_t n)
{
    app_scenario_t *s=ctx;
    if(s->capture) s->capture(s->capture_ctx,channel,p,n);
    if(channel==BOARD_SITL_LINK) {
        s->link_bytes+=n; cs_link_decoder_feed(&s->decoder,p,n,received,s);
    } else if(channel==BOARD_SITL_RADIO) {
        s->radio_bytes+=n; s->radio_packets++;
        if(!cs_telemetry_decode(&s->last_packet,p,n)) s->radio_bad++;
    } else if(channel==BOARD_SITL_GNSS_TX && board_sitl_stats()->baud==s->receiver_baud) {
        /* Fake receiver accepts the VALSET at its actual current rate. */
        if(n>6 && p[0]==0xb5 && p[1]==0x62 && p[2]==6 && p[3]==0x8a) {
            static const uint8_t ack[]={0xb5,0x62,5,1,2,0,6,0x8a,0x98,0xc1};
            (void)board_sitl_gnss_receive(ack,sizeof ack); s->receiver_baud=115200;
        }
    }
}
double app_scenario_altitude(double t)
{ return t<120?0:t<125?(t-120)*60:t<155?(155-t)*10:0; }
static void send_pvt(uint64_t world_us,double height)
{
    uint8_t p[100]={0xb5,0x62,1,7,92,0}; uint8_t *v=p+6;
    put(v,(uint32_t)(world_us/1000u),4); put(v+4,2026,2); v[6]=10; v[7]=27;
    v[8]=1; v[9]=(uint8_t)((world_us/60000000u)%60u); v[10]=(uint8_t)((world_us/1000000u)%60u); v[11]=7;
    v[20]=3; v[21]=1; v[23]=14;
    put(v+24,1140000000u,4); put(v+28,220000000u,4); put(v+36,(uint32_t)(height*1000.0+10000.0),4);
    put(v+40,1000,4); put(v+44,1500,4); put(v+68,100,4); put(v+76,100,2);
    uint8_t a=0,b=0; for(size_t i=2;i<98;i++) { a=(uint8_t)(a+p[i]); b=(uint8_t)(b+a); } p[98]=a; p[99]=b;
    (void)board_sitl_gnss_receive(p,sizeof p);
}
void app_scenario_step(app_scenario_t *s,double t)
{
    uint64_t world_us=(uint64_t)(t*1000000.0+.5); board_sitl_inputs_t *in=board_sitl_inputs();
    double h=app_scenario_altitude(t);
    in->pressure_pa=cs_pressure_at(h,101325,283.15);
    /* ENU with body-to-world LEVEL=diag(1,-1,-1), nadir camera: stationary
     * specific force is -g on body z. Shocks are synthetic FSM stimuli. */
    in->accel_v[2]=(t>=120 && t<120.4)? -60.0f:(t>=155 && t<155.1)? -70.0f: -9.80665f;
    in->baro=!(s->sensor_failures && t>=130); in->temp=!(s->sensor_failures && t>=130);
    in->gyro=!(s->sensor_failures && t>=130); in->accel=!(s->sensor_failures && t>=130);
    if(s->gnss_enabled && in->gnss && board_sitl_stats()->baud==s->receiver_baud) {
        if(world_us>=s->next_pps) { (void)board_sitl_pps(board_tick()); s->next_pps=world_us+1000000u; }
        if(world_us>=s->next_gnss) { send_pvt(world_us,h); s->next_gnss=world_us+100000u; }
    }
    if(s->pi_enabled && board_pi_powered() && board_sitl_stats()->uart_enabled) {
        if(world_us>=s->next_pi) {
            cs_link_pi_status_t m={0}; uint8_t p[CS_LINK_PI_STATUS_SIZE];
            m.proto=CS_PROTO_VERSION; m.state=CS_PI_READY;
            m.flags=(uint16_t)(CS_PIFLAG_CAMERA|CS_PIFLAG_MAP|CS_PIFLAG_STORAGE|CS_PIFLAG_EPOCH_ECHO|((uint16_t)board_epoch()<<CS_PIFLAG_EPOCH_SHIFT));
            m.uptime_ms=(uint32_t)(world_us/1000u); m.tick_echo=board_tick(); m.echo_age_ms=0;
            (void)cs_link_pack_pi_status(p,sizeof p,&m); pi_send(s,CS_LINK_PI_STATUS_ID,p,sizeof p); s->next_pi=world_us+500000u;
        }
        if(world_us>=s->next_nav) {
            cs_link_nav_t m={0}; uint8_t p[CS_LINK_NAV_SIZE]; m.tick=board_tick(); m.flags=CS_FLAG_TRN_VALID|CS_FLAG_PRED_VALID|CS_FLAG_CAMERA_OK;
            m.trn_e=123; m.trn_n=45; m.landmarks=8; (void)cs_link_pack_nav(p,sizeof p,&m);
            pi_send(s,CS_LINK_NAV_ID,p,sizeof p); s->next_nav=world_us+200000u;
        }
        if(world_us>=s->next_cfg) {
            cs_link_trig_cfg_t m={t>=119 && t<160?1u:0u,0,200,100}; uint8_t p[CS_LINK_TRIG_CFG_SIZE];
            (void)cs_link_pack_trig_cfg(p,sizeof p,&m); pi_send(s,CS_LINK_TRIG_CFG_ID,p,sizeof p); s->next_cfg=world_us+1000000u;
        }
    }
}
