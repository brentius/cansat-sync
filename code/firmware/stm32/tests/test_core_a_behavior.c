/* GPL-3.0-only. CORE-A edge cases, asynchronous analytic motion, TX priority.
 * CS_SOURCES: cs_crc16.c cs_cobs.c cs_link.c cs_txqueue.c cs_imu.c cs_telemetry.c */
#include "cs_test.h"
#include "cs_crc16.h"
#include "cs_cobs.h"
#include "cs_link.h"
#include "cs_imu.h"
#include "cs_txqueue.h"
#include "cs_telemetry.h"

static void rounding_and_na(void)
{
    CS_CHECK_EQ(cs_link_scale_signed(-1.25,0.1,16),-12);
    CS_CHECK_EQ(cs_link_scale_signed(1.25,0.1,16),13);
    CS_CHECK_EQ(cs_link_scale_signed((double)NAN,0.1,16),-32768);
    CS_CHECK_EQ(cs_link_scale_signed(-1e100,0.1,16),-32767);
    CS_CHECK_EQ_U(cs_link_scale_unsigned(1e100,0.01,32),4294967294u);
    CS_CHECK_EQ_U(cs_link_scale_unsigned((double)NAN,0.1,8),255);
    CS_CHECK_EQ_U(cs_link_scale_unsigned(-1,0.1,16),0);
    CS_CHECK_EQ(cs_telemetry_i16(1.25,0.1),12);
    CS_CHECK_EQ(cs_telemetry_i16(-1.25,0.1),-12);
    CS_CHECK_EQ(cs_telemetry_i16(1.75,0.1),18);
    CS_CHECK_EQ(cs_telemetry_i16((double)NAN,1),-32768);
    CS_CHECK_EQ_U(cs_telemetry_u16(101325,1,50000),51325);
    CS_CHECK_EQ_U(cs_telemetry_u16((double)INFINITY,1,0),65535);
    CS_CHECK_EQ_U(cs_telemetry_u8(3.875,0.01,2.5),138);
    CS_CHECK_EQ_U(cs_link_angle(-0.1),248);
    CS_CHECK_EQ_U(cs_telemetry_angle(-0.1),248);
    CS_CHECK_EQ_U(cs_link_angle((double)NAN),0);
}

static void nav_age_and_crc(void)
{
    cs_telemetry_t t, back;
    cs_link_nav_t n={0}; uint8_t b[40];
    cs_telemetry_init(&t); t.flags=CS_FLAG_GPS_VALID; t.seq=99;
    n.tick=0xfffffff0u; n.flags=0xff; n.trn_e=123; n.trn_n=-88;
    n.trn_major_m=7; n.protection_m=9; n.pred_n=-99; n.landmarks=4;
    cs_apply_nav(&t,&n,0.001,984);
    CS_CHECK_EQ(t.trn_e,123); CS_CHECK_EQ_U(t.flags,0xf0); CS_CHECK_EQ_U(t.seq,99);
    cs_apply_nav(&t,&n,0.001,985);
    CS_CHECK_EQ(t.trn_e,-32768); CS_CHECK_EQ_U(t.trn_major_m,255);
    CS_CHECK_EQ_U(t.flags,0x10); CS_CHECK_EQ_U(t.landmarks,0);
    cs_apply_nav(&t,&n,-1,985); CS_CHECK_EQ(t.pred_n,-99);
    cs_apply_nav(&t,NULL,-1,985); CS_CHECK_EQ(t.pred_n,-32768);
    CS_CHECK_EQ_U(cs_telemetry_encode(b,39,&t),0);
    CS_CHECK_EQ_U(cs_telemetry_encode(b,40,&t),40);
    CS_CHECK(cs_telemetry_decode(&back,b,40));
    b[9]^=1; CS_CHECK(!cs_telemetry_decode(&back,b,40));
    b[9]^=1; b[0]=0; { uint16_t c=cs_crc16(b,38,65535); b[38]=(uint8_t)c; b[39]=(uint8_t)(c>>8); }
    CS_CHECK(!cs_telemetry_decode(&back,b,40));
    CS_CHECK(!cs_telemetry_decode(&back,b,39));
}

static void queue_priority_and_overflow(void)
{
    cs_txqueue_t q; size_t len; const uint8_t *p;
    uint8_t a[]={1,2,0}, b[]={3,4,0}, c[]={5,6,0}; size_t i;
    cs_txqueue_init(&q);
    CS_CHECK(cs_txqueue_push(&q,a,3,false));
    p=cs_txqueue_peek(&q,&len); CS_CHECK_EQ_U(len,3); CS_CHECK_EQ_U(p[0],1);
    CS_CHECK(cs_txqueue_consume(&q,1));
    CS_CHECK(cs_txqueue_push(&q,b,3,false)); CS_CHECK(cs_txqueue_push(&q,c,3,true));
    p=cs_txqueue_peek(&q,&len); CS_CHECK_EQ_U(len,2); CS_CHECK_EQ_U(p[0],2);
    CS_CHECK(!cs_txqueue_consume(&q,3)); CS_CHECK(cs_txqueue_consume(&q,2));
    p=cs_txqueue_peek(&q,&len); CS_CHECK_EQ_U(p[0],5); CS_CHECK(cs_txqueue_consume(&q,3));
    p=cs_txqueue_peek(&q,&len); CS_CHECK_EQ_U(p[0],3); CS_CHECK(cs_txqueue_consume(&q,3));
    CS_CHECK(cs_txqueue_peek(&q,&len)==NULL);
    for(i=0;i<CS_TXQUEUE_CAPACITY;i++) CS_CHECK(cs_txqueue_push(&q,a,3,false));
    CS_CHECK(!cs_txqueue_push(&q,b,3,false)); CS_CHECK_EQ_U(q.tx_dropped,1);
    CS_CHECK(cs_txqueue_push(&q,c,3,true)); CS_CHECK_EQ_U(q.tx_dropped,2);
    p=cs_txqueue_peek(&q,&len); CS_CHECK_EQ_U(p[0],5);
    cs_txqueue_init(&q);
    for(i=0;i<CS_TXQUEUE_CAPACITY;i++) CS_CHECK(cs_txqueue_push(&q,c,3,true));
    CS_CHECK(!cs_txqueue_push(&q,c,3,true)); CS_CHECK_EQ_U(q.tx_dropped,1);
}

static void cobs_capacity_and_encoder(void)
{
    uint8_t buf[300], out[300], back[300], malformed[]={0}, overrun[]={4,1};
    size_t n=99; cs_link_encoder_t e;
    memset(buf,1,sizeof buf);
    CS_CHECK(!cs_cobs_encode(buf,254,out,255,&n)); CS_CHECK_EQ_U(n,0);
    CS_CHECK(cs_cobs_encode(buf,254,out,256,&n)); CS_CHECK_EQ_U(n,256);
    CS_CHECK(!cs_cobs_decode(out,n,back,253,&n));
    CS_CHECK(!cs_cobs_decode(malformed,1,back,sizeof back,&n));
    CS_CHECK(!cs_cobs_decode(overrun,2,back,sizeof back,&n));
    cs_link_encoder_init(&e); e.seq=255;
    CS_CHECK_EQ_U(cs_link_encode(&e,out,1,0x90,NULL,0),0); CS_CHECK_EQ_U(e.seq,255);
    CS_CHECK_EQ_U(cs_link_encode(&e,out,sizeof out,0x90,NULL,0),6); CS_CHECK_EQ_U(e.seq,0);
    CS_CHECK_EQ_U(cs_link_frame(out,sizeof out,1,0,buf,250),0);
    CS_CHECK_EQ_U(cs_link_frame(out,sizeof out,1,0,buf,249),255);
}

static void async_analytic_motion(void)
{
    cs_imu_t s; cs_link_imu_t msg; uint32_t g=0,a=0, origin=0xffff0000u;
    const float rate[3]={0,0,2}, force[3]={3,0,9};
    unsigned snapshots=0;
    cs_imu_init(&s,7);
    while(g<=2002000u || a<=2002000u) {
        if(g<=a) { CS_CHECK(cs_imu_add_gyro(&s,origin+g,rate,0)); g+=1000; }
        else { CS_CHECK(cs_imu_add_accel(&s,origin+a,force,0)); a+=625; }
        if(g%10000==1000 && cs_imu_snapshot_latest(&s,&msg)) {
            double t=(double)(uint32_t)(msg.tick-origin)*1e-6;
            CS_CHECK_NEAR((double)msg.q[0]/1073741824.0,cos(t),1e-8);
            CS_CHECK_NEAR((double)msg.q[3]/1073741824.0,sin(t),1e-8);
            CS_CHECK_NEAR((double)msg.u[0]/65536.0,1.5*sin(2*t),2e-5);
            CS_CHECK_NEAR((double)msg.u[1]/65536.0,1.5*(1-cos(2*t)),2e-5);
            CS_CHECK_NEAR((double)msg.u[2]/65536.0,9*t,1e-5);
            CS_CHECK_EQ_U(msg.epoch,7); snapshots++;
        }
    }
    CS_CHECK(snapshots>190); CS_CHECK_EQ_U(s.rejected,0);
}

static void async_bracketing_counts_and_restart(void)
{
    cs_imu_t s; cs_link_imu_t msg;
    const float r[3]={0,0,0}, f0[3]={0,0,0}, f1[3]={2,0,0};
    cs_imu_init(&s,255);
    CS_CHECK(cs_imu_add_gyro(&s,0,r,0)); CS_CHECK(cs_imu_add_accel(&s,0,f0,0));
    CS_CHECK(cs_imu_add_gyro(&s,1000,r,CS_IMU_GYRO_SATURATED));
    CS_CHECK(!cs_imu_snapshot(&s,1000,&msg));
    CS_CHECK(cs_imu_add_accel(&s,1250,f1,CS_IMU_ACCEL_SATURATED));
    CS_CHECK(!cs_imu_snapshot(&s,1000,&msg));
    CS_CHECK(cs_imu_add_gyro(&s,2000,r,0));
    CS_CHECK(cs_imu_snapshot(&s,1000,&msg));
    CS_CHECK_NEAR((double)msg.u[0]/65536.0,0.0008,1e-5);
    CS_CHECK_EQ_U(msg.n_gyro,2); CS_CHECK_EQ_U(msg.n_accel,1);
    CS_CHECK((msg.flags&(CS_IMU_RESTART|CS_IMU_GYRO_SATURATED|CS_IMU_ACCEL_SATURATED))==19);
    CS_CHECK(!cs_imu_snapshot(&s,1000,&msg));
    CS_CHECK(!cs_imu_add_gyro(&s,1000,r,0)); CS_CHECK_EQ_U(s.rejected,1);
    cs_imu_init(&s,0); CS_CHECK(cs_imu_add_sample(&s,r,f0,0.001,0));
    s.u[0]=65536; s.u[1]=-32768; s.u[2]=100000.00001;
    CS_CHECK(cs_imu_snapshot(&s,0xffffffffu,&msg));
    CS_CHECK_EQ(msg.u[0],0); CS_CHECK_EQ(msg.u[1],(-2147483647-1));
    CS_CHECK_EQ(msg.u[2],-2036334591); /* retains a count at 100000 m/s */
    CS_CHECK_EQ_U(msg.epoch,0); CS_CHECK(msg.flags&CS_IMU_RESTART);
}

static void imu_bad_samples_overflow_and_counts(void)
{
    cs_imu_t s; cs_link_imu_t msg; size_t i;
    const float r[3]={0,0,0}, f[3]={0,0,9}, bad[3]={0,NAN,0};
    cs_imu_init(&s,1);
    for(i=0;i<300;i++) CS_CHECK(cs_imu_add_sample(&s,r,f,0.001,0));
    CS_CHECK(cs_imu_snapshot(&s,300000,&msg));
    CS_CHECK_EQ_U(msg.n_gyro,255); CS_CHECK_EQ_U(msg.n_accel,255);
    CS_CHECK(!cs_imu_add_sample(&s,bad,f,0.001,0));
    CS_CHECK(!cs_imu_add_sample(&s,r,f,(double)NAN,0));
    CS_CHECK(!cs_imu_add_sample(&s,r,f,1e300,0));
    CS_CHECK(cs_imu_snapshot(&s,301000,&msg));
    CS_CHECK_EQ_U(msg.n_gyro,0); CS_CHECK_EQ_U(msg.n_accel,0);
    CS_CHECK_EQ_U(msg.flags,CS_IMU_GYRO_FAULT|CS_IMU_ACCEL_FAULT);
    cs_imu_init(&s,2);
    for(i=0;i<CS_IMU_HISTORY;i++) CS_CHECK(cs_imu_add_accel(&s,(uint32_t)i*625,f,0));
    CS_CHECK(!cs_imu_add_accel(&s,40000,f,0)); CS_CHECK_EQ_U(s.rejected,1);
    CS_CHECK_EQ_U(s.pending_count,CS_IMU_HISTORY);
    CS_CHECK(cs_imu_add_gyro(&s,0,r,0));
    CS_CHECK(cs_imu_add_gyro(&s,1000,r,0));
    CS_CHECK(cs_imu_snapshot_latest(&s,&msg));
    CS_CHECK(msg.flags&CS_IMU_ACCEL_MISSED);
    CS_CHECK(!cs_imu_add_sample(&s,r,f,0.001,0));
}

static size_t callbacks;
static void count_frame(void *ctx,uint8_t id,uint8_t seq,const uint8_t *p,size_t len)
{
    (void)ctx;(void)id;(void)seq;(void)p;(void)len;callbacks++;
}
static void decoder_unknown_repeats_and_recovery(void)
{
    cs_link_decoder_t d; uint8_t wire[255], huge[1000]; size_t n;
    cs_link_decoder_init(&d,true);callbacks=0;
    n=cs_link_frame(wire,sizeof wire,0xf0,255,NULL,0);
    cs_link_decoder_feed(&d,wire,n,count_frame,NULL);
    cs_link_decoder_feed(&d,wire,n,count_frame,NULL);
    CS_CHECK_EQ_U(callbacks,2); CS_CHECK_EQ_U(d.stats.repeats,1); CS_CHECK_EQ_U(d.stats.unknown,2);
    cs_link_decoder_reset_sequence(&d);
    n=cs_link_frame(wire,sizeof wire,0xf0,17,NULL,0);
    cs_link_decoder_feed(&d,wire,n,count_frame,NULL); CS_CHECK_EQ_U(d.stats.lost,0);
    cs_link_decoder_feed(&d,NULL,1,count_frame,NULL); CS_CHECK_EQ_U(d.stats.ignored,1);
    memset(huge,1,sizeof huge);cs_link_decoder_feed(&d,huge,sizeof huge,count_frame,NULL);
    CS_CHECK_EQ_U(d.stats.too_long,1); CS_CHECK_EQ_U(d.used,0);
    cs_link_decoder_feed(&d,(const uint8_t *)"",1,count_frame,NULL);
    cs_link_decoder_feed(&d,wire,n,count_frame,NULL); CS_CHECK_EQ_U(callbacks,4);
    cs_link_decoder_feed(&d,(const uint8_t *)"\1",2,count_frame,NULL);
    CS_CHECK_EQ_U(d.stats.short_frame,1); CS_CHECK_EQ_U(d.stats.faults,0);
}

CS_MAIN(rounding_and_na,nav_age_and_crc,queue_priority_and_overflow,cobs_capacity_and_encoder,async_analytic_motion,async_bracketing_counts_and_restart,imu_bad_samples_overflow_and_counts,decoder_unknown_repeats_and_recovery)
