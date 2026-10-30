/* GPL-3.0-only. Protocol section 8 state/timer paths, simulated inputs only.
 * CS_SOURCES: cs_pimgr.c */
#include "cs_test.h"
#include "cs_pimgr.h"
static cs_pimgr_output_t poll(cs_pimgr_t *m,cs_pimgr_input_t *in,uint32_t t)
{ in->now_ms=t; return cs_pimgr_poll(m,in); }
static void start_config(cs_pimgr_t *m,cs_pimgr_input_t *in,uint32_t t,const cs_pimgr_config_t *cfg)
{
    memset(in,0,sizeof *in); CS_CHECK(cs_pimgr_init(m,cfg,t,254));
    cs_pimgr_output_t o=poll(m,in,t); CS_CHECK(o.oe_high && !o.pwr_en);
    in->pgood=true; o=poll(m,in,t+1); CS_CHECK(o.oe_high && o.pwr_en);
    o=poll(m,in,t+100); CS_CHECK(o.oe_high);
    o=poll(m,in,t+101); CS_CHECK(!o.oe_high && o.pwr_en && !o.alive);
}
static void start(cs_pimgr_t *m,cs_pimgr_input_t *in,uint32_t t)
{ start_config(m,in,t,NULL); }
/* Silence recovery is an explicit bench policy, never the flight default. */
static void start_with_bench_recovery(cs_pimgr_t *m,cs_pimgr_input_t *in,uint32_t t)
{
    cs_pimgr_config_t cfg=cs_pimgr_default_config(); cfg.cycle_silence_ms=10000;
    start_config(m,in,t,&cfg);
}
CS_TEST(default_slow_boot_continuous_power)
{
    cs_pimgr_t m; cs_pimgr_input_t in; start(&m,&in,0);
    bool continuous=true;
    /* Observe every simulated millisecond of a 120 s Linux startup budget. */
    for(uint32_t t=102;t<=120101;t++) {
        cs_pimgr_output_t o=poll(&m,&in,t);
        continuous=continuous && o.pwr_en && !o.oe_high && !o.alive;
    }
    CS_CHECK(continuous); CS_CHECK_EQ(m.phase,CS_PI_RUNNING);
    in.rx_valid=true; cs_pimgr_output_t o=poll(&m,&in,120102);
    CS_CHECK(o.send_hello && o.alive && o.pwr_en); CS_CHECK_EQ(in.flight,CS_PAD);
}
CS_TEST(default_three_hour_pad_then_silent_flight)
{
    cs_pimgr_t m; cs_pimgr_input_t in; start(&m,&in,0);
    bool continuous=true;
    /* Full three-hour FSM interval; no hours-long application execution. */
    in.rx_valid=true;
    for(uint32_t t=201;t<=10800201;t+=100) {
        cs_pimgr_output_t o=poll(&m,&in,t);
        continuous=continuous && o.pwr_en && !o.oe_high && o.alive;
    }
    CS_CHECK(continuous);
    in.rx_valid=false;
    cs_pimgr_output_t o=poll(&m,&in,10803201);
    CS_CHECK(!o.alive && o.pwr_en); /* Health expires at 3 s; power does not. */
    continuous=true;
    for(uint32_t t=10803301;t<=10890201;t+=100) {
        in.flight=t<10830201?CS_PAD:(t<10860201?CS_ASCENT:CS_DESCENT);
        o=poll(&m,&in,t);
        continuous=continuous && o.pwr_en && !o.oe_high && !o.alive;
    }
    CS_CHECK(continuous); CS_CHECK(!m.flight_cycle_used);
    in.rx_valid=true; o=poll(&m,&in,10890301);
    CS_CHECK(o.alive && o.send_hello && o.pwr_en);
}
CS_TEST(default_silent_launch_across_wrap)
{
    cs_pimgr_t m; cs_pimgr_input_t in; uint32_t base=UINT32_MAX-500;
    start(&m,&in,base); bool continuous=true;
    /* No heartbeat has ever arrived, including at launch. */
    for(uint32_t dt=102;dt<=60102;dt+=100) {
        in.flight=dt<10002?CS_PAD:(dt<30002?CS_ASCENT:CS_DESCENT);
        cs_pimgr_output_t o=poll(&m,&in,base+dt);
        continuous=continuous && o.pwr_en && !o.oe_high && !o.alive;
    }
    CS_CHECK(continuous); CS_CHECK(!m.flight_cycle_used);
}
CS_TEST(cycle_config_validation)
{
    cs_pimgr_t m; cs_pimgr_config_t cfg=cs_pimgr_default_config();
    cfg.cycle_silence_ms=0; CS_CHECK(cs_pimgr_init(&m,&cfg,0,0));
    cfg.cycle_silence_ms=cfg.alive_ms-1; CS_CHECK(!cs_pimgr_init(&m,&cfg,0,0));
    cfg.cycle_silence_ms=cfg.alive_ms; CS_CHECK(cs_pimgr_init(&m,&cfg,0,0));
    cfg.cycle_silence_ms=UINT32_MAX; CS_CHECK(!cs_pimgr_init(&m,&cfg,0,0));
}
CS_TEST(opt_in_startup_and_alive)
{
    cs_pimgr_t m; cs_pimgr_input_t in; start_with_bench_recovery(&m,&in,0);
    in.rx_valid=true; cs_pimgr_output_t o=poll(&m,&in,200); CS_CHECK(o.send_hello && o.alive);
    o=poll(&m,&in,700); CS_CHECK(!o.send_hello && o.alive);
    in.rx_valid=false; o=poll(&m,&in,3700); CS_CHECK(!o.alive && o.pwr_en);
    in.rx_valid=true; o=poll(&m,&in,3701); CS_CHECK(o.send_hello && o.alive);
    in.rx_valid=false; o=poll(&m,&in,13701); CS_CHECK(o.oe_high && !o.pwr_en);
    o=poll(&m,&in,15700); CS_CHECK(!o.pwr_en);
    o=poll(&m,&in,15701); CS_CHECK(o.pwr_en && o.oe_high);
    o=poll(&m,&in,15801); CS_CHECK(o.pwr_en && !o.oe_high);
}
CS_TEST(opt_in_flight_cycle_policy)
{
    cs_pimgr_t m; cs_pimgr_input_t in; start_with_bench_recovery(&m,&in,0);
    in.rx_valid=true; (void)poll(&m,&in,1000); in.rx_valid=false; in.flight=CS_ASCENT;
    (void)poll(&m,&in,1100); cs_pimgr_output_t o=poll(&m,&in,20000); CS_CHECK(o.pwr_en && !o.oe_high);
    start_with_bench_recovery(&m,&in,0); in.flight=CS_ASCENT; (void)poll(&m,&in,5000);
    o=poll(&m,&in,10101); CS_CHECK(!o.pwr_en); CS_CHECK(m.flight_cycle_used);
    (void)poll(&m,&in,12101); (void)poll(&m,&in,12201);
    o=poll(&m,&in,50000); CS_CHECK(o.pwr_en); /* Only one in-flight attempt. */
}
CS_TEST(shutdown_ack_and_timeout)
{
    cs_pimgr_t m; cs_pimgr_input_t in; start(&m,&in,0);
    in.request_shutdown=true; cs_pimgr_output_t o=poll(&m,&in,1000);
    CS_CHECK(o.send_cmd && o.stop_triggers && o.pwr_en); CS_CHECK_EQ(o.cmd.cmd,CS_CMD_SHUTDOWN); CS_CHECK_EQ(o.cmd.cmd_seq,255);
    o=poll(&m,&in,1499); CS_CHECK(!o.send_cmd); o=poll(&m,&in,1500); CS_CHECK(o.send_cmd); CS_CHECK_EQ(o.cmd.cmd_seq,255);
    cs_link_ack_t ack={254,CS_CMD_SHUTDOWN,CS_ACK_DONE}; in.ack=&ack;
    o=poll(&m,&in,2000); CS_CHECK(o.send_cmd); CS_CHECK(!m.done);
    ack.cmd_seq=255; ack.result=CS_ACK_ACCEPTED;
    o=poll(&m,&in,2001); CS_CHECK(!o.send_cmd && o.pwr_en);
    ack.result=CS_ACK_DONE; o=poll(&m,&in,2500); CS_CHECK(o.pwr_en); CS_CHECK(m.done);
    /* Repeated DONE must not restart the 2 s wait. */
    o=poll(&m,&in,4499); CS_CHECK(o.pwr_en); o=poll(&m,&in,4500); CS_CHECK(!o.pwr_en && o.oe_high);
    start(&m,&in,0); in.request_shutdown=true; (void)poll(&m,&in,1000);
    o=poll(&m,&in,31000); CS_CHECK(!o.pwr_en && o.oe_high && o.stop_triggers);
}
CS_TEST(timer_wrap_and_pgood)
{
    cs_pimgr_t m; cs_pimgr_input_t in; uint32_t t=UINT32_MAX-500; start(&m,&in,t);
    in.rx_valid=true; cs_pimgr_output_t o=poll(&m,&in,t+200); CS_CHECK(o.alive);
    in.rx_valid=false; o=poll(&m,&in,t+3200); CS_CHECK(!o.alive && o.pwr_en);
    CS_CHECK(cs_pimgr_init(&m,NULL,0,0)); memset(&in,0,sizeof in); in.pgood=true;
    (void)poll(&m,&in,0); in.pgood=false; o=poll(&m,&in,50); CS_CHECK(o.oe_high && !o.pwr_en);
}
CS_MAIN(default_slow_boot_continuous_power,default_three_hour_pad_then_silent_flight,default_silent_launch_across_wrap,cycle_config_validation,opt_in_startup_and_alive,opt_in_flight_cycle_policy,shutdown_ack_and_timeout,timer_wrap_and_pgood)
