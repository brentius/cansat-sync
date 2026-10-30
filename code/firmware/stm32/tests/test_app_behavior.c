/* GPL-3.0-only. Application integration, actual core + timed board model.
CS_SOURCES: app/app.c app/persist.c board/sitl/board.c hal/hal_logic.c sitl/scenario.c cs_crc16.c cs_cobs.c cs_link.c cs_txqueue.c cs_imu.c cs_telemetry.c cs_flight_state.c cs_atmos.c cs_lora.c cs_downlink.c cs_ms5611.c cs_ntc.c cs_ubx.c cs_geo.c cs_pimgr.c cs_log.c
*/
#include "cs_test.h"
#include "app.h"
#include "scenario.h"
static app_t app;
static app_scenario_t scenario;
static void start(uint32_t size)
{
    board_sitl_cold_start(size); board_init(); app_scenario_init(&scenario);
    board_sitl_set_sink(app_scenario_sink,&scenario); CS_CHECK(app_init(&app));
}
static void run(double begin,double end)
{
    uint64_t elapsed=(uint64_t)(begin*1000000.0),until=(uint64_t)(end*1000000.0);
    while(elapsed<until) {
        uint64_t before=board_time_us(); app_scenario_step(&scenario,(double)elapsed/1000000.0); app_run_once(&app);
        elapsed+=board_time_us()-before;
    }
}
static void send(uint8_t id,const uint8_t *p,size_t n)
{ uint8_t wire[255]; size_t z=cs_link_encode(&scenario.encoder,wire,sizeof wire,id,p,n); CS_CHECK(board_sitl_pi_receive(wire,z)); app_poll(&app); }
static void mount_independent_primary(void)
{
    start(BOARD_SITL_FLASH_MAX); scenario.pi_enabled=false; scenario.gnss_enabled=false;
    run(0,2); CS_CHECK(!app.log_ready); CS_CHECK(app.log_unavailable>100);
    CS_CHECK(app.log.mount_reads<=20000u); CS_CHECK(board_radio_tx_count()>0); CS_CHECK(app.temp_valid);
    run(2,5);
    CS_CHECK(app.pressure_valid); CS_CHECK(app.fsm.baseline_pa>101300); CS_CHECK(!app.temp_disabled);
    run(5,10); CS_CHECK(app.log_ready); CS_CHECK_EQ_U(app.log.mount_reads,65536);
    CS_CHECK_EQ_U(board_sitl_stats()->max_reads_per_poll,2); /* mount + erase-ahead after mount */
    CS_CHECK_EQ_U(app.log.dropped,0); CS_CHECK_EQ_U(app.log_queue_dropped,0);
    CS_CHECK(board_sitl_stats()->watchdog_max_gap_us<=800u);
}
static void full_flight_and_retention(void)
{
    start(BOARD_SITL_FLASH_MAX); run(0,470);
    CS_CHECK_EQ(app.fsm.state,CS_LANDED); CS_CHECK_EQ_U(scenario.state_mask,15);
    CS_CHECK(app.log.protected); CS_CHECK(app.origin_valid); CS_CHECK(!app.log.stopped);
    CS_CHECK_EQ_U(app.log.dropped,0); CS_CHECK_EQ_U(app.log_queue_dropped,0);
    CS_CHECK_EQ_U(board_sitl_stats()->flash_rule_errors,0); CS_CHECK_EQ_U(board_sitl_stats()->power_order_errors,0);
    CS_CHECK(scenario.frames[CS_LINK_TRIG_ID]>150); CS_CHECK(scenario.frames[CS_LINK_GNSS_ID]>4000);
    CS_CHECK_EQ_U(scenario.imu_restart,1); CS_CHECK_EQ_U(scenario.imu_missed,0); /* startup miss logged while OE disabled */
    CS_CHECK(scenario.prepare>0 && scenario.shutdown>0); CS_CHECK(!board_pi_powered());
    CS_CHECK(board_sitl_stats()->trigger_low); CS_CHECK(app.log_suppressed>10000);
    CS_CHECK_EQ_U(app.txqueue.tx_dropped,0); CS_CHECK_EQ_U(app.txqueue.count,0); CS_CHECK(app.tx_suppressed>0);
    CS_CHECK_EQ_U(scenario.radio_bad,0); CS_CHECK(scenario.radio_packets>60);
    CS_CHECK_EQ_U(app.fsm.overflow,0);
    CS_CHECK((double)app.temp_on_us/(double)board_time_us()<=.06);
}
static void nav_and_sensor_staleness(void)
{
    start(65536); run(0,4); app_build_packet(&app,&app.packet);
    CS_CHECK(app.packet.flags&CS_FLAG_TRN_VALID); CS_CHECK_EQ(app.packet.trn_e,123);
    /* 8000 mV 2S pack: packet carries 4.00 V per cell, not a saturated 5.04 V. */
    CS_CHECK_EQ_U(app.packet.battery_v,150u);
    scenario.pi_enabled=false; scenario.gnss_enabled=false; board_sitl_inputs()->baro=false; board_sitl_inputs()->temp=false;
    /* Run without scenario_step, which otherwise restores baro/temp controls. */
    for(unsigned i=0;i<7000;i++) app_run_once(&app);
    app_build_packet(&app,&app.packet); app_build_status(&app,&app.status);
    CS_CHECK_EQ_U(app.packet.flags&(CS_FLAG_TRN_VALID|CS_FLAG_PRED_VALID|CS_FLAG_CAMERA_OK|CS_FLAG_GPS_VALID),0);
    CS_CHECK_EQ(app.packet.trn_e,INT16_MIN); CS_CHECK_EQ(app.packet.temperature_c,INT16_MIN);
    CS_CHECK_EQ_U(app.packet.pressure_pa,UINT16_MAX);
    CS_CHECK_EQ_U(app.status.health&(CS_HEALTH_BARO|CS_HEALTH_TEMP|CS_HEALTH_GNSS|CS_HEALTH_FIX|CS_HEALTH_PI_ALIVE),0);
}
static void late_pi_heartbeat_recovers_on_powered_pad(void)
{
    start(65536); scenario.pi_enabled=false; run(0,1);
    CS_CHECK_EQ(app.fsm.state,CS_PAD); CS_CHECK(board_pi_powered());
    /* Sample silence boundaries and a slow 120 s startup without running a
     * multi-hour application replay. The pure FSM checks continuous power. */
    const uint32_t samples_ms[]={10101,12101,30000,60000,120000};
    for(unsigned i=0;i<sizeof samples_ms/sizeof samples_ms[0];i++) {
        board_sitl_set_time((uint64_t)samples_ms[i]*1000u); app_poll(&app);
        CS_CHECK(board_pi_powered()); CS_CHECK(app.uart_enabled);
        CS_CHECK(!app.pi_alive); CS_CHECK_EQ(app.fsm.state,CS_PAD);
    }
    scenario.pi_enabled=true; run(1,2); /* Still stationary PAD inputs. */
    CS_CHECK(board_pi_powered() && app.pi_alive && app.nav_epoch_ready);
    CS_CHECK_EQ(app.fsm.state,CS_PAD);
    app_build_status(&app,&app.status); app_build_packet(&app,&app.packet);
    CS_CHECK(app.status.health&CS_HEALTH_PI_ALIVE);
    CS_CHECK(app.packet.flags&CS_FLAG_TRN_VALID);
    CS_CHECK(app.pressure_valid && app.temp_valid); CS_CHECK(board_radio_tx_count()>0);
    /* Launch after recovery: boot and health already happened on PAD. */
    run(120,122); CS_CHECK_EQ(app.fsm.state,CS_ASCENT);
    CS_CHECK(board_pi_powered() && app.pi_alive);
    CS_CHECK_EQ_U(board_sitl_stats()->power_order_errors,0);
}
static void reset_midflight_and_corruption(void)
{
    start(BOARD_SITL_FLASH_MAX); run(0,136); CS_CHECK_EQ(app.fsm.state,CS_DESCENT);
    uint32_t protection=app.log.protected_start; double launch=app.fsm.t_launch,mission=app_mission_time(&app);
    uint16_t seq=app.telemetry_seq; double used=cs_duty_used(&app.downlink.limiter,mission);
    app_persist_save(&app); board_sitl_warm_reset(BOARD_RESET_WATCHDOG); board_init();
    cs_link_decoder_init(&scenario.decoder,false); CS_CHECK(app_init(&app));
    CS_CHECK(app.restored); CS_CHECK_EQ(app.fsm.state,CS_DESCENT); CS_CHECK_EQ(app.telemetry_seq,seq);
    CS_CHECK_NEAR(app.fsm.t_launch,launch,1e-9); CS_CHECK_NEAR(app_mission_time(&app),mission,1e-9);
    /* Histogram buckets round each up by <1 ms: restored use never understates. */
    { double restored=cs_duty_used(&app.downlink.limiter,mission); CS_CHECK(restored>=used-1e-9 && restored<=used+.008+1e-9); }
    run(136,170); CS_CHECK(app.log_ready); CS_CHECK_EQ_U(app.log.protected_start,protection);
    CS_CHECK_EQ_U(app.log_unavailable,0); CS_CHECK_EQ_U(app.log_queue_dropped,0);
    CS_CHECK(app.log.mount_reads<=128); CS_CHECK(!app.resume_failed);
    CS_CHECK_EQ(app.fsm.state,CS_LANDED); CS_CHECK_EQ_U(scenario.imu_restart,2);
    app_persist_save(&app); board_sitl_corrupt_persist(88,0xff);
    board_sitl_warm_reset(BOARD_RESET_SOFTWARE); board_init(); CS_CHECK(app_init(&app));
    CS_CHECK(!app.restored); CS_CHECK(app.persist_invalid);
    CS_CHECK(!cs_downlink_due(&app.downlink,app_mission_time(&app),CS_ASCENT));
    /* A second reset must not legitimize the prior invalid PAD assumption. */
    app_persist_save(&app); board_sitl_warm_reset(BOARD_RESET_WATCHDOG); board_init();
    CS_CHECK(app_init(&app)); CS_CHECK(app.persist_invalid); CS_CHECK(!app.restored);
}
static void warm_reset_ages_radio_airtime(void)
{
    /* An hour of 1 Hz PAD packets, then a warm reset. A single current-time
     * reservation would block ~296 s for another full hour; buckets let the
     * older half expire on schedule, never earlier than the true history. */
    start(65536); run(0,2);
    double air=app.downlink.airtime,now;
    app.mission_offset=3600.0-app_mission_time(&app)+app.mission_offset; now=app_mission_time(&app);
    CS_CHECK(cs_duty_init(&app.downlink.limiter,APP_RADIO_DUTY,APP_RADIO_WINDOW_S));
    for(unsigned i=1;i<=3600u;i++) CS_CHECK(cs_duty_record(&app.downlink.limiter,(double)i,air));
    double used=cs_duty_used(&app.downlink.limiter,now);
    app_persist_save(&app); board_sitl_warm_reset(BOARD_RESET_WATCHDOG); board_init();
    cs_link_decoder_init(&scenario.decoder,false); CS_CHECK(app_init(&app)); CS_CHECK(app.restored);
    CS_CHECK(cs_duty_used(&app.downlink.limiter,now)>=used-1e-9);
    double later=cs_duty_used(&app.downlink.limiter,now+1800.0),truth=1800.0*air;
    CS_CHECK(later>=truth-1e-9); CS_CHECK(later<=truth+450.0*air+.01);
    CS_CHECK(cs_duty_used(&app.downlink.limiter,now+3600.0)<1e-9);
}
static void power_on_ignores_backup(void)
{
    start(65536); run(0,4); app.telemetry_seq=900; app_persist_save(&app);
    board_sitl_warm_reset(BOARD_RESET_POWER); board_init(); CS_CHECK(app_init(&app));
    CS_CHECK(!app.restored); CS_CHECK(!app.persist_invalid); CS_CHECK_EQ_U(app.telemetry_seq,0);
}
static void temperature_fault_and_duty(void)
{
    start(65536); run(0,1); CS_CHECK(app.temp_valid);
    board_sitl_inputs()->temp_open=true;
    for(unsigned i=0;i<500;i++) app_run_once(&app);
    CS_CHECK(!app.temp_valid); CS_CHECK(app.temp_faults>0);
    board_sitl_inputs()->temp_open=false; board_sitl_inputs()->temp_excitation_us=9000;
    for(unsigned i=0;i<500;i++) app_run_once(&app);
    CS_CHECK(app.temp_disabled); CS_CHECK(!app.temp_valid);
}
static void triggers_and_shutdown(void)
{
    start(65536); run(0,1);
    cs_link_trig_cfg_t cfg={1,1,200,100}; uint8_t p[8];
    (void)cs_link_pack_trig_cfg(p,sizeof p,&cfg); send(CS_LINK_TRIG_CFG_ID,p,sizeof p);
    CS_CHECK(board_trig_running()); uint32_t starts=board_sitl_stats()->trigger_starts;
    send(CS_LINK_TRIG_CFG_ID,p,sizeof p); CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,starts);
    cfg.period_ms=1; (void)cs_link_pack_trig_cfg(p,sizeof p,&cfg); send(CS_LINK_TRIG_CFG_ID,p,sizeof p);
    CS_CHECK(app.trigger_rejected>0);
    scenario.ack_shutdown=false; app.shutdown_requested=true;
    for(unsigned i=0;i<65000;i++) app_run_once(&app);
    CS_CHECK(!board_pi_powered()); CS_CHECK(!board_trig_running()); CS_CHECK(board_sitl_stats()->trigger_low);
    CS_CHECK_EQ_U(board_sitl_stats()->power_order_errors,0);
}
static void raw_nor_and_wrap(void)
{
    start(65536); uint8_t p[2]={0x0f,0xf0},q[2]={0xff,0xff},r[2];
    CS_CHECK(board_flash_program(0,p,2)); CS_CHECK(board_flash_busy()); CS_CHECK(!board_flash_read(0,r,2));
    board_sitl_advance(1000); CS_CHECK(board_flash_read(0,r,2)); CS_CHECK_MEM(p,r,2);
    CS_CHECK(!board_flash_program(0,q,2)); CS_CHECK(!board_flash_program(255,p,2)); CS_CHECK(!board_flash_erase_sector(1));
    CS_CHECK(board_flash_erase_sector(0)); board_sitl_advance(45000); CS_CHECK(board_flash_read(0,r,2)); CS_CHECK_MEM(q,r,2);
    board_sitl_cold_start(65536); board_init(); board_sitl_set_time(UINT64_C(4294960000));
    app_scenario_init(&scenario); board_sitl_set_sink(app_scenario_sink,&scenario); CS_CHECK(app_init(&app));
    run(0,4); CS_CHECK_EQ_U(board_tick_hi(),1); CS_CHECK_EQ_U(scenario.imu_restart,1);
    CS_CHECK_EQ_U(scenario.imu_missed,0); CS_CHECK(app.pressure_valid && app.temp_valid);
}
static void flash_wrap_protection(void)
{
    static uint8_t protected_sector[4096];
    start(65536); run(0,20); CS_CHECK(board_sitl_stats()->flash_erases>0);
    CS_CHECK(board_sitl_stats()->flash_programs*256u>board_flash_size());
    board_sitl_inputs()->accel_v[2]=60;
    for(unsigned i=0;i<1000;i++) app_run_once(&app);
    CS_CHECK_EQ(app.fsm.state,CS_ASCENT); CS_CHECK(app.log.protected);
    uint32_t start_addr=app.log.protected_start;
    memcpy(protected_sector,board_sitl_flash_data()+start_addr,sizeof protected_sector);
    for(unsigned i=0;i<10000;i++) app_run_once(&app);
    CS_CHECK(app.log.stopped); CS_CHECK(app.log.full_drops>0);
    CS_CHECK_MEM(protected_sector,board_sitl_flash_data()+start_addr,sizeof protected_sector);
    CS_CHECK_EQ_U(board_sitl_stats()->flash_rule_errors,0);
}
static void low_battery_and_origin_freeze(void)
{
    start(BOARD_SITL_FLASH_MAX); run(0,121); CS_CHECK_EQ(app.fsm.state,CS_ASCENT);
    int32_t origin_h=app.origin_h; run(121,126); CS_CHECK_EQ(app.origin_h,origin_h);
    CS_CHECK(app.gnss.h_msl_m>origin_h);
    board_sitl_inputs()->battery_mv=6000;
    run(126,133); CS_CHECK(!app.shutdown_requested); CS_CHECK(board_pi_powered());
    /* Ground low voltage must request shutdown after the hold. */
    start(65536); board_sitl_inputs()->battery_mv=6000; run(0,8);
    CS_CHECK(app.shutdown_requested); CS_CHECK(!board_pi_powered());
}
static void nav_epoch_gate(void)
{
    start(65536); scenario.pi_enabled=false; run(0,1); CS_CHECK(!app.nav_epoch_ready);
    cs_link_nav_t nav={0}; uint8_t payload[32],wire[255]; nav.tick=board_tick(); nav.flags=CS_FLAG_TRN_VALID; nav.trn_e=123;
    (void)cs_link_pack_nav(payload,sizeof payload,&nav);
    size_t n=cs_link_encode(&scenario.encoder,wire,sizeof wire,CS_LINK_NAV_ID,payload,sizeof payload);
    CS_CHECK(board_sitl_pi_receive(wire,n/2)); app_poll(&app); CS_CHECK(!app.nav_valid);
    CS_CHECK(board_sitl_pi_receive(wire+n/2,n-n/2)); app_poll(&app); CS_CHECK(!app.nav_valid);
    cs_link_pi_status_t ps={0}; uint8_t status_bytes[24]; ps.proto=CS_PROTO_VERSION;
    ps.flags=(uint16_t)(CS_PIFLAG_EPOCH_ECHO|((uint16_t)(uint8_t)(board_epoch()+1u)<<CS_PIFLAG_EPOCH_SHIFT));
    (void)cs_link_pack_pi_status(status_bytes,sizeof status_bytes,&ps); send(CS_LINK_PI_STATUS_ID,status_bytes,sizeof status_bytes);
    send(CS_LINK_NAV_ID,payload,sizeof payload); CS_CHECK(!app.nav_epoch_ready && !app.nav_valid);
    ps.flags=(uint16_t)(CS_PIFLAG_EPOCH_ECHO|((uint16_t)board_epoch()<<CS_PIFLAG_EPOCH_SHIFT));
    (void)cs_link_pack_pi_status(status_bytes,sizeof status_bytes,&ps); send(CS_LINK_PI_STATUS_ID,status_bytes,sizeof status_bytes);
    CS_CHECK(app.nav_epoch_ready); send(CS_LINK_NAV_ID,payload,sizeof payload); CS_CHECK(app.nav_valid);
    app_build_packet(&app,&app.packet); CS_CHECK(app.packet.flags&CS_FLAG_TRN_VALID); CS_CHECK_EQ(app.packet.trn_e,123);
    /* Silence expiration is checked before old Nav can revive the alive timer. */
    board_sitl_advance(3100000); send(CS_LINK_NAV_ID,payload,sizeof payload);
    CS_CHECK(!app.nav_epoch_ready && !app.nav_valid);
    ps.flags=(uint16_t)((uint16_t)board_epoch()<<CS_PIFLAG_EPOCH_SHIFT);
    (void)cs_link_pack_pi_status(status_bytes,sizeof status_bytes,&ps); send(CS_LINK_PI_STATUS_ID,status_bytes,sizeof status_bytes);
    CS_CHECK(!app.nav_epoch_ready);
}
static void trigger_reconfiguration_discards_old_captures(void)
{
    start(65536); scenario.pi_enabled=false; run(0,1);
    cs_link_trig_cfg_t cfg={1,1,200,100}; uint8_t payload[8];
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    board_sitl_advance(10); app_poll(&app); CS_CHECK_EQ(app.frame,1);
    board_sitl_advance(210000); board_poll(); /* old pulse(s) captured, not yet drained */
    uint32_t previous_drops=board_stats()->trig_dropped;
    cfg.period_ms=100; cfg.pulse_us=200;
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK(board_stats()->trig_dropped>previous_drops);
    /* No old edge may consume the new run's first flag with new parameters. */
    CS_CHECK(app.trigger_first);
    CS_CHECK_EQ(app.frame,0); /* protocol counter is since the last actual start */
    board_sitl_advance(10); app_poll(&app);
    CS_CHECK(!app.trigger_first); /* first new edge occurs one timer quantum later */
    CS_CHECK_EQ(app.frame,1);
    CS_CHECK_EQ(app.trigger_period,100000); CS_CHECK_EQ(app.trigger_pulse,200);
    board_sitl_advance(100000); board_poll(); board_trig_stop(true);
    uint32_t tick; CS_CHECK(!board_trig_pop(&tick)); CS_CHECK(!board_trig_running());
}
typedef struct {
    cs_link_decoder_t decoder;
    cs_link_trig_t messages[8];
    unsigned count;
} trigger_capture_t;
static void capture_trigger(void *ctx,uint8_t id,uint8_t seq,const uint8_t *p,size_t n)
{
    trigger_capture_t *c=ctx; (void)seq;
    if(id==CS_LINK_TRIG_ID && c->count<8u && cs_link_unpack_trig(&c->messages[c->count],p,n)) c->count++;
}
static void capture_trigger_wire(void *ctx,int channel,const uint8_t *p,size_t n)
{
    trigger_capture_t *c=ctx;
    if(channel==BOARD_SITL_LINK) cs_link_decoder_feed(&c->decoder,p,n,capture_trigger,c);
}
static void drain_trigger_wire(void)
{ for(unsigned i=0;i<20;i++) app_run_once(&app); }
static void rounded_trigger_refresh(uint16_t period_ms,uint16_t pulse_us,
                                    uint32_t actual_period,uint16_t actual_pulse,uint32_t quantum)
{
    start(65536); run(0,1); scenario.pi_enabled=false;
    /* Remove synthetic SPI read-time charges for exact timer-boundary checks. */
    board_sitl_inputs()->flash=false;
    trigger_capture_t capture={0}; cs_link_decoder_init(&capture.decoder,false);
    scenario.capture=capture_trigger_wire; scenario.capture_ctx=&capture;
    cs_link_trig_cfg_t cfg={1,1,period_ms,pulse_us}; uint8_t payload[8];
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    uint64_t first_tick=board_time_us()+quantum;
    uint32_t starts=board_sitl_stats()->trigger_starts,drops=board_stats()->trig_dropped;
    CS_CHECK(board_trig_running()); CS_CHECK_EQ_U(app.trigger_period,actual_period);
    CS_CHECK_EQ_U(app.trigger_pulse,actual_pulse); CS_CHECK(app.trigger_first);
    send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,starts);
    CS_CHECK_EQ_U(app.frame,0); CS_CHECK(app.trigger_first);
    board_sitl_advance(quantum); app_poll(&app); drain_trigger_wire();
    CS_CHECK_EQ_U(app.frame,1); CS_CHECK(!app.trigger_first);
    CS_CHECK_EQ_U(capture.count,1);
    CS_CHECK_EQ_U(capture.messages[0].tick,first_tick);
    CS_CHECK_EQ_U(capture.messages[0].frame,0); CS_CHECK_EQ_U(capture.messages[0].flags,1);
    CS_CHECK_EQ_U(capture.messages[0].period_us,actual_period);
    CS_CHECK_EQ_U(capture.messages[0].pulse_us,actual_pulse);
    /* Queue an edge before receiving the identical refresh: it must survive. */
    board_sitl_set_time(first_tick+actual_period); board_poll();
    send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload); drain_trigger_wire();
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,starts);
    CS_CHECK_EQ_U(board_stats()->trig_dropped,drops);
    CS_CHECK_EQ_U(app.frame,2); CS_CHECK(!app.trigger_first);
    CS_CHECK_EQ_U(capture.count,2);
    CS_CHECK_EQ_U(capture.messages[1].tick,first_tick+actual_period);
    CS_CHECK_EQ_U(capture.messages[1].frame,1); CS_CHECK_EQ_U(capture.messages[1].flags,0);
    CS_CHECK_EQ_U(capture.messages[1].period_us,actual_period);
    CS_CHECK_EQ_U(capture.messages[1].pulse_us,actual_pulse);
    /* The next falling edge keeps the original phase, down to one microsecond. */
    board_sitl_set_time(first_tick+2u*actual_period-1u); app_poll(&app);
    CS_CHECK_EQ_U(app.frame,2);
    board_sitl_advance(1); app_poll(&app); drain_trigger_wire();
    CS_CHECK_EQ_U(app.frame,3); CS_CHECK_EQ_U(capture.count,3);
    CS_CHECK_EQ_U(capture.messages[2].tick,first_tick+2u*actual_period);
    CS_CHECK_EQ_U(capture.messages[2].frame,2); CS_CHECK_EQ_U(capture.messages[2].flags,0);
    scenario.capture=NULL; scenario.capture_ctx=NULL;
}
static void rounded_pulse_refresh_keeps_captures_and_phase(void)
{ rounded_trigger_refresh(200,101,200000,100,4); }
static void rounded_period_refresh_keeps_captures_and_phase(void)
{ rounded_trigger_refresh(400,98,400001,98,7); }
static void trigger_request_rejection_and_restart(void)
{
    start(65536); run(0,1); scenario.pi_enabled=false;
    cs_link_trig_cfg_t cfg={1,1,200,101}; uint8_t payload[8];
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    board_sitl_advance(10); app_poll(&app);
    uint32_t starts=board_sitl_stats()->trigger_starts,rejected=app.trigger_rejected;
    cfg.period_ms=1; /* App-invalid request must not replace the accepted request. */
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(app.trigger_rejected,++rejected); CS_CHECK(board_trig_running());
    cfg.period_ms=200;
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,starts); CS_CHECK_EQ_U(app.frame,1);
    CS_CHECK(!app.trigger_first);
    cfg.pulse_us=1; /* App-valid, but rounds to zero and fails at the board. */
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(app.trigger_rejected,++rejected); CS_CHECK(!board_trig_running());
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,starts);
    send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(app.trigger_rejected,++rejected); CS_CHECK(!board_trig_running());
    cfg.pulse_us=101;
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,++starts);
    CS_CHECK_EQ_U(app.frame,0); CS_CHECK(app.trigger_first);
    send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,starts);
    /* A changed request must restart even if it rounds to the same actual pulse. */
    cfg.pulse_us=100;
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,++starts);
    CS_CHECK_EQ_U(app.trigger_pulse,100); CS_CHECK_EQ_U(app.frame,0); CS_CHECK(app.trigger_first);
    cfg.enable=0;
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK(!board_trig_running()); CS_CHECK(!app.trigger_first);
    cfg.enable=1;
    (void)cs_link_pack_trig_cfg(payload,sizeof payload,&cfg); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,++starts);
    CS_CHECK_EQ_U(app.frame,0); CS_CHECK(app.trigger_first);
    /* A board stop also defeats the equality shortcut, despite a cached request. */
    board_trig_stop(true); send(CS_LINK_TRIG_CFG_ID,payload,sizeof payload);
    CS_CHECK_EQ_U(board_sitl_stats()->trigger_starts,++starts);
    CS_CHECK_EQ_U(app.frame,0); CS_CHECK(app.trigger_first);
}
CS_MAIN(late_pi_heartbeat_recovers_on_powered_pad,mount_independent_primary,full_flight_and_retention,nav_and_sensor_staleness,reset_midflight_and_corruption,warm_reset_ages_radio_airtime,power_on_ignores_backup,temperature_fault_and_duty,triggers_and_shutdown,raw_nor_and_wrap,flash_wrap_protection,low_battery_and_origin_freeze,nav_epoch_gate,trigger_reconfiguration_discards_old_captures,rounded_pulse_refresh_keeps_captures_and_phase,rounded_period_refresh_keeps_captures_and_phase,trigger_request_rejection_and_restart)
