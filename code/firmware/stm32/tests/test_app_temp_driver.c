/* GPL-3.0-only. Verify default app policy against actual temperature driver.
CS_SOURCES: app/app.c app/persist.c drivers/drv_sensors.c hal/hal_logic.c cs_crc16.c cs_cobs.c cs_link.c cs_txqueue.c cs_imu.c cs_telemetry.c cs_flight_state.c cs_atmos.c cs_lora.c cs_downlink.c cs_ms5611.c cs_ntc.c cs_ubx.c cs_geo.c cs_pimgr.c cs_log.c
*/
#include "cs_test.h"
#include "app.h"
#include "drv_sensors.h"
static uint32_t excite(void *ctx,bool on,uint32_t budget) { (void)ctx; (void)on; (void)budget; return 0; }
static bool is_on(void *ctx) { (void)ctx; return false; }
static void default_schedule(void)
{
    drv_temp_t d; drv_i2c_t bus={0}; drv_excitation_t gate={NULL,excite,is_on};
    drv_temp_init(&d,bus,gate);
    board_temp_cfg_t c={APP_TEMP_PERIOD_US,APP_TEMP_SETTLE_US,APP_TEMP_SPS,2048,4096};
    CS_CHECK(drv_temp_configure(&d,&c,0)); CS_CHECK_EQ_U(d.budget,6256);
    CS_CHECK((uint64_t)d.budget*100u<=(uint64_t)c.period_us*6u);
    CS_CHECK_EQ_U(c.settle_us,2300); CS_CHECK_EQ_U(c.sps,860); CS_CHECK_EQ_U(c.period_us,125000);
    c.period_us=100000; CS_CHECK(!drv_temp_configure(&d,&c,0));
}
/* Use the actual ADS1115 sequencing for the app-facing temperature methods;
 * all other board services are the timed SITL implementation. No successful
 * synthetic temperature generator substitutes for the real driver here. */
#define board_temp_configure sitl_unused_temp_configure
#define board_temp_pop sitl_unused_temp_pop
#define board_temp_ok sitl_unused_temp_ok
#include "../board/sitl/board.c"
#undef board_temp_configure
#undef board_temp_pop
#undef board_temp_ok

static drv_temp_t test_driver;
static app_t test_app;
static bool test_gate_on;
static uint32_t test_gate_start,test_gate_budget,test_done,test_first_read;
static uint8_t test_reg,test_channel;
static unsigned test_samples,test_injected;
static uint32_t test_injected_on;
static uint32_t test_excite(void *ctx,bool on,uint32_t budget)
{
    (void)ctx;
    if(on) { test_gate_on=true; test_gate_start=board_tick(); test_gate_budget=budget; return 0; }
    uint32_t elapsed=board_tick()-test_gate_start;
    test_gate_on=false; return elapsed>test_gate_budget?test_gate_budget:elapsed;
}
static bool test_gate_active(void *ctx)
{ (void)ctx; return test_gate_on && board_tick()-test_gate_start<test_gate_budget; }
static bool test_bus_start(void *ctx,uint8_t address,const uint8_t *tx,size_t nt,size_t nr)
{
    (void)ctx; (void)address; (void)nr; test_reg=tx[0];
    if(nt==3) test_channel=((tx[1]>>4)&7u)==5u;
    test_done=board_tick()+100u; return true;
}
static int test_bus_result(void *ctx,uint8_t *rx,size_t n)
{
    (void)ctx; if((int32_t)(board_tick()-test_done)<0) return 0;
    if(rx && n==2) {
        uint16_t code=test_reg==1?0x8000u:(test_channel?26400u:10560u);
        rx[0]=(uint8_t)(code>>8); rx[1]=(uint8_t)code;
    }
    return 1;
}
bool board_temp_configure(const board_temp_cfg_t *cfg)
{ return drv_temp_configure(&test_driver,cfg,board_tick()); }
bool board_temp_ok(void) { return test_driver.ok; }
bool board_temp_pop(board_temp_raw_t *raw)
{
    if(test_injected) {
        test_injected--;
        *raw=(board_temp_raw_t){board_tick(),10560,26400,2048,4096,test_injected_on};
        return true;
    }
    if(!drv_temp_poll(&test_driver,board_tick(),raw)) return false;
    if(!test_samples) test_first_read=board_tick();
    test_samples++; return true;
}
static void start_driver_app(void)
{
    board_sitl_cold_start(4096); board_init();
    board_sitl_inputs()->gyro=false; board_sitl_inputs()->accel=false;
    test_gate_on=false; test_samples=test_injected=0; test_first_read=0;
    drv_temp_init(&test_driver,(drv_i2c_t){0,test_bus_start,test_bus_result},
                  (drv_excitation_t){0,test_excite,test_gate_active});
    CS_CHECK(app_init(&test_app));
}
CS_TEST(immediate_first_driver_conversion_remains_healthy)
{
    start_driver_app();
    while(board_time_us()<400000u) { app_poll(&test_app); board_sitl_advance(20); }
    CS_CHECK(test_first_read>0 && test_first_read<APP_TEMP_PERIOD_US);
    CS_CHECK(test_samples>=3); CS_CHECK(!test_app.temp_disabled && test_app.temp_valid);
    CS_CHECK_EQ(test_driver.cfg.period_us,APP_TEMP_PERIOD_US);
    CS_CHECK_EQ(test_app.temp_faults,0);
    CS_CHECK(test_app.temp_on_us<4u*6256u);
}
CS_TEST(pulse_overrun_and_excess_frequency_still_disable_excitation)
{
    start_driver_app(); test_injected=1; test_injected_on=APP_TEMP_MAX_ON_US+1u;
    app_poll(&test_app); CS_CHECK(test_app.temp_disabled); CS_CHECK_EQ(test_driver.cfg.period_us,0);
    start_driver_app(); test_injected=3; test_injected_on=6256;
    app_poll(&test_app); CS_CHECK(test_app.temp_disabled); CS_CHECK_EQ(test_driver.cfg.period_us,0);
}
CS_MAIN(default_schedule,immediate_first_driver_conversion_remains_healthy,pulse_overrun_and_excess_frequency_still_disable_excitation)
