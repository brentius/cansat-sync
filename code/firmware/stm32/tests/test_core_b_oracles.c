/* GPL-3.0-only. Synthetic reference comparisons, not sensor measurements.
 * CS_SOURCES: cs_atmos.c cs_flight_state.c cs_lora.c cs_downlink.c cs_geo.c cs_ntc.c */
#include "cs_test.h"
#include <stdlib.h>
#include "cs_atmos.h"
#include "cs_flight_state.h"
#include "cs_lora.h"
#include "cs_downlink.h"
#include "cs_geo.h"
#include "cs_ntc.h"
#include "vectors_core_b.h"
static void same(double a,double b,double tol) { if(isnan(b)) CS_CHECK(isnan(a)); else CS_CHECK_NEAR(a,b,tol); }
CS_TEST(atmosphere)
{
    for(int h=-100;h<=12000;h+=10) {
        double p=cs_pressure_at((double)h,101325,283.15);
        CS_CHECK_NEAR(cs_pressure_altitude(p,101325,283.15),(double)h,1e-9);
    }
    CS_CHECK_NEAR(cs_air_density(101325,288.15),1.22501226599,1e-10);
}
CS_TEST(fsm_python)
{
    static cs_fsm_t f; unsigned scenario=UINT32_MAX;
    for(size_t i=0;i<sizeof fsm_vectors/sizeof fsm_vectors[0];i++) {
        const fsm_vector_t *v=&fsm_vectors[i];
        if(v->scenario!=scenario) { CS_CHECK(cs_fsm_init(&f,NULL)); scenario=v->scenario; }
        CS_CHECK_EQ(cs_fsm_update(&f,v->t,v->accel,v->p),v->state);
        CS_CHECK_EQ(f.events,v->events); CS_CHECK_EQ(f.reason,v->reason);
        same(f.baseline_pa,v->baseline,1e-7); same(f.altitude,v->altitude,1e-7);
        same(f.vertical_speed,v->speed,1e-7); same(f.max_altitude,v->maximum,1e-7);
        CS_CHECK_EQ(f.overflow,0);
    }
}
CS_TEST(fsm_restore_and_bounds)
{
    static cs_fsm_t f;
    CS_CHECK(cs_fsm_init(&f,NULL));
    CS_CHECK(cs_fsm_restore(&f,CS_ASCENT,101325,300,-16,(double)NAN,(double)NAN));
    CS_CHECK_EQ(cs_fsm_update(&f,0,(double)NAN,(double)NAN),CS_DESCENT); CS_CHECK_EQ(f.reason,CS_REASON_TIMEOUT);
    CS_CHECK(cs_fsm_restore(&f,CS_DESCENT,101325,300,-200,-181,(double)NAN));
    CS_CHECK_EQ(cs_fsm_update(&f,0,(double)NAN,(double)NAN),CS_LANDED);
    CS_CHECK(!cs_fsm_restore(&f,CS_ASCENT,101325,0,(double)NAN,(double)NAN,(double)NAN));
    CS_CHECK(cs_fsm_restore(&f,CS_PAD,101325,0,(double)NAN,(double)NAN,(double)NAN));
    for(unsigned i=0;i<40;i++) (void)cs_fsm_update(&f,(double)i*.1,(double)NAN,101000);
    CS_CHECK(isfinite(f.baseline_pa)); CS_CHECK(f.baseline_pa<101325);
    cs_fsm_config_t c=cs_fsm_default_config(); c.speed_window_s=2;
    CS_CHECK(!cs_fsm_init(&f,&c)); CS_CHECK(cs_fsm_init(&f,NULL));
    for(unsigned i=0;i<2000;i++) (void)cs_fsm_update(&f,(double)i*.001,(double)NAN,101325);
    CS_CHECK(f.overflow>0); CS_CHECK(f.npending<=CS_FSM_PENDING_CAP);
    unsigned bad=f.invalid_time; (void)cs_fsm_update(&f,0,(double)NAN,(double)NAN); CS_CHECK_EQ(f.invalid_time,bad+1);
}
CS_TEST(lora_python)
{
    for(size_t i=0;i<sizeof lora_vectors/sizeof lora_vectors[0];i++) {
        const lora_vector_t *v=&lora_vectors[i];
        cs_lora_config_t c={v->sf,v->cr,8,v->bw,v->crc!=0,v->implicit!=0,v->ldro};
        CS_CHECK_NEAR(cs_lora_time_on_air(v->bytes,&c),v->toa,1e-12);
    }
    cs_lora_config_t c=cs_lora_default_config(); c.sf=5; CS_CHECK(isnan(cs_lora_time_on_air(40,&c)));
}
CS_TEST(downlink_python)
{
    static cs_downlink_t s; CS_CHECK(cs_downlink_init(&s,.06,.01,3600,1,60,0));
    size_t n=0;
    for(unsigned i=0;i<127200;i++) {
        cs_flight_state_t state=i<108000?CS_PAD:i<109200?CS_DESCENT:CS_LANDED;
        bool expect=n<sizeof downlink_due_ticks/sizeof downlink_due_ticks[0] && downlink_due_ticks[n]==i;
        bool due=cs_downlink_due(&s,(double)i*.1,state); CS_CHECK_EQ(due,expect); if(expect) n++;
    }
    CS_CHECK_EQ(n,sizeof downlink_due_ticks/sizeof downlink_due_ticks[0]);
    CS_CHECK(!cs_downlink_init(&s,.1,.001,3600,1,60,0));
}
CS_TEST(duty_fail_closed)
{
    static cs_duty_t d; CS_CHECK(cs_duty_init(&d,1,3600));
    for(unsigned i=0;i<CS_DUTY_CAP;i++) CS_CHECK(cs_duty_record(&d,0,.001));
    CS_CHECK(!cs_duty_allowed(&d,0,.001,0)); CS_CHECK(!cs_duty_record(&d,0,.001));
    CS_CHECK_NEAR(cs_duty_used(&d,3599),4.096,1e-10);
    CS_CHECK_NEAR(cs_duty_used(&d,3600),0,0); CS_CHECK(cs_duty_record(&d,3600,1));
    CS_CHECK(!cs_duty_allowed(&d,3599,1,0));
}
CS_TEST(geodesy_python)
{
    for(size_t i=0;i<sizeof geo_vectors/sizeof geo_vectors[0];i++) {
        const geo_vector_t *v=&geo_vectors[i]; double enu[3];
        CS_CHECK(cs_geo_enu_from_link(v->lat,v->lon,v->h,v->lat0,v->lon0,v->h0,enu));
        CS_CHECK_NEAR(enu[0],v->e,1e-8); CS_CHECK_NEAR(enu[1],v->n,1e-8); CS_CHECK_NEAR(enu[2],v->u,1e-8);
    }
    double out[3]; CS_CHECK(!cs_geo_enu_from_link(INT32_MAX,0,0,0,0,0,out));
}
CS_TEST(ntc_python)
{
    for(size_t i=0;i<sizeof ntc_vectors/sizeof ntc_vectors[0];i++) {
        const ntc_vector_t *v=&ntc_vectors[i]; cs_ntc_config_t c=cs_ntc_default_config(); cs_ntc_result_t out;
        c.use_sh=v->sh!=0; c.sh_a=ntc_sh_coeff[0]; c.sh_b=ntc_sh_coeff[1]; c.sh_c=ntc_sh_coeff[2];
        CS_CHECK_EQ(cs_ntc_convert(&c,v->a,v->pga0,v->b,v->pga1,&out),CS_NTC_OK);
        CS_CHECK_NEAR(out.resistance,v->r,.02);
        CS_CHECK(abs((int)out.temperature_centi_c-v->centi)<=1);
    }
}
CS_TEST(ntc_faults_and_duty)
{
    cs_ntc_config_t c=cs_ntc_default_config(); cs_ntc_result_t r;
    CS_CHECK(c.use_sh);
    CS_CHECK_EQ(cs_ntc_convert(&c,3000,2,24000,1,&r),CS_NTC_OK);
    CS_CHECK_EQ(r.temperature_centi_c,2500);
    CS_CHECK_EQ(cs_ntc_convert(&c,0,2,24000,1,&r),CS_NTC_SHORT);
    CS_CHECK_EQ(cs_ntc_convert(&c,16000,1,16000,1,&r),CS_NTC_OPEN);
    CS_CHECK_EQ(cs_ntc_convert(&c,32767,2,24000,1,&r),CS_NTC_SATURATED);
    CS_CHECK_EQ(cs_ntc_convert(&c,3000,2,0,1,&r),CS_NTC_DOMAIN);
    CS_CHECK_EQ(cs_ntc_convert(&c,3000,8,24000,1,&r),CS_NTC_CONFIG);
    cs_ntc_duty_t d; CS_CHECK(cs_ntc_duty_init(&d,.06f));
    for(unsigned i=0;i<100;i++) {
        uint64_t t=(uint64_t)i*100000;
        CS_CHECK(cs_ntc_duty_begin(&d,t,4634));
        CS_CHECK(!cs_ntc_duty_begin(&d,t+1,4634));
        cs_ntc_duty_finish(&d,t+4634);
        CS_CHECK(!cs_ntc_duty_begin(&d,t+5000,4634));
    }
    CS_CHECK_EQ_U(d.total_on_us,463400); CS_CHECK_EQ(d.overruns,0);
    CS_CHECK(cs_ntc_duty_begin(&d,10000000,4634)); cs_ntc_duty_finish(&d,10005000);
    CS_CHECK_EQ(d.overruns,1); CS_CHECK(!cs_ntc_duty_begin(&d,20000000,4634));
    CS_CHECK(!cs_ntc_duty_init(&d,.061f));
}
CS_MAIN(atmosphere,fsm_python,fsm_restore_and_bounds,lora_python,downlink_python,duty_fail_closed,geodesy_python,ntc_python,ntc_faults_and_duty)
