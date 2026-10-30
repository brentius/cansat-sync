/* GPL-3.0-only. */
#include "cs_flight_state.h"
#include "cs_atmos.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <math.h>
#include <string.h>
#else
extern double exp(double);
extern double fabs(double);
extern void *memset(void *,int,size_t);
extern void *memmove(void *,const void *,size_t);
#define isfinite(x) __builtin_isfinite(x)
#define NAN __builtin_nan("")
#endif
cs_fsm_config_t cs_fsm_default_config(void)
{
    cs_fsm_config_t c = {4*CS_G0,.15,30,20,.5,30,3,.25,1,283.15,
        3,3*CS_G0,5,.3,15,5,4*CS_G0,60,1,2,180};
    return c;
}
bool cs_fsm_init(cs_fsm_t *f, const cs_fsm_config_t *config)
{
    cs_fsm_config_t c = config ? *config : cs_fsm_default_config();
    if (!f || !isfinite(c.baseline_tau_s) || c.baseline_tau_s <= 0 ||
        !isfinite(c.baseline_delay_s) || c.baseline_delay_s < 0 || c.baseline_delay_s > 3 ||
        !isfinite(c.median_window_s) || c.median_window_s < 0 || c.median_window_s > .25 ||
        !isfinite(c.speed_window_s) || c.speed_window_s <= 0 || c.speed_window_s > 1 ||
        !isfinite(c.mean_temp_k) || c.mean_temp_k <= 0) return false;
    memset(f,0,sizeof *f); f->cfg=c;
    f->baseline_pa=f->altitude=f->vertical_speed=f->max_altitude=(double)NAN;
    f->t_launch=f->t_descent=f->t_landed=f->baseline_t=f->last_t=(double)NAN;
    for (unsigned i=0;i<4;i++) f->hold_since[i]=(double)NAN;
    return true;
}
bool cs_fsm_restore(cs_fsm_t *f, cs_flight_state_t state, double p0, double max_alt,
                    double launch, double descent, double landed)
{
    if (!f || state < CS_PAD || state > CS_LANDED ||
        (state >= CS_ASCENT && !isfinite(launch)) ||
        (state >= CS_DESCENT && (!isfinite(descent) || descent<launch)) ||
        (state == CS_LANDED && (!isfinite(landed) || landed<descent)) ||
        (isfinite(p0) && p0<=0)) return false;
    cs_fsm_config_t c=f->cfg;
    if (!cs_fsm_init(f,&c)) return false;
    f->state=state; f->baseline_pa=p0; f->max_altitude=max_alt;
    f->t_launch=launch; f->t_descent=descent; f->t_landed=landed;
    return true;
}
static void pop(cs_fsm_sample_t *a, size_t *n)
{ if (*n) { --*n; memmove(a,a+1,*n*sizeof *a); } }
static void push(cs_fsm_t *f, cs_fsm_sample_t *a, size_t *n, size_t cap, double t, double x)
{
    if (*n==cap) { pop(a,n); f->overflow++; }
    a[*n].t=t; a[*n].value=x; ++*n;
}
static bool hold(cs_fsm_t *f, unsigned k, double t, bool condition, double duration)
{
    if (!condition) { f->hold_since[k]=(double)NAN; return false; }
    if (!isfinite(f->hold_since[k])) f->hold_since[k]=t;
    return t-f->hold_since[k]>=duration;
}
static void go(cs_fsm_t *f, cs_flight_state_t state, cs_fsm_reason_t reason)
{ f->state=state; f->reason=reason; f->events++; }
cs_flight_state_t cs_fsm_update(cs_fsm_t *f, double t, double accel, double p)
{
    if (!f) return CS_PAD;
    if (!isfinite(t) || (isfinite(f->last_t) && t<f->last_t)) { f->invalid_time++; return f->state; }
    f->last_t=t;
    const cs_fsm_config_t *c=&f->cfg;
    if (f->state==CS_PAD && isfinite(p) && p>0) {
        push(f,f->pending,&f->npending,CS_FSM_PENDING_CAP,t,p);
        while (f->npending && f->pending[0].t<=t-c->baseline_delay_s) {
            cs_fsm_sample_t s=f->pending[0]; pop(f->pending,&f->npending);
            if (!isfinite(f->baseline_pa)) f->baseline_pa=s.value;
            else if (isfinite(f->baseline_t)) f->baseline_pa+=(1-exp(-(s.t-f->baseline_t)/c->baseline_tau_s))*(s.value-f->baseline_pa);
            f->baseline_t=s.t;
        }
    }
    if (isfinite(p) && p>0 && isfinite(f->baseline_pa))
        push(f,f->recent,&f->nrecent,CS_FSM_MEDIAN_CAP,t,cs_pressure_altitude(p,f->baseline_pa,c->mean_temp_k));
    while (f->nrecent && f->recent[0].t<t-c->median_window_s) pop(f->recent,&f->nrecent);
    if (!f->nrecent) { f->altitude=f->vertical_speed=(double)NAN; f->nhistory=0; }
    else {
        double ordered[CS_FSM_MEDIAN_CAP];
        for (size_t i=0;i<f->nrecent;i++) {
            size_t j=i; double x=f->recent[i].value;
            while(j && ordered[j-1]>x) { ordered[j]=ordered[j-1]; --j; }
            ordered[j]=x;
        }
        f->altitude=ordered[f->nrecent/2];
        push(f,f->history,&f->nhistory,CS_FSM_HISTORY_CAP,t,f->altitude);
        while(f->nhistory && f->history[0].t<t-c->speed_window_s) pop(f->history,&f->nhistory);
        double span=t-f->history[0].t;
        f->vertical_speed=span>=.8*c->speed_window_s ? (f->altitude-f->history[0].value)/span : (double)NAN;
    }
    double alt=f->altitude, speed=f->vertical_speed;
    bool ha=isfinite(alt), hc=isfinite(accel);
    if (f->state==CS_PAD) {
        bool ba=hold(f,0,t,hc && accel>c->launch_accel,c->launch_hold_s);
        bool bh=hold(f,1,t,ha && alt>c->launch_alt_m && isfinite(speed) && speed>c->launch_climb_rate,c->launch_alt_hold_s);
        if (ba || bh) {
            if (!isfinite(f->baseline_pa) && f->npending) f->baseline_pa=f->pending[0].value;
            f->npending=0; f->t_launch=t; f->max_altitude=ha?alt:(double)NAN;
            go(f,CS_ASCENT,ba?CS_REASON_ACCELERATION:CS_REASON_ALTITUDE);
        }
    } else if(f->state==CS_ASCENT) {
        if (ha && !(alt<=f->max_altitude)) f->max_altitude=alt;
        double since=t-f->t_launch;
        bool armed=since>=c->eject_lockout_s;
        bool falling=hold(f,2,t,armed && ha && alt<f->max_altitude-c->apogee_drop_m,c->apogee_hold_s);
        cs_fsm_reason_t reason=armed && hc && accel>c->eject_accel ? CS_REASON_ACCELERATION :
            falling ? CS_REASON_ALTITUDE : since>=c->ascent_timeout_s ? CS_REASON_TIMEOUT : CS_REASON_NONE;
        if(reason) { f->t_descent=t; go(f,CS_DESCENT,reason); }
    } else if(f->state==CS_DESCENT) {
        double since=t-f->t_descent;
        bool resting=hold(f,3,t,isfinite(speed) && fabs(speed)<c->rest_speed,c->rest_hold_s);
        cs_fsm_reason_t reason=since>=c->touchdown_lockout_s && (!ha || alt<c->touchdown_alt_m) && hc && accel>c->touchdown_accel ? CS_REASON_ACCELERATION :
            since>=c->touchdown_lockout_s && resting ? CS_REASON_ALTITUDE : since>=c->descent_timeout_s ? CS_REASON_TIMEOUT : CS_REASON_NONE;
        if(reason) { f->t_landed=t; go(f,CS_LANDED,reason); }
    }
    return f->state;
}
