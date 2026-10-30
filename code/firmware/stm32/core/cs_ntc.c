/* GPL-3.0-only. */
#include "cs_ntc.h"
#include <stddef.h>
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <math.h>
#include <string.h>
#else
extern float logf(float);
extern float floorf(float);
extern double ceil(double);
extern void *memset(void *,int,size_t);
#define isfinite(x) __builtin_isfinite(x)
#define NAN __builtin_nanf("")
#endif
cs_ntc_config_t cs_ntc_default_config(void)
{
    /* Nominal three-point SH fit from sim/temperature/models.py, not per-probe
     * calibration. Beta remains available. Fault limits are [assumed]. */
    cs_ntc_config_t c={150000.0f,10000.0f,3976.0f,0.00112911968419f,
        0.000234128784187f,8.76609546711e-08f,100.0f,2000000.0f,true}; return c;
}
float cs_ntc_code_volts(int16_t code,unsigned pga)
{
    /* TI ADS111x SBAS444E, Table 7-1 and Config register PGA[2:0], read at
     * https://www.ti.com/lit/ds/symlink/ads1115.pdf (Rev E, Dec 2024). */
    static const float fs[8]={6.144f,4.096f,2.048f,1.024f,.512f,.256f,.256f,.256f};
    return pga<8 ? (float)code*fs[pga]/32768.0f : NAN;
}
cs_ntc_fault_t cs_ntc_convert(const cs_ntc_config_t *c,int16_t a,unsigned pga0,int16_t b,unsigned pga1,cs_ntc_result_t *out)
{
    if(!out) return CS_NTC_CONFIG;
    memset(out,0,sizeof *out); out->temperature_centi_c=INT16_MIN;
    out->fault=CS_NTC_CONFIG;
    if(!c || pga0>7 || pga1>7 || !isfinite(c->r_ref) || c->r_ref<=0 ||
        !isfinite(c->min_ohm) || c->min_ohm<0 || !isfinite(c->max_ohm) || c->max_ohm<=c->min_ohm) return out->fault;
    out->v_ntc=cs_ntc_code_volts(a,pga0); out->v_exc=cs_ntc_code_volts(b,pga1);
    if(a==INT16_MIN || a==INT16_MAX || b==INT16_MIN || b==INT16_MAX) return out->fault=CS_NTC_SATURATED;
    if(out->v_exc<=0) return out->fault=CS_NTC_DOMAIN;
    if(out->v_ntc<=0) return out->fault=CS_NTC_SHORT;
    if(out->v_ntc>=out->v_exc) return out->fault=CS_NTC_OPEN;
    float r=c->r_ref*out->v_ntc/(out->v_exc-out->v_ntc); out->resistance=r;
    if(r<c->min_ohm) return out->fault=CS_NTC_SHORT;
    if(r>c->max_ohm) return out->fault=CS_NTC_OPEN;
    float inv;
    if(c->use_sh) { float x=logf(r); inv=c->sh_a+c->sh_b*x+c->sh_c*x*x*x; }
    else {
        if(!isfinite(c->r25) || c->r25<=0 || !isfinite(c->beta) || c->beta<=0) return out->fault;
        inv=1.0f/298.15f+logf(r/c->r25)/c->beta;
    }
    if(!isfinite(inv) || inv<=0) return out->fault=CS_NTC_DOMAIN;
    float centi=floorf((1.0f/inv-273.15f)*100.0f+.5f);
    if(!isfinite(centi) || centi<=-32768.0f || centi>32767.0f) return out->fault=CS_NTC_DOMAIN;
    out->temperature_centi_c=(int16_t)centi; return out->fault=CS_NTC_OK;
}
bool cs_ntc_duty_init(cs_ntc_duty_t *d,float ceiling)
{
    if(!d || !isfinite(ceiling) || ceiling<=0 || ceiling>.06f) return false;
    memset(d,0,sizeof *d); d->ceiling=ceiling; return true;
}
bool cs_ntc_duty_begin(cs_ntc_duty_t *d,uint64_t now,uint32_t on)
{
    if(!d || !on || d->active || now<d->next_us || d->ceiling<=0 || d->ceiling>.06f) { if(d) d->denied++; return false; }
    /* double is needed for microsecond ceiling arithmetic at extended ticks. */
    uint64_t period=(uint64_t)ceil((double)on/(double)d->ceiling);
    if(now>UINT64_MAX-period) { d->denied++; return false; }
    d->active=true; d->started_us=now; d->reserved_us=on; d->next_us=now+period; return true;
}
void cs_ntc_duty_finish(cs_ntc_duty_t *d,uint64_t now)
{
    if(!d || !d->active) return;
    uint64_t on=now>=d->started_us?now-d->started_us:UINT64_MAX;
    if(UINT64_MAX-d->total_on_us<on) d->total_on_us=UINT64_MAX; else d->total_on_us+=on;
    if(on>d->reserved_us) { d->overruns++; d->next_us=UINT64_MAX; }
    d->active=false;
}
