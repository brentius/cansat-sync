/* GPL-3.0-only. */
#include "cs_lora.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <math.h>
#include <string.h>
#else
extern double ceil(double);
extern double fmax(double,double);
extern void *memset(void *,int,size_t);
#define isfinite(x) __builtin_isfinite(x)
#define NAN __builtin_nan("")
#define INFINITY __builtin_inf()
#endif
cs_lora_config_t cs_lora_default_config(void)
{ cs_lora_config_t c={7,1,8,125000,true,false,-1}; return c; }
double cs_lora_time_on_air(size_t bytes, const cs_lora_config_t *config)
{
    cs_lora_config_t c=config?*config:cs_lora_default_config();
    if(c.sf<6 || c.sf>12 || c.coding_rate<1 || c.coding_rate>4 || !isfinite(c.bw_hz) || c.bw_hz<=0 || c.ldro < -1 || c.ldro>1) return (double)NAN;
    double symbol=(double)(1u<<c.sf)/c.bw_hz;
    bool ldro=c.ldro<0 ? symbol>.016 : c.ldro!=0;
    double numerator=8*(double)bytes-4*(double)c.sf+28+16*(double)c.crc-20*(double)c.implicit_header;
    double blocks=ceil(numerator/(4*((double)c.sf-2*(double)ldro)));
    double payload=8+fmax(blocks*((double)c.coding_rate+4),0);
    return ((double)c.preamble+4.25+payload)*symbol;
}
bool cs_duty_init(cs_duty_t *d, double limit, double window)
{
    if(!d || !isfinite(limit) || limit<=0 || limit>1 || !isfinite(window) || window<=0) return false;
    memset(d,0,sizeof *d); d->duty_limit=limit; d->window_s=window; d->last_time=-(double)INFINITY; return true;
}
static bool expire(cs_duty_t *d,double now)
{
    if(!isfinite(now) || now<d->last_time) return false;
    d->last_time=now;
    while(d->count && d->time[d->head]<=now-d->window_s) {
        d->used_s-=d->airtime[d->head]; d->head=(d->head+1)%CS_DUTY_CAP; --d->count;
    }
    if(!d->count) d->used_s=0;
    return true;
}
double cs_duty_used(cs_duty_t *d,double now) { return d && expire(d,now)?d->used_s:(double)NAN; }
double cs_duty_remaining(cs_duty_t *d,double now) { return d?d->duty_limit*d->window_s-cs_duty_used(d,now):(double)NAN; }
bool cs_duty_allowed(cs_duty_t *d,double now,double airtime,double reserve)
{
    return d && isfinite(airtime) && airtime>=0 && isfinite(reserve) && reserve>=0 &&
        expire(d,now) && d->count<CS_DUTY_CAP && airtime<=d->duty_limit*d->window_s-d->used_s-reserve+1e-12;
}
bool cs_duty_record(cs_duty_t *d,double now,double airtime)
{
    if(!cs_duty_allowed(d,now,airtime,0)) { if(d) d->rejected++; return false; }
    size_t i=(d->head+d->count)%CS_DUTY_CAP;
    d->time[i]=now; d->airtime[i]=airtime; ++d->count; d->used_s+=airtime; return true;
}
