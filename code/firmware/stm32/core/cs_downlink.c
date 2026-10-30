/* GPL-3.0-only. */
#include "cs_downlink.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <math.h>
#else
extern double fmax(double,double);
#define isfinite(x) __builtin_isfinite(x)
#define NAN __builtin_nan("")
#endif
bool cs_downlink_init(cs_downlink_t *s,double air,double limit,double window,double interval,double flight,double ground)
{
    if(!s || !isfinite(air) || air<=0 || !isfinite(interval) || interval<=0 ||
        !isfinite(flight) || flight<0 || !isfinite(ground) || ground<0 || !cs_duty_init(&s->limiter,limit,window)) return false;
    s->airtime=air; s->flight_interval=interval; s->reserve=flight/interval*air;
    double spare=limit*window-s->reserve;
    if(spare<air) return false;
    s->ground_interval=fmax(fmax(interval,window*air/spare),ground); s->last=(double)NAN; return true;
}
bool cs_downlink_due(cs_downlink_t *s,double now,cs_flight_state_t state)
{
    if(!s || !isfinite(now)) return false;
    double interval=(state==CS_ASCENT || state==CS_DESCENT)?s->flight_interval:s->ground_interval;
    if(isfinite(s->last) && now-s->last<interval-1e-9) return false;
    double reserve=state==CS_PAD?s->reserve:0;
    if(!cs_duty_allowed(&s->limiter,now,s->airtime,reserve) || !cs_duty_record(&s->limiter,now,s->airtime)) return false;
    s->last=now; return true;
}
