/* GPL-3.0-only. C twin of telemetry.encode/decode and link.apply_nav. */
#include "cs_telemetry.h"
/* The bare Zig target supplies no libc headers; these are standard C99
 * runtime declarations, resolved by the final firmware runtime/libm. */
#if __STDC_HOSTED__
#include <math.h>
#include <string.h>
#else
extern void *memset(void *,int,size_t);
extern double floor(double);
extern double fmod(double,double);
#define isfinite(x) __builtin_isfinite(x)
#endif
#include "cs_crc16.h"
static double even_round(double x) {
    double lo=floor(x), f=x-lo;
    if(f>0.5 || (f==0.5 && fmod(lo,2.0)!=0)) lo+=1;
    return lo;
}
int16_t cs_telemetry_i16(double value,double step) {
    double x;
    if(!isfinite(value) || !isfinite(step) || step<=0) return INT16_MIN;
    x=even_round(value/step);
    if(x>32767) x=32767;
    if(x< -32767) x=-32767;
    return (int16_t)x;
}
static uint32_t unsigned_scaled(double value,double step,double offset,uint32_t na) {
    double x;
    if(!isfinite(value) || !isfinite(step) || !isfinite(offset) || step<=0) return na;
    x=even_round((value-offset)/step);
    if(x<0) x=0;
    if(x>(double)(na-1u)) x=(double)(na-1u);
    return (uint32_t)x;
}
uint8_t cs_telemetry_u8(double value,double step,double offset) { return (uint8_t)unsigned_scaled(value,step,offset,255); }
uint16_t cs_telemetry_u16(double value,double step,double offset) { return (uint16_t)unsigned_scaled(value,step,offset,65535); }
uint8_t cs_telemetry_angle(double radians) {
    const double pi=3.141592653589793; double x;
    if(!isfinite(radians)) return 0;
    x=fmod(radians,pi);if(x<0) x+=pi;
    return (uint8_t)((unsigned)even_round(x/pi*256.0)&255u);
}
void cs_telemetry_init(cs_telemetry_t *t) {
    memset(t,0,sizeof *t);
    t->pressure_pa=UINT16_MAX;
    t->temperature_c=t->altitude_m=t->gps_e=t->gps_n=t->gps_u=t->trn_e=t->trn_n=t->pred_e=t->pred_n=INT16_MIN;
    t->trn_major_m=t->trn_minor_m=t->protection_m=t->time_to_go_s=t->pred_major_m=t->pred_minor_m=t->battery_v=UINT8_MAX;
}
static void put(uint8_t *p,uint32_t value,size_t n) { size_t i;for(i=0;i<n;i++) p[i]=(uint8_t)(value>>(8u*i)); }
static uint32_t get(const uint8_t *p,size_t n) { size_t i;uint32_t v=0;for(i=0;i<n;i++) v|=(uint32_t)p[i]<<(8u*i);return v; }
static int16_t get_i16(const uint8_t *p) { uint32_t v=get(p,2);return v>32767u ? (int16_t)(-1-(int32_t)(65535u-v)) : (int16_t)v; }
size_t cs_telemetry_encode(uint8_t *dst,size_t cap,const cs_telemetry_t *t) {
    uint16_t crc;
    if(!dst || !t || cap<40) return 0;
    dst[0]=CS_TELEMETRY_MAGIC;
    put(dst+1,t->seq,2);put(dst+3,t->t_ms,4);dst[7]=(uint8_t)((t->state&15u)|(t->flags&240u));
    put(dst+8,t->pressure_pa,2);put(dst+10,(uint32_t)t->temperature_c,2);put(dst+12,(uint32_t)t->altitude_m,2);
    put(dst+14,(uint32_t)t->gps_e,2);put(dst+16,(uint32_t)t->gps_n,2);put(dst+18,(uint32_t)t->gps_u,2);
    put(dst+20,(uint32_t)t->trn_e,2);put(dst+22,(uint32_t)t->trn_n,2);
    dst[24]=t->trn_major_m;dst[25]=t->trn_minor_m;dst[26]=t->trn_angle_rad;dst[27]=t->protection_m;dst[28]=t->landmarks;
    put(dst+29,(uint32_t)t->pred_e,2);put(dst+31,(uint32_t)t->pred_n,2);
    dst[33]=t->time_to_go_s;dst[34]=t->pred_major_m;dst[35]=t->pred_minor_m;dst[36]=t->pred_angle_rad;dst[37]=t->battery_v;
    crc=cs_crc16(dst,38,65535);put(dst+38,crc,2);return 40;
}
bool cs_telemetry_decode(cs_telemetry_t *t,const uint8_t *src,size_t len) {
    if(!t || !src || len!=40 || src[0]!=CS_TELEMETRY_MAGIC || cs_crc16(src,38,65535)!=get(src+38,2)) return false;
    t->seq=(uint16_t)get(src+1,2);t->t_ms=get(src+3,4);t->state=src[7]&15u;t->flags=src[7]&240u;
    t->pressure_pa=(uint16_t)get(src+8,2);t->temperature_c=get_i16(src+10);t->altitude_m=get_i16(src+12);
    t->gps_e=get_i16(src+14);t->gps_n=get_i16(src+16);t->gps_u=get_i16(src+18);t->trn_e=get_i16(src+20);t->trn_n=get_i16(src+22);
    t->trn_major_m=src[24];t->trn_minor_m=src[25];t->trn_angle_rad=src[26];t->protection_m=src[27];t->landmarks=src[28];
    t->pred_e=get_i16(src+29);t->pred_n=get_i16(src+31);t->time_to_go_s=src[33];t->pred_major_m=src[34];t->pred_minor_m=src[35];t->pred_angle_rad=src[36];t->battery_v=src[37];
    return true;
}
void cs_apply_nav(cs_telemetry_t *t,const cs_link_nav_t *nav,double max_age_s,uint32_t now_tick) {
    const uint8_t mask=CS_FLAG_TRN_VALID|CS_FLAG_PRED_VALID|CS_FLAG_CAMERA_OK;
    bool stale=nav && max_age_s>=0 && ((double)(uint32_t)(now_tick-nav->tick)/1000000.0)>max_age_s;
    if(!t) return;
    t->flags&=(uint8_t)~mask;
    if(!nav || stale) {
        t->trn_e=t->trn_n=t->pred_e=t->pred_n=INT16_MIN;
        t->trn_major_m=t->trn_minor_m=t->protection_m=t->time_to_go_s=t->pred_major_m=t->pred_minor_m=UINT8_MAX;
        t->trn_angle_rad=t->pred_angle_rad=t->landmarks=0;
        return;
    }
    t->flags|=nav->flags&mask;
    t->trn_e=nav->trn_e;t->trn_n=nav->trn_n;t->trn_major_m=nav->trn_major_m;t->trn_minor_m=nav->trn_minor_m;t->trn_angle_rad=nav->trn_angle_rad;
    t->protection_m=nav->protection_m;t->landmarks=nav->landmarks;t->pred_e=nav->pred_e;t->pred_n=nav->pred_n;t->time_to_go_s=nav->time_to_go_s;
    t->pred_major_m=nav->pred_major_m;t->pred_minor_m=nav->pred_minor_m;t->pred_angle_rad=nav->pred_angle_rad;
}
