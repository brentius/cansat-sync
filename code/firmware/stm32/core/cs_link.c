/* GPL-3.0-only. C twin of link message layouts; explicit little endian. */
#include "cs_link.h"
/* The bare Zig target supplies no libc headers; these are standard C99
 * runtime declarations, resolved by the final firmware runtime/libm. */
#if __STDC_HOSTED__
#include <string.h>
#include <math.h>
#else
extern void *memcpy(void *,const void *,size_t);
extern void *memset(void *,int,size_t);
extern double floor(double);
extern double ldexp(double,int);
extern double fmod(double,double);
#define isfinite(x) __builtin_isfinite(x)
#endif
#include "cs_crc16.h"
#include "cs_cobs.h"
static void put(uint8_t *p, uint32_t value, size_t bytes) {
    size_t i; for (i=0;i<bytes;i++) p[i]=(uint8_t)(value>>(8u*i));
}
static uint32_t get(const uint8_t *p, size_t bytes) {
    size_t i; uint32_t value=0; for(i=0;i<bytes;i++) value|=(uint32_t)p[i]<<(8u*i); return value;
}
/* Interpret signed wire bits without implementation-defined unsigned casts. */
static int32_t signed_get(const uint8_t *p, size_t bytes) {
    uint32_t value=get(p,bytes), sign=1u<<(bytes*8u-1u);
    if(value&sign) return -1-(int32_t)((sign*2u-1u)-value);
    return (int32_t)value;
}
size_t cs_link_pack_hello(uint8_t *dst, size_t cap, const cs_link_hello_t *msg) {
    if(!dst || !msg || cap<16) return 0;
    memset(dst,0,16);
    put(dst+0,(uint32_t)msg->proto,1);
    put(dst+1,(uint32_t)msg->epoch,1);
    put(dst+2,(uint32_t)msg->fw_version,2);
    put(dst+4,(uint32_t)msg->reset_cause,1);
    put(dst+5,(uint32_t)msg->flags,1);
    put(dst+6,(uint32_t)msg->tick_hi,2);
    put(dst+8,(uint32_t)msg->tick,4);
    put(dst+12,(uint32_t)msg->tick_hz,4);
    return 16;
}
bool cs_link_unpack_hello(cs_link_hello_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=16) return false;
    msg->proto=(uint8_t)get(src+0,1);
    msg->epoch=(uint8_t)get(src+1,1);
    msg->fw_version=(uint16_t)get(src+2,2);
    msg->reset_cause=(uint8_t)get(src+4,1);
    msg->flags=(uint8_t)get(src+5,1);
    msg->tick_hi=(uint16_t)get(src+6,2);
    msg->tick=(uint32_t)get(src+8,4);
    msg->tick_hz=(uint32_t)get(src+12,4);
    return true;
}
size_t cs_link_pack_imu(uint8_t *dst, size_t cap, const cs_link_imu_t *msg) {
    if(!dst || !msg || cap<36) return 0;
    memset(dst,0,36);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->q[0],4);
    put(dst+8,(uint32_t)msg->q[1],4);
    put(dst+12,(uint32_t)msg->q[2],4);
    put(dst+16,(uint32_t)msg->q[3],4);
    put(dst+20,(uint32_t)msg->u[0],4);
    put(dst+24,(uint32_t)msg->u[1],4);
    put(dst+28,(uint32_t)msg->u[2],4);
    put(dst+32,(uint32_t)msg->epoch,1);
    put(dst+33,(uint32_t)msg->flags,1);
    put(dst+34,(uint32_t)msg->n_gyro,1);
    put(dst+35,(uint32_t)msg->n_accel,1);
    return 36;
}
bool cs_link_unpack_imu(cs_link_imu_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=36) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->q[0]=(int32_t)signed_get(src+4,4);
    msg->q[1]=(int32_t)signed_get(src+8,4);
    msg->q[2]=(int32_t)signed_get(src+12,4);
    msg->q[3]=(int32_t)signed_get(src+16,4);
    msg->u[0]=(int32_t)signed_get(src+20,4);
    msg->u[1]=(int32_t)signed_get(src+24,4);
    msg->u[2]=(int32_t)signed_get(src+28,4);
    msg->epoch=(uint8_t)get(src+32,1);
    msg->flags=(uint8_t)get(src+33,1);
    msg->n_gyro=(uint8_t)get(src+34,1);
    msg->n_accel=(uint8_t)get(src+35,1);
    return true;
}
size_t cs_link_pack_baro(uint8_t *dst, size_t cap, const cs_link_baro_t *msg) {
    if(!dst || !msg || cap<12) return 0;
    memset(dst,0,12);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->pressure_pa,4);
    put(dst+8,(uint32_t)msg->temperature_c,2);
    put(dst+10,(uint32_t)msg->n_samples,1);
    put(dst+11,(uint32_t)msg->flags,1);
    return 12;
}
bool cs_link_unpack_baro(cs_link_baro_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=12) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->pressure_pa=(uint32_t)get(src+4,4);
    msg->temperature_c=(int16_t)signed_get(src+8,2);
    msg->n_samples=(uint8_t)get(src+10,1);
    msg->flags=(uint8_t)get(src+11,1);
    return true;
}
size_t cs_link_pack_gnss(uint8_t *dst, size_t cap, const cs_link_gnss_t *msg) {
    if(!dst || !msg || cap<60) return 0;
    memset(dst,0,60);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->itow_ms,4);
    put(dst+8,(uint32_t)msg->year,2);
    put(dst+10,(uint32_t)msg->month,1);
    put(dst+11,(uint32_t)msg->day,1);
    put(dst+12,(uint32_t)msg->hour,1);
    put(dst+13,(uint32_t)msg->minute,1);
    put(dst+14,(uint32_t)msg->second,1);
    put(dst+15,(uint32_t)msg->valid,1);
    put(dst+16,(uint32_t)msg->fix_type,1);
    put(dst+17,(uint32_t)msg->fix_flags,1);
    put(dst+18,(uint32_t)msg->num_sv,1);
    put(dst+19,(uint32_t)msg->link_flags,1);
    put(dst+20,(uint32_t)msg->lon_deg,4);
    put(dst+24,(uint32_t)msg->lat_deg,4);
    put(dst+28,(uint32_t)msg->h_msl_m,4);
    put(dst+32,(uint32_t)msg->h_acc_m,4);
    put(dst+36,(uint32_t)msg->v_acc_m,4);
    put(dst+40,(uint32_t)msg->vel_n,4);
    put(dst+44,(uint32_t)msg->vel_e,4);
    put(dst+48,(uint32_t)msg->vel_d,4);
    put(dst+52,(uint32_t)msg->s_acc,4);
    put(dst+56,(uint32_t)msg->p_dop,2);
    return 60;
}
bool cs_link_unpack_gnss(cs_link_gnss_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=60) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->itow_ms=(uint32_t)get(src+4,4);
    msg->year=(uint16_t)get(src+8,2);
    msg->month=(uint8_t)get(src+10,1);
    msg->day=(uint8_t)get(src+11,1);
    msg->hour=(uint8_t)get(src+12,1);
    msg->minute=(uint8_t)get(src+13,1);
    msg->second=(uint8_t)get(src+14,1);
    msg->valid=(uint8_t)get(src+15,1);
    msg->fix_type=(uint8_t)get(src+16,1);
    msg->fix_flags=(uint8_t)get(src+17,1);
    msg->num_sv=(uint8_t)get(src+18,1);
    msg->link_flags=(uint8_t)get(src+19,1);
    msg->lon_deg=(int32_t)signed_get(src+20,4);
    msg->lat_deg=(int32_t)signed_get(src+24,4);
    msg->h_msl_m=(int32_t)signed_get(src+28,4);
    msg->h_acc_m=(uint32_t)get(src+32,4);
    msg->v_acc_m=(uint32_t)get(src+36,4);
    msg->vel_n=(int32_t)signed_get(src+40,4);
    msg->vel_e=(int32_t)signed_get(src+44,4);
    msg->vel_d=(int32_t)signed_get(src+48,4);
    msg->s_acc=(uint32_t)get(src+52,4);
    msg->p_dop=(uint16_t)get(src+56,2);
    return true;
}
size_t cs_link_pack_trig(uint8_t *dst, size_t cap, const cs_link_trig_t *msg) {
    if(!dst || !msg || cap<16) return 0;
    memset(dst,0,16);
    put(dst+0,(uint32_t)msg->frame,4);
    put(dst+4,(uint32_t)msg->tick,4);
    put(dst+8,(uint32_t)msg->period_us,4);
    put(dst+12,(uint32_t)msg->pulse_us,2);
    put(dst+14,(uint32_t)msg->flags,1);
    return 16;
}
bool cs_link_unpack_trig(cs_link_trig_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=16) return false;
    msg->frame=(uint32_t)get(src+0,4);
    msg->tick=(uint32_t)get(src+4,4);
    msg->period_us=(uint32_t)get(src+8,4);
    msg->pulse_us=(uint16_t)get(src+12,2);
    msg->flags=(uint8_t)get(src+14,1);
    return true;
}
size_t cs_link_pack_pps(uint8_t *dst, size_t cap, const cs_link_pps_t *msg) {
    if(!dst || !msg || cap<16) return 0;
    memset(dst,0,16);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->itow_ms,4);
    put(dst+8,(uint32_t)msg->interval,4);
    put(dst+12,(uint32_t)msg->count,2);
    put(dst+14,(uint32_t)msg->flags,1);
    return 16;
}
bool cs_link_unpack_pps(cs_link_pps_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=16) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->itow_ms=(uint32_t)get(src+4,4);
    msg->interval=(uint32_t)get(src+8,4);
    msg->count=(uint16_t)get(src+12,2);
    msg->flags=(uint8_t)get(src+14,1);
    return true;
}
size_t cs_link_pack_status(uint8_t *dst, size_t cap, const cs_link_status_t *msg) {
    if(!dst || !msg || cap<48) return 0;
    memset(dst,0,48);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->tick_hi,2);
    put(dst+6,(uint32_t)msg->epoch,1);
    put(dst+7,(uint32_t)msg->state,1);
    put(dst+8,(uint32_t)msg->tick_state,4);
    put(dst+12,(uint32_t)msg->health,2);
    put(dst+14,(uint32_t)msg->battery_v,2);
    put(dst+16,(uint32_t)msg->baseline_pa,4);
    put(dst+20,(uint32_t)msg->origin_lat_deg,4);
    put(dst+24,(uint32_t)msg->origin_lon_deg,4);
    put(dst+28,(uint32_t)msg->origin_h_msl_m,4);
    put(dst+32,(uint32_t)msg->altitude_m,2);
    put(dst+34,(uint32_t)msg->temperature_c,2);
    put(dst+36,(uint32_t)msg->telemetry_seq,2);
    put(dst+38,(uint32_t)msg->rx_crc,2);
    put(dst+40,(uint32_t)msg->rx_lost,2);
    put(dst+42,(uint32_t)msg->tx_dropped,2);
    put(dst+44,(uint32_t)msg->pi_age_s,1);
    put(dst+45,(uint32_t)msg->mcu_temp_c,1);
    return 48;
}
bool cs_link_unpack_status(cs_link_status_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=48) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->tick_hi=(uint16_t)get(src+4,2);
    msg->epoch=(uint8_t)get(src+6,1);
    msg->state=(uint8_t)get(src+7,1);
    msg->tick_state=(uint32_t)get(src+8,4);
    msg->health=(uint16_t)get(src+12,2);
    msg->battery_v=(uint16_t)get(src+14,2);
    msg->baseline_pa=(uint32_t)get(src+16,4);
    msg->origin_lat_deg=(int32_t)signed_get(src+20,4);
    msg->origin_lon_deg=(int32_t)signed_get(src+24,4);
    msg->origin_h_msl_m=(int32_t)signed_get(src+28,4);
    msg->altitude_m=(int16_t)signed_get(src+32,2);
    msg->temperature_c=(int16_t)signed_get(src+34,2);
    msg->telemetry_seq=(uint16_t)get(src+36,2);
    msg->rx_crc=(uint16_t)get(src+38,2);
    msg->rx_lost=(uint16_t)get(src+40,2);
    msg->tx_dropped=(uint16_t)get(src+42,2);
    msg->pi_age_s=(uint8_t)get(src+44,1);
    msg->mcu_temp_c=(int8_t)signed_get(src+45,1);
    return true;
}
size_t cs_link_pack_cmd(uint8_t *dst, size_t cap, const cs_link_cmd_t *msg) {
    if(!dst || !msg || cap<8) return 0;
    memset(dst,0,8);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->cmd_seq,1);
    put(dst+5,(uint32_t)msg->cmd,1);
    put(dst+6,(uint32_t)msg->arg,2);
    return 8;
}
bool cs_link_unpack_cmd(cs_link_cmd_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=8) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->cmd_seq=(uint8_t)get(src+4,1);
    msg->cmd=(uint8_t)get(src+5,1);
    msg->arg=(uint16_t)get(src+6,2);
    return true;
}
size_t cs_link_pack_pi_status(uint8_t *dst, size_t cap, const cs_link_pi_status_t *msg) {
    if(!dst || !msg || cap<24) return 0;
    memset(dst,0,24);
    put(dst+0,(uint32_t)msg->uptime_ms,4);
    put(dst+4,(uint32_t)msg->tick_echo,4);
    put(dst+8,(uint32_t)msg->echo_age_ms,2);
    put(dst+10,(uint32_t)msg->proto,1);
    put(dst+11,(uint32_t)msg->state,1);
    put(dst+12,(uint32_t)msg->flags,2);
    put(dst+14,(uint32_t)msg->sw_version,2);
    put(dst+16,(uint32_t)msg->rx_crc,2);
    put(dst+18,(uint32_t)msg->rx_lost,2);
    put(dst+20,(uint32_t)msg->frames,2);
    put(dst+22,(uint32_t)msg->cpu_temp_c,1);
    put(dst+23,(uint32_t)msg->cpu_load,1);
    return 24;
}
bool cs_link_unpack_pi_status(cs_link_pi_status_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=24) return false;
    msg->uptime_ms=(uint32_t)get(src+0,4);
    msg->tick_echo=(uint32_t)get(src+4,4);
    msg->echo_age_ms=(uint16_t)get(src+8,2);
    msg->proto=(uint8_t)get(src+10,1);
    msg->state=(uint8_t)get(src+11,1);
    msg->flags=(uint16_t)get(src+12,2);
    msg->sw_version=(uint16_t)get(src+14,2);
    msg->rx_crc=(uint16_t)get(src+16,2);
    msg->rx_lost=(uint16_t)get(src+18,2);
    msg->frames=(uint16_t)get(src+20,2);
    msg->cpu_temp_c=(int8_t)signed_get(src+22,1);
    msg->cpu_load=(uint8_t)get(src+23,1);
    return true;
}
size_t cs_link_pack_nav(uint8_t *dst, size_t cap, const cs_link_nav_t *msg) {
    if(!dst || !msg || cap<32) return 0;
    memset(dst,0,32);
    put(dst+0,(uint32_t)msg->tick,4);
    put(dst+4,(uint32_t)msg->frame,2);
    put(dst+6,(uint32_t)msg->flags,1);
    put(dst+7,(uint32_t)msg->landmarks,1);
    put(dst+8,(uint32_t)msg->trn_e,2);
    put(dst+10,(uint32_t)msg->trn_n,2);
    put(dst+12,(uint32_t)msg->trn_u,2);
    put(dst+14,(uint32_t)msg->trn_major_m,1);
    put(dst+15,(uint32_t)msg->trn_minor_m,1);
    put(dst+16,(uint32_t)msg->trn_angle_rad,1);
    put(dst+17,(uint32_t)msg->protection_m,1);
    put(dst+18,(uint32_t)msg->pred_e,2);
    put(dst+20,(uint32_t)msg->pred_n,2);
    put(dst+22,(uint32_t)msg->time_to_go_s,1);
    put(dst+23,(uint32_t)msg->pred_major_m,1);
    put(dst+24,(uint32_t)msg->pred_minor_m,1);
    put(dst+25,(uint32_t)msg->pred_angle_rad,1);
    put(dst+26,(uint32_t)msg->vel_e,2);
    put(dst+28,(uint32_t)msg->vel_n,2);
    put(dst+30,(uint32_t)msg->vel_u,2);
    return 32;
}
bool cs_link_unpack_nav(cs_link_nav_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=32) return false;
    msg->tick=(uint32_t)get(src+0,4);
    msg->frame=(uint16_t)get(src+4,2);
    msg->flags=(uint8_t)get(src+6,1);
    msg->landmarks=(uint8_t)get(src+7,1);
    msg->trn_e=(int16_t)signed_get(src+8,2);
    msg->trn_n=(int16_t)signed_get(src+10,2);
    msg->trn_u=(int16_t)signed_get(src+12,2);
    msg->trn_major_m=(uint8_t)get(src+14,1);
    msg->trn_minor_m=(uint8_t)get(src+15,1);
    msg->trn_angle_rad=(uint8_t)get(src+16,1);
    msg->protection_m=(uint8_t)get(src+17,1);
    msg->pred_e=(int16_t)signed_get(src+18,2);
    msg->pred_n=(int16_t)signed_get(src+20,2);
    msg->time_to_go_s=(uint8_t)get(src+22,1);
    msg->pred_major_m=(uint8_t)get(src+23,1);
    msg->pred_minor_m=(uint8_t)get(src+24,1);
    msg->pred_angle_rad=(uint8_t)get(src+25,1);
    msg->vel_e=(int16_t)signed_get(src+26,2);
    msg->vel_n=(int16_t)signed_get(src+28,2);
    msg->vel_u=(int16_t)signed_get(src+30,2);
    return true;
}
size_t cs_link_pack_ack(uint8_t *dst, size_t cap, const cs_link_ack_t *msg) {
    if(!dst || !msg || cap<4) return 0;
    memset(dst,0,4);
    put(dst+0,(uint32_t)msg->cmd_seq,1);
    put(dst+1,(uint32_t)msg->cmd,1);
    put(dst+2,(uint32_t)msg->result,1);
    return 4;
}
bool cs_link_unpack_ack(cs_link_ack_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=4) return false;
    msg->cmd_seq=(uint8_t)get(src+0,1);
    msg->cmd=(uint8_t)get(src+1,1);
    msg->result=(uint8_t)get(src+2,1);
    return true;
}
size_t cs_link_pack_trig_cfg(uint8_t *dst, size_t cap, const cs_link_trig_cfg_t *msg) {
    if(!dst || !msg || cap<8) return 0;
    memset(dst,0,8);
    put(dst+0,(uint32_t)msg->enable,1);
    put(dst+1,(uint32_t)msg->cfg_seq,1);
    put(dst+2,(uint32_t)msg->period_ms,2);
    put(dst+4,(uint32_t)msg->pulse_us,2);
    return 8;
}
bool cs_link_unpack_trig_cfg(cs_link_trig_cfg_t *msg,const uint8_t *src,size_t len) {
    if(!msg || !src || len!=8) return false;
    msg->enable=(uint8_t)get(src+0,1);
    msg->cfg_seq=(uint8_t)get(src+1,1);
    msg->period_ms=(uint16_t)get(src+2,2);
    msg->pulse_us=(uint16_t)get(src+4,2);
    return true;
}
size_t cs_link_payload_size(uint8_t id) { switch(id) {
case 1: return 16;
case 16: return 36;
case 17: return 12;
case 18: return 60;
case 19: return 16;
case 20: return 16;
case 21: return 48;
case 22: return 8;
case 128: return 24;
case 129: return 32;
case 130: return 4;
case 131: return 8;
default: return 0; }}
/* SI helpers reserve one code for unavailable. */
int32_t cs_link_scale_signed(double value, double step, unsigned bits)
{
    double raw, limit;
    if(bits<2 || bits>32) return 0;
    limit=ldexp(1.0,(int)bits-1)-1.0;
    if(!isfinite(value) || !isfinite(step) || step<=0) return (int32_t)(-limit-1.0);
    raw=floor(value/step+0.5);
    if(raw>limit) raw=limit;
    if(raw < -limit) raw=-limit;
    return (int32_t)raw;
}
uint32_t cs_link_scale_unsigned(double value, double step, unsigned bits)
{
    double raw, limit;
    if(bits<1 || bits>32) return 0;
    limit=ldexp(1.0,(int)bits)-1.0;
    if(!isfinite(value) || !isfinite(step) || step<=0) return (uint32_t)limit;
    raw=floor(value/step+0.5);
    if(raw>limit-1.0) raw=limit-1.0;
    if(raw<0) raw=0;
    return (uint32_t)raw;
}
uint8_t cs_link_angle(double radians)
{
    const double pi=3.141592653589793;
    double value;
    if(!isfinite(radians)) return 0;
    value=fmod(radians,pi);
    if(value<0) value+=pi;
    return (uint8_t)((unsigned)floor(value/pi*256.0+0.5)&255u);
}
size_t cs_link_frame(uint8_t *dst,size_t cap,uint8_t id,uint8_t seq,const uint8_t *payload,size_t len)
{
    uint8_t body[CS_LINK_MAX_FRAME]; uint16_t crc; size_t n=0;
    if(!dst || cap<1 || len>CS_LINK_MAX_PAYLOAD || (!payload && len)) return 0;
    body[0]=id; body[1]=seq;
    if(len) memcpy(body+2,payload,len);
    crc=cs_crc16(body,len+2,0xffffu);
    put(body+len+2,crc,2);
    if(!cs_cobs_encode(body,len+4,dst,cap-1,&n)) return 0;
    dst[n++]=0;
    return n;
}
void cs_link_encoder_init(cs_link_encoder_t *encoder) { memset(encoder,0,sizeof *encoder); }
size_t cs_link_encode(cs_link_encoder_t *encoder,uint8_t *dst,size_t cap,uint8_t id,const uint8_t *payload,size_t len)
{
    size_t n;
    if(!encoder) return 0;
    n=cs_link_frame(dst,cap,id,encoder->seq,payload,len);
    if(n) encoder->seq++;
    return n;
}
void cs_link_decoder_init(cs_link_decoder_t *decoder,bool raw_unknown)
{
    memset(decoder,0,sizeof *decoder); decoder->raw_unknown=raw_unknown;
}
void cs_link_decoder_reset_sequence(cs_link_decoder_t *decoder) { decoder->have_seq=false; }
static void decode_frame(cs_link_decoder_t *d,cs_link_receive_fn receive,void *ctx)
{
    uint8_t body[CS_LINK_MAX_WIRE]; size_t n=0,expected; uint8_t id,seq;
    if(!cs_cobs_decode(d->buf,d->used,body,sizeof body,&n)) { d->stats.stuffing++;return; }
    if(n<4) { d->stats.short_frame++;return; }
    if(n>CS_LINK_MAX_FRAME) { d->stats.too_long++;return; }
    if(cs_crc16(body,n-2,0xffffu)!=get(body+n-2,2)) { d->stats.crc++;return; }
    d->stats.frames++;
    id=body[0]; seq=body[1];
    if(d->have_seq) {
        uint8_t step=(uint8_t)(seq-d->last_seq);
        if(step==0) d->stats.repeats++; else d->stats.lost+=(uint64_t)step-1u;
    }
    d->have_seq=true; d->last_seq=seq;
    expected=cs_link_payload_size(id);
    if(!expected) { d->stats.unknown++;if(!d->raw_unknown) return; }
    else if(n-4!=expected) { d->stats.length++;return; }
    if(receive) receive(ctx,id,seq,body+2,n-4);
}
void cs_link_decoder_feed(cs_link_decoder_t *d,const uint8_t *data,size_t len,cs_link_receive_fn receive,void *ctx)
{
    size_t i;
    if(!d) return;
    if(!data && len) { d->stats.ignored++;return; }
    d->stats.bytes+=len;
    for(i=0;i<len;i++) {
        uint8_t b=data[i];
        if(b==0) {
            if(d->skip) d->skip=false;
            else if(d->used==0) d->stats.empty++;
            else decode_frame(d,receive,ctx);
            d->used=0;
        } else if(!d->skip) {
            if(d->used==sizeof d->buf) { d->stats.too_long++;d->skip=true;d->used=0; }
            else d->buf[d->used++]=b;
        }
    }
}
