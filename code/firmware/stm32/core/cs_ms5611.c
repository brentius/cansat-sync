/* GPL-3.0-only. Equations: TE MS5611-01BA03 06/2017 pp8-9;
 * CRC: TEConnectivity/MS5611_Generic_C_Driver ms5611.c (AN520).
 * Signed division uses explicit floor to match arithmetic-shift equations. */
#include "cs_ms5611.h"
#include <stddef.h>
static int64_t div_floor(int64_t n,int64_t d) { return n/d-(n<0 && n%d!=0 ? 1:0); }
uint8_t cs_ms5611_crc4(const uint16_t prom[8])
{
    if(!prom) return 0;
    uint16_t rem=0;
    for(unsigned i=0;i<16;i++) {
        uint16_t w=prom[i/2]; if(i/2==7) w&=0xff00u;
        rem^=(i&1u)?(uint16_t)(w&0xffu):(uint16_t)(w>>8);
        for(unsigned j=0;j<8;j++) rem=(uint16_t)((rem&0x8000u)?((uint32_t)rem<<1)^0x3000u:((uint32_t)rem<<1));
    }
    return (uint8_t)((rem>>12)&15u);
}
bool cs_ms5611_prom_valid(const uint16_t p[8])
{
    if(!p || cs_ms5611_crc4(p)!=(p[7]&15u)) return false;
    for(unsigned i=1;i<=6;i++) if(p[i]==0 || p[i]==UINT16_MAX) return false;
    return true;
}
bool cs_ms5611_compensate(const uint16_t p[8],uint32_t d1,uint32_t d2,bool second,cs_ms5611_result_t *out)
{
    if(!out || !cs_ms5611_prom_valid(p) || !d1 || !d2 || d1>=0xffffffu || d2>=0xffffffu) return false;
    int64_t dt=(int64_t)d2-(int64_t)p[5]*256;
    int64_t temp=2000+div_floor(dt*p[6],8388608);
    int64_t off=(int64_t)p[2]*65536+div_floor((int64_t)p[4]*dt,128);
    int64_t sens=(int64_t)p[1]*32768+div_floor((int64_t)p[3]*dt,256);
    if(second && temp<2000) {
        int64_t q=(temp-2000)*(temp-2000), off2=5*q/2, sens2=5*q/4;
        if(temp < -1500) { q=(temp+1500)*(temp+1500); off2+=7*q; sens2+=11*q/2; }
        temp-=dt*dt/INT64_C(2147483648); off-=off2; sens-=sens2;
    }
    /* Products fit signed 64 bits for 24-bit ADC and 16-bit coefficients.
     * Preserve eight extra pressure bits by dividing 2^15 by 2^8 = 2^7. */
    int64_t pq8=div_floor(div_floor((int64_t)d1*sens,2097152)-off,128);
    if(temp<INT32_MIN || temp>INT32_MAX || pq8<=0 || pq8>(int64_t)UINT32_MAX*256/100) return false;
    out->temperature_centi_c=(int32_t)temp; out->pressure_q8_pa=pq8;
    out->pressure_cpa=(uint32_t)((pq8*100+128)/256); return true;
}
