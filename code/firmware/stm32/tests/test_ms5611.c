/* GPL-3.0-only. Datasheet example and independent floating equation checks.
 * CS_SOURCES: cs_ms5611.c */
#include "cs_test.h"
#include "cs_ms5611.h"
static void example_prom(uint16_t p[8])
{
    const uint16_t src[8]={0,40127,36924,23317,23282,33464,28312,0};
    memcpy(p,src,sizeof src); p[7]=cs_ms5611_crc4(p);
}
CS_TEST(crc_and_faults)
{
    uint16_t p[8],q[8]; example_prom(p);
    CS_CHECK_EQ(p[7],0); CS_CHECK(cs_ms5611_prom_valid(p));
    for(unsigned i=0;i<8;i++) for(unsigned bit=0;bit<16;bit++) {
        if(i==7 && bit>=4 && bit<8) continue; /* AN520 clears the whole CRC byte. */
        memcpy(q,p,sizeof q); q[i]^=(uint16_t)(1u<<bit); CS_CHECK(!cs_ms5611_prom_valid(q));
    }
    cs_ms5611_result_t out;
    CS_CHECK(!cs_ms5611_compensate(p,0,8569150,true,&out));
    CS_CHECK(!cs_ms5611_compensate(p,9085466,0xffffff,true,&out));
    memset(q,0,sizeof q); CS_CHECK(!cs_ms5611_prom_valid(q));
    memset(q,0xff,sizeof q); CS_CHECK(!cs_ms5611_prom_valid(q));
}
CS_TEST(datasheet_example)
{
    uint16_t p[8]; example_prom(p); cs_ms5611_result_t out;
    CS_CHECK(cs_ms5611_compensate(p,9085466,8569150,true,&out));
    CS_CHECK_EQ(out.temperature_centi_c,2007);
    CS_CHECK_NEAR((double)out.pressure_cpa*.01,100009.07,.02);
    CS_CHECK(out.pressure_cpa%100!=0); /* Sub-Pa information is preserved. */
}
CS_TEST(all_temperature_branches)
{
    uint16_t p[8]; example_prom(p);
    for(uint32_t d2=6500000;d2<11000000;d2+=12501) for(unsigned second=0;second<2;second++) {
        cs_ms5611_result_t out;
        int64_t dt=(int64_t)d2-(int64_t)p[5]*256;
        double temp=2000+floor((double)dt*(double)p[6]/8388608);
        double off=(double)p[2]*65536+floor((double)p[4]*(double)dt/128);
        double sens=(double)p[1]*32768+floor((double)p[3]*(double)dt/256);
        if(second && temp<2000) {
            double q=(temp-2000)*(temp-2000), off2=floor(5*q/2),sens2=floor(5*q/4);
            if(temp < -1500) { q=(temp+1500)*(temp+1500); off2+=7*q; sens2+=floor(11*q/2); }
            temp-=floor((double)dt*(double)dt/2147483648.0); off-=off2; sens-=sens2;
        }
        double pressure=((9085466*sens/2097152)-off)/32768;
        CS_CHECK(cs_ms5611_compensate(p,9085466,d2,second!=0,&out));
        CS_CHECK_EQ(out.temperature_centi_c,(int32_t)temp);
        CS_CHECK_NEAR((double)out.pressure_cpa*.01,pressure,.01);
    }
}
CS_MAIN(crc_and_faults,datasheet_example,all_temperature_branches)
