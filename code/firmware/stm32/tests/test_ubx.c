/* GPL-3.0-only. UBX malformed streams, NAV-PVT layout and configuration.
 * CS_SOURCES: cs_ubx.c */
#include "cs_test.h"
#include "cs_ubx.h"
static size_t frame(uint8_t *wire,uint8_t cls,uint8_t id,const uint8_t *p,size_t n)
{
    wire[0]=0xb5; wire[1]=0x62; wire[2]=cls; wire[3]=id; wire[4]=(uint8_t)n; wire[5]=(uint8_t)(n>>8);
    if(n) memcpy(wire+6,p,n);
    uint8_t a=0,b=0; for(size_t i=2;i<n+6;i++) { a=(uint8_t)(a+wire[i]); b=(uint8_t)(b+a); }
    wire[n+6]=a; wire[n+7]=b; return n+8;
}
static unsigned feed(cs_ubx_t *p,const uint8_t *wire,size_t n)
{ unsigned count=0; for(size_t i=0;i<n;i++) if(cs_ubx_feed(p,wire[i])) count++; return count; }
CS_TEST(stream_and_noise)
{
    cs_ubx_t p; cs_ubx_init(&p); uint8_t wire[264],payload[256];
    for(unsigned i=0;i<256;i++) payload[i]=(uint8_t)i;
    size_t n=frame(wire,1,7,payload,92);
    (void)cs_ubx_feed(&p,0xb5); CS_CHECK_EQ(feed(&p,wire,n),1); CS_CHECK_EQ(p.frame.len,92);
    CS_CHECK_MEM(p.frame.payload,payload,92);
    for(size_t j=2;j<n;j++) {
        wire[j]^=1; (void)feed(&p,wire,n); wire[j]^=1;
        /* A corrupted length may consume the following bytes; two known frames
         * bound recovery even after an interrupted/corrupt packet. */
        (void)feed(&p,wire,n); CS_CHECK(feed(&p,wire,n)>0);
    }
    cs_ubx_init(&p); const uint8_t huge[]={0xb5,0x62,1,7,1,1};
    CS_CHECK_EQ(feed(&p,huge,sizeof huge),0); CS_CHECK_EQ(p.oversize,1); CS_CHECK_EQ(feed(&p,wire,n),1);
    n=frame(wire,5,1,NULL,0); CS_CHECK_EQ(feed(&p,wire,n),1); CS_CHECK_EQ(p.frame.len,0);
    n=frame(wire,1,0x99,payload,256); CS_CHECK_EQ(feed(&p,wire,n),1); CS_CHECK_EQ(p.frame.len,256);
    wire[n-1]^=1; CS_CHECK_EQ(feed(&p,wire,n),0); CS_CHECK_EQ(p.checksum_errors,1);
}
CS_TEST(nav_pvt)
{
    cs_ubx_frame_t f; memset(&f,0,sizeof f); f.cls=1; f.id=7; f.len=92;
    for(unsigned i=0;i<92;i++) f.payload[i]=(uint8_t)i;
    cs_link_gnss_t g; CS_CHECK(cs_ubx_nav_pvt(&f,0xfedcba98,1,&g));
    CS_CHECK_EQ_U(g.tick,0xfedcba98); CS_CHECK_EQ_U(g.itow_ms,0x03020100);
    CS_CHECK_EQ(g.year,0x0504); CS_CHECK_EQ(g.month,6); CS_CHECK_EQ(g.day,7); CS_CHECK_EQ(g.hour,8);
    CS_CHECK_EQ(g.minute,9); CS_CHECK_EQ(g.second,10); CS_CHECK_EQ(g.valid,11);
    CS_CHECK_EQ(g.fix_type,20); CS_CHECK_EQ(g.fix_flags,21); CS_CHECK_EQ(g.num_sv,23); CS_CHECK_EQ(g.link_flags,1);
    CS_CHECK_EQ_U(g.lon_deg,0x1b1a1918); CS_CHECK_EQ_U(g.lat_deg,0x1f1e1d1c);
    CS_CHECK_EQ_U(g.h_msl_m,0x27262524); CS_CHECK_EQ_U(g.h_acc_m,0x2b2a2928); CS_CHECK_EQ_U(g.v_acc_m,0x2f2e2d2c);
    CS_CHECK_EQ_U(g.vel_n,0x33323130); CS_CHECK_EQ_U(g.vel_e,0x37363534); CS_CHECK_EQ_U(g.vel_d,0x3b3a3938);
    CS_CHECK_EQ_U(g.s_acc,0x47464544); CS_CHECK_EQ(g.p_dop,0x4d4c);
    memset(f.payload+24,0xff,4); CS_CHECK(cs_ubx_nav_pvt(&f,0,0,&g)); CS_CHECK_EQ(g.lon_deg,-1);
    f.len=91; CS_CHECK(!cs_ubx_nav_pvt(&f,0,0,&g));
}
CS_TEST(config_builder)
{
    uint8_t wire[264]; cs_ubx_t p; cs_ubx_init(&p);
    size_t n=cs_ubx_flight_config(wire,sizeof wire); CS_CHECK(n>0); CS_CHECK_EQ(feed(&p,wire,n),1);
    CS_CHECK_EQ(p.frame.cls,6); CS_CHECK_EQ(p.frame.id,0x8a);
    CS_CHECK_EQ(p.frame.payload[0],0); CS_CHECK_EQ(p.frame.payload[1],1);
    const uint8_t baud[]={1,0,0x52,0x40,0,0xc2,1,0}; CS_CHECK_MEM(p.frame.payload+4,baud,sizeof baud);
    CS_CHECK_EQ(cs_ubx_flight_config(wire,12),0);
    cs_ubx_kv_t item={0x00010001,1}; CS_CHECK_EQ(cs_ubx_valset(&item,1,1,wire,sizeof wire),0);
    item.key=0x1031001f; item.value=2; CS_CHECK_EQ(cs_ubx_valset(&item,1,1,wire,sizeof wire),0);
    item.value=1; CS_CHECK_EQ(cs_ubx_valset(&item,1,8,wire,sizeof wire),0);
    const cs_ubx_kv_t all[]={{0x10010001,1},{0x20010001,255},{0x30010001,65535},{0x40010001,UINT32_MAX},{0x50010001,UINT64_MAX}};
    n=cs_ubx_valset(all,5,3,wire,sizeof wire); CS_CHECK(n>0); CS_CHECK_EQ(feed(&p,wire,n),1);
}
CS_MAIN(stream_and_noise,nav_pvt,config_builder)
