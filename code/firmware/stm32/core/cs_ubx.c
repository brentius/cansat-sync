/* GPL-3.0-only. Key IDs/layout read from UBX-21035062 R03 (27 Jun 2023),
 * https://content.u-blox.com/sites/default/files/u-blox-M10-SPG-5.10_InterfaceDescription_UBX-21035062.pdf */
#include "cs_ubx.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <string.h>
#else
extern void *memset(void *,int,size_t);
#endif
static uint16_t u16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|(uint16_t)((uint16_t)p[1]<<8)); }
static uint32_t u32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static int32_t i32(const uint8_t *p)
{ uint32_t u=u32(p); return u<=INT32_MAX?(int32_t)u:-1-(int32_t)(UINT32_MAX-u); }
void cs_ubx_init(cs_ubx_t *p) { if(p) memset(p,0,sizeof *p); }
static void checksum(cs_ubx_t *p,uint8_t b) { p->a=(uint8_t)(p->a+b); p->b=(uint8_t)(p->b+p->a); }
bool cs_ubx_feed(cs_ubx_t *p,uint8_t byte)
{
    if(!p) return false;
    switch(p->state) {
    case 0: if(byte==0xb5) p->state=1; break;
    case 1: if(byte==0x62) { p->state=2; p->a=p->b=0; } else p->state=byte==0xb5?1:0; break;
    case 2: p->frame.cls=byte; checksum(p,byte); p->state=3; break;
    case 3: p->frame.id=byte; checksum(p,byte); p->state=4; break;
    case 4: p->frame.len=byte; checksum(p,byte); p->state=5; break;
    case 5:
        p->frame.len=(uint16_t)(p->frame.len|((uint16_t)byte<<8)); checksum(p,byte); p->pos=0;
        if(p->frame.len>CS_UBX_MAX_PAYLOAD) { p->oversize++; p->state=byte==0xb5?1:0; }
        else p->state=p->frame.len?6:7;
        break;
    case 6: p->frame.payload[p->pos++]=byte; checksum(p,byte); if(p->pos==p->frame.len) p->state=7; break;
    case 7: p->received_a=byte; p->state=8; break;
    default:
        if(p->received_a==p->a && byte==p->b) { p->frames++; p->state=0; return true; }
        p->checksum_errors++; p->state=byte==0xb5?1:0; break;
    }
    return false;
}
bool cs_ubx_nav_pvt(const cs_ubx_frame_t *f,uint32_t tick,uint8_t flags,cs_link_gnss_t *g)
{
    if(!f || !g || f->cls!=1 || f->id!=7 || f->len!=92) return false;
    const uint8_t *p=f->payload; memset(g,0,sizeof *g);
    g->tick=tick; g->itow_ms=u32(p); g->year=u16(p+4); g->month=p[6]; g->day=p[7];
    g->hour=p[8]; g->minute=p[9]; g->second=p[10]; g->valid=p[11];
    g->fix_type=p[20]; g->fix_flags=p[21]; g->num_sv=p[23]; g->link_flags=flags;
    g->lon_deg=i32(p+24); g->lat_deg=i32(p+28); g->h_msl_m=i32(p+36);
    g->h_acc_m=u32(p+40); g->v_acc_m=u32(p+44);
    g->vel_n=i32(p+48); g->vel_e=i32(p+52); g->vel_d=i32(p+56);
    g->s_acc=u32(p+68); g->p_dop=u16(p+76); return true;
}
static size_t key_size(uint32_t k)
{
    switch((k>>28)&7u) { case 1: case 2:return 1; case 3:return 2; case 4:return 4; case 5:return 8; default:return 0; }
}
size_t cs_ubx_valset(const cs_ubx_kv_t *items,size_t count,uint8_t layers,uint8_t *out,size_t cap)
{
    if(!out || (!items && count) || !layers || (layers&~7u) || cap<12) return 0;
    size_t n=4;
    for(size_t i=0;i<count;i++) {
        size_t z=key_size(items[i].key);
        if(!z || n>CS_UBX_MAX_PAYLOAD-4-z || (z<8 && items[i].value>(((uint64_t)1<<(z*8))-1)) ||
           (((items[i].key>>28)&7u)==1 && items[i].value>1)) return 0;
        n+=4+z;
    }
    if(n+8>cap) return 0;
    out[0]=0xb5; out[1]=0x62; out[2]=6; out[3]=0x8a;
    out[4]=(uint8_t)n; out[5]=(uint8_t)(n>>8);
    out[6]=0; out[7]=layers; out[8]=0; out[9]=0;
    size_t at=10;
    for(size_t i=0;i<count;i++) {
        for(unsigned j=0;j<4;j++) out[at++]=(uint8_t)(items[i].key>>(j*8));
        size_t z=key_size(items[i].key);
        for(size_t j=0;j<z;j++) out[at++]=(uint8_t)(items[i].value>>(j*8));
    }
    uint8_t a=0,b=0;
    for(size_t j=2;j<at;j++) { a=(uint8_t)(a+out[j]); b=(uint8_t)(b+a); }
    out[at++]=a; out[at++]=b; return at;
}
size_t cs_ubx_flight_config(uint8_t *out,size_t cap)
{
    static const cs_ubx_kv_t items[]={
        {0x40520001,115200}, /* UART1 baud */
        {0x10730001,1},{0x10730002,0},{0x10740001,1},{0x10740002,0},
        {0x30210001,100},{0x30210002,1},{0x20910007,1},{0x20110021,8},
        {0x1031001f,1},{0x10310001,1},{0x10310021,1},{0x10310007,1},
        {0x10310020,0},{0x10310022,0},{0x10310024,0},{0x10310025,0}
    };
    return cs_ubx_valset(items,sizeof items/sizeof items[0],1,out,cap);
}
