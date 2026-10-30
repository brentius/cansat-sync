/* GPL-3.0-only. Command-level NOR and SX1262 device models.
 * CS_SOURCES: drivers/drv_devices.c */
#include "cs_test.h"
#include "../drivers/drv_devices.h"
typedef struct {
    uint8_t mem[8192], packet[255], status, packet_type, last[32];
    size_t len, max_n;
    uint16_t irq;
    unsigned calls, reset_count, tx_start;
    bool fail, busy, txen, reset;
} fake_t;
static bool xfer(void *ctx,drv_spi_device_t dev,const uint8_t *tx,uint8_t *rx,size_t n)
{
    fake_t *f=ctx; size_t i; uint32_t addr;
    f->calls++; if(f->fail) return false;
    if(rx) memset(rx,0,n);
    if(dev==DRV_FLASH) {
        switch(tx[0]) {
        case 0x9f:
            /* W25Q128JV ignores identification commands during BUSY. */
            if(rx && !(f->status&1u)) { rx[1]=0xef; rx[2]=0x40; rx[3]=0x18; }
            break;
        case 0x05: if(rx) rx[1]=f->status; break;
        case 0x06: f->status|=2; break;
        case 0x03:
            addr=(uint32_t)tx[1]<<16|(uint32_t)tx[2]<<8|tx[3];
            for(i=4;i<n;i++) rx[i]=f->mem[(addr+i-4)%sizeof f->mem];
            break;
        case 0x02:
            if(!(f->status&2)) return false;
            addr=(uint32_t)tx[1]<<16|(uint32_t)tx[2]<<8|tx[3];
            for(i=4;i<n;i++) f->mem[(addr+i-4)%sizeof f->mem]&=tx[i];
            f->status=1; break;
        case 0x20:
            if(!(f->status&2)) return false;
            addr=(uint32_t)tx[1]<<16|(uint32_t)tx[2]<<8|tx[3];
            memset(f->mem+addr%sizeof f->mem,255,4096); f->status=1; break;
        default: break;
        }
    } else {
        if(n>f->max_n) f->max_n=n;
        f->len=n; memcpy(f->last,tx,n>32?32:n);
        switch(tx[0]) {
        case 0x8a: f->packet_type=tx[1]; break;
        case 0x11: if(rx) rx[2]=f->packet_type; break;
        case 0x12: if(rx) { rx[2]=(uint8_t)(f->irq>>8); rx[3]=(uint8_t)f->irq; } break;
        case 0x02: f->irq=0; break;
        case 0x0e: for(i=2;i<n;i++) f->packet[tx[1]+i-2]=tx[i]; break;
        case 0x83: f->tx_start++; break;
        default: break;
        }
    }
    return true;
}
static bool busy(void *ctx) { return ((fake_t *)ctx)->busy; }
static void reset(void *ctx,bool on) { fake_t *f=ctx; f->reset=on; f->reset_count++; }
static void txen(void *ctx,bool on) { ((fake_t *)ctx)->txen=on; }
CS_TEST(nor_bounds_busy_and_failure)
{
    fake_t f={0}; drv_flash_t d; uint8_t buf[256],out[256];
    memset(f.mem,255,sizeof f.mem); memset(buf,0xa5,sizeof buf);
    drv_flash_init(&d,(drv_spi_t){&f,xfer}); CS_CHECK(d.ok);
    CS_CHECK(!drv_flash_program(&d,250,buf,10,0)); CS_CHECK(!drv_flash_erase(&d,1,0));
    CS_CHECK(!drv_flash_read(&d,0xffffffu,out,2)); CS_CHECK(!drv_flash_read(&d,0,out,257));
    CS_CHECK(drv_flash_program(&d,256,buf,256,0)); CS_CHECK(d.busy);
    CS_CHECK(!drv_flash_read(&d,256,out,256)); drv_flash_poll(&d,1000); CS_CHECK(d.busy);
    f.status=0; drv_flash_poll(&d,2000); CS_CHECK(!d.busy);
    CS_CHECK(drv_flash_read(&d,256,out,256)); CS_CHECK_MEM(out,buf,256);
    CS_CHECK(drv_flash_erase(&d,0,3000)); f.status=0; drv_flash_poll(&d,4000);
    CS_CHECK(drv_flash_read(&d,256,out,256)); memset(buf,255,sizeof buf); CS_CHECK_MEM(out,buf,256);
    CS_CHECK(drv_flash_program(&d,0,buf,1,5000)); drv_flash_poll(&d,1000000);
    CS_CHECK(!d.ok); CS_CHECK(d.errors>0); CS_CHECK(!d.busy);
    f.fail=true; drv_flash_init(&d,(drv_spi_t){&f,xfer}); CS_CHECK(!d.ok);
}
CS_TEST(radio_staged_tx_and_timeouts)
{
    fake_t f={0}; drv_radio_t d; uint8_t p[255]; uint32_t now;
    board_radio_cfg_t cfg={869525000,7,125000,1,8,true,false,false,14,1800};
    memset(p,0xa5,sizeof p);
    drv_radio_init(&d,(drv_spi_t){&f,xfer},(drv_radio_io_t){&f,busy,reset,txen},0);
    cfg.tcxo_mv=1900; CS_CHECK(!drv_radio_configure(&d,&cfg,0)); cfg.tcxo_mv=1800;
    CS_CHECK(drv_radio_configure(&d,&cfg,0));
    CS_CHECK(!drv_radio_send(&d,p,40,0));
    for(now=0;now<100000;now+=100) drv_radio_poll(&d,now);
    CS_CHECK(d.ok); CS_CHECK(!f.txen); CS_CHECK(f.reset_count>=2);
    /* round(869525000 * 2^25 / 32000000), computed independently. */
    CS_CHECK_EQ(drv_radio_pll(869525000),911763046u);
    CS_CHECK(drv_radio_send(&d,p,255,now)); CS_CHECK(d.transmitting);
    CS_CHECK(!drv_radio_send(&d,p,40,now));
    for(;now<200000;now+=100) drv_radio_poll(&d,now);
    CS_CHECK_EQ(f.tx_start,1); CS_CHECK(f.txen); CS_CHECK_MEM(p,f.packet,255);
    CS_CHECK(f.max_n<=18); f.irq=1; drv_radio_poll(&d,now+2000);
    CS_CHECK_EQ(d.tx_count,1); CS_CHECK(!f.txen && !d.transmitting);
    CS_CHECK(drv_radio_send(&d,p,40,now+3000));
    drv_radio_poll(&d,now+6000000); CS_CHECK(!f.txen && !d.ok); CS_CHECK(d.errors>0);
    f.busy=true;
    for(now=7000000;now<7400000;now+=1000) drv_radio_poll(&d,now);
    CS_CHECK(!d.ok); CS_CHECK(!f.txen);
}
CS_TEST(nor_reset_during_write)
{
    fake_t f={0}; drv_flash_t d;
    f.status=1;
    drv_flash_init(&d,(drv_spi_t){&f,xfer});
    CS_CHECK(!d.ok);
    drv_flash_poll(&d,1000); CS_CHECK(!d.ok);
    f.status=0;
    drv_flash_poll(&d,2000);
    /* Complete bounded startup probing after the interrupted erase finishes. */
    drv_flash_poll(&d,3000); drv_flash_poll(&d,4000); drv_flash_poll(&d,5000);
    CS_CHECK(d.ok); CS_CHECK(!d.busy); CS_CHECK_EQ(d.errors,0);
}
CS_MAIN(nor_bounds_busy_and_failure,radio_staged_tx_and_timeouts,nor_reset_during_write)
