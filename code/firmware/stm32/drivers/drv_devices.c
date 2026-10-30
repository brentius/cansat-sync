/* GPL-3.0-only. NOR and radio commands from primary sources in SOURCES.md. */
#include "drv_devices.h"

#define FLASH_SIZE 0x1000000u
static bool flash_xfer(drv_flash_t *d,const uint8_t *tx,uint8_t *rx,size_t n)
{
    if(!d->bus.transfer(d->bus.ctx,DRV_FLASH,tx,rx,n)) {
        d->errors++; d->ok=false; d->busy=false; return false;
    }
    return true;
}
static bool flash_status(drv_flash_t *d,uint8_t *status)
{
    const uint8_t tx[2]={0x05,0}; uint8_t rx[2];
    if(!flash_xfer(d,tx,rx,2)) return false;
    *status=rx[1]; return true;
}
void drv_flash_init(drv_flash_t *d,drv_spi_t bus)
{
    const uint8_t tx[4]={0x9f,0,0,0}; uint8_t rx[4],sr=0;
    const uint8_t status2[2]={0x35,0},status3[2]={0x15,0};
    *d=(drv_flash_t){0}; d->bus=bus;
    /* A warm MCU reset does not reset NOR. Only status reads are reliable
     * until a previously launched program/erase completes. Never reset NOR
     * to make JEDEC identification work: that could interrupt stored data. */
    if(!flash_status(d,&sr)) return;
    if(sr&1u) { d->probing=true; d->busy=true; d->deadline=1000000u; return; }
    if(!flash_xfer(d,tx,rx,4)) return;
    if(rx[1]!=0xef || rx[2]!=0x40 || rx[3]!=0x18) { d->errors++; return; }
    /* Never clear protection or reset a possibly in-progress program/erase.
     * A watchdog reset can occur during a NOR operation. */
    if(sr&0x7cu) { d->errors++; return; }
    if(!flash_xfer(d,status2,rx,2)) return;
    if(rx[1]&0x40u) { d->errors++; return; } /* CMP can invert the BP region */
    if(!flash_xfer(d,status3,rx,2)) return;
    if(rx[1]&4u) { d->errors++; return; } /* WPS: individual-lock regime */
    d->ok=true; d->busy=(sr&1u)!=0; d->deadline=1000000u;
}
void drv_flash_poll(drv_flash_t *d,uint32_t now)
{
    uint8_t sr;
    if((!d->ok && !d->probing) || !d->busy || !drv_due(now,d->next_poll)) return;
    d->next_poll=now+1000u;
    if(!flash_status(d,&sr)) return;
    if(!(sr&1u)) {
        if(d->probing) drv_flash_init(d,d->bus);
        else d->busy=false;
    } else if(drv_due(now,d->deadline)) {
        d->errors++; d->ok=false; d->busy=false; d->probing=false;
    }
}
static bool flash_bounds(drv_flash_t *d,uint32_t addr,size_t n)
{ return d->ok && !d->busy && n>0 && n<=256 && addr<FLASH_SIZE && n<=FLASH_SIZE-addr; }
bool drv_flash_read(drv_flash_t *d,uint32_t addr,uint8_t *out,size_t n)
{
    uint8_t tx[260]={0},rx[260]; size_t i;
    if(!out || !flash_bounds(d,addr,n)) return false;
    tx[0]=3; tx[1]=(uint8_t)(addr>>16); tx[2]=(uint8_t)(addr>>8); tx[3]=(uint8_t)addr;
    if(!flash_xfer(d,tx,rx,n+4)) return false;
    for(i=0;i<n;i++) out[i]=rx[i+4];
    return true;
}
static bool flash_write_enable(drv_flash_t *d)
{
    const uint8_t cmd=6; uint8_t sr;
    if(!flash_xfer(d,&cmd,0,1) || !flash_status(d,&sr)) return false;
    if((sr&3u)!=2u) { d->errors++; d->ok=false; return false; }
    return true;
}
bool drv_flash_program(drv_flash_t *d,uint32_t addr,const uint8_t *data,size_t n,uint32_t now)
{
    uint8_t tx[260]; size_t i;
    if(!data || !flash_bounds(d,addr,n) || n>256u-(addr&255u)) return false;
    tx[0]=2; tx[1]=(uint8_t)(addr>>16); tx[2]=(uint8_t)(addr>>8); tx[3]=(uint8_t)addr;
    for(i=0;i<n;i++) tx[i+4]=data[i];
    if(!flash_write_enable(d) || !flash_xfer(d,tx,0,n+4)) return false;
    d->busy=true; d->deadline=now+10000u; d->next_poll=now+100u; return true;
}
bool drv_flash_erase(drv_flash_t *d,uint32_t addr,uint32_t now)
{
    uint8_t tx[4]={0x20,(uint8_t)(addr>>16),(uint8_t)(addr>>8),(uint8_t)addr};
    if(!flash_bounds(d,addr,1) || (addr&4095u)) return false;
    if(!flash_write_enable(d) || !flash_xfer(d,tx,0,4)) return false;
    d->busy=true; d->deadline=now+1000000u; d->next_poll=now+1000u; return true;
}

uint32_t drv_radio_pll(uint32_t hz)
{ return (uint32_t)(((uint64_t)hz*33554432u+16000000u)/32000000u); }
static int tcxo_code(uint16_t mv)
{
    static const uint16_t volts[]={1600,1700,1800,2200,2400,2700,3000,3300}; unsigned i;
    for(i=0;i<8;i++) if(volts[i]==mv) return (int)i;
    return -1;
}
static void radio_fault(drv_radio_t *d,uint32_t now)
{
    d->errors++; d->ok=false; d->transmitting=false; d->tx_stage=0;
    d->io.txen(d->io.ctx,false); d->io.reset(d->io.ctx,true);
    d->stage=0; d->deadline=now+100000u;
}
static bool radio_xfer(drv_radio_t *d,uint8_t *tx,uint8_t *rx,size_t n,uint32_t now)
{
    if(!d->bus.transfer(d->bus.ctx,DRV_RADIO,tx,rx,n)) { radio_fault(d,now); return false; }
    return true;
}
void drv_radio_init(drv_radio_t *d,drv_spi_t bus,drv_radio_io_t io,uint32_t now)
{
    *d=(drv_radio_t){0}; d->bus=bus; d->io=io; d->deadline=now+1000u;
    io.txen(io.ctx,false); io.reset(io.ctx,true);
}
bool drv_radio_configure(drv_radio_t *d,const board_radio_cfg_t *cfg,uint32_t now)
{
    if(!cfg || d->transmitting || cfg->frequency_hz<863000000u || cfg->frequency_hz>870000000u ||
       cfg->sf<5 || cfg->sf>12 || cfg->cr<1 || cfg->cr>4 || cfg->preamble<4 ||
       (cfg->bw_hz!=125000 && cfg->bw_hz!=250000 && cfg->bw_hz!=500000) ||
       cfg->power_dbm< -9 || cfg->power_dbm>22 || (cfg->tcxo_mv && tcxo_code(cfg->tcxo_mv)<0)) return false;
    d->cfg=*cfg; d->configured=true; d->ok=false; d->stage=0; d->deadline=now+1000u;
    d->io.txen(d->io.ctx,false); d->io.reset(d->io.ctx,true); return true;
}
static size_t packet_params(const drv_radio_t *d,uint8_t *tx,uint8_t len)
{
    tx[0]=0x8c; tx[1]=(uint8_t)(d->cfg.preamble>>8); tx[2]=(uint8_t)d->cfg.preamble;
    tx[3]=d->cfg.implicit_header?1:0; tx[4]=len; tx[5]=d->cfg.crc_on?1:0; tx[6]=0;
    return 7;
}
bool drv_radio_send(drv_radio_t *d,const uint8_t *p,size_t n,uint32_t now)
{
    size_t i;
    if(!p || !n || n>255 || !d->ok || d->transmitting) return false;
    for(i=0;i<n;i++) d->packet[i]=p[i];
    d->len=(uint8_t)n; d->offset=0; d->tx_stage=1; d->transmitting=true;
    d->deadline=now+5500000u; return true;
}
void drv_radio_poll(drv_radio_t *d,uint32_t now)
{
    uint8_t tx[18]={0},rx[18]={0}; size_t n=0;
    if(!d->ok) {
        if(!drv_due(now,d->deadline) && d->stage<2) return;
        if(d->stage==0) {
            d->io.reset(d->io.ctx,false); d->stage=1; d->deadline=now+10000u; return;
        }
        if(d->io.busy(d->io.ctx)) {
            if(drv_due(now,d->deadline+100000u)) radio_fault(d,now);
            return;
        }
        /* Reset to standby even before an app supplies a radio configuration. */
        if(d->stage==1) { tx[0]=0x80; tx[1]=0; n=2; }
        else if(!d->configured) return;
        else switch(d->stage) {
        case 2: tx[0]=0x96; tx[1]=0; n=2; break; /* LDO, conservative module mode */
        case 3:
            if(!d->cfg.tcxo_mv) { d->stage++; return; }
            tx[0]=0x97; tx[1]=(uint8_t)tcxo_code(d->cfg.tcxo_mv);
            tx[2]=0; tx[3]=1; tx[4]=0x40; n=5; break; /* 5 ms at 64 kHz */
        case 4: tx[0]=0x89; tx[1]=0x7f; n=2; break;
        case 5: tx[0]=0x98; tx[1]=0xd7; tx[2]=0xdb; n=3; break;
        case 6: tx[0]=0x07; n=3; break; /* clear prior oscillator errors */
        case 7: tx[0]=0x8a; tx[1]=1; n=2; break;
        case 8: {
            uint32_t pll=drv_radio_pll(d->cfg.frequency_hz);
            tx[0]=0x86; tx[1]=(uint8_t)(pll>>24); tx[2]=(uint8_t)(pll>>16);
            tx[3]=(uint8_t)(pll>>8); tx[4]=(uint8_t)pll; n=5; break;
        }
        case 9: tx[0]=0x95; tx[1]=4; tx[2]=7; tx[3]=0; tx[4]=1; n=5; break;
        case 10: tx[0]=0x8e; tx[1]=(uint8_t)d->cfg.power_dbm; tx[2]=4; n=3; break;
        case 11:
            tx[0]=0x8b; tx[1]=d->cfg.sf;
            tx[2]=d->cfg.bw_hz==125000?4:(d->cfg.bw_hz==250000?5:6);
            tx[3]=d->cfg.cr; tx[4]=d->cfg.ldro?1:0; n=5; break;
        case 12: n=packet_params(d,tx,40); break;
        case 13: tx[0]=0x8f; n=3; break; /* buffer bases zero */
        case 14: tx[0]=0x9d; n=2; break; /* GPIO TXEN/RXEN, DIO2 switch disabled */
        case 15: tx[0]=0x08; tx[1]=2; tx[2]=1; tx[3]=2; tx[4]=1; n=9; break;
        case 16: tx[0]=0x93; tx[1]=0x20; n=2; break; /* standby RC fallback */
        case 17: tx[0]=0x1d; tx[1]=0x08; tx[2]=0xd8; n=5; break;
        case 18: tx[0]=0x0d; tx[1]=0x08; tx[2]=0xd8; tx[3]=d->packet[0]|0x1e; n=4; break;
        case 19: tx[0]=0x1d; tx[1]=0x08; tx[2]=0x89; n=5; break;
        case 20:
            tx[0]=0x0d; tx[1]=0x08; tx[2]=0x89;
            tx[3]=d->cfg.bw_hz==500000?(uint8_t)(d->packet[0]&~4u):(uint8_t)(d->packet[0]|4u);
            n=4; break;
        case 21: tx[0]=0x0d; tx[1]=0x07; tx[2]=0x40; tx[3]=0x14; tx[4]=0x24; n=5; break;
        case 22: tx[0]=0x11; n=3; break;
        case 23: tx[0]=0x17; n=4; break;
        default: d->ok=true; return;
        }
        if(!radio_xfer(d,tx,rx,n,now)) return;
        if(d->stage==17 || d->stage==19) d->packet[0]=rx[4];
        if((d->stage==22 && rx[2]!=1) || (d->stage==23 && (rx[2] || rx[3]))) {
            radio_fault(d,now); return;
        }
        d->stage++; d->deadline=now; return;
    }
    if(!d->transmitting) return;
    if(drv_due(now,d->deadline)) { radio_fault(d,now); return; }
    if(d->io.busy(d->io.ctx)) return;
    switch(d->tx_stage) {
    case 1: tx[0]=0x02; tx[1]=0xff; tx[2]=0xff; n=3; break;
    case 2: n=packet_params(d,tx,d->len); break;
    case 3: {
        size_t i,count=(size_t)d->len-d->offset;
        if(count>16) count=16;
        tx[0]=0x0e; tx[1]=d->offset;
        for(i=0;i<count;i++) tx[i+2]=d->packet[d->offset+i];
        if(!radio_xfer(d,tx,rx,count+2,now)) return;
        d->offset=(uint8_t)(d->offset+count);
        if(d->offset==d->len) d->tx_stage++;
        return;
    }
    case 4:
        d->io.txen(d->io.ctx,true);
        tx[0]=0x83; tx[1]=4; tx[2]=0xe2; tx[3]=0; n=4; break; /* 5 s */
    case 5:
        if(!drv_due(now,d->next_poll)) return;
        d->next_poll=now+1000u; tx[0]=0x12;
        if(!radio_xfer(d,tx,rx,4,now)) return;
        if(rx[2]&2u) { radio_fault(d,now); return; }
        if(rx[3]&1u) {
            d->io.txen(d->io.ctx,false); d->transmitting=false; d->tx_count++;
        }
        return;
    default: radio_fault(d,now); return;
    }
    if(radio_xfer(d,tx,rx,n,now)) d->tx_stage++;
}
