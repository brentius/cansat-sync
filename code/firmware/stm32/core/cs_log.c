/* GPL-3.0-only. Fixed-buffer, whole-page NOR journal. */
#include "cs_log.h"
#include "cs_crc16.h"
/* Bare Zig supplies no target libc headers; final firmware supplies these
 * standard C99 memory/libm symbols. Host builds use their system headers. */
#if __STDC_HOSTED__
#include <string.h>
#else
extern void *memset(void *,int,size_t);
extern void *memcpy(void *,const void *,size_t);
#endif
static uint16_t get16(const uint8_t *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
static uint32_t get32(const uint8_t *p) { return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24); }
static uint64_t get64(const uint8_t *p) { return (uint64_t)get32(p)|((uint64_t)get32(p+4)<<32); }
static void put16(uint8_t *p,uint16_t x) { p[0]=(uint8_t)x; p[1]=(uint8_t)(x>>8); }
static void put32(uint8_t *p,uint32_t x) { for(unsigned i=0;i<4;i++) p[i]=(uint8_t)(x>>(8*i)); }
static void put64(uint8_t *p,uint64_t x) { for(unsigned i=0;i<8;i++) p[i]=(uint8_t)(x>>(8*i)); }
static uint32_t page_crc(const uint8_t *p)
{
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<256;i++) {
        if(i>=28 && i<32) continue;
        crc^=p[i];
        for(unsigned bit=0;bit<8;bit++) crc=(crc>>1)^((crc&1u)?UINT32_C(0xedb88320):0u);
    }
    return ~crc;
}
static bool blank(const uint8_t *p) { for(unsigned i=0;i<256;i++) if(p[i]!=0xff) return false; return true; }
bool cs_log_page_valid(const uint8_t p[256])
{
    return p && get32(p)==CS_LOG_MAGIC && get16(p+10)>=32 && get16(p+10)<=256 && get32(p+28)==page_crc(p);
}
static bool newer(uint32_t a,uint32_t b) { return a!=b && (uint32_t)(a-b)<UINT32_C(0x80000000); }
static uint32_t distance(const cs_log_t *l,uint32_t from,uint32_t to)
{ return to>=from?to-from:l->flash.size-from+to; }
static uint32_t sector(uint32_t addr) { return addr&~(CS_LOG_SECTOR_SIZE-1u); }
static void reset_staging(cs_log_t *l) { memset(l->staging,0xff,256); l->used=32; l->first_tick=0; }
static uint64_t page_last_tick(const uint8_t *p)
{
    uint64_t tick=get64(p+12); size_t at=32,used=get16(p+10);
    while(at+12<=used && at+12+p[at+1]<=used) {
        tick=get64(p+at+2); at+=12+p[at+1];
    }
    return tick;
}
static void remember_page(cs_log_t *l,uint32_t addr,const uint8_t *p)
{
    cs_log_recent_t item={get64(p+12),page_last_tick(p),get32(p+4),sector(addr),get16(p+8)};
    size_t n=l->recent_count;
    for(size_t i=0;i<n;i++) {
        cs_log_recent_t *r=&l->recent[i];
        if(r->addr==item.addr && r->epoch==item.epoch) {
            if(newer(r->sequence,item.sequence)) { r->sequence=item.sequence; r->first_tick=item.first_tick; }
            if(item.last_tick>r->last_tick) r->last_tick=item.last_tick;
            return;
        }
    }
    if(n==CS_LOG_RECENT_CAP) {
        l->recent_complete=false;
        if(!newer(item.sequence,l->recent[0].sequence)) return;
        for(size_t i=1;i<n;i++) l->recent[i-1]=l->recent[i];
        n--;
    }
    size_t at=n;
    while(at && newer(l->recent[at-1].sequence,item.sequence)) {
        l->recent[at]=l->recent[at-1]; at--;
    }
    l->recent[at]=item; l->recent_count=n+1;
}
static void forget_sector(cs_log_t *l,uint32_t addr)
{
    size_t n=0;
    for(size_t i=0;i<l->recent_count;i++) if(l->recent[i].addr!=addr) l->recent[n++]=l->recent[i];
    l->recent_count=n;
}
bool cs_log_mount_begin(cs_log_t *l,const cs_flash_t *f,uint16_t epoch)
{
    if(!l || !f || f->size<4096 || f->size%4096 || !f->busy || !f->read || !f->program || !f->erase_sector) return false;
    memset(l,0,sizeof *l); l->flash=*f; l->epoch=epoch; l->protected_start=CS_LOG_NO_PROTECTION;
    l->ready_sector=UINT32_MAX; l->scan_protection=UINT32_MAX; l->erase_scan_sector=UINT32_MAX;
    l->mounting=true; l->recent_complete=true; reset_staging(l); return true;
}
bool cs_log_mount_step(cs_log_t *l)
{
    if(l && l->resuming) return cs_log_resume_step(l);
    if(!l || !l->mounting) return l && l->mounted;
    if(l->flash.busy(l->flash.ctx)) return false;
    uint8_t p[256]; uint32_t addr=l->scan_addr;
    l->mount_reads++;
    if(!l->flash.read(l->flash.ctx,addr,p,256)) { l->io_errors++; l->mounting=false; return false; }
    if(!blank(p)) {
        l->scan_nonempty=true;
        if(!cs_log_page_valid(p)) l->corrupt_pages++;
        else {
            uint32_t seq=get32(p+4); remember_page(l,addr,p);
            if(!l->scan_found || newer(seq,l->scan_newest)) {
                l->scan_newest=seq; l->scan_newaddr=addr; l->scan_protection=get32(p+20);
                l->last_tick=get16(p+8)==l->epoch?page_last_tick(p):0;
            }
            if(!l->scan_found || newer(l->scan_oldest,seq)) { l->scan_oldest=seq; l->scan_oldaddr=addr; }
            l->scan_found=true;
        }
    }
    l->scan_addr+=256;
    if(l->scan_addr<l->flash.size) return false;
    if(l->scan_found) {
        l->writer=(l->scan_newaddr+256)%l->flash.size; l->sequence=l->scan_newest+1;
        l->protected=true; l->protected_start=sector(l->scan_oldaddr);
        uint32_t protection=l->scan_protection;
        if(protection!=UINT32_MAX && protection<l->flash.size && protection%4096==0) l->protected_start=protection;
        l->protected_end=distance(l,l->writer,l->protected_start);
        l->stopped=l->protected_end==0;
    } else if(l->scan_nonempty) {
        l->protected=true; l->protected_start=0; l->stopped=true;
    }
    l->mounting=false; l->mounted=true; return true;
}
bool cs_log_mount(cs_log_t *l,const cs_flash_t *f,uint16_t epoch)
{
    if(!f || !f->busy || f->busy(f->ctx) || !cs_log_mount_begin(l,f,epoch)) return false;
    while(l->mounting) {
        if(f->busy(f->ctx)) return false;
        (void)cs_log_mount_step(l);
    }
    return l->mounted;
}
bool cs_log_resume_begin(cs_log_t *l,const cs_flash_t *f,uint16_t epoch,
                         uint32_t writer,uint32_t next_sequence,uint32_t protection)
{
    if(!f || writer>=f->size || writer%256 || protection>=f->size || protection%4096 ||
       !cs_log_mount_begin(l,f,epoch)) return false;
    l->resuming=true; l->writer=writer; l->sequence=next_sequence;
    l->resume_sequence=next_sequence; l->resume_protection=protection;
    l->recent_complete=false; return true;
}
bool cs_log_resume_step(cs_log_t *l)
{
    if(!l || !l->resuming) return l && l->mounted;
    if(l->flash.busy(l->flash.ctx)) return false;
    if(l->resume_reads>=CS_LOG_RESUME_MAX_READS) {
        l->io_errors++; l->resuming=l->mounting=false; return false;
    }
    uint8_t p[256];
    uint32_t addr=l->resume_checked?l->writer:(l->writer?l->writer-256:l->flash.size-256);
    l->resume_reads++; l->mount_reads++;
    if(!l->flash.read(l->flash.ctx,addr,p,256)) {
        l->io_errors++; l->resuming=l->mounting=false; return false;
    }
    bool valid=cs_log_page_valid(p);
    if(!l->resume_checked) {
        if(!valid || get32(p+4)!=l->resume_sequence-1u) {
            l->io_errors++; l->resuming=l->mounting=false; return false;
        }
        remember_page(l,addr,p); l->scan_nonempty=true; l->resume_checked=true; return false;
    }
    if(valid && get32(p+4)==l->sequence) {
        remember_page(l,addr,p); l->sequence++; l->writer=(addr+256)%l->flash.size;
        return false;
    }
    if(!blank(p)) { l->scan_nonempty=true; if(!valid) l->corrupt_pages++; }
    l->protected=true; l->protected_start=l->resume_protection;
    l->protected_end=distance(l,l->writer,l->protected_start);
    l->stopped=l->protected_end==0; l->dirty_protection=true;
    l->mounted=true; l->mounting=l->resuming=false; return true;
}
void cs_log_allow_prelaunch_ring(cs_log_t *l)
{
    if(!l || !l->mounted) return;
    l->protected=false; l->protected_start=UINT32_MAX; l->protected_end=0; l->stopped=false;
}
bool cs_log_restore_protection(cs_log_t *l,uint32_t start)
{
    if(!l || !l->mounted || start>=l->flash.size || start%4096) return false;
    l->protected=true; l->protected_start=start; l->protected_end=distance(l,l->writer,start);
    l->stopped=l->protected_end==0; l->dirty_protection=true; return true;
}
bool cs_log_flush(cs_log_t *l)
{
    if(!l || !l->mounted || l->stopped || l->pending_valid) return false;
    if(l->used==32 && !l->dirty_protection) return true;
    put32(l->staging,CS_LOG_MAGIC); put32(l->staging+4,l->sequence); put16(l->staging+8,l->epoch);
    put16(l->staging+10,l->used); put64(l->staging+12,l->used==32?l->last_tick:l->first_tick);
    put32(l->staging+20,l->protected?l->protected_start:UINT32_MAX); put32(l->staging+24,0);
    put32(l->staging+28,page_crc(l->staging)); memcpy(l->pending,l->staging,256);
    l->pending_valid=true; l->dirty_protection=false; reset_staging(l); return true;
}
bool cs_log_append(cs_log_t *l,uint8_t type,uint64_t tick,const void *payload,size_t len)
{
    if(!l) return false;
    if(!l->mounted || len>CS_LOG_MAX_PAYLOAD || (!payload && len) || l->stopped || tick<l->last_tick) {
        l->dropped++; if(l->stopped) l->full_drops++; return false;
    }
    if((size_t)l->used+len+12>256 && !cs_log_flush(l)) { l->dropped++; return false; }
    uint8_t *p=l->staging+l->used;
    p[0]=type; p[1]=(uint8_t)len; put64(p+2,tick);
    if(len) memcpy(p+10,payload,len);
    put16(p+10+len,cs_crc16(p,10+len,0xffff));
    if(l->used==32) l->first_tick=tick;
    l->used=(uint16_t)(l->used+12+len); l->last_tick=tick; return true;
}
static bool erase_allowed(const cs_log_t *l,uint32_t addr)
{
    if(!l->protected) return true;
    uint32_t ahead=distance(l,l->writer,addr);
    return ahead<l->protected_end && l->protected_end-ahead>=4096;
}
static void skip_tail(cs_log_t *l)
{
    uint32_t skip=4096-(l->writer%4096);
    if(l->protected && skip>=l->protected_end) { l->stopped=true; return; }
    if(l->protected) l->protected_end-=skip;
    l->writer=(l->writer+skip)%l->flash.size;
}
void cs_log_poll(cs_log_t *l)
{
    if(!l || !l->mounted || l->stopped || l->flash.busy(l->flash.ctx)) return;
    if(!l->pending_valid && l->dirty_protection) (void)cs_log_flush(l);
    if(l->pending_valid) {
        if(l->protected && l->protected_end<256) { l->stopped=true; return; }
        uint8_t p[256];
        if(!l->flash.read(l->flash.ctx,l->writer,p,256)) { l->io_errors++; l->stopped=true; return; }
        if(!blank(p)) {
            /* A reset/torn page in a partially used sector must not erase its
             * other valid pages. Skip its tail, then retry next poll. */
            if(l->writer%4096) { skip_tail(l); return; }
            if(!erase_allowed(l,l->writer)) { l->stopped=true; return; }
            if(!l->flash.erase_sector(l->flash.ctx,l->writer)) l->io_errors++;
            else { l->ready_sector=l->writer; forget_sector(l,l->writer); }
            return;
        }
        if(!l->flash.program(l->flash.ctx,l->writer,l->pending,256)) { l->io_errors++; return; }
        remember_page(l,l->writer,l->pending);
        l->pending_valid=false; l->pages_written++; l->sequence++; l->writer=(l->writer+256)%l->flash.size;
        if(l->protected) { l->protected_end-=256; if(!l->protected_end) l->stopped=true; }
        return;
    }
    /* Erase ahead during idle polls. Never erase the writer's own sector. */
    uint32_t next=(sector(l->writer)+4096)%l->flash.size;
    if(next==sector(l->writer) || next==l->ready_sector || !erase_allowed(l,next)) return;
    if(l->erase_scan_sector!=next) { l->erase_scan_sector=next; l->erase_scan_page=0; }
    uint8_t p[256];
    if(!l->flash.read(l->flash.ctx,next+256u*l->erase_scan_page,p,256)) { l->io_errors++; l->stopped=true; return; }
    if(blank(p)) {
        if(++l->erase_scan_page==16) { l->ready_sector=next; l->erase_scan_sector=UINT32_MAX; }
        return;
    }
    if(!l->flash.erase_sector(l->flash.ctx,next)) l->io_errors++;
    else { l->ready_sector=next; l->erase_scan_sector=UINT32_MAX; forget_sector(l,next); }
}
bool cs_log_mark_launch(cs_log_t *l,uint64_t retain)
{
    if(!l || !l->mounted || l->protected || l->flash.busy(l->flash.ctx)) return false;
    uint64_t cutoff=l->last_tick>retain?l->last_tick-retain:0;
    uint8_t p[256]; bool any=false, before=false, previous_epoch=false;
    uint32_t oldest_seq=0,chosen_seq=0,oldaddr=l->writer,chosen=l->writer;
    for(uint32_t addr=0;addr<l->flash.size;addr+=256) {
        if(!l->flash.read(l->flash.ctx,addr,p,256)) { l->io_errors++; return false; }
        if(!cs_log_page_valid(p)) continue;
        uint32_t seq=get32(p+4);
        if(!any || newer(oldest_seq,seq)) { oldest_seq=seq; oldaddr=addr; }
        any=true;
        /* Prior boot epochs have a different tick origin: retain them rather
         * than compare unrelated clocks. They may only widen retention. */
        if(get16(p+8)!=l->epoch) previous_epoch=true;
        if(get16(p+8)==l->epoch && get64(p+12)<=cutoff && (!before || newer(seq,chosen_seq))) {
            before=true; chosen_seq=seq; chosen=addr;
        }
    }
    if(!before || (previous_epoch && l->last_tick<retain)) chosen=any?oldaddr:l->writer;
    /* If cutoff falls in a RAM-only page newer than all committed pages, its
     * predecessor is still conservative, so no special RAM-page index needed. */
    l->protected=true; l->protected_start=sector(chosen);
    l->protected_end=distance(l,l->writer,l->protected_start);
    if(!l->protected_end) {
        /* Equal pointers mean either an empty device or all retained sectors.
         * Never interpret a full device as free space. */
        if(!l->flash.read(l->flash.ctx,l->writer,p,256)) { l->io_errors++; l->stopped=true; return false; }
        if(blank(p)) l->protected_end=l->flash.size;
        else l->stopped=true;
    }
    l->dirty_protection=true;
    if(l->pending_valid) {
        put32(l->pending+20,l->protected_start); put32(l->pending+28,page_crc(l->pending));
    }
    return true;
}

bool cs_log_mark_launch_recent(cs_log_t *l,uint64_t retain)
{
    if(!l || !l->mounted || l->protected || l->flash.busy(l->flash.ctx)) return false;
    uint64_t anchor=l->last_tick,remaining=retain; uint16_t epoch=l->epoch;
    uint32_t chosen=l->writer; bool found=false;
    for(size_t i=l->recent_count;i>0;i--) {
        const cs_log_recent_t *r=&l->recent[i-1];
        if(r->epoch!=epoch) {
            remaining=remaining>anchor?remaining-anchor:0;
            epoch=r->epoch; anchor=r->last_tick;
        }
        uint64_t cutoff=anchor>remaining?anchor-remaining:0;
        if(r->first_tick<=cutoff && (anchor>=remaining || !remaining)) {
            chosen=r->addr; found=true; break;
        }
    }
    if(!found && l->recent_count) {
        if(l->recent_complete) chosen=l->recent[0].addr;
        else {
            /* The index cannot prove where the older requested interval starts.
             * Preserve all closed sectors. Only the writer's unused tail may be
             * consumed; full-device retention takes precedence over new writes. */
            chosen=(sector(l->writer)+4096)%l->flash.size;
            if(l->writer%4096==0 && l->ready_sector!=l->writer &&
               !(!l->scan_nonempty && l->pages_written<l->flash.size/256)) {
                /* At a sector boundary after a full ring/reset, its contents
                 * may be unindexed retained data. No read-free proof of free
                 * space exists; stop rather than erase it. */
                chosen=l->writer;
            }
        }
    }
    l->protected=true; l->protected_start=sector(chosen);
    l->protected_end=distance(l,l->writer,l->protected_start);
    if(!l->protected_end) {
        if(!l->recent_count && !l->scan_nonempty && !l->pages_written) l->protected_end=l->flash.size;
        else l->stopped=true;
    }
    l->dirty_protection=true;
    if(l->pending_valid) {
        put32(l->pending+20,l->protected_start); put32(l->pending+28,page_crc(l->pending));
    }
    return true;
}
