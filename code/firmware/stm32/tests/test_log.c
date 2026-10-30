/* GPL-3.0-only. Synthetic NOR with bit-clearing writes and timed busy.
 * CS_SOURCES: cs_log.c cs_crc16.c */
#include "cs_test.h"
#include "cs_log.h"
#define FLASH_SIZE (8u*4096u)
typedef struct { uint8_t data[FLASH_SIZE]; unsigned busy_ticks,operations,erases,programs,reads; bool fail_read,fail_program,fail_erase; } ram_flash_t;
static ram_flash_t ram;
static cs_log_t log1,journal2;
static bool is_busy(void *ctx) { ram_flash_t *r=ctx; return r->busy_ticks!=0; }
static bool read_flash(void *ctx,uint32_t addr,uint8_t *p,size_t n)
{
    ram_flash_t *r=ctx; r->reads++;
    if(r->fail_read || n>256 || addr>FLASH_SIZE || n>FLASH_SIZE-addr) return false;
    memcpy(p,r->data+addr,n); return true;
}
static bool program(void *ctx,uint32_t addr,const uint8_t *p,size_t n)
{
    ram_flash_t *r=ctx;
    CS_CHECK(!is_busy(ctx)); CS_CHECK(n<=256 && addr/256==(addr+n-1)/256);
    if(r->fail_program || r->busy_ticks || addr+n>FLASH_SIZE) return false;
    for(size_t i=0;i<n;i++) { CS_CHECK((r->data[addr+i]&p[i])==p[i]); r->data[addr+i]&=p[i]; }
    r->busy_ticks=2; r->operations++; r->programs++; return true;
}
static bool erase(void *ctx,uint32_t addr)
{
    ram_flash_t *r=ctx; CS_CHECK(!is_busy(ctx)); CS_CHECK(addr%4096==0);
    if(r->fail_erase || r->busy_ticks || addr+4096>FLASH_SIZE) return false;
    memset(r->data+addr,0xff,4096); r->busy_ticks=5; r->operations++; r->erases++; return true;
}
static cs_flash_t setup(uint32_t size)
{
    memset(&ram,0,sizeof ram); memset(ram.data,0xff,sizeof ram.data);
    cs_flash_t f={&ram,size,is_busy,read_flash,program,erase}; return f;
}
static void step(cs_log_t *l)
{
    unsigned before=ram.operations; bool busy=is_busy(&ram); cs_log_poll(l);
    CS_CHECK(ram.operations-before<=1); if(busy) CS_CHECK_EQ(ram.operations,before);
    if(ram.busy_ticks) ram.busy_ticks--;
}
static void drain(cs_log_t *l)
{
    for(unsigned i=0;i<40 && (l->pending_valid || is_busy(&ram) || l->dirty_protection);i++) step(l);
    CS_CHECK(!is_busy(&ram));
}
static void one_page(cs_log_t *l,uint64_t t)
{
    uint8_t payload[212]; memset(payload,(int)(t&255u),sizeof payload);
    CS_CHECK(cs_log_append(l,0x10,t,payload,sizeof payload)); CS_CHECK(cs_log_flush(l)); drain(l);
}
CS_TEST(packing_busy_and_dump_image)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1)); CS_CHECK_EQ(log1.mount_reads,FLASH_SIZE/256);
    for(unsigned i=0;i<8;i++) { uint8_t p[10]; memset(p,(int)i,10); CS_CHECK(cs_log_append(&log1,0x11,1000000+i,p,10)); }
    CS_CHECK(cs_log_flush(&log1)); CS_CHECK_EQ(ram.programs,0); drain(&log1);
    CS_CHECK_EQ(ram.programs,1); CS_CHECK(cs_log_page_valid(ram.data));
    uint8_t page[256]; memcpy(page,ram.data,256); page[40]^=1; CS_CHECK(!cs_log_page_valid(page));
    FILE *file=fopen("../build/host/core_b_flash.bin","wb"); CS_CHECK(file!=NULL);
    if(file) { CS_CHECK_EQ(fwrite(ram.data,1,FLASH_SIZE,file),FLASH_SIZE); CS_CHECK_EQ(fclose(file),0); }
}
CS_TEST(ring_and_erase_ahead)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<300;i++) {
        one_page(&log1,(uint64_t)i*1000000);
        /* Idle time permits erasing the next sector before writer reaches it. */
        for(unsigned j=0;j<6;j++) step(&log1);
    }
    CS_CHECK_EQ(log1.pages_written,300); CS_CHECK(ram.erases>0); CS_CHECK(!log1.stopped);
    unsigned ops=ram.operations; CS_CHECK(cs_log_mount(&journal2,&f,2)); CS_CHECK_EQ(ram.operations,ops);
    CS_CHECK_EQ(journal2.sequence,300); CS_CHECK(journal2.protected);
    /* New boot has its own tick origin; log epoch separates the times. */
    CS_CHECK(cs_log_append(&journal2,0x11,10,NULL,0));
    cs_log_allow_prelaunch_ring(&journal2); CS_CHECK(!journal2.protected && !journal2.stopped);
}
CS_TEST(retention_wrap_stop_and_reset)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<180;i++) one_page(&log1,(uint64_t)i*1000000);
    CS_CHECK(cs_log_mark_launch(&log1,UINT64_C(20000000)));
    CS_CHECK_EQ(log1.protected_start,4096); /* seq159 at page31 -> sector1. */
    uint8_t saved[4096]; memcpy(saved,ram.data+4096,4096);
    uint32_t protected_start=log1.protected_start;
    unsigned i=180;
    while(!log1.stopped && i<500) one_page(&log1,(uint64_t)i++*1000000);
    CS_CHECK(log1.stopped); CS_CHECK_MEM(saved,ram.data+4096,4096);
    CS_CHECK(!cs_log_append(&log1,1,(uint64_t)i*1000000,NULL,0)); CS_CHECK_EQ(log1.full_drops,1);
    CS_CHECK(cs_log_mount(&journal2,&f,2)); CS_CHECK_EQ(journal2.protected_start,protected_start); CS_CHECK(journal2.stopped);
    CS_CHECK(cs_log_restore_protection(&journal2,protected_start)); CS_CHECK(journal2.stopped);
    CS_CHECK_MEM(saved,ram.data+4096,4096);
}
CS_TEST(full_retention_does_not_erase)
{
    cs_flash_t f=setup(4096); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<16;i++) one_page(&log1,(uint64_t)i*1000000);
    unsigned erases=ram.erases; CS_CHECK(cs_log_mark_launch(&log1,UINT64_MAX));
    CS_CHECK(log1.stopped); for(unsigned i=0;i<10;i++) step(&log1); CS_CHECK_EQ(ram.erases,erases);
    f=setup(4096); CS_CHECK(cs_log_mount(&log1,&f,1)); CS_CHECK(cs_log_mark_launch(&log1,0));
    for(unsigned i=0;i<16;i++) one_page(&log1,(uint64_t)i*1000000);
    CS_CHECK(log1.stopped); CS_CHECK_EQ(ram.erases,0);
}
CS_TEST(overflow_corruption_io)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    uint8_t payload[212]; memset(payload,0xaa,sizeof payload);
    CS_CHECK(!cs_log_append(&log1,1,0,payload,213)); CS_CHECK_EQ(log1.dropped,1);
    CS_CHECK(cs_log_append(&log1,1,1,payload,212)); CS_CHECK(cs_log_flush(&log1));
    CS_CHECK(cs_log_append(&log1,1,2,payload,212));
    CS_CHECK(!cs_log_append(&log1,1,3,payload,212)); CS_CHECK_EQ(log1.dropped,2);
    ram.fail_program=true; step(&log1); CS_CHECK_EQ(log1.io_errors,1); CS_CHECK(log1.pending_valid);
    ram.fail_program=false; drain(&log1); CS_CHECK(cs_log_flush(&log1)); drain(&log1);
    /* Simulated torn third page. Mount must never erase either earlier page. */
    memset(ram.data+512,0,100); unsigned erases=ram.erases;
    CS_CHECK(cs_log_mount(&journal2,&f,2)); CS_CHECK_EQ(journal2.corrupt_pages,1);
    CS_CHECK(cs_log_append(&journal2,2,0,NULL,0)); CS_CHECK(cs_log_flush(&journal2)); drain(&journal2);
    CS_CHECK_EQ(ram.erases,erases); CS_CHECK(cs_log_page_valid(ram.data)); CS_CHECK(cs_log_page_valid(ram.data+256));
    CS_CHECK(cs_log_page_valid(ram.data+4096));
    ram.fail_read=true; CS_CHECK(!cs_log_mount(&journal2,&f,3)); CS_CHECK_EQ(journal2.io_errors,1);
}
CS_TEST(launch_metadata_and_persisted_fallback)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    one_page(&log1,1000); CS_CHECK(cs_log_mark_launch(&log1,60000000));
    uint32_t start=log1.protected_start;
    /* Reset before the launch header reaches NOR still mounts conservatively. */
    CS_CHECK(cs_log_mount(&journal2,&f,2)); CS_CHECK(journal2.protected);
    CS_CHECK(cs_log_restore_protection(&journal2,start)); drain(&journal2);
    CS_CHECK_EQ(journal2.pages_written,1);
    CS_CHECK(cs_log_mount(&log1,&f,3)); CS_CHECK_EQ(log1.protected_start,start);
}
CS_TEST(launch_busy_and_previous_boot_retention)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<20;i++) one_page(&log1,(uint64_t)i*1000000);
    CS_CHECK(cs_log_mount(&journal2,&f,2)); cs_log_allow_prelaunch_ring(&journal2);
    one_page(&journal2,0); one_page(&journal2,1000000);
    ram.busy_ticks=1; CS_CHECK(!cs_log_mark_launch(&journal2,60000000));
    step(&journal2); /* busy poll issues no operation */
    CS_CHECK(cs_log_mark_launch(&journal2,60000000));
    CS_CHECK_EQ(journal2.protected_start,0); /* current-boot tick zero must not hide epoch1 */
    drain(&journal2); CS_CHECK(journal2.protected);
}
static bool huge_read(void *ctx,uint32_t addr,uint8_t *p,size_t n)
{
    unsigned *reads=ctx; ++*reads;
    CS_CHECK(n<=256 && addr<16u*1024u*1024u && n<=16u*1024u*1024u-addr);
    memset(p,0xff,n); return true;
}
static bool huge_idle(void *ctx) { (void)ctx; return false; }
CS_TEST(production_geometry_mount_bound)
{
    unsigned reads=0;
    /* Writes are deliberately not invoked in this geometry/read-count check;
     * the separate RAM model tests their busy/erase/program rules. */
    cs_flash_t f={&reads,16u*1024u*1024u,huge_idle,huge_read,program,erase};
    CS_CHECK(cs_log_mount(&log1,&f,1)); CS_CHECK_EQ(reads,65536); CS_CHECK_EQ(log1.mount_reads,65536);
}
CS_TEST(erase_failure_is_retryable)
{
    cs_flash_t f=setup(4096); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<16;i++) one_page(&log1,i);
    CS_CHECK(cs_log_append(&log1,1,17,NULL,0)); CS_CHECK(cs_log_flush(&log1));
    ram.fail_erase=true; step(&log1); CS_CHECK_EQ(log1.io_errors,1); CS_CHECK(log1.pending_valid);
    ram.fail_erase=false; drain(&log1); CS_CHECK_EQ(log1.pages_written,17); CS_CHECK_EQ(ram.erases,1);
}
CS_TEST(incremental_mount_and_bounded_poll)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount_begin(&log1,&f,1));
    CS_CHECK_EQ(ram.reads,0); CS_CHECK(!log1.mounted);
    ram.busy_ticks=1; CS_CHECK(!cs_log_mount_step(&log1)); CS_CHECK_EQ(ram.reads,0); ram.busy_ticks=0;
    for(unsigned i=0;i<FLASH_SIZE/256;i++) {
        unsigned reads=ram.reads; bool done=cs_log_mount_step(&log1);
        CS_CHECK_EQ(ram.reads-reads,1); CS_CHECK_EQ(done,i+1==FLASH_SIZE/256);
    }
    CS_CHECK(log1.mounted && !log1.mounting);
    for(unsigned i=0;i<40;i++) { unsigned reads=ram.reads; step(&log1); CS_CHECK(ram.reads-reads<=1); }
    f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount_begin(&log1,&f,1)); ram.fail_read=true;
    CS_CHECK(!cs_log_mount_step(&log1)); CS_CHECK(!log1.mounting && !log1.mounted); CS_CHECK_EQ(log1.io_errors,1);
}
CS_TEST(recent_retention_no_flash_scan)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<180;i++) one_page(&log1,(uint64_t)i*1000000);
    unsigned reads=ram.reads; CS_CHECK(cs_log_mark_launch_recent(&log1,20000000));
    CS_CHECK_EQ(ram.reads,reads); CS_CHECK_EQ(log1.protected_start,4096);
    uint8_t saved[4096]; memcpy(saved,ram.data+4096,4096);
    for(unsigned i=180;!log1.stopped && i<500;i++) one_page(&log1,(uint64_t)i*1000000);
    CS_CHECK(log1.stopped); CS_CHECK_MEM(saved,ram.data+4096,4096);
    f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    CS_CHECK(cs_log_mark_launch_recent(&log1,60000000)); CS_CHECK_EQ(log1.protected_end,FLASH_SIZE);
}
CS_TEST(recent_retention_across_boot_epochs)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<100;i++) one_page(&log1,(uint64_t)i*1000000);
    CS_CHECK(cs_log_mount(&journal2,&f,2)); cs_log_allow_prelaunch_ring(&journal2);
    one_page(&journal2,1000000); one_page(&journal2,2000000);
    unsigned reads=ram.reads; CS_CHECK(cs_log_mark_launch_recent(&journal2,20000000));
    CS_CHECK_EQ(ram.reads,reads); CS_CHECK_EQ(journal2.protected_start,5*4096);
    /* 2 s in this boot + 18 s before the last epoch's final 99 s: 81 s,
     * rounded down to the sector beginning at 80 s. Unknown downtime only
     * widens the retained interval. */
}
CS_TEST(checkpoint_resume_keeps_commits_and_separates_boot_ticks)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<20;i++) one_page(&log1,(uint64_t)i*1000000);
    CS_CHECK(cs_log_mark_launch_recent(&log1,10000000)); drain(&log1);
    uint32_t writer=log1.writer,seq=log1.sequence,protection=log1.protected_start;
    for(unsigned i=20;i<27;i++) one_page(&log1,(uint64_t)i*1000000);
    uint32_t expected_writer=log1.writer,expected_seq=log1.sequence; unsigned ops=ram.operations;
    CS_CHECK(cs_log_resume_begin(&journal2,&f,2,writer,seq,protection));
    for(unsigned i=0;journal2.mounting && i<CS_LOG_RESUME_MAX_READS;i++) {
        unsigned reads=ram.reads; (void)cs_log_mount_step(&journal2); CS_CHECK_EQ(ram.reads-reads,1);
    }
    CS_CHECK(journal2.mounted); CS_CHECK_EQ(journal2.mount_reads,9);
    CS_CHECK_EQ(journal2.writer,expected_writer); CS_CHECK_EQ(journal2.sequence,expected_seq);
    CS_CHECK_EQ(journal2.protected_start,protection); CS_CHECK_EQ(ram.operations,ops);
    CS_CHECK(cs_log_append(&journal2,1,0,NULL,0)); CS_CHECK(cs_log_flush(&journal2)); drain(&journal2);
    CS_CHECK(cs_log_page_valid(ram.data+expected_writer));
}
CS_TEST(checkpoint_rejection_and_torn_tail_preserve_committed_pages)
{
    cs_flash_t f=setup(FLASH_SIZE); CS_CHECK(cs_log_mount(&log1,&f,1));
    for(unsigned i=0;i<3;i++) one_page(&log1,i);
    uint32_t writer=log1.writer,seq=log1.sequence;
    CS_CHECK(cs_log_resume_begin(&journal2,&f,2,writer,seq+1,0));
    CS_CHECK(!cs_log_resume_step(&journal2)); CS_CHECK(!journal2.mounting && !journal2.mounted);
    CS_CHECK_EQ(journal2.io_errors,1);
    memset(ram.data+writer,0,100); unsigned ops=ram.operations;
    CS_CHECK(cs_log_resume_begin(&journal2,&f,2,writer,seq,0));
    CS_CHECK(!cs_log_resume_step(&journal2)); CS_CHECK(cs_log_resume_step(&journal2));
    CS_CHECK_EQ(journal2.corrupt_pages,1); CS_CHECK_EQ(ram.operations,ops);
    CS_CHECK(cs_log_append(&journal2,1,0,NULL,0)); CS_CHECK(cs_log_flush(&journal2)); drain(&journal2);
    CS_CHECK(cs_log_page_valid(ram.data)); CS_CHECK(cs_log_page_valid(ram.data+4096));
    CS_CHECK_EQ(ram.erases,0);
}
CS_MAIN(packing_busy_and_dump_image,ring_and_erase_ahead,retention_wrap_stop_and_reset,full_retention_does_not_erase,overflow_corruption_io,launch_metadata_and_persisted_fallback,launch_busy_and_previous_boot_retention,production_geometry_mount_bound,erase_failure_is_retryable,incremental_mount_and_bounded_poll,recent_retention_no_flash_scan,recent_retention_across_boot_epochs,checkpoint_resume_keeps_commits_and_separates_boot_ticks,checkpoint_rejection_and_torn_tail_preserve_committed_pages)

