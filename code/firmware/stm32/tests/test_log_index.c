/* GPL-3.0-only. Production-size history eviction and reset retention.
 * CS_SOURCES: cs_log.c cs_crc16.c */
#include "cs_test.h"
#include "cs_log.h"
#define SIZE (2u*1024u*1024u)
static uint8_t memory[SIZE];
static unsigned reads,erases;
static cs_log_t journal;
static bool idle(void *ctx) { (void)ctx; return false; }
static bool read_page(void *ctx,uint32_t a,uint8_t *p,size_t n)
{ (void)ctx; reads++; if(a>SIZE || n>SIZE-a || n>256) return false; memcpy(p,memory+a,n); return true; }
static bool write_page(void *ctx,uint32_t a,const uint8_t *p,size_t n)
{
    (void)ctx; if(a>SIZE || n>SIZE-a || n>256 || a/256!=(a+n-1)/256) return false;
    for(size_t i=0;i<n;i++) { CS_CHECK((memory[a+i]&p[i])==p[i]); memory[a+i]&=p[i]; }
    return true;
}
static bool erase_sector(void *ctx,uint32_t a)
{ (void)ctx; if(a%4096 || a>SIZE-4096) return false; memset(memory+a,255,4096); erases++; return true; }
static cs_flash_t flash={NULL,SIZE,idle,read_page,write_page,erase_sector};
static void page(uint64_t tick)
{
    uint8_t p[212]={0};
    CS_CHECK(cs_log_append(&journal,0x10,tick,p,sizeof p)); CS_CHECK(cs_log_flush(&journal));
    for(unsigned i=0;i<3 && journal.pending_valid;i++) cs_log_poll(&journal);
    CS_CHECK(!journal.pending_valid);
}
static void populated(void)
{
    memset(memory,255,sizeof memory); reads=erases=0;
    CS_CHECK(cs_log_mount(&journal,&flash,1));
    for(unsigned i=0;i<300u*16u;i++) page((uint64_t)(i+1u)*1000000u);
    CS_CHECK_EQ(journal.recent_count,CS_LOG_RECENT_CAP); CS_CHECK(!journal.recent_complete);
}
CS_TEST(index_eviction_still_preserves_recent_interval_across_reset)
{
    populated(); CS_CHECK(cs_log_mount(&journal,&flash,2)); cs_log_allow_prelaunch_ring(&journal);
    page(1000000); page(2000000);
    unsigned before=reads; CS_CHECK(cs_log_mark_launch_recent(&journal,60000000));
    CS_CHECK_EQ(reads,before); CS_CHECK_EQ(journal.protected_start,296u*4096u);
    uint8_t retained[4096]; memcpy(retained,memory+296u*4096u,sizeof retained);
    for(unsigned i=3;!journal.stopped && i<10000;i++) page((uint64_t)i*1000000u);
    CS_CHECK(journal.stopped); CS_CHECK_MEM(retained,memory+296u*4096u,sizeof retained);
}
CS_TEST(unindexed_interval_fails_closed_without_scanning_or_erasing)
{
    populated(); uint8_t oldest[4096]; memcpy(oldest,memory,sizeof oldest);
    unsigned before=reads; CS_CHECK(cs_log_mark_launch_recent(&journal,UINT64_MAX));
    CS_CHECK_EQ(reads,before); CS_CHECK_EQ(journal.protected_start,301u*4096u);
    for(unsigned i=4801;!journal.stopped && i<5000;i++) page((uint64_t)i*1000000u);
    CS_CHECK(journal.stopped); CS_CHECK_EQ(erases,0); CS_CHECK_MEM(oldest,memory,sizeof oldest);
}
CS_TEST(unindexed_full_ring_never_erases_the_writer_sector)
{
    populated();
    for(unsigned i=4801;i<=540u*16u;i++) page((uint64_t)i*1000000u);
    CS_CHECK_EQ(journal.writer%4096,0);
    uint8_t saved[4096]; memcpy(saved,memory+journal.writer,sizeof saved);
    unsigned before=reads,old_erases=erases;
    CS_CHECK(cs_log_mark_launch_recent(&journal,UINT64_MAX)); CS_CHECK_EQ(reads,before);
    CS_CHECK(journal.stopped);
    for(unsigned i=0;i<32;i++) cs_log_poll(&journal);
    CS_CHECK_EQ(erases,old_erases); CS_CHECK_MEM(saved,memory+journal.writer,sizeof saved);
}
CS_MAIN(index_eviction_still_preserves_recent_interval_across_reset,unindexed_interval_fails_closed_without_scanning_or_erasing,unindexed_full_ring_never_erases_the_writer_sector)
