/* GPL-3.0-only. Flight log section of flight-software-interfaces.md.
 * On-flash little endian v1: 32-byte header (magic u32, sequence u32, epoch
 * u16, used u16, first_tick u64, protected_start u32, reserved u32, CRC32 u32).
 * CRC32/IEEE covers bytes 0..27 + 32..255. Remaining page bytes are FF.
 * Records: type u8, len u8, tick u64, payload[len], CRC16/CCITT-FALSE u16.
 * Max payload 212. No split records. Two fixed RAM page buffers; append drops
 * if the next buffer is still pending. flush seals a partial page.
 * Mount reads exactly size/256 pages (one read each), no erase/program and
 * resumes at the next physical page; invalid/torn non-FF pages consume space.
 * Existing data is conservatively protected on mount until the app explicitly
 * calls allow_prelaunch_ring after persisted state proves PAD, or restores
 * protection. This also protects a reset between launch detection and flush.
 * mark_launch returns false while busy (caller must retry BEFORE next poll or
 * accepting more records); scans <=size/256+1 pages for the retention boundary; uses the last
 * appended tick as launch time. Protection rounds down to a whole 4KiB sector
 * (retains extra data) and is repeated in every new page header. Persist the
 * protected_start immediately in app backup state, then call restore_protection
 * after reset. Poll makes <=1 erase/program call and never waits for busy.
 * Erase-ahead uses the next sector only, never a protected sector. All times
 * are extended monotonic microseconds supplied by the app (not raw TIM2).
 * Failures drop/count or stop; no flight data is overwritten to make room. */
#ifndef CS_LOG_H
#define CS_LOG_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define CS_LOG_PAGE_SIZE 256u
#define CS_LOG_SECTOR_SIZE 4096u
#define CS_LOG_HEADER_SIZE 32u
#define CS_LOG_MAX_PAYLOAD 212u
#define CS_LOG_MAGIC UINT32_C(0x314c5343)
#define CS_LOG_NO_PROTECTION UINT32_MAX
#define CS_LOG_RECENT_CAP 256u
#define CS_LOG_RESUME_MAX_READS 128u
typedef struct {
    uint64_t first_tick, last_tick;
    uint32_t sequence, addr;
    uint16_t epoch;
} cs_log_recent_t;
typedef struct {
    void *ctx; uint32_t size;
    bool (*busy)(void *ctx);
    bool (*read)(void *ctx, uint32_t addr, uint8_t *buf, size_t n);
    bool (*program)(void *ctx, uint32_t addr, const uint8_t *buf, size_t n);
    bool (*erase_sector)(void *ctx, uint32_t addr);
} cs_flash_t;
typedef struct {
    cs_flash_t flash; uint8_t staging[256], pending[256];
    uint32_t writer, sequence, protected_start, protected_end, ready_sector;
    uint16_t epoch, used; uint64_t first_tick, last_tick;
    bool pending_valid, protected, stopped, mounted, dirty_protection;
    uint32_t dropped, full_drops, io_errors, corrupt_pages, mount_reads, pages_written;
    /* Incremental boot scan and bounded, chronologically ordered sector/epoch
     * index. 256 sectors cover 1 MiB at the nominal <10 KiB/s record rate.
     * The index may retain extra history, never fabricate missing history. */
    cs_log_recent_t recent[CS_LOG_RECENT_CAP];
    size_t recent_count;
    uint32_t scan_addr, scan_newest, scan_oldest, scan_newaddr, scan_oldaddr, scan_protection;
    uint32_t erase_scan_sector;
    uint8_t erase_scan_page;
    bool mounting, scan_found, scan_nonempty, recent_complete;
    bool resuming, resume_checked;
    uint32_t resume_sequence, resume_protection, resume_reads;
} cs_log_t;
/* Flight startup: begin does no I/O; each step reads at most one page.
 * step returns true only on completion. On failure mounting=false,
 * mounted=false and io_errors increments. Busy is retryable. Service sensors
 * and watchdog between steps. The synchronous wrapper is for host tools. */
bool cs_log_mount_begin(cs_log_t *log, const cs_flash_t *flash, uint16_t epoch);
bool cs_log_mount_step(cs_log_t *log);
bool cs_log_mount(cs_log_t *log, const cs_flash_t *flash, uint16_t epoch);
/* Warm-flight recovery from a CRC-checked backup checkpoint saved at most
 * one second ago. Validates its predecessor, then follows exact consecutive
 * sequences forward, stopping before an unwritten/torn/old page. Each step
 * reads <=1 page, total <=128. Invalid/stale checkpoints fail without writes;
 * caller can fall back to the full incremental mount. protected_start must be
 * a valid retained-flight sector, never NO_PROTECTION. New records use epoch.
 * Mounting buffers belong to the application; already committed NOR is kept. */
bool cs_log_resume_begin(cs_log_t *log, const cs_flash_t *flash, uint16_t epoch,
                         uint32_t writer, uint32_t next_sequence, uint32_t protected_start);
bool cs_log_resume_step(cs_log_t *log);
void cs_log_allow_prelaunch_ring(cs_log_t *log);
bool cs_log_restore_protection(cs_log_t *log, uint32_t protected_start);
bool cs_log_append(cs_log_t *log, uint8_t type, uint64_t tick_us, const void *payload, size_t len);
bool cs_log_flush(cs_log_t *log);
void cs_log_poll(cs_log_t *log);
bool cs_log_mark_launch(cs_log_t *log, uint64_t retain_us);
/* Flight launch: bounded RAM-index lookup, zero flash reads, busy is retryable.
 * Across boots, unknown downtime is zero, conservatively widening retention.
 * If the requested interval exceeds the bounded index, protect all remaining
 * device data (may stop early) rather than discard unindexed prelaunch data. */
bool cs_log_mark_launch_recent(cs_log_t *log, uint64_t retain_us);
bool cs_log_page_valid(const uint8_t page[256]);
#endif
