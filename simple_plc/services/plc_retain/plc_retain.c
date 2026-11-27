/*
 * plc_retain.c - Layer 3 (PLC Application Services)
 *
 * Implementation of the retain scheme declared in plc_retain.h. See that
 * file's header comment, splc_flash_define.h (Flash layout/size constants,
 * section "Retain page log") and docs/handoff.md.
 *
 * NOT built with the real toolchain, NOT run on a board yet.
 *
 * ---------------------------------------------------------------------------
 * Format v1 ("page log"): ONE sector, append-only, 144-byte pages.
 * ---------------------------------------------------------------------------
 * Only the FIRST sector of the retain region (SPLC_FLASH_RETAIN_BASE_ADDR) is
 * written. The region itself stays SPLC_FLASH_RETAIN_SECTOR_COUNT (3) sectors
 * long so no other address moves; sectors +1 and +2 are reserved.
 *
 * Page layout (big-endian), SPLC_RETAIN_PAGE_SIZE = 144 bytes:
 *   offset 0..3     uint32_t seq_num   newer page = larger number
 *   offset 4..5     uint16_t fmt_count high byte SPLC_RETAIN_PAGE_FMT (0xA5),
 *                                      low byte = number of values that follow
 *   offset 6..7     uint16_t crc16     CRC-16/MODBUS over the WHOLE 144 bytes,
 *                                      this field taken as 0 (padding included)
 *   offset 8..135   int32_t value[32]  value i belongs to the i-th retain tag
 *                                      (tag_vreg_retain_base_index() + i)
 *   offset 136..143 padding, written as 0
 *
 * Page use inside the sector (56 pages):
 *   pages 0..53   (SPLC_RETAIN_MAIN_PAGES)        main loop
 *   pages 54..55  (SPLC_RETAIN_PVD_RESERVE_PAGES) PVD interrupt only
 * Pages are programmed in increasing order and each exactly once. When the
 * main loop needs a page and only the reserve is left it COMPACTS: erases the
 * sector and writes the current RAM values to page 0. The PVD interrupt never
 * erases: if no blank page is left it returns false and the main loop retries.
 *
 * Known limit of a one-sector design: from the start of the erase until page 0
 * is fully programmed (2 erase steps + 9 quad-words) the sector holds no valid
 * page, so a power loss exactly there loses the stored values (the RAM values
 * are written right after the erase). Compaction is therefore lazy: only when
 * a write is really needed and no main page is left (about once per 54 saves),
 * or at boot when not even one blank page is left.
 *
 * ---------------------------------------------------------------------------
 * Boot: scan, load, and one-time migration from the old format
 * ---------------------------------------------------------------------------
 * Old format (v0): 208-byte records (8 B header + 32 x {tag_index 2, value 4}
 * + 8 B padding), ring over 3 sectors. Boot rules:
 *   1. A valid v1 page exists in sector 0: use the newest one (highest seq).
 *      The old format is not looked at (see "downgrade" in docs/handoff.md).
 *   2. No v1 page, a valid v0 record exists: MIGRATE (retain_migrate_legacy()).
 *   3. Neither: first boot, every retain tag stays 0; sector 0 is erased only
 *      if it holds garbage.
 * Migration order, safe against a power loss at every step: (a) if the newest
 * old record is in sector 0, copy it byte for byte to sector 1 and verify it;
 * (b) erase sector 0; (c) write page 0 with seq = old seq + 1; (d) read it back
 * and check the CRC; (e) only then erase sectors 1 and 2 (stale old records).
 * A power loss before (d) leaves a valid old record, so the next boot migrates
 * again. If a step fails, s_migration_pending stays set, the write position
 * stays unknown (no write, no PVD write) and every retain_snapshot_write()
 * retries; retain_values_changed() stays true, so retain_service() retries
 * once per RETAIN_SNAPSHOT_PERIOD_MS.
 */

#include "plc_retain.h"

#include <string.h>

#include "splc_flash_define.h"
#include "sx_flash.h"
#include "sx_time.h"
#include "plc_wdt.h"    /* plc_wdt_grace(): a sector erase is slow (main loop only) */
#include "splc_nmbs.h"
#include "plc_tag.h"
#include "splc_log.h"

static const char *TAG = "PLC_RETAIN";

/* Both formats keep seq_num at 0 and crc16 at 6. */
#define RETAIN_SEQ_OFFSET        0U
#define RETAIN_CRC_OFFSET        6U

/* v1 page */
#define RETAIN_PAGE_FMTCNT_OFFSET 4U
#define RETAIN_PAGE_VALUES_OFFSET SPLC_RETAIN_PAGE_HEADER_SIZE

/* v0 record (read only, for migration) */
#define RETAIN_LEG_COUNT_OFFSET   4U
#define RETAIN_LEG_ENTRIES_OFFSET 8U
#define RETAIN_LEG_ENTRY_SIZE     6U

#ifndef __cplusplus
_Static_assert(SPLC_RETAIN_ACTIVE_SECTORS == 1U,
               "plc_retain.c implements the one-sector page log only");
_Static_assert(SPLC_RETAIN_PAGE_SIZE * SPLC_RETAIN_PAGES_PER_SECTOR <= SPLC_FLASH_SECTOR_SIZE,
               "the retain pages must fit in one sector");
_Static_assert(SPLC_RETAIN_RECORD_SIZE % 16U == 0U,
               "the old retain record must be a whole number of quad-words");
_Static_assert(SPLC_RETAIN_MAIN_PAGES >= 1U,
               "the main loop needs at least one page");
#endif

static uint32_t s_next_seq_num      = 0;     /* seq_num for the NEXT page */
static uint32_t s_write_page        = 0;     /* page index for the NEXT write (0..SPLC_RETAIN_PAGES_PER_SECTOR) */
static bool     s_write_pos_known   = false; /* true once boot has established where to write next */
static bool     s_migration_pending = false; /* old-format migration failed, retry */

static uint32_t s_last_snapshot_tick_ms = 0;

/*
 * Flash-operation guard, shared by every main-loop Flash operation (this
 * file's own writes/erases and plc_rule_flash.c's save, which brackets its
 * erase+write with retain_flash_op_begin()/end()). The PVD interrupt can
 * fire at any instruction; if it fired while the main loop was inside
 * HAL_FLASH_Program()/HAL_FLASHEx_Erase(), the HAL's process lock would
 * make the interrupt's own call fail with BUSY -- and the interrupt must
 * not touch s_write_page while the main loop is part-way through updating
 * it. While the depth is non-zero the interrupt does nothing except raise
 * s_emergency_pending; retain_service() then does the write from the main
 * loop once the Flash is free.
 *
 * A plain volatile counter is enough on this single-core MCU: the
 * interrupt never changes the depth by a net amount (it only runs when the
 * depth is 0 and leaves it 0), so the main loop's non-atomic increment
 * cannot be corrupted by it.
 */
static volatile uint32_t s_flash_op_depth         = 0U;
static volatile bool     s_emergency_pending      = false;
static volatile bool     s_emergency_seen         = false; /* an emergency write has been attempted since boot */
static volatile uint32_t s_last_emergency_tick_ms = 0U;

/*
 * Values of the 32 retain tags as stored in the newest valid Flash page
 * (or as loaded at boot). Lets the periodic and emergency paths skip a
 * write -- and the Flash wear -- when nothing changed since the last save.
 */
static int32_t s_saved_values[SPLC_RETAIN_TAG_COUNT];
static bool    s_saved_valid = false;

/*
 * A board's VREG_RETAIN group starts at a board-dependent offset (see
 * tag_vreg_retain_base_index(), plc_tag.h), so the tag indices are computed
 * once, lazily, on first use.
 */
static uint16_t s_retain_tag_indices[SPLC_RETAIN_TAG_COUNT];
static bool     s_retain_tag_indices_ready = false;

static void retain_tag_indices_init(void)
{
    if (s_retain_tag_indices_ready) {
        return;
    }
    uint16_t base = tag_vreg_retain_base_index();
    for (uint16_t i = 0; i < SPLC_RETAIN_TAG_COUNT; i++) {
        s_retain_tag_indices[i] = (uint16_t)(base + i);
    }
    s_retain_tag_indices_ready = true;
}

/* ---------------------------------------------------------------- helpers */

static uint32_t read_u32_be(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static void write_u32_be(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
}

static uint16_t read_u16_be(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

static void write_u16_be(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v >> 8);
    p[1] = (uint8_t)v;
}

static int32_t read_i32_be(const uint8_t *p)
{
    return (int32_t)read_u32_be(p);
}

static void write_i32_be(uint8_t *p, int32_t v)
{
    write_u32_be(p, (uint32_t)v);
}

/*
 * CRC-16/MODBUS over a whole record/page with the crc16 field (offset 6..7)
 * taken as 0. nanoMODBUS's nmbs_crc_calc() returns the result byte-swapped
 * for RTU wire order; this swaps it back to a plain arithmetic uint16_t.
 * raw[6..7] is zeroed temporarily and restored; the rest is read-only.
 */
static uint16_t retain_crc(uint8_t *raw, uint32_t len)
{
    uint8_t saved0 = raw[RETAIN_CRC_OFFSET];
    uint8_t saved1 = raw[RETAIN_CRC_OFFSET + 1U];

    raw[RETAIN_CRC_OFFSET]      = 0;
    raw[RETAIN_CRC_OFFSET + 1U] = 0;

    uint16_t wire_order_crc = nmbs_crc_calc(raw, len, NULL);
    uint16_t crc = (uint16_t)((wire_order_crc << 8) | (wire_order_crc >> 8));

    raw[RETAIN_CRC_OFFSET]      = saved0;
    raw[RETAIN_CRC_OFFSET + 1U] = saved1;

    return crc;
}

static uint32_t retain_sector_addr(uint32_t sector_idx)
{
    return SPLC_FLASH_RETAIN_BASE_ADDR + sector_idx * SPLC_FLASH_SECTOR_SIZE;
}

/* v1 page `page` of sector 0 */
static uint32_t retain_page_addr(uint32_t page)
{
    return retain_sector_addr(0U) + page * SPLC_RETAIN_PAGE_SIZE;
}

/* v0 record slot `slot_idx` of sector `sector_idx` (migration only) */
static uint32_t retain_leg_addr(uint32_t sector_idx, uint32_t slot_idx)
{
    return retain_sector_addr(sector_idx) + slot_idx * SPLC_RETAIN_RECORD_SIZE;
}

/* True if [addr, addr+len) reads as erased (all 0xFF). Reads through
 * sx_flash_read() (plain loads, safe from the PVD interrupt), 16 bytes at a
 * time to keep the interrupt's stack use small. */
static bool retain_range_is_blank(uint32_t addr, uint32_t len)
{
    uint8_t chunk[16];

    for (uint32_t off = 0; off < len; off += sizeof(chunk)) {
        sx_flash_read(addr + off, chunk, sizeof(chunk));
        for (uint32_t i = 0; i < sizeof(chunk); i++) {
            if (chunk[i] != 0xFFU) {
                return false;
            }
        }
    }
    return true;
}

static bool retain_page_is_blank(uint32_t page)
{
    return retain_range_is_blank(retain_page_addr(page), SPLC_RETAIN_PAGE_SIZE);
}

static bool retain_sector_is_blank(uint32_t sector_idx)
{
    return retain_range_is_blank(retain_sector_addr(sector_idx), SPLC_FLASH_SECTOR_SIZE);
}

/* Erases one retain sector. Flash must be unlocked for an erase exactly as
 * for a program (plc_rule_flash.c does unlock -> erase -> write -> lock).
 * Main loop only. */
static void retain_erase_sector(uint32_t sector_idx)
{
    plc_wdt_grace(PLC_WDT_CH_PLC, SPLC_WDG_FLASH_GRACE_MS);
    sx_flash_unlock();
    sx_flash_erase(retain_sector_addr(sector_idx), SPLC_FLASH_SECTOR_SIZE);
    sx_flash_lock();
}

void retain_flash_op_begin(void)
{
    s_flash_op_depth++;
}

void retain_flash_op_end(void)
{
    if (s_flash_op_depth != 0U) {
        s_flash_op_depth--;
    }
}

/* -------------------------------------------------------------- v1 pages */

/*
 * Reads and validates page `page`: matching CRC, format byte 0xA5 and a value
 * count that fits. Blank flash (0xFF) and partly programmed pages fail.
 */
static bool retain_page_is_valid(uint32_t page, uint32_t *seq_out, uint8_t *count_out)
{
    uint8_t raw[SPLC_RETAIN_PAGE_SIZE];
    sx_flash_read(retain_page_addr(page), raw, SPLC_RETAIN_PAGE_SIZE);

    if (raw[RETAIN_PAGE_FMTCNT_OFFSET] != SPLC_RETAIN_PAGE_FMT) {
        return false;
    }
    if (raw[RETAIN_PAGE_FMTCNT_OFFSET + 1U] > SPLC_RETAIN_TAG_COUNT) {
        return false;
    }
    if (read_u16_be(&raw[RETAIN_CRC_OFFSET]) != retain_crc(raw, SPLC_RETAIN_PAGE_SIZE)) {
        return false;
    }

    if (seq_out != NULL) {
        *seq_out = read_u32_be(&raw[RETAIN_SEQ_OFFSET]);
    }
    if (count_out != NULL) {
        *count_out = raw[RETAIN_PAGE_FMTCNT_OFFSET + 1U];
    }
    return true;
}

static void retain_build_page(uint8_t *raw, uint32_t seq, const int32_t *vals)
{
    memset(raw, 0, SPLC_RETAIN_PAGE_SIZE);
    write_u32_be(&raw[RETAIN_SEQ_OFFSET], seq);
    raw[RETAIN_PAGE_FMTCNT_OFFSET]      = (uint8_t)SPLC_RETAIN_PAGE_FMT;
    raw[RETAIN_PAGE_FMTCNT_OFFSET + 1U] = (uint8_t)SPLC_RETAIN_TAG_COUNT;
    for (uint32_t i = 0; i < SPLC_RETAIN_TAG_COUNT; i++) {
        write_i32_be(&raw[RETAIN_PAGE_VALUES_OFFSET + i * SPLC_RETAIN_PAGE_VALUE_SIZE], vals[i]);
    }
    write_u16_be(&raw[RETAIN_CRC_OFFSET], retain_crc(raw, SPLC_RETAIN_PAGE_SIZE));
}

/* Programs one page (the page must be blank). in_isr: quiet write, no log. */
static void retain_program_page(uint32_t page, uint32_t seq, const int32_t *vals, bool in_isr)
{
    uint8_t raw[SPLC_RETAIN_PAGE_SIZE];
    retain_build_page(raw, seq, vals);

    sx_flash_unlock();
    if (in_isr) {
        (void)sx_flash_write_quiet(retain_page_addr(page), raw, SPLC_RETAIN_PAGE_SIZE);
    } else {
        sx_flash_write(retain_page_addr(page), raw, SPLC_RETAIN_PAGE_SIZE);
    }
    sx_flash_lock();
}

/* Newest valid page of sector 0 (highest seq_num). */
static bool retain_scan_pages(uint32_t *seq_out, uint32_t *page_out)
{
    bool     found = false;
    uint32_t best_seq = 0U;
    uint32_t best_page = 0U;

    for (uint32_t page = 0; page < SPLC_RETAIN_PAGES_PER_SECTOR; page++) {
        uint32_t seq;
        if (!retain_page_is_valid(page, &seq, NULL)) {
            continue;
        }
        if (!found || seq > best_seq) {
            found = true;
            best_seq = seq;
            best_page = page;
        }
    }
    *seq_out = best_seq;
    *page_out = best_page;
    return found;
}

/* ------------------------------------------------- v0 records (migration) */

static bool retain_leg_is_valid(uint32_t sector_idx, uint32_t slot_idx, uint32_t *seq_out)
{
    uint8_t raw[SPLC_RETAIN_RECORD_SIZE];
    sx_flash_read(retain_leg_addr(sector_idx, slot_idx), raw, SPLC_RETAIN_RECORD_SIZE);

    if (read_u16_be(&raw[RETAIN_CRC_OFFSET]) != retain_crc(raw, SPLC_RETAIN_RECORD_SIZE)) {
        return false;
    }
    if (seq_out != NULL) {
        *seq_out = read_u32_be(&raw[RETAIN_SEQ_OFFSET]);
    }
    return true;
}

/* Newest valid old-format record in the whole 3-sector region. */
static bool retain_scan_legacy(uint32_t *seq_out, uint32_t *sector_out, uint32_t *slot_out)
{
    bool     found = false;
    uint32_t best_seq = 0U, best_sector = 0U, best_slot = 0U;

    for (uint32_t sector = 0; sector < SPLC_FLASH_RETAIN_SECTOR_COUNT; sector++) {
        for (uint32_t slot = 0; slot < SPLC_RETAIN_RECORDS_PER_SECTOR; slot++) {
            uint32_t seq;
            if (!retain_leg_is_valid(sector, slot, &seq)) {
                continue;
            }
            if (!found || seq > best_seq) {
                found = true;
                best_seq = seq;
                best_sector = sector;
                best_slot = slot;
            }
        }
    }
    *seq_out = best_seq;
    *sector_out = best_sector;
    *slot_out = best_slot;
    return found;
}

/* ------------------------------------------------------ change detection */

static bool retain_values_changed(void)
{
    if (!s_saved_valid) {
        return true;
    }
    retain_tag_indices_init();
    for (uint16_t i = 0; i < SPLC_RETAIN_TAG_COUNT; i++) {
        if (tag_read(s_retain_tag_indices[i]) != s_saved_values[i]) {
            return true;
        }
    }
    return false;
}

bool retain_has_unsaved_changes(void)
{
    return retain_values_changed();
}

static void retain_saved_from_tags(void)
{
    for (uint16_t i = 0; i < SPLC_RETAIN_TAG_COUNT; i++) {
        s_saved_values[i] = tag_read(s_retain_tag_indices[i]);
    }
    s_saved_valid = true;
}

/* ------------------------------------------------------------- migration */

/*
 * Old format -> v1 page log. Main loop only (erases). `seq` / `sector` /
 * `slot` locate the newest valid old record. load_tags: copy its values into
 * the tags first (boot); false on a retry, when the tags already hold the
 * loaded (and possibly newer) values. Returns true when page 0 is written and
 * verified. See the file header for the step order.
 */
static bool retain_migrate_legacy(uint32_t seq, uint32_t sector, uint32_t slot, bool load_tags)
{
    uint8_t raw[SPLC_RETAIN_RECORD_SIZE];
    sx_flash_read(retain_leg_addr(sector, slot), raw, SPLC_RETAIN_RECORD_SIZE);

    log_info(TAG, "migrate: old record seq=%lu in sector %u slot %u",
             (unsigned long)seq, (unsigned)sector, (unsigned)slot);

    if (load_tags) {
        uint16_t count = read_u16_be(&raw[RETAIN_LEG_COUNT_OFFSET]);
        if (count > SPLC_RETAIN_TAG_COUNT) {
            count = SPLC_RETAIN_TAG_COUNT;
        }
        for (uint16_t i = 0; i < count; i++) {
            const uint8_t *entry = &raw[RETAIN_LEG_ENTRIES_OFFSET + i * RETAIN_LEG_ENTRY_SIZE];
            uint16_t tag_idx = read_u16_be(&entry[0]);
            int32_t  value   = read_i32_be(&entry[2]);
            if (tag_idx < MAX_TAGS && tag_get_kind(tag_idx) == TAG_VREG_RETAIN) {
                tag_write(tag_idx, value);
            }
        }
    }

    /* (a) the newest old record is about to be destroyed with sector 0: keep a
     * verified copy in sector 1 first. Sector 1 then holds only older records
     * (the newest one is in sector 0), so it may be erased. */
    if (sector == 0U) {
        if (!retain_sector_is_blank(1U)) {
            retain_erase_sector(1U);
        }
        sx_flash_unlock();
        sx_flash_write(retain_leg_addr(1U, 0U), raw, SPLC_RETAIN_RECORD_SIZE);
        sx_flash_lock();

        uint32_t seq_back = 0U;
        if (!retain_leg_is_valid(1U, 0U, &seq_back) || seq_back != seq) {
            log_error(TAG, "migrate: backup copy in sector 1 failed, sector 0 untouched");
            return false;
        }
        log_debug(TAG, "migrate: backup copy in sector 1 verified");
    }

    /* (b) erase sector 0 */
    retain_erase_sector(0U);
    if (!retain_sector_is_blank(0U)) {
        log_error(TAG, "migrate: sector 0 is not blank after erase");
        return false;
    }

    /* (c) page 0 = current tag values */
    int32_t snap[SPLC_RETAIN_TAG_COUNT];
    for (uint16_t i = 0; i < SPLC_RETAIN_TAG_COUNT; i++) {
        snap[i] = tag_read(s_retain_tag_indices[i]);
    }
    const uint32_t new_seq = seq + 1U;
    retain_program_page(0U, new_seq, snap, false);

    /* (d) read back */
    uint32_t seq_back = 0U;
    if (!retain_page_is_valid(0U, &seq_back, NULL) || seq_back != new_seq) {
        log_error(TAG, "migrate: page 0 did not verify");
        return false;
    }

    memcpy(s_saved_values, snap, sizeof(s_saved_values));
    s_saved_valid     = true;
    s_next_seq_num    = new_seq + 1U;
    s_write_page      = 1U;
    s_write_pos_known = true;
    s_migration_pending = false;

    /* (e) stale old records: erase them so a downgraded firmware cannot find
     * an out-of-date record. Only done here, never at a normal boot (sectors
     * 1 and 2 are reserved for a later extension record). */
    for (uint32_t s = 1U; s < SPLC_FLASH_RETAIN_SECTOR_COUNT; s++) {
        if (!retain_sector_is_blank(s)) {
            retain_erase_sector(s);
        }
    }

    log_info(TAG, "migrate: done, page 0 seq=%lu verified, old sectors cleared",
             (unsigned long)new_seq);
    return true;
}

/* ------------------------------------------------------------------ boot */

/*
 * Establishes the write position (and, on the first call, loads the tags).
 * load_tags is true from retain_store_restore() and false on a retry from
 * retain_snapshot_write(). Main loop only.
 */
static void retain_boot_init(bool load_tags)
{
    retain_tag_indices_init();
    retain_flash_op_begin();

    uint32_t seq = 0U, page = 0U;

    if (retain_scan_pages(&seq, &page)) {
        /* v1 pages present: newest one wins. */
        uint8_t raw[SPLC_RETAIN_PAGE_SIZE];
        sx_flash_read(retain_page_addr(page), raw, SPLC_RETAIN_PAGE_SIZE);

        uint32_t count = raw[RETAIN_PAGE_FMTCNT_OFFSET + 1U];
        int32_t  page_vals[SPLC_RETAIN_TAG_COUNT];
        memset(page_vals, 0, sizeof(page_vals));
        for (uint32_t i = 0; i < count; i++) {
            page_vals[i] = read_i32_be(&raw[RETAIN_PAGE_VALUES_OFFSET + i * SPLC_RETAIN_PAGE_VALUE_SIZE]);
        }

        if (load_tags) {
            for (uint32_t i = 0; i < count; i++) {
                uint16_t idx = s_retain_tag_indices[i];
                if (tag_get_kind(idx) == TAG_VREG_RETAIN) {
                    tag_write(idx, page_vals[i]);
                }
            }
            retain_saved_from_tags();
        } else {
            memcpy(s_saved_values, page_vals, sizeof(s_saved_values));
            s_saved_valid = true;
        }

        s_next_seq_num      = seq + 1U;
        s_write_page        = page + 1U;
        s_write_pos_known   = true;
        s_migration_pending = false;

        /* A partly programmed page (power lost mid-write) is not blank: skip. */
        while (s_write_page < SPLC_RETAIN_PAGES_PER_SECTOR && !retain_page_is_blank(s_write_page)) {
            s_write_page++;
        }
        log_info(TAG, "restored: page %lu seq=%lu next page %lu",
                 (unsigned long)page, (unsigned long)seq, (unsigned long)s_write_page);
    } else {
        uint32_t leg_seq = 0U, leg_sector = 0U, leg_slot = 0U;

        if (retain_scan_legacy(&leg_seq, &leg_sector, &leg_slot)) {
            if (retain_migrate_legacy(leg_seq, leg_sector, leg_slot, load_tags)) {
                /* state already set by the migration */
            } else {
                s_write_pos_known   = false;
                s_migration_pending = true;
                s_saved_valid       = false;
                log_error(TAG, "migrate: failed, will retry (retain writes disabled until it succeeds)");
            }
        } else {
            /* First boot, or nothing readable. Tags keep their power-on value (0). */
            if (!retain_sector_is_blank(0U)) {
                log_info(TAG, "no valid retain data, erasing the unreadable retain sector");
                retain_erase_sector(0U);
            }
            retain_saved_from_tags();
            s_next_seq_num      = 0U;
            s_write_page        = 0U;
            s_write_pos_known   = true;
            s_migration_pending = false;
            log_info(TAG, "no retain data in Flash (first boot or cleared)");
        }
    }

    retain_flash_op_end();
}

/* --------------------------------------------------------------- writing */

/*
 * Writes one page with the current retain tag values. Shared by the
 * main-loop paths (in_isr == false) and the PVD interrupt (in_isr == true);
 * the caller has already made sure no other Flash operation is running
 * (retain_flash_op_begin(), or depth 0 for the interrupt).
 *
 * Main loop: uses pages 0..SPLC_RETAIN_MAIN_PAGES-1; with none left it
 * compacts (erase the sector, write page 0).
 * Interrupt: may also use the reserve pages; never erases, never logs; with
 * no blank page it returns false and the caller defers to the main loop.
 *
 * Returns true only if the page was read back with a valid CRC and the
 * expected seq_num.
 */
static bool retain_write_record(bool in_isr)
{
    if (!s_write_pos_known) {
        /* Defensive: boot must run first (plc_engine_init() guarantees this),
         * or a migration is pending. Refusing to write with an unknown
         * position avoids clobbering data whose validity is not established. */
        return false;
    }

    retain_tag_indices_init();

    const uint32_t limit = in_isr ? SPLC_RETAIN_PAGES_PER_SECTOR : SPLC_RETAIN_MAIN_PAGES;

    /* Skip pages that are not blank (earlier failed or interrupted write). */
    while (s_write_page < limit && !retain_page_is_blank(s_write_page)) {
        s_write_page++;
    }

    if (s_write_page >= limit) {
        if (in_isr) {
            return false;
        }
        log_info(TAG, "compaction: sector full at page %lu, erasing and restarting at page 0",
                 (unsigned long)s_write_page);
        retain_erase_sector(0U);
        s_write_page = 0U;
        if (!retain_sector_is_blank(0U)) {
            log_error(TAG, "compaction: sector 0 is not blank after erase");
            return false;
        }
    }

    int32_t snap[SPLC_RETAIN_TAG_COUNT];
    for (uint16_t i = 0; i < SPLC_RETAIN_TAG_COUNT; i++) {
        snap[i] = tag_read(s_retain_tag_indices[i]);
    }

    const uint32_t written_seq  = s_next_seq_num;
    const uint32_t written_page = s_write_page;

    retain_program_page(written_page, written_seq, snap, in_isr);

    /* The page is consumed even if the write failed: a partly programmed
     * page cannot be reused. */
    s_next_seq_num++;
    s_write_page++;

    /* Read back and verify (Wire Contract section 7, step 3). */
    uint32_t seq_back = 0U;
    bool verified = retain_page_is_valid(written_page, &seq_back, NULL) &&
                    (seq_back == written_seq);

    if (verified) {
        memcpy(s_saved_values, snap, sizeof(s_saved_values));
        s_saved_valid = true;
        if (!in_isr) {
            log_debug(TAG, "page %lu seq=%lu written",
                      (unsigned long)written_page, (unsigned long)written_seq);
        }
    } else if (!in_isr) {
        log_error(TAG, "page %lu seq=%lu did not verify",
                  (unsigned long)written_page, (unsigned long)written_seq);
    }

    return verified;
}

void retain_store_restore(void)
{
    /* Must run after tag_table_load_from_flash() (plc_engine_init()'s call
     * order guarantees this) so tag_vreg_retain_base_index() already
     * reflects this board's real layout. */
    s_write_pos_known   = false;
    s_saved_valid       = false;
    s_migration_pending = false;

    retain_boot_init(true);

    /* No blank page is left at all (the PVD reserve is used up too): compact
     * now (main loop), otherwise a PVD write could not save anything. While
     * at least one reserve page is blank nothing is erased here: compaction
     * is lazy and happens at the next write that is really needed, so a boot
     * with unchanged values never erases (smaller power-loss window). */
    if (s_write_pos_known && s_write_page >= SPLC_RETAIN_PAGES_PER_SECTOR) {
        (void)retain_snapshot_write();
    }

    s_last_snapshot_tick_ms = sx_get_tick_ms();
}

bool retain_snapshot_write(void)
{
    retain_flash_op_begin();
    if (!s_write_pos_known && s_migration_pending) {
        retain_boot_init(false);
    }
    bool ok = retain_write_record(false);
    retain_flash_op_end();
    return ok;
}

void retain_emergency_snapshot(void)
{
    /* Interrupt context. No logging, no erase, no blocking beyond the
     * quad-word programming of one page. */
    if (!s_write_pos_known) {
        return;
    }

    uint32_t now = sx_get_tick_ms();

    /* The detector can chatter around its threshold; one emergency write
     * per interval is enough and bounds the Flash wear. */
    if (s_emergency_seen &&
        (now - s_last_emergency_tick_ms) < RETAIN_EMERGENCY_MIN_INTERVAL_MS) {
        return;
    }

    if (s_flash_op_depth != 0U) {
        /* The main loop is inside a Flash operation: do not touch Flash or
         * the write position. retain_service() writes once it is free. */
        s_emergency_pending = true;
        return;
    }

    if (!retain_values_changed()) {
        return;
    }

    s_flash_op_depth++;
    bool ok = retain_write_record(true);
    s_flash_op_depth--;

    s_emergency_seen         = true;
    s_last_emergency_tick_ms = now;

    if (!ok) {
        /* No blank page left (needs an erase) or the write did not verify:
         * let the main loop retry, where an erase is allowed. */
        s_emergency_pending = true;
    }
}

void retain_service(void)
{
    uint32_t now = sx_get_tick_ms();

    if (s_emergency_pending && s_flash_op_depth == 0U) {
        s_emergency_pending = false;
        if (retain_values_changed()) {
            (void)retain_snapshot_write();
            s_last_snapshot_tick_ms = now;
        }
    }

    /* Unsigned subtraction handles tick wraparound correctly as long as
     * the actual elapsed time never exceeds UINT32_MAX ms (~49.7 days) --
     * true for any period this project uses. Safety net only: the PVD
     * interrupt is the primary power-loss path, so a period with no change
     * costs no Flash write. While a migration is pending, retain_values_changed()
     * is true, so this is also the periodic retry of the migration. */
    if ((now - s_last_snapshot_tick_ms) >= RETAIN_SNAPSHOT_PERIOD_MS) {
        if (retain_values_changed()) {
            (void)retain_snapshot_write();
        }
        s_last_snapshot_tick_ms = now;
    }
}