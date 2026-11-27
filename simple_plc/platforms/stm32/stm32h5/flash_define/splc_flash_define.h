#ifndef SPLC_FLASH_DEFINE_H
#define SPLC_FLASH_DEFINE_H

#include "sx_platform_config.h"

/*
 * splc_flash_define.h - Layer 0 (platforms/stm32/stm32h5/flash_define)
 *
 * Flash memory map of the persistent-storage regions, as a set of macros --
 * not a driver, not business logic. Defines WHERE each region lives; the
 * driver (platforms/stm32/stm32h5/flash/stm32h5_flash.c) knows HOW to erase/
 * program a sector and services/plc_retain, services/plc_rule_flash know WHAT
 * goes in each region.
 *
 * The regions are 5 contiguous sectors: by default the last 5 sectors of the
 * chip's Flash, counted backwards from its end, or -- when a host firmware owns
 * the end of the Flash -- a 40 KB hole at SPLC_FLASH_DATA_ADDR (splcopts.h). The only product input is the chip's Flash size,
 * SPLC_FLASH_SIZE_KB (splcopts.h, default in config/splc_opt.h); every
 * address below is derived from it, so moving to another STM32H5 part needs
 * no edit in this file. Every stm32h5_*.h file already #includes
 * sx_platform_config.h (which includes splc_opt.h), and the file is reached
 * by lower and upper layers through the PUBLIC include path of
 * platforms/stm32/stm32h5/CMakeLists.txt.
 *
 * Hardware facts this file is built on (STM32H5, RM0481 / datasheets):
 *   - Sector size 8 KB on every STM32H5 part (FLASH_SECTOR_SIZE in CMSIS).
 *   - Two banks of equal size (SPLC_FLASH_SIZE_KB / 2 each), so the last
 *     sectors are always in bank 2 and the code running from bank 1 keeps
 *     executing while they are erased/programmed. ASSUMPTION to re-check
 *     against the datasheet of any part not yet used: that part is dual-bank
 *     with equal banks (the checks below only prove the 5 sectors fit in
 *     bank 2).
 *   - FLASH_BASE (0x08000000) comes from the CMSIS device header; the size
 *     macros of that header (FLASH_SIZE/FLASH_BANK_SIZE) read a register
 *     that Hard-Faults on STM32H5 and must NOT be used.
 *
 * Layout (offsets counted back from END = FLASH_BASE + total size; the
 * example column is a 256 KB part, STM32H523CC):
 *
 *   Sector (from the end) | Address        | 256 KB example | Region
 *   ----------------------+----------------+----------------+-----------------
 *   last                  | END - 1 sector | 0x0803E000     | Rule Table A (running copy)
 *   2nd from last         | END - 2        | 0x0803C000     | Rule Table B (backup copy)
 *   3rd from last         | END - 3        | 0x0803A000     | Retain sector 3 of 3
 *   4th from last         | END - 4        | 0x08038000     | Retain sector 2 of 3
 *   5th from last         | END - 5        | 0x08036000     | Retain sector 1 of 3 (lowest)
 *
 * The firmware image (.text/.data) must stay below SPLC_FLASH_DATA_BASE_ADDR
 * (END - 5 sectors): set the linker script's FLASH LENGTH to
 * SPLC_FLASH_SIZE_KB minus SPLC_FLASH_RESERVED_SIZE (40 KB).
 *
 * Rule Table: 2 sectors (A/B, 8 KB each), per docs/handoff.md section 1 --
 * see services/plc_rule_flash/plc_rule_flash.h for the full A/B recovery
 * mechanism this supports. Each sector holds one on-Flash record: an
 * 8-byte header (seq_num/rule_count/crc16) plus up to
 * MAX_RULES(100) * sizeof(SPLC_RuleRecord)(32) = 3200 bytes of rule wire
 * data (3208 bytes total max for rules + header alone; with every optional section incl. the phase-2 EXT section the worst case is 7756 bytes, 95% of one 8 KB sector) -- see
 * core/plc_rule/plc_rule.h. No wear-leveling within a sector (each holds
 * exactly one record, rewritten in place): the Active Rule Table only
 * changes when the App commits a new rule set (rare, human-triggered),
 * unlike Retain which can change every scan cycle -- see
 * docs/architecture.md section 2.6.2 for the commit protocol (staging
 * happens over Modbus registers in RAM/App side; only the final
 * committed table is written to Flash, to BOTH A and B).
 *
 * Retain: 3 contiguous sectors (24 KB) using the rotating EEPROM-emulation
 * scheme from docs/SimplePLC_RuleStruct_MCU_Spec_v0.1.md section 7,
 * updated for v1.9's 32-slot VREG_RETAIN range (the v0.1 spec's own
 * numbers -- 104 byte/record, ~78 records/sector, 64 KB/8 sectors -- were
 * computed for the OLD 16-slot VREG_RETAIN layout and do not apply
 * as-is; see the recomputed constants below). Reduced from the original
 * 4 sectors to 3 to free one sector (the former 4th Retain sector) for Rule Table B
 * -- per docs/handoff.md section 1.1 point 1, this trade was made
 * deliberately with the user rather than growing the total reserved
 * Flash region.
 *
 * Endurance at these 3 sectors, 32 retain slots, RETAIN_SNAPSHOT_PERIOD_MS
 * = 5 minutes (docs/SimplePLC_RuleStruct_MCU_Spec_v0.1.md section 7.1):
 * still comfortably sufficient (on the order of ten years) before
 * exhausting the ~10,000 erase-cycle endurance typically quoted for
 * STM32H5 Flash sectors, even after losing one sector to Rule Table B --
 * accepted as sufficient for this product's expected service life (see
 * chat discussion; no consumer gateway device is expected to still be the
 * same physical unit in active use after 10+ years). If
 * RETAIN_SNAPSHOT_PERIOD_MS is ever configured shorter over Modbus
 * (docs/architecture.md's config knob), this endurance budget shrinks
 * proportionally -- worth re-checking before allowing very short periods.
 *
 * UPDATE (retain page log, format v1; NOT built, NOT run on a board yet):
 * only the FIRST of the 3 retain sectors is written now. It is used as a
 * single-sector append log of 144-byte pages (see "Retain page log" below).
 * The region stays 3 sectors long (SPLC_FLASH_RETAIN_SECTOR_COUNT) so that
 * SPLC_FLASH_DATA_BASE_ADDR, Rule Table A/B, the cert store and the Wi-Fi
 * store do NOT move. Sectors +1 and +2 are reserved: they hold stale
 * old-format records until migration has run, and are kept free for a later
 * extension record. The old 208-byte constants below stay: the migration
 * from the old format needs them.
 */

#if STM32H5_PLATFORM

#include <stdint.h>

/* FLASH_BASE (0x08000000) is a CMSIS device-header macro (Drivers/CMSIS/
 * Device/ST/STM32H5xx/Include/stm32h523xx.h), needed below by
 * SPLC_FLASH_RULE_TABLE_ADDR/SPLC_FLASH_RETAIN_BASE_ADDR. This file is
 * consumed from Layer 3 (services/plc_retain.c) as well as Layer 0
 * (platforms/stm32/stm32h5/), and Layer 3 has no reason to already have
 * included any HAL/CMSIS header itself before reaching for "where is
 * Flash laid out" -- unlike a stm32h5_*.h file, which always sits behind
 * `#if STM32H5_PLATFORM` and can assume stm32h5xx_hal.h is already
 * pulled in by the same guard. So this header includes it directly,
 * making itself self-contained rather than relying on whoever included
 * it to have done so first. Safe to include unconditionally (not gated
 * behind STM32H5_PLATFORM) because this file, by construction, only
 * ever describes the STM32H5's own Flash layout -- a build for a
 * different chip would use a different splc_flash_define.h entirely,
 * not this same file with a different platform branch inside it. */
#include "stm32h5xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* --- Flash geometry (derived from SPLC_FLASH_SIZE_KB) ------------------ */
#define SPLC_FLASH_SECTOR_SIZE          0x2000U                                  /* 8 KB, all STM32H5 */
#define SPLC_FLASH_TOTAL_SIZE           ((uint32_t)SPLC_FLASH_SIZE_KB * 1024U)
#define SPLC_FLASH_BANK_SIZE            (SPLC_FLASH_TOTAL_SIZE / 2U)             /* two equal banks */
#define SPLC_FLASH_SECTOR_COUNT         (SPLC_FLASH_TOTAL_SIZE / SPLC_FLASH_SECTOR_SIZE)
#define SPLC_FLASH_END_ADDR             (FLASH_BASE + SPLC_FLASH_TOTAL_SIZE)     /* one past the last byte */

/* Sectors reserved for data at the end of Flash: Rule Table A/B + Retain. */
#define SPLC_FLASH_RULE_TABLE_SECTORS   2U
#define SPLC_FLASH_RETAIN_SECTOR_COUNT  3U
#define SPLC_FLASH_RESERVED_SECTORS     (SPLC_FLASH_RULE_TABLE_SECTORS + SPLC_FLASH_RETAIN_SECTOR_COUNT)
#define SPLC_FLASH_RESERVED_SIZE        (SPLC_FLASH_RESERVED_SECTORS * SPLC_FLASH_SECTOR_SIZE)   /* 40 KB */
/* Lowest address of the reserved region; the firmware must end below it.
 * Default: the last 5 sectors of the chip. A host that owns the end of the
 * Flash declares the hole with SPLC_FLASH_DATA_ADDR (config/splc_opt.h). */
#ifdef SPLC_FLASH_DATA_ADDR
#define SPLC_FLASH_DATA_BASE_ADDR       ((uint32_t)(SPLC_FLASH_DATA_ADDR))
#else
#define SPLC_FLASH_DATA_BASE_ADDR       (SPLC_FLASH_END_ADDR - SPLC_FLASH_RESERVED_SIZE)
#endif

/* A bad SPLC_FLASH_SIZE_KB must stop the build, not erase code at run time. */
#ifndef __cplusplus
_Static_assert((SPLC_FLASH_SIZE_KB) % 16 == 0,
               "SPLC_FLASH_SIZE_KB must be a multiple of 16 (two banks of whole 8 KB sectors)");
_Static_assert((SPLC_FLASH_SIZE_KB) >= 128 && (SPLC_FLASH_SIZE_KB) <= 2048,
               "SPLC_FLASH_SIZE_KB outside the 128..2048 KB range of the STM32H5 family");
_Static_assert(SPLC_FLASH_RESERVED_SIZE <= SPLC_FLASH_BANK_SIZE,
               "data sectors do not fit in bank 2 of this Flash size");
_Static_assert(SPLC_FLASH_DATA_BASE_ADDR % SPLC_FLASH_SECTOR_SIZE == 0U,
               "SPLC_FLASH_DATA_ADDR must be 8 KB (sector) aligned");
_Static_assert(SPLC_FLASH_DATA_BASE_ADDR >= FLASH_BASE + SPLC_FLASH_BANK_SIZE &&
               SPLC_FLASH_DATA_BASE_ADDR + SPLC_FLASH_RESERVED_SIZE <= SPLC_FLASH_END_ADDR,
               "the 5 data sectors must lie entirely inside bank 2 of the Flash");
#endif

/* --- Rule Table A/B (see services/plc_rule_flash/plc_rule_flash.h) ------
 *
 * Last two sectors in Flash. A is the
 * "running" copy that plc_rule_flash_load() reads first at boot; B is its
 * backup, used to self-heal A if A's CRC is ever bad (e.g. power loss
 * mid-write) -- see plc_rule_flash.h for the full mechanism. Both sectors
 * use the identical on-Flash record format; app/architecture docs refer
 * to this pair collectively as "the Rule Table Flash region". See
 * docs/architecture.md section 2.6.2 for the Modbus staging/commit
 * protocol that produces what eventually gets written here.
 */
#define SPLC_FLASH_RULE_TABLE_A_ADDR    (SPLC_FLASH_DATA_BASE_ADDR + 4U * SPLC_FLASH_SECTOR_SIZE)
#define SPLC_FLASH_RULE_TABLE_B_ADDR    (SPLC_FLASH_DATA_BASE_ADDR + 3U * SPLC_FLASH_SECTOR_SIZE)
#define SPLC_FLASH_RULE_TABLE_SIZE      SPLC_FLASH_SECTOR_SIZE   /* 1 sector each, 8 KB */

/* --- Retain (VREG_RETAIN rotating EEPROM-emulation store) ---------------
 *
 * 3 contiguous sectors immediately before Rule Table B (the 5th..3rd
 * sectors from the end of Flash), per docs/SimplePLC_RuleStruct_MCU_Spec_v0.1.md section 7.1's
 * rotating-record scheme: services/plc_retain/plc_retain.c scans this
 * whole region at boot for the record with the highest seq_num and a
 * valid CRC to find the "active" write position, rather than persisting
 * a separate pointer anywhere. Reduced from 4 to 3 sectors to free a sector
 * for Rule Table B -- see the header comment above.
 *
 * SPLC_FLASH_RETAIN_BASE_ADDR is the LOWEST address of the 3-sector
 * region (5th sector from the end) -- i.e. writing/scanning proceeds
 * from SPLC_FLASH_RETAIN_BASE_ADDR upward through
 * SPLC_FLASH_RETAIN_BASE_ADDR + SPLC_FLASH_RETAIN_TOTAL_SIZE - 1, which
 * ends exactly where SPLC_FLASH_RULE_TABLE_B_ADDR begins (no gap, no
 * overlap -- checked at compile time just below).
 */
#define SPLC_FLASH_RETAIN_BASE_ADDR     (SPLC_FLASH_DATA_BASE_ADDR)
#define SPLC_FLASH_RETAIN_TOTAL_SIZE    (SPLC_FLASH_RETAIN_SECTOR_COUNT * SPLC_FLASH_SECTOR_SIZE) /* 24 KB */

#ifndef __cplusplus
_Static_assert(SPLC_FLASH_RETAIN_BASE_ADDR + SPLC_FLASH_RETAIN_TOTAL_SIZE == SPLC_FLASH_RULE_TABLE_B_ADDR,
               "Retain region must end exactly where Rule Table B begins");
_Static_assert(SPLC_FLASH_RETAIN_BASE_ADDR == SPLC_FLASH_DATA_BASE_ADDR,
               "Retain region must start at the bottom of the reserved data region");
#endif

/*
 * Retain record layout, per docs/SimplePLC_RuleStruct_MCU_Spec_v0.1.md
 * section 7.1's RetainSnapshotHeader, with the per-tag entry count
 * updated from that spec's original 16 (v0.1's VREG_RETAIN range) to
 * v1.9's 32 (see board/board_tag_define.h, VREG_RETAIN0..31):
 *
 *   struct {
 *       uint32_t seq_num;
 *       uint16_t count;
 *       uint16_t crc16;
 *       // followed by: count x { uint16_t tag_index; int32_t value; }
 *   };
 *
 * These are size/count constants only -- services/plc_retain.c (not yet
 * written) is where the actual struct and read/write logic belong; this
 * file intentionally does not declare a struct, to keep this a pure
 * "where and how big" memory-map header that any layer can include
 * without pulling in Layer 3 business logic.
 */
#define SPLC_RETAIN_TAG_COUNT           32U   /* v1.9 VREG_RETAIN range size */
#define SPLC_RETAIN_HEADER_SIZE         8U    /* seq_num(4) + count(2) + crc16(2) */
#define SPLC_RETAIN_ENTRY_SIZE          6U    /* tag_index(2) + value(4), per retained tag */

/*
 * BUGFIX (found on real hardware, confirmed via STM32CubeProgrammer's
 * raw SWD read -- bypasses ICACHE, so this is the true Flash content,
 * not a stale-cache artifact): the natural record size
 * (SPLC_RETAIN_HEADER_SIZE + SPLC_RETAIN_ENTRY_SIZE * SPLC_RETAIN_TAG_COUNT)
 * is 200 bytes, which is NOT a multiple of 16.
 *
 * STM32H5's HAL_FLASH_Program() only supports FLASH_TYPEPROGRAM_QUADWORD
 * (16-byte-aligned writes) -- there is no smaller program granularity on
 * this part. retain_record_addr() places record N at
 * sector_base + N * SPLC_RETAIN_RECORD_SIZE, so with a 200-byte record,
 * every ODD-numbered slot (1, 3, 5, ...) starts 8 bytes off a 16-byte
 * boundary (200 % 16 == 8). sx_flash_write() (platforms/stm32/stm32h5/
 * flash/stm32h5_flash.c) walks the buffer in 16-byte quad-words starting
 * from that misaligned address, so EVERY quad-word it programs for that
 * record lands on a misaligned address and HAL_FLASH_Program() rejects
 * all of them (observed on real hardware: 13/13 quad-words of the
 * affected record failed, logged as repeated
 * "HAL_FLASH_Program failed ... status=1" -- not a partial/random
 * failure, and not caused by the destination not being erased: a raw
 * SWD read of the failing address showed it was still cleanly erased
 * (0xFF) right up until the misalignment kicked in). Even-numbered slots
 * (0, 2, 4, ...) always happened to land on a 16-byte boundary again
 * (200 * 2 = 400 = 16 * 25), which is why slot 0 wrote fine and only
 * every second write ever failed -- easy to miss in short test runs.
 *
 * This was invisible in normal testing because it only bites once
 * retain_snapshot_write() has been called enough times to reach an odd
 * slot index -- with RETAIN_SNAPSHOT_PERIOD_MS's default of 5 minutes,
 * that only surfaces after the device has been left running
 * uninterrupted for 5+ minutes, which most short manual test sessions
 * never reach.
 *
 * Fix: round SPLC_RETAIN_RECORD_SIZE itself up to the next 16-byte
 * multiple (200 -> 208). Every record slot is then a whole multiple of
 * 16 bytes apart, so slot N's address (sector_base + N * 208) is always
 * 16-byte aligned for every N, not just even ones. The extra 8 bytes are
 * unused padding at the end of every record (never read: every read
 * path in plc_retain.c bounds its use of a record by `count`, the
 * header, or the CRC region -- none of them iterate past
 * RETAIN_RECORD_ENTRIES_OFFSET + count * SPLC_RETAIN_ENTRY_SIZE). No
 * other change is required anywhere else: services/plc_retain/
 * plc_retain.c already computes every record address, buffer size, and
 * CRC span purely from SPLC_RETAIN_RECORD_SIZE (see that file's own
 * header comment), so a single-point fix here is sufficient -- verified
 * by grep across plc_retain.c/.h for every use of this constant.
 *
 * Trade-off: SPLC_RETAIN_RECORDS_PER_SECTOR drops from 40 to 39 (8 KB /
 * 208, floored) -- a ~2.5% reduction in usable retain history per
 * sector, well within the Flash-endurance budget this file's header
 * comment already computes (~10 years) for RETAIN_SNAPSHOT_PERIOD_MS's
 * default 5-minute period.
 */
#define SPLC_RETAIN_RECORD_SIZE         (((SPLC_RETAIN_HEADER_SIZE + SPLC_RETAIN_ENTRY_SIZE * SPLC_RETAIN_TAG_COUNT) + 15U) & ~15U) /* 208 bytes (200 rounded up to a 16-byte/quad-word multiple -- see bugfix comment above) */
#define SPLC_RETAIN_RECORDS_PER_SECTOR  (SPLC_FLASH_SECTOR_SIZE / SPLC_RETAIN_RECORD_SIZE) /* 39 */
#define SPLC_RETAIN_TOTAL_RECORDS       (SPLC_RETAIN_RECORDS_PER_SECTOR * SPLC_FLASH_RETAIN_SECTOR_COUNT) /* 117 */

/*
 * Retain page log (format v1): ONE sector, append-only, 144-byte pages.
 * NOT built, NOT run on a board yet.
 *
 * The old format above (208 B, 6 bytes per tag, 3-sector ring) is kept ONLY so
 * services/plc_retain/plc_retain.c can read it once at boot and migrate.
 *
 * Page layout (big-endian, like every other record in this project):
 *   offset 0..3     uint32_t seq_num   newer page = larger number
 *   offset 4..5     uint16_t fmt_count high byte SPLC_RETAIN_PAGE_FMT (0xA5),
 *                                      low byte = number of values that follow
 *   offset 6..7     uint16_t crc16     CRC-16/MODBUS over the WHOLE page, with
 *                                      this field taken as 0 (padding included)
 *   offset 8..135   32 x int32_t value value i belongs to the i-th retain tag
 *                                      (tag_vreg_retain_base_index() + i)
 *   offset 136..143 padding            written as 0, inside the CRC
 *
 * 144 = 9 x 16, so every page starts on a 16-byte (quad-word) boundary and the
 * H5 driver can program it (each quad-word is programmed once between erases).
 *
 * Sector use: pages are written in increasing order. The main loop uses pages
 * 0..SPLC_RETAIN_MAIN_PAGES-1. The last SPLC_RETAIN_PVD_RESERVE_PAGES pages are
 * kept for the low-voltage (PVD) emergency write, which never erases. When the
 * main loop needs a page and only the reserve is left, it erases the sector and
 * writes the current values to page 0.
 */
#define SPLC_RETAIN_ACTIVE_SECTORS      1U    /* sectors actually written: the first of the region */
#define SPLC_RETAIN_PAGE_HEADER_SIZE    8U    /* seq_num(4) + fmt_count(2) + crc16(2) */
#define SPLC_RETAIN_PAGE_VALUE_SIZE     4U    /* int32 value, no tag_index */
#define SPLC_RETAIN_PAGE_FMT            0xA5U /* high byte of fmt_count: marks the new format */
#define SPLC_RETAIN_PAGE_SIZE           ((SPLC_RETAIN_PAGE_HEADER_SIZE + SPLC_RETAIN_PAGE_VALUE_SIZE * SPLC_RETAIN_TAG_COUNT + 15U) & ~15U) /* 144 bytes (136 rounded up to a 16-byte multiple) */
#define SPLC_RETAIN_PAGES_PER_SECTOR    (SPLC_FLASH_SECTOR_SIZE / SPLC_RETAIN_PAGE_SIZE) /* 56 */
#define SPLC_RETAIN_PVD_RESERVE_PAGES   2U    /* last pages, for the PVD interrupt write only */
#define SPLC_RETAIN_MAIN_PAGES          (SPLC_RETAIN_PAGES_PER_SECTOR - SPLC_RETAIN_PVD_RESERVE_PAGES) /* 54 */

#ifndef __cplusplus
_Static_assert(SPLC_RETAIN_PAGE_SIZE % 16U == 0U,
               "a retain page must be a whole number of 16-byte quad-words");
_Static_assert(SPLC_RETAIN_PAGE_SIZE >= SPLC_RETAIN_PAGE_HEADER_SIZE + SPLC_RETAIN_PAGE_VALUE_SIZE * SPLC_RETAIN_TAG_COUNT,
               "a retain page must hold the header and every tag value");
_Static_assert(SPLC_RETAIN_TAG_COUNT <= 255U,
               "the tag count is stored in one byte of fmt_count");
_Static_assert(SPLC_RETAIN_PVD_RESERVE_PAGES >= 1U && SPLC_RETAIN_PAGES_PER_SECTOR > SPLC_RETAIN_PVD_RESERVE_PAGES + 1U,
               "the retain sector must hold more pages than the PVD reserve");
_Static_assert(SPLC_RETAIN_ACTIVE_SECTORS >= 1U && SPLC_RETAIN_ACTIVE_SECTORS <= SPLC_FLASH_RETAIN_SECTOR_COUNT,
               "the active retain sectors must lie inside the reserved retain region");
#endif

/*
 * Rule Table A/B on-Flash record layout, per
 * services/plc_rule_flash/plc_rule_flash.h:
 *
 *   struct {
 *       uint32_t seq_num;
 *       uint16_t rule_count; // bit 15 = "FB section present" flag,
 *                             // bit 14 = "FB section is the V2 128-byte
 *                             //   image" (only meaningful with bit 15;
 *                             //   clear = the legacy 112-byte image),
 *                             // bits 0..13 = the real rule count
 *       uint16_t crc16;      // field-embedded CRC-16/MODBUS over the
 *                             // whole record (this field itself zeroed
 *                             // during calculation), NOT the same scope
 *                             // as rule_table_wire_crc16()'s own CRC --
 *                             // see plc_rule_flash.h for why.
 *       // followed by: rule_count x 32-byte wire image, via
 *       // rule_record_to_wire() (plc_modbus_cfg.h)
 *       // then, only if the FB flag is set: the Function Block config
 *       // image (plc_fb_export(): 128 bytes, or 112 bytes in a record
 *       // written before Step 8d), covered by the same crc16.
 *       // then, only if the MQTT flag (bit 13) is set: the MQTT config
 *       // image (plc_mqtt_export(): 960 bytes), covered by the same crc16.
 *   };
 *
 * As with the Retain record layout above, this file intentionally does
 * not declare the actual struct -- only size constants -- to stay a pure
 * "where and how big" memory-map header. SPLC_RULE_FLASH_RECORD_MAX_SIZE
 * is the upper bound at MAX_RULES rules (100 by default); actual on-Flash records
 * are usually smaller (SPLC_RULE_FLASH_HEADER_SIZE + rule_count * 32
 * [+ the FB image size if the FB flag is set]) and
 * services/plc_rule_flash/plc_rule_flash.c reads/writes exactly that
 * many bytes, not this fixed maximum.
 */
#define SPLC_RULE_FLASH_HEADER_SIZE       8U    /* seq_num(4) + rule_count(2) + crc16(2) */
#define SPLC_RULE_FLASH_RECORD_WIRE_SIZE  32U   /* one rule's wire image, matches SPLC_RuleRecord's Modbus layout */
/* MAX_RULES is the product option from config/splc_opt.h (reached through
 * sx_platform_config.h, which this file already includes), not from Layer 2:
 * a Flash memory map still needs nothing from core/. plc_rule_flash.c
 * cross-checks the resulting size with a compile-time static assertion. */
#define SPLC_RULE_FLASH_FB_SIZE           128U  /* FB config image written by Step 8d firmware (plc_fb.h PLC_FB_FLASH_SIZE), cross-checked in plc_rule_flash.c */
#define SPLC_RULE_FLASH_FB_SIZE_V1        112U  /* Legacy image (before Step 8d: no Timer rule binding), read-only (plc_fb.h PLC_FB_FLASH_SIZE_V1) */
#define SPLC_RULE_FLASH_FB_FLAG           0x8000U /* bit 15 of the rule_count field: FB section present */
#define SPLC_RULE_FLASH_FB_V2_FLAG        0x4000U /* bit 14 of the rule_count field: the FB section is the 128-byte image (else 112) */
/* Bit 13 of the rule_count field: an MQTT section (SPLC_RULE_FLASH_MQTT_SIZE
 * bytes, docs/plc_mqtt_design.md section 6) follows the FB section. Only a
 * build with SPLC_MQTT_ENABLE=1 knows this flag. A build with it 0 keeps the
 * old 14-bit count mask, so it reads such a record as "count > MAX_RULES" and
 * rejects it exactly like any other record it does not understand (the same
 * downgrade rule as for the FB flags: a fresh Deploy is needed). That keeps
 * the MQTT=0 build byte-for-byte the same size as before. */
#define SPLC_RULE_FLASH_MQTT_FLAG         0x2000U /* bit 13 of the rule_count field: MQTT section present */
#define SPLC_RULE_FLASH_MQTT_SIZE         960U    /* 4 telemetry blocks x 192 B + broker image 192 B; cross-checked against PLC_MQTT_FLASH_SIZE in plc_rule_flash.c */
/* Bit 11 of the rule_count field: a tag-type table (SPLC_RULE_FLASH_TYPES_SIZE
 * bytes, one per tag; plc_tagtype.h, decision 41) follows the MQTT section
 * (layout: header | rules | FB | MQTT | MB (bit 12) | TYPES). Only a
 * build with SPLC_REAL_ENABLE=1 knows this flag; a build with it 0 keeps its
 * old count mask and rejects such a record as "count > MAX_RULES" (the same
 * downgrade rule as for the FB and MQTT flags), so it stays byte-for-byte the
 * same size as before. */
/* Bit 12 of the rule_count field: an MB_READ section (SPLC_RULE_FLASH_MB_SIZE
 * bytes = 4 blocks x 64 registers, big-endian as on the wire; plc_mbm.h,
 * decision 41) sits between the MQTT and the tag-type section. Only a build with
 * SPLC_MBM_ENABLE=1 knows this flag; a build with it 0 keeps bit 12 in its count
 * mask (same downgrade rule as the other flags), so it stays byte-for-byte the
 * same size as before. */
#define SPLC_RULE_FLASH_MB_FLAG           0x1000U /* bit 12 of the rule_count field: MB_READ section present */
#define SPLC_RULE_FLASH_MB_SIZE           640U    /* 4 x 128 B blocks + 128 B RS-485 port table (0x1400); cross-checked against PLC_MBM_FLASH_SIZE in plc_rule_flash.c */
#define SPLC_RULE_FLASH_TYPES_FLAG        0x0800U /* bit 11 of the rule_count field: tag-type section present */
#define SPLC_RULE_FLASH_TYPES_SIZE        128U    /* one byte per tag; cross-checked against PLC_TAGTYPE_FLASH_SIZE in plc_rule_flash.c */

/* Bit 10 of the rule_count field: an MQTT Subscribe section (SPLC_RULE_FLASH_SUB_SIZE
 * bytes = 2 blocks x 64 registers, big-endian as on the wire; plc_mqtt_sub.h) follows
 * the tag-type section -- the LAST section, so a build with SPLC_MQTT_SUB_ENABLE=0
 * moves no offset (layout: header | rules | FB | MQTT | MB | TYPES | SUB). Only a
 * build with SPLC_MQTT_SUB_ENABLE=1 knows this flag; a build with it 0 keeps bit 10
 * in its count mask (same downgrade rule as the other flags). */
#define SPLC_RULE_FLASH_SUB_FLAG          0x0400U /* bit 10 of the rule_count field: MQTT Subscribe section present */
#define SPLC_RULE_FLASH_SUB_SIZE          256U    /* 2 x 128 B blocks; cross-checked against PLC_MQTT_SUB_FLASH_SIZE in plc_rule_flash.c */

/* Bit 9 of the rule_count field: an MB_WRITE section (SPLC_RULE_FLASH_MBW_SIZE bytes =
 * 4 blocks x 64 registers, big-endian as on the wire; plc_mbw.h) follows the MQTT
 * Subscribe section -- the LAST section, so a build with SPLC_MBW_ENABLE=0 moves no
 * offset (layout: header | rules | FB | MQTT | MB | TYPES | SUB | MBW). Only a build
 * with SPLC_MBW_ENABLE=1 knows this flag; a build with it 0 keeps bit 9 in its count
 * mask (same downgrade rule as the other flags). */
#define SPLC_RULE_FLASH_MBW_FLAG          0x0200U /* bit 9 of the rule_count field: MB_WRITE section present */
#define SPLC_RULE_FLASH_MBW_SIZE          512U    /* 4 x 128 B blocks; cross-checked against PLC_MBW_FLASH_SIZE in plc_rule_flash.c */

/* Bit 8 of the rule_count field: a Zigbee configuration section
 * (SPLC_RULE_FLASH_ZIGBEE_SIZE bytes = registers 0x1700..0x171F, big-endian as on
 * the wire, 32 registers; SPLC-SPEC-ZIGBEE-006) follows the MB_WRITE section -- the
 * LAST section, so a build with SPLC_ZIGBEE_ENABLE=0 moves no offset (layout:
 * header | rules | FB | MQTT | MB | TYPES | SUB | MBW | ZIGBEE). The network key is
 * stored as written (plain, like the MQTT broker password); the PRESERVE_KEY bit is a
 * command and is never stored. Only a build with SPLC_ZIGBEE_ENABLE=1 knows this
 * flag; a build with it 0 keeps bit 8 in its count mask (same downgrade rule as the
 * other flags). With the flag owned the count mask is 0x00FF at most, enough for
 * MAX_RULES = 100. */
#define SPLC_RULE_FLASH_ZIGBEE_FLAG       0x0100U /* bit 8 of the rule_count field: Zigbee configuration section present */
#define SPLC_RULE_FLASH_ZIGBEE_SIZE       64U     /* 32 registers; must be cross-checked against the Zigbee module's flash size once it exists */

/* Bit 7 of the rule_count field: an MQTT Last Will section (SPLC_RULE_FLASH_WILL_SIZE bytes =
 * registers 0x0E60..0x0E7F, big-endian as on the wire, 32 registers; SPLC-SPEC-MQTT-004) follows
 * the ZIGBEE section -- the LAST section (layout: header | rules | FB | MQTT | MB | TYPES | SUB |
 * MBW | ZIGBEE | WILL). It is a section of its own because the MQTT section (960 B) has no room:
 * its broker image is 84 config registers + 12 reserved zero registers, which plc_mqtt_import()
 * insists on, and growing it would change the meaning of every record already in Flash.
 * The section is written ONLY when the Will is not all-zero (plc_mqtt_will_is_default() == false),
 * so a device that never uses the Last Will keeps writing records byte-identical to the ones
 * before this section existed (no downgrade problem for it, no extra bytes). Absent flag = Will
 * all-zero = disabled. Only a build with SPLC_MQTT_ENABLE=1 knows this flag; a build with it 0
 * keeps bit 7 in its count mask (same downgrade rule as the other flags). */
#define SPLC_RULE_FLASH_WILL_FLAG         0x0080U /* bit 7 of the rule_count field: Last Will section present */
#define SPLC_RULE_FLASH_WILL_SIZE         64U     /* 32 registers; cross-checked against PLC_MQTT_WILL_FLASH_SIZE in plc_rule_flash.c */

/* Phase 2 EXTENSION section (NOT built, NOT run on a board yet). It sits after the WILL section, i.e. at
 * the very end of the record (layout: header | rules | FB | MQTT | MB | TYPES | SUB | MBW | ZIGBEE | WILL |
 * EXT). It has no flag bit of its own: the rule_count field has none left. It is announced by the
 * combination (bit 15, bit 14) = (0, 1) of the rule_count field:
 *     (1,0)  FB present, legacy 112-byte image     (old records)
 *     (1,1)  FB present, V2 128-byte image         (every record written so far, 0xC000)
 *     (0,1)  FB V2 128-byte image present AND an EXT section follows   <-- new, phase 2
 *     (0,0)  no FB section
 * Before phase 2 the combination (0,1) was never written; rule_flash_split_field() used to read bit 15 = 0
 * as "no FB section", so a firmware without phase 2 reads such a record with a wrong size and rejects it
 * (CRC) -- downgrade to an older firmware loses the rules (see docs/handoff.md). The EXT section is written
 * ONLY when at least one phase-2 block has content, so a device that does not use them keeps writing
 * records byte-identical to the old ones (bit 15 and bit 14 both set, 0xC000).
 *
 * EXT section = header SPLC_RULE_FLASH_EXT_HDR_SIZE (4 B, big-endian: 16 extension flag bits, then 16
 * reserved bits written as 0) followed by the segments whose flag is set, in bit order:
 *     bit 0  PULSE        SPLC_RULE_FLASH_EXT_PULSE_SIZE  256 B  (8 blocks x 32 B, plc_pulse.h)
 *     bit 1  MQTT pub 4..7 SPLC_RULE_FLASH_EXT_PUB_SIZE   768 B  (plc_mqtt_ext_export())
 *     bit 2  MQTT sub 2..7 SPLC_RULE_FLASH_EXT_SUB_SIZE   768 B  (plc_mqtt_sub_ext_export())
 * All of it is covered by the same crc16 as the rest of the record. */
#define SPLC_RULE_FLASH_EXT_FLAG_VALUE    0x4000U /* (bit 15, bit 14) = (0, 1): FB V2 present + EXT section follows */
#define SPLC_RULE_FLASH_EXT_FLAG_MASK     0xC000U /* the two bits that carry that combination */
#define SPLC_RULE_FLASH_EXT_HDR_SIZE      4U      /* 16 extension flag bits + 16 reserved bits */
#define SPLC_RULE_FLASH_EXT_BIT_PULSE     0x0001U /* header flag bit 0: PULSE segment present */
#define SPLC_RULE_FLASH_EXT_BIT_PUB       0x0002U /* header flag bit 1: MQTT publish blocks 4..7 present */
#define SPLC_RULE_FLASH_EXT_BIT_SUB       0x0004U /* header flag bit 2: MQTT subscribe blocks 2..7 present */
#define SPLC_RULE_FLASH_EXT_KNOWN_BITS    (SPLC_RULE_FLASH_EXT_BIT_PULSE | SPLC_RULE_FLASH_EXT_BIT_PUB | SPLC_RULE_FLASH_EXT_BIT_SUB)
#define SPLC_RULE_FLASH_EXT_PULSE_SIZE    256U    /* 8 x 32 B; cross-checked against PLC_PULSE_FLASH_SIZE in plc_rule_flash.c */
#define SPLC_RULE_FLASH_EXT_PUB_SIZE      768U    /* 4 x 192 B; cross-checked against PLC_MQTT_EXT_FLASH_SIZE in plc_rule_flash.c */
#define SPLC_RULE_FLASH_EXT_SUB_SIZE      768U    /* 6 x 128 B; cross-checked against PLC_MQTT_SUB_EXT_FLASH_SIZE in plc_rule_flash.c */

/* The flag bits THIS build owns (0 for a feature that is compiled out): they are
 * taken out of the count mask. */
#if SPLC_MQTT_ENABLE
#define SPLC_RULE_FLASH_MQTT_RESERVE      SPLC_RULE_FLASH_MQTT_SIZE
#define SPLC_RULE_FLASH_MQTT_OWNED_FLAG   SPLC_RULE_FLASH_MQTT_FLAG
#else
#define SPLC_RULE_FLASH_MQTT_RESERVE      0U
#define SPLC_RULE_FLASH_MQTT_OWNED_FLAG   0U
#endif
#if SPLC_MBM_ENABLE
#define SPLC_RULE_FLASH_MB_RESERVE        SPLC_RULE_FLASH_MB_SIZE
#define SPLC_RULE_FLASH_MB_OWNED_FLAG     SPLC_RULE_FLASH_MB_FLAG
#else
#define SPLC_RULE_FLASH_MB_RESERVE        0U
#define SPLC_RULE_FLASH_MB_OWNED_FLAG     0U
#endif
#if SPLC_REAL_ENABLE
#define SPLC_RULE_FLASH_TYPES_RESERVE     SPLC_RULE_FLASH_TYPES_SIZE
#define SPLC_RULE_FLASH_TYPES_OWNED_FLAG  SPLC_RULE_FLASH_TYPES_FLAG
#else
#define SPLC_RULE_FLASH_TYPES_RESERVE     0U
#define SPLC_RULE_FLASH_TYPES_OWNED_FLAG  0U
#endif
#if SPLC_MQTT_SUB_ENABLE
#define SPLC_RULE_FLASH_SUB_RESERVE       SPLC_RULE_FLASH_SUB_SIZE
#define SPLC_RULE_FLASH_SUB_OWNED_FLAG    SPLC_RULE_FLASH_SUB_FLAG
#else
#define SPLC_RULE_FLASH_SUB_RESERVE       0U
#define SPLC_RULE_FLASH_SUB_OWNED_FLAG    0U
#endif
#if SPLC_MBW_ENABLE
#define SPLC_RULE_FLASH_MBW_RESERVE       SPLC_RULE_FLASH_MBW_SIZE
#define SPLC_RULE_FLASH_MBW_OWNED_FLAG    SPLC_RULE_FLASH_MBW_FLAG
#else
#define SPLC_RULE_FLASH_MBW_RESERVE       0U
#define SPLC_RULE_FLASH_MBW_OWNED_FLAG    0U
#endif
#if SPLC_ZIGBEE_ENABLE
#define SPLC_RULE_FLASH_ZIGBEE_RESERVE    SPLC_RULE_FLASH_ZIGBEE_SIZE
#define SPLC_RULE_FLASH_ZIGBEE_OWNED_FLAG SPLC_RULE_FLASH_ZIGBEE_FLAG
#else
#define SPLC_RULE_FLASH_ZIGBEE_RESERVE    0U
#define SPLC_RULE_FLASH_ZIGBEE_OWNED_FLAG 0U
#endif
#if SPLC_MQTT_ENABLE
#define SPLC_RULE_FLASH_WILL_RESERVE      SPLC_RULE_FLASH_WILL_SIZE   /* worst case: the section is optional per record */
#define SPLC_RULE_FLASH_WILL_OWNED_FLAG   SPLC_RULE_FLASH_WILL_FLAG
#else
#define SPLC_RULE_FLASH_WILL_RESERVE      0U
#define SPLC_RULE_FLASH_WILL_OWNED_FLAG   0U
#endif
/* Worst-case bytes of the EXT section for THIS build: the header plus every segment whose feature is
 * compiled in (0 when none is: then the build has no EXT section at all). The section itself is optional
 * per record. It owns no bit of the count mask (it uses the bit 15/14 combination, see above). */
#if SPLC_MQTT_ENABLE
#define SPLC_RULE_FLASH_EXT_PUB_RESERVE   SPLC_RULE_FLASH_EXT_PUB_SIZE
#else
#define SPLC_RULE_FLASH_EXT_PUB_RESERVE   0U
#endif
#if SPLC_MQTT_SUB_ENABLE
#define SPLC_RULE_FLASH_EXT_SUB_RESERVE   SPLC_RULE_FLASH_EXT_SUB_SIZE
#else
#define SPLC_RULE_FLASH_EXT_SUB_RESERVE   0U
#endif
#if SPLC_PULSE_ENABLE
#define SPLC_RULE_FLASH_EXT_PULSE_RESERVE SPLC_RULE_FLASH_EXT_PULSE_SIZE
#else
#define SPLC_RULE_FLASH_EXT_PULSE_RESERVE 0U
#endif
#define SPLC_RULE_FLASH_EXT_SEGS_RESERVE  (SPLC_RULE_FLASH_EXT_PUB_RESERVE + SPLC_RULE_FLASH_EXT_SUB_RESERVE + SPLC_RULE_FLASH_EXT_PULSE_RESERVE)
#define SPLC_RULE_FLASH_EXT_RESERVE       ((SPLC_RULE_FLASH_EXT_SEGS_RESERVE) != 0U ? (SPLC_RULE_FLASH_EXT_HDR_SIZE + SPLC_RULE_FLASH_EXT_SEGS_RESERVE) : 0U)
/* The real rule count: bits 0..13 minus the flags this build owns
 * (0x3FFF; minus bit 13 with MQTT, bit 12 with MBM, bit 11 with REAL, bit 10 with MQTT SUB, bit 9 with MB_WRITE, bit 8 with Zigbee, bit 7 with MQTT = the Last Will section; with every flag owned the mask is 0x007F = 127 rules, MAX_RULES is at most 100). */
#define SPLC_RULE_FLASH_COUNT_MASK        ((uint16_t)(0x3FFFU & ~(SPLC_RULE_FLASH_MQTT_OWNED_FLAG | SPLC_RULE_FLASH_MB_OWNED_FLAG | SPLC_RULE_FLASH_TYPES_OWNED_FLAG | SPLC_RULE_FLASH_SUB_OWNED_FLAG | SPLC_RULE_FLASH_MBW_OWNED_FLAG | SPLC_RULE_FLASH_ZIGBEE_OWNED_FLAG | SPLC_RULE_FLASH_WILL_OWNED_FLAG)))
#define SPLC_RULE_FLASH_RECORD_MAX_SIZE   (SPLC_RULE_FLASH_HEADER_SIZE + (uint32_t)MAX_RULES * SPLC_RULE_FLASH_RECORD_WIRE_SIZE + SPLC_RULE_FLASH_FB_SIZE + SPLC_RULE_FLASH_MQTT_RESERVE + SPLC_RULE_FLASH_MB_RESERVE + SPLC_RULE_FLASH_TYPES_RESERVE + SPLC_RULE_FLASH_SUB_RESERVE + SPLC_RULE_FLASH_MBW_RESERVE + SPLC_RULE_FLASH_ZIGBEE_RESERVE + SPLC_RULE_FLASH_WILL_RESERVE + SPLC_RULE_FLASH_EXT_RESERVE) /* 3336 bytes at MAX_RULES = 100 (4296 with SPLC_MQTT_ENABLE=1, +640 with SPLC_MBM_ENABLE=1, +128 with SPLC_REAL_ENABLE=1, +256 with SPLC_MQTT_SUB_ENABLE=1, +512 with SPLC_MBW_ENABLE=1, +64 with SPLC_ZIGBEE_ENABLE=1, +64 with SPLC_MQTT_ENABLE=1 for the optional Last Will section, + up to 1796 for the optional phase-2 EXT section: 4 B header + 768 pub + 768 sub + 256 PULSE; 7756 with everything on) */
_Static_assert(SPLC_RULE_FLASH_RECORD_MAX_SIZE <= SPLC_FLASH_SECTOR_SIZE,
               "a rule record must fit in one Flash sector");

#endif // STM32H5_PLATFORM

#ifdef __cplusplus
}
#endif

#endif /* SPLC_FLASH_DEFINE_H */