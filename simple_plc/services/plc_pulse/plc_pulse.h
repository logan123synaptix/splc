#ifndef PLC_PULSE_H
#define PLC_PULSE_H

/*
 * plc_pulse.h - Layer 3 (PLC Application Services)
 *
 * PULSE_OUT (SPLC-SPEC-DIRECT-WRITE-003 / PULSE-003): STORAGE ONLY, phase 2.
 * This module HOLDS the configuration of up to 8 PULSE blocks and gives it back
 * (FC03), stages it (FC16 -> draft -> COMMIT), keeps it in the Flash Rule Table
 * record and hands it to the host through plc_pulse_get(). It does NOT drive any
 * output: no GPIO, no PWM, no PTO, no addressable-bus code lives here. The PULSE
 * BEHAVIOUR is ported by the user on top of plc_pulse_get().
 *
 * Wire (FC03 / FC06 / FC16): block i (0..7) at 0x1200 + i * 16, 16 registers = 32 B.
 * The 8 blocks therefore fill 0x1200..0x127F. The documents label the area
 * 0x1200..0x12FF; the upper half 0x1280..0x12FF holds no block: FC03 returns 0 there,
 * a write that reaches it is BAD_ADDRESS (0x02).
 *
 *   Reg 0     (mode << 8) | target_pin   mode 0 DO, 1 PWM, 2 pulse train (PTO), 3 addressable Bus
 *                                        target_pin 0..7 = DO0..DO7
 *   Reg 1     channel_count              Bus: 1..256 channels; PTO: number of pulses
 *   Reg 2     (trigger_mode << 8) | flags
 *                                        trigger_mode 0 = rising edge, 1 = continuous (only 0/1)
 *                                        flags bit 0 active high, bit 1 exclusive channel,
 *                                        bit 2 fail-safe state, bit 3 chip (0 WS2812B GRB,
 *                                        1 UCS16703 RGB)
 *   Reg 3     pulse_width_us
 *   Reg 4..5  pwm_freq_hz                32 bit, High Word first
 *   Reg 6     (duty_max << 8) | duty_min
 *   Reg 7     channel_tag_index
 *   Reg 8     param_tag_index
 *   Reg 9..10 default_color_rgb          32 bit, High Word first
 *   Reg 11    trig_tag    Reg 12 done_tag    Reg 13 busy_tag    Reg 14 error_tag   (0xFFFF = unmapped)
 *   Reg 15    reserved, always 0
 * The registers are big-endian words: every register is decoded ONE BY ONE. Never
 * overlay the C struct of the ICD on the wire words (its byte order differs).
 * The source of truth is the App codec + the team's reply to the PULSE questions;
 * spec v2.2.0 is NOT in the repository yet (docs/firmware/SPLC_SPEC_DIRECT_WRITE_003_ICD.md
 * is v2.1.0 and differs in a few fields).
 *
 * "Empty" block = all 16 registers 0 (not configured). A block that was configured and
 * then switched off still has content, so it is NOT empty.
 *
 * Staging (same idea as plc_mqtt_sub): host writes go to a DRAFT block (copied from the
 * running block on the first write to it, so a partial write keeps the rest) and are
 * checked at COMMIT:
 *     plc_pulse_validate_draft() : false = reject the COMMIT, nothing changed
 *     plc_pulse_commit_draft()   : the drafted blocks become the running blocks
 *     plc_pulse_discard_draft()  : drop the draft
 * A Deploy that does not write a block keeps it running.
 *
 * What COMMIT checks in a non-empty block (every rule below is grounded in the ICD or in
 * the App team's reply; nothing else is rejected, so the App is never refused for a field
 * this module does not understand yet): mode 0..3, target_pin 0..7, trigger_mode 0..1,
 * reserved Reg 15 = 0, and for the Bus mode 1 <= channel_count <= 256.
 *
 * Flash: PLC_PULSE_FLASH_SIZE bytes = the 128 RUNNING registers, big-endian, as a section
 * of the optional extension part of the Rule Table record (bit 0 of the extension flags,
 * see plc_rule_flash.c). The section is left out of the record while every block is empty.
 *
 * Threading: single-threaded, engine context only. Compiled only with SPLC_PULSE_ENABLE = 1,
 * so a build with it 0 keeps its exact baseline size.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "splc_opt.h"

#if SPLC_PULSE_ENABLE

#ifdef __cplusplus
extern "C" {
#endif

#define PLC_PULSE_BASE          0x1200U
#define PLC_PULSE_BLOCKS        8U
#define PLC_PULSE_BLOCK_REGS    16U
#define PLC_PULSE_REGS          (PLC_PULSE_BLOCKS * PLC_PULSE_BLOCK_REGS)   /* 128: 0x1200..0x127F */
#define PLC_PULSE_WINDOW_REGS   256U     /* the documented window 0x1200..0x12FF */
#define PLC_PULSE_FLASH_SIZE    (PLC_PULSE_REGS * 2U)                        /* 256 */

#define PLC_PULSE_TAG_UNMAPPED  0xFFFFU

#define PLC_PULSE_MODE_DO       0U
#define PLC_PULSE_MODE_PWM      1U
#define PLC_PULSE_MODE_PTO      2U
#define PLC_PULSE_MODE_BUS      3U
#define PLC_PULSE_MAX_PIN       7U
#define PLC_PULSE_BUS_MAX_CHANNELS 256U

typedef enum {
    PLC_PULSE_WRITE_OK = 0,
    PLC_PULSE_WRITE_BAD_ADDRESS        /* -> Modbus 0x02 */
} plc_pulse_write_result_t;

/* One block, decoded register by register (see the wire table above). */
typedef struct {
    uint8_t  mode;
    uint8_t  target_pin;
    uint16_t channel_count;
    uint8_t  trigger_mode;
    bool     active_high;
    bool     exclusive_channel;
    bool     fail_safe_on;
    uint8_t  chip;                 /* 0 = WS2812B GRB, 1 = UCS16703 RGB */
    uint16_t pulse_width_us;
    uint32_t pwm_freq_hz;
    uint8_t  duty_min;
    uint8_t  duty_max;
    uint16_t channel_tag;
    uint16_t param_tag;
    uint32_t default_color_rgb;
    uint16_t trig_tag;             /* PLC_PULSE_TAG_UNMAPPED = none */
    uint16_t done_tag;
    uint16_t busy_tag;
    uint16_t error_tag;
} plc_pulse_cfg_t;

/* Boot: every block empty, draft dropped. Call once, before plc_rule_flash_load(). */
void plc_pulse_init(void);

/* FC03 of `quantity` registers at `offset` (0..255): the RUNNING blocks; 0 from 128 up. */
void plc_pulse_read(uint16_t offset, uint16_t quantity, uint16_t *registers_out);

/* FC16 (FC06 arrives as quantity 1) into the DRAFT. A request that reaches offset 128
 * (0x1280) or beyond is BAD_ADDRESS and changes nothing. Content is checked at COMMIT. */
plc_pulse_write_result_t plc_pulse_write(uint16_t offset, uint16_t quantity,
                                         const uint16_t *registers);

bool plc_pulse_validate_draft(void);   /* true also when nothing was written */
void plc_pulse_commit_draft(void);     /* call only after validate returned true */
void plc_pulse_discard_draft(void);

/* Flash image: PLC_PULSE_FLASH_SIZE bytes = the 128 RUNNING registers, big-endian. */
void plc_pulse_export(uint8_t *out);

/* Load an image written by plc_pulse_export(): the whole image is checked with the COMMIT
 * rules; on failure NOTHING is applied and false is returned. Drops any draft. */
bool plc_pulse_import(const uint8_t *in);

/* true when every register of every block is 0 (plc_rule_flash then leaves the section out). */
bool plc_pulse_is_default(void);

/* plc_pulse_clear(): every RUNNING block empty (a record without the section, or a rollback of a
 * failed CLEAR_RULES); the draft is not touched. plc_pulse_clear_all(): the same for CLEAR_RULES /
 * FACTORY_RESET, and the draft is dropped too. */
void plc_pulse_clear(void);
void plc_pulse_clear_all(void);

/* Host side: the running block `block` (0..7) decoded. Returns false when the block is empty
 * or `block` is out of range; `out` may be NULL (then it only tells whether the block is in use). */
bool plc_pulse_get(uint8_t block, plc_pulse_cfg_t *out);

/* Moves whenever the running configuration changed (COMMIT with new content, Flash import,
 * clear). The host re-reads its blocks when it differs from the value it saw. */
uint32_t plc_pulse_generation(void);

#ifdef __cplusplus
}
#endif

#endif /* SPLC_PULSE_ENABLE */

#endif /* PLC_PULSE_H */