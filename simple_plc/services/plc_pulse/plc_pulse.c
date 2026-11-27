/*
 * plc_pulse.c - Layer 3. See plc_pulse.h. Storage only: no output is driven here.
 */
#include "plc_pulse.h"

#if SPLC_PULSE_ENABLE

#include <string.h>

#include "splc_log.h"

static const char *TAG = "PLC_PULSE";

#define BLOCKS   PLC_PULSE_BLOCKS
#define REGS     PLC_PULSE_BLOCK_REGS

_Static_assert(PLC_PULSE_BLOCKS <= 8U, "s_draft_mask is 8 bits: one bit per block");
_Static_assert(PLC_PULSE_BLOCK_REGS == 16U, "a PULSE block is 16 registers = 32 bytes");
_Static_assert(PLC_PULSE_FLASH_SIZE == PLC_PULSE_BLOCKS * PLC_PULSE_BLOCK_REGS * 2U,
               "PULSE Flash image = 2 bytes per register");
_Static_assert(PLC_PULSE_WINDOW_REGS >= PLC_PULSE_REGS, "the Modbus window must hold every block");

/* Register index inside a block. */
#define R_MODE_PIN    0U
#define R_COUNT       1U
#define R_TRIG_FLAGS  2U
#define R_WIDTH       3U
#define R_FREQ_HI     4U
#define R_FREQ_LO     5U
#define R_DUTY        6U
#define R_CH_TAG      7U
#define R_PARAM_TAG   8U
#define R_COLOR_HI    9U
#define R_COLOR_LO    10U
#define R_TRIG_TAG    11U
#define R_DONE_TAG    12U
#define R_BUSY_TAG    13U
#define R_ERR_TAG     14U
#define R_RESERVED    15U

/* Flags (low byte of Reg 2). */
#define FLAG_ACTIVE_HIGH  0x01U
#define FLAG_EXCLUSIVE    0x02U
#define FLAG_FAILSAFE_ON  0x04U
#define FLAG_CHIP         0x08U

static uint16_t s_run[BLOCKS][REGS];       /* RUNNING */
static uint16_t s_draft[BLOCKS][REGS];     /* valid only where the mask bit is set */
static uint8_t  s_draft_mask;
static uint32_t s_gen;

static bool block_empty(const uint16_t *b)
{
    for (unsigned r = 0U; r < REGS; r++) {
        if (b[r] != 0U) { return false; }
    }
    return true;
}

static bool block_valid(const uint16_t *b, unsigned idx)
{
    if (block_empty(b)) {
        return true;                       /* the App writes 0 into a block it no longer uses */
    }
    const unsigned mode = (unsigned)(b[R_MODE_PIN] >> 8);
    const unsigned pin  = (unsigned)(b[R_MODE_PIN] & 0xFFU);
    const unsigned trig = (unsigned)(b[R_TRIG_FLAGS] >> 8);

    if (mode > PLC_PULSE_MODE_BUS) { log_warn(TAG, "block %u: mode %u", idx, mode); return false; }
    if (pin > PLC_PULSE_MAX_PIN)   { log_warn(TAG, "block %u: target pin %u", idx, pin); return false; }
    if (trig > 1U)                 { log_warn(TAG, "block %u: trigger mode %u", idx, trig); return false; }
    if (b[R_RESERVED] != 0U)       { log_warn(TAG, "block %u: reserved register is not 0", idx); return false; }
    if (mode == PLC_PULSE_MODE_BUS &&
        (b[R_COUNT] < 1U || b[R_COUNT] > PLC_PULSE_BUS_MAX_CHANNELS)) {
        log_warn(TAG, "block %u: bus channel count %u", idx, (unsigned)b[R_COUNT]);
        return false;
    }
    return true;
}

static uint32_t get_u32(const uint16_t *r) { return ((uint32_t)r[0] << 16) | r[1]; }

static uint16_t be16(const uint8_t *in, unsigned idx)
{
    return (uint16_t)(((uint16_t)in[idx * 2U] << 8) | in[idx * 2U + 1U]);
}

/* ------------------------------------------------------------------------ */

void plc_pulse_clear_all(void)
{
    memset(s_run, 0, sizeof(s_run));
    memset(s_draft, 0, sizeof(s_draft));
    s_draft_mask = 0U;
    s_gen++;
}

void plc_pulse_init(void)
{
    s_gen = 0U;
    plc_pulse_clear_all();
}

void plc_pulse_clear(void)
{
    if (!plc_pulse_is_default()) {
        s_gen++;
    }
    memset(s_run, 0, sizeof(s_run));
}

void plc_pulse_read(uint16_t offset, uint16_t quantity, uint16_t *registers_out)
{
    for (uint16_t i = 0U; i < quantity; i++) {
        const uint16_t o = (uint16_t)(offset + i);
        registers_out[i] = (o < PLC_PULSE_REGS) ? s_run[o / REGS][o % REGS] : 0U;
    }
}

plc_pulse_write_result_t plc_pulse_write(uint16_t offset, uint16_t quantity, const uint16_t *registers)
{
    if ((uint32_t)offset + quantity > PLC_PULSE_REGS) {
        log_warn(TAG, "write rejected: offset=%u quantity=%u runs past 0x127F", offset, quantity);
        return PLC_PULSE_WRITE_BAD_ADDRESS;
    }
    for (uint16_t i = 0U; i < quantity; i++) {
        const uint16_t o = (uint16_t)(offset + i);
        const unsigned blk = o / REGS;
        if ((s_draft_mask & (1U << blk)) == 0U) {
            memcpy(s_draft[blk], s_run[blk], sizeof(s_draft[blk]));    /* a partial write keeps the rest */
            s_draft_mask |= (uint8_t)(1U << blk);
        }
        s_draft[blk][o % REGS] = registers[i];
    }
    return PLC_PULSE_WRITE_OK;
}

bool plc_pulse_validate_draft(void)
{
    for (unsigned i = 0U; i < BLOCKS; i++) {
        if ((s_draft_mask & (1U << i)) != 0U && !block_valid(s_draft[i], i)) { return false; }
    }
    return true;
}

void plc_pulse_commit_draft(void)
{
    bool changed = false;
    for (unsigned i = 0U; i < BLOCKS; i++) {
        if ((s_draft_mask & (1U << i)) != 0U) {
            if (memcmp(s_run[i], s_draft[i], sizeof(s_run[i])) != 0) { changed = true; }
            memcpy(s_run[i], s_draft[i], sizeof(s_run[i]));
        }
    }
    if (s_draft_mask != 0U) {
        if (changed) { s_gen++; }
        log_info(TAG, "COMMIT: draft_mask=0x%02X, changed=%d", (unsigned)s_draft_mask, (int)changed);
    }
    s_draft_mask = 0U;
}

void plc_pulse_discard_draft(void) { s_draft_mask = 0U; }

void plc_pulse_export(uint8_t *out)
{
    for (unsigned b = 0U; b < BLOCKS; b++) {
        for (unsigned r = 0U; r < REGS; r++) {
            const uint16_t v = s_run[b][r];
            out[(b * REGS + r) * 2U]      = (uint8_t)(v >> 8);
            out[(b * REGS + r) * 2U + 1U] = (uint8_t)(v & 0xFFU);
        }
    }
}

bool plc_pulse_import(const uint8_t *in)
{
    uint16_t blk[REGS];                          /* 32 B on the stack: no static copy */
    for (unsigned b = 0U; b < BLOCKS; b++) {
        for (unsigned r = 0U; r < REGS; r++) { blk[r] = be16(in, b * REGS + r); }
        if (!block_valid(blk, b)) {
            log_warn(TAG, "Flash PULSE image failed validation -- PULSE config left as it was");
            return false;                                              /* nothing applied yet */
        }
    }
    for (unsigned b = 0U; b < BLOCKS; b++) {
        for (unsigned r = 0U; r < REGS; r++) { s_run[b][r] = be16(in, b * REGS + r); }
    }
    s_draft_mask = 0U;
    s_gen++;
    return true;
}

bool plc_pulse_is_default(void)
{
    for (unsigned b = 0U; b < BLOCKS; b++) {
        if (!block_empty(s_run[b])) { return false; }
    }
    return true;
}

bool plc_pulse_get(uint8_t block, plc_pulse_cfg_t *out)
{
    if (block >= BLOCKS || block_empty(s_run[block])) { return false; }
    if (out == NULL) { return true; }

    const uint16_t *b = s_run[block];
    out->mode              = (uint8_t)(b[R_MODE_PIN] >> 8);
    out->target_pin        = (uint8_t)(b[R_MODE_PIN] & 0xFFU);
    out->channel_count     = b[R_COUNT];
    out->trigger_mode      = (uint8_t)(b[R_TRIG_FLAGS] >> 8);
    out->active_high       = (b[R_TRIG_FLAGS] & FLAG_ACTIVE_HIGH) != 0U;
    out->exclusive_channel = (b[R_TRIG_FLAGS] & FLAG_EXCLUSIVE) != 0U;
    out->fail_safe_on      = (b[R_TRIG_FLAGS] & FLAG_FAILSAFE_ON) != 0U;
    out->chip              = (uint8_t)(((b[R_TRIG_FLAGS] & FLAG_CHIP) != 0U) ? 1U : 0U);
    out->pulse_width_us    = b[R_WIDTH];
    out->pwm_freq_hz       = get_u32(&b[R_FREQ_HI]);
    out->duty_min          = (uint8_t)(b[R_DUTY] & 0xFFU);
    out->duty_max          = (uint8_t)(b[R_DUTY] >> 8);
    out->channel_tag       = b[R_CH_TAG];
    out->param_tag         = b[R_PARAM_TAG];
    out->default_color_rgb = get_u32(&b[R_COLOR_HI]);
    out->trig_tag          = b[R_TRIG_TAG];
    out->done_tag          = b[R_DONE_TAG];
    out->busy_tag          = b[R_BUSY_TAG];
    out->error_tag         = b[R_ERR_TAG];
    return true;
}

uint32_t plc_pulse_generation(void) { return s_gen; }

#endif /* SPLC_PULSE_ENABLE */