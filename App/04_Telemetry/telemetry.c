/**
 * telemetry.c
 * =========================================================================
 * Binary telemetry frame builder + W25Q64 offline ring-buffer logger.
 * =========================================================================
 */
#include "telemetry.h"
#include "debug_cli.h"
#include <string.h>

/* ── CRC-16 (Modbus, poly 0xA001) ───────────────────────────────── */
static uint16_t crc16(const uint8_t *d, uint16_t l)
{
    uint16_t c=0xFFFF;
    while(l--){ c^=*d++; for(uint8_t i=0;i<8;i++) c=(c&1)?(c>>1)^0xA001:(c>>1); }
    return c;
}

/* ── Frame builder ───────────────────────────────────────────────── */
static uint16_t seq_counter = 0;

void Telem_Init(void) { seq_counter = 0; }

void Telem_BuildFrame(TelemFrame_t *f, const uint16_t *regs,
                       uint8_t num_regs, uint32_t timestamp)
{
    const GatewayConfig_t *cfg = Config_Get();
    f->magic     = TELEM_MAGIC;
    f->version   = TELEM_VERSION;
    f->device_id = cfg->device_id;
    f->timestamp = timestamp;
    f->seq       = seq_counter++;
    f->num_regs  = num_regs;
    memset(f->regs, 0, sizeof(f->regs));
    memcpy(f->regs, regs, num_regs * sizeof(uint16_t));
    /* CRC over everything except crc16 field */
    f->crc16 = crc16((const uint8_t*)f,
                     sizeof(TelemFrame_t) - sizeof(uint16_t));
}

uint16_t Telem_BuildDeltaFrame(uint8_t *out, uint16_t out_size,
                                const uint16_t *cur, const uint16_t *last,
                                uint8_t num_regs, uint32_t ts)
{
    const GatewayConfig_t *cfg = Config_Get();
    uint8_t count = 0;

    /* Find changed registers */
    uint8_t  changed_idx[CONFIG_MAX_REGS];
    uint16_t changed_val[CONFIG_MAX_REGS];
    for (uint8_t i = 0; i < num_regs; i++) {
        uint16_t diff = (cur[i]>last[i])?(cur[i]-last[i]):(last[i]-cur[i]);
        if (diff >= cfg->delta_threshold) {
            changed_idx[count] = i;
            changed_val[count] = cur[i];
            count++;
        }
    }
    if (!count) return 0;

    /* Build delta frame */
    uint16_t needed = 1+2+4+2+1 + count*3 + 2;  /* header+pairs+crc */
    if (needed > out_size) return 0;

    uint8_t *p = out;
    *p++=TELEM_DELTA_MAGIC;
    *p++=(uint8_t)(cfg->device_id&0xFF); *p++=(uint8_t)(cfg->device_id>>8);
    *p++=(uint8_t)(ts&0xFF);*p++=(uint8_t)((ts>>8)&0xFF);
    *p++=(uint8_t)((ts>>16)&0xFF);*p++=(uint8_t)((ts>>24)&0xFF);
    *p++=(uint8_t)(seq_counter&0xFF);*p++=(uint8_t)(seq_counter>>8);
    seq_counter++;
    *p++=count;
    for (uint8_t i = 0; i < count; i++) {
        *p++=changed_idx[i];
        *p++=(uint8_t)(changed_val[i]&0xFF);
        *p++=(uint8_t)(changed_val[i]>>8);
    }
    uint16_t c=crc16(out,needed-2);
    *p++=(uint8_t)(c&0xFF); *p++=(uint8_t)(c>>8);
    return needed;
}

/* ── SPI log ring buffer ─────────────────────────────────────────── */
#pragma pack(1)
typedef struct {
    uint32_t magic;
    uint32_t write_idx;   /* total frames written */
    uint32_t read_idx;    /* total frames flushed */
    uint32_t crc32;
} LogMeta_t;
#pragma pack()

static LogMeta_t s_lm;
static bool      s_lm_loaded = false;

static uint32_t lm_crc(const LogMeta_t *m)
{
    uint32_t c=0xFFFFFFFFUL;
    const uint8_t *d=(const uint8_t*)m; uint32_t l=sizeof(LogMeta_t)-4U;
    while(l--){ c^=*d++; for(uint8_t b=0;b<8;b++) c=(c&1)?((c>>1)^0xEDB88320UL):(c>>1); }
    return c^0xFFFFFFFFUL;
}

static void load_meta(void)
{
    if (s_lm_loaded) return;
    W25Q_Read(LOG_REGION_START,(uint8_t*)&s_lm,sizeof(LogMeta_t));
    if (s_lm.magic!=LOG_META_MAGIC||s_lm.crc32!=lm_crc(&s_lm)) {
        s_lm.magic=LOG_META_MAGIC; s_lm.write_idx=0; s_lm.read_idx=0;
        s_lm.crc32=lm_crc(&s_lm);
        W25Q_EraseSector(LOG_REGION_START);
        W25Q_Write(LOG_REGION_START,(uint8_t*)&s_lm,sizeof(LogMeta_t));
        Debug_Print("[LOG] Log area initialised\r\n");
    }
    s_lm_loaded=true;
}

static void save_meta(void)
{
    s_lm.crc32=lm_crc(&s_lm);
    W25Q_EraseSector(LOG_REGION_START);
    W25Q_Write(LOG_REGION_START,(uint8_t*)&s_lm,sizeof(LogMeta_t));
}

static uint32_t frame_addr(uint32_t idx)
{
    uint32_t slot = idx % LOG_MAX_FRAMES;
    return LOG_REGION_START + sizeof(LogMeta_t) + slot * TELEM_FRAME_SIZE;
}

void SpiLog_Write(const uint8_t *frame, uint16_t len)
{
    load_meta();
    uint32_t addr = frame_addr(s_lm.write_idx);
    if ((addr % W25Q_SECTOR_SIZE) == 0) W25Q_EraseSector(addr);
    W25Q_Write(addr, frame, len);
    s_lm.write_idx++;
    save_meta();
}

uint32_t SpiLog_Pending(void)
{ load_meta(); return s_lm.write_idx - s_lm.read_idx; }

bool SpiLog_ReadNext(uint8_t *buf, uint16_t *len)
{
    load_meta();
    if (s_lm.read_idx >= s_lm.write_idx) return false;
    W25Q_Read(frame_addr(s_lm.read_idx), buf, TELEM_FRAME_SIZE);
    *len = TELEM_FRAME_SIZE;
    return true;
}

void SpiLog_ConfirmRead(void)
{ load_meta(); s_lm.read_idx++; save_meta(); }

void SpiLog_Clear(void)
{
    load_meta();
    s_lm.read_idx=s_lm.write_idx;
    save_meta();
    Debug_Print("[LOG] Log cleared\r\n");
}
