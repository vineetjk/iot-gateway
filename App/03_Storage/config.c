/**
 * config.c
 * =========================================================================
 * Flash-backed config with ping-pong wear levelling.
 * Pages: 0x0801F000 (A) and 0x0801F800 (B), 2KB each.
 * =========================================================================
 */
#include "config.h"
#include "debug_cli.h"
#include <string.h>
#include <stdio.h>

#define CFG_PAGE_A  0x0801E000UL   /* Page at 120KB */
#define CFG_PAGE_B  0x0801E800UL   /* Page at 122KB */
#define CFG_PAGE_SZ 2048U

static GatewayConfig_t s_cfg;

/* Temp read buffers — kept as statics to avoid stack overflow
 * (two GatewayConfig_t on the stack = ~3KB > 2KB min stack) */
static GatewayConfig_t s_tmp_a;
static GatewayConfig_t s_tmp_b;

/* ── CRC-32 ──────────────────────────────────────────────────────── */
static uint32_t crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    while (len--) {
        crc ^= *data++;
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFFUL;
}

static uint32_t cfg_crc(const GatewayConfig_t *c)
{
    return crc32((const uint8_t *)c, sizeof(GatewayConfig_t) - 4U);
}

/* ── MCU Unique ID → 16-bit hash ────────────────────────────────── */
uint16_t Config_GetMcuId(void)
{
    /* STM32F103 96-bit unique ID at 0x1FFFF7E8 */
    uint32_t u0 = *(volatile uint32_t *)(STM32_UID_BASE);
    uint32_t u1 = *(volatile uint32_t *)(STM32_UID_BASE + 4U);
    uint32_t u2 = *(volatile uint32_t *)(STM32_UID_BASE + 8U);

    if ((u0 | u1 | u2) != 0) {
        /* Normal: XOR-fold 96-bit UID to 16 bits */
        uint32_t h = u0 ^ u1 ^ u2;
        return (uint16_t)((h ^ (h >> 16)) & 0xFFFFU);
    }

    /* Fallback: UID blank (some STM32 batches).
     * Return 0 here — caller (Config_LoadDefaults / Config_Init)
     * will detect 0 and generate a random ID using SysTick,
     * then persist it with 'save' so it stays stable. */
    return 0;
}

/* ── Defaults ────────────────────────────────────────────────────── */
void Config_LoadDefaults(void)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.magic              = CONFIG_MAGIC;
    s_cfg.cfg_version        = CONFIG_VERSION;
    s_cfg.write_count        = 0;
    s_cfg.device_id          = Config_GetMcuId();
    if (s_cfg.device_id == 0) {
        /* UID unavailable — generate pseudo-random ID from SysTick + IDCODE.
         * Will be stable once saved to flash with 'save' command. */
        uint32_t seed = HAL_GetTick() ^ *(volatile uint32_t *)(0xE0042000UL);
        seed ^= (seed >> 13); seed *= 0x5BD1E995UL; seed ^= (seed >> 15);
        s_cfg.device_id = (uint16_t)(seed & 0xFFFFU);
        if (s_cfg.device_id == 0) s_cfg.device_id = 0x0001;
    }
    snprintf(s_cfg.device_name, 32, "Delta-GW-%04X", s_cfg.device_id);
    s_cfg.modbus_baud        = 9600;
    s_cfg.modbus_parity      = 0;
    s_cfg.modbus_stop_bits   = 1;
    s_cfg.modbus_timeout_ms  = 500;
    s_cfg.poll_interval_ms   = 5000;
    s_cfg.num_regs           = 0;
    s_cfg.publish_interval_ms = 10000;
    s_cfg.delta_mode         = 1;
    s_cfg.delta_threshold    = 1;
    strncpy(s_cfg.apn,           "airtelgprs.com",        31);
    strncpy(s_cfg.broker,        "broker.emqx.io",        63);
    s_cfg.broker_port        = 1883;
    snprintf(s_cfg.mqtt_client_id, 32, "delta-gw-%04X", s_cfg.device_id);
    strncpy(s_cfg.mqtt_user,     "",                      31);
    strncpy(s_cfg.mqtt_pass,     "",                      31);
    snprintf(s_cfg.mqtt_topic, 64, "delta/gw/%04X/data", s_cfg.device_id);
    snprintf(s_cfg.mqtt_alarm_topic, 64, "delta/gw/%04X/alarms", s_cfg.device_id);
    s_cfg.modem_type         = MODEM_QUECTEL_EC200U;
    s_cfg.mqtt_qos           = 1;
    strncpy(s_cfg.ota_manifest_url,
            "https://yourserver.com/fw/manifest.json",    127);
    s_cfg.fw_version         = 0x00010000UL;
    s_cfg.ota_enabled        = 1;
    s_cfg.num_alarm_bits     = 0;
    s_cfg.crc32              = cfg_crc(&s_cfg);
}

bool Config_Validate(const GatewayConfig_t *c)
{
    return (c->magic       == CONFIG_MAGIC   &&
            c->cfg_version == CONFIG_VERSION &&
            c->crc32       == cfg_crc(c));
}

/* ── Flash helpers ───────────────────────────────────────────────── */
static void flash_read(uint32_t addr, GatewayConfig_t *dst)
{
    memcpy(dst, (const void *)addr, sizeof(GatewayConfig_t));
}

static HAL_StatusTypeDef flash_write(uint32_t page_addr,
                                      const GatewayConfig_t *cfg)
{
    /* STM32F103 medium-density has 1KB pages.
     * GatewayConfig_t is >1KB, so we must erase 2 pages per slot.
     * Erasing only 1 page caused writes beyond 1KB to fail
     * because you cannot program a non-0xFF cell. */
    uint32_t n_pages = (sizeof(GatewayConfig_t) + 1023U) / 1024U;

    FLASH_EraseInitTypeDef e = {
        .TypeErase   = FLASH_TYPEERASE_PAGES,
        .PageAddress = page_addr,
        .NbPages     = n_pages
    };
    uint32_t err = 0;
    HAL_FLASH_Unlock();
    if (HAL_FLASHEx_Erase(&e, &err) != HAL_OK) {
        Debug_Printf("[CFG] Erase FAILED at 0x%08lX SR=0x%08lX\r\n",
                     err, FLASH->SR);
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    const uint16_t *src  = (const uint16_t *)cfg;
    uint32_t        addr = page_addr;
    uint32_t        n    = (sizeof(GatewayConfig_t) + 1U) / 2U;
    for (uint32_t i = 0; i < n; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                              addr, src[i]) != HAL_OK) {
            Debug_Printf("[CFG] Write FAILED at 0x%08lX SR=0x%08lX\r\n",
                         addr, FLASH->SR);
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        addr += 2U;
    }
    HAL_FLASH_Lock();
    return HAL_OK;
}

/* ── Init ────────────────────────────────────────────────────────── */
void Config_Init(void)
{
    flash_read(CFG_PAGE_A, &s_tmp_a);
    flash_read(CFG_PAGE_B, &s_tmp_b);
    bool va = Config_Validate(&s_tmp_a);
    bool vb = Config_Validate(&s_tmp_b);

    if (va && vb)
        memcpy(&s_cfg,
               s_tmp_a.write_count >= s_tmp_b.write_count ? &s_tmp_a : &s_tmp_b,
               sizeof(s_cfg));
    else if (va)
        memcpy(&s_cfg, &s_tmp_a, sizeof(s_cfg));
    else if (vb)
        memcpy(&s_cfg, &s_tmp_b, sizeof(s_cfg));
    else {
        Debug_Print("[CFG] No valid config — loading defaults\r\n");
        Config_LoadDefaults();
        /* Skip save on first boot — flash pages may be outside 64KB on C8 */
        Debug_Print("[CFG] Defaults loaded (not saved to flash)\r\n");
        return;
    }
    /* Re-derive device_id if it was saved as 0 */
    if (s_cfg.device_id == 0) {
        uint32_t seed = HAL_GetTick() ^ *(volatile uint32_t *)(0xE0042000UL);
        seed ^= (seed >> 13); seed *= 0x5BD1E995UL; seed ^= (seed >> 15);
        s_cfg.device_id = (uint16_t)(seed & 0xFFFFU);
        if (s_cfg.device_id == 0) s_cfg.device_id = 0x0001;
        snprintf(s_cfg.device_name, 32, "Delta-GW-%04X", s_cfg.device_id);
        snprintf(s_cfg.mqtt_client_id, 32, "delta-gw-%04X", s_cfg.device_id);
        snprintf(s_cfg.mqtt_topic, 64, "delta/gw/%04X/data", s_cfg.device_id);
        snprintf(s_cfg.mqtt_alarm_topic, 64, "delta/gw/%04X/alarms", s_cfg.device_id);
        Debug_Printf("[CFG] Generated device_id=0x%04X (UID blank)\r\n", s_cfg.device_id);
    }
    Debug_Print("[CFG] Config loaded from flash\r\n");
}

/* ── Save (ping-pong) ────────────────────────────────────────────── */
HAL_StatusTypeDef Config_Save(void)
{
    flash_read(CFG_PAGE_A, &s_tmp_a);
    flash_read(CFG_PAGE_B, &s_tmp_b);
    bool va = Config_Validate(&s_tmp_a);
    bool vb = Config_Validate(&s_tmp_b);

    uint32_t target;
    if      (!va)  target = CFG_PAGE_A;
    else if (!vb)  target = CFG_PAGE_B;
    else           target = (s_tmp_a.write_count <= s_tmp_b.write_count)
                            ? CFG_PAGE_A : CFG_PAGE_B;

    uint32_t max_wc = 0;
    if (va) max_wc = s_tmp_a.write_count;
    if (vb && s_tmp_b.write_count > max_wc) max_wc = s_tmp_b.write_count;

    s_cfg.write_count = max_wc + 1U;
    s_cfg.crc32       = cfg_crc(&s_cfg);

    HAL_StatusTypeDef st = flash_write(target, &s_cfg);
    if (st == HAL_OK) Debug_Print("[CFG] Saved to flash OK\r\n");
    else              Debug_Print("[CFG] Flash write FAILED\r\n");
    return st;
}

const GatewayConfig_t *Config_Get(void)       { return &s_cfg; }
GatewayConfig_t       *Config_GetMutable(void){ return &s_cfg; }

/* ── Print ───────────────────────────────────────────────────────── */
void Config_Print(void)
{
    const GatewayConfig_t *c = &s_cfg;
    char buf[256];

    Debug_Print("\r\n╔══════════════════════════════════════╗\r\n");
    Debug_Print(  "║       GATEWAY CONFIGURATION          ║\r\n");
    Debug_Print(  "╚══════════════════════════════════════╝\r\n");
    snprintf(buf,sizeof(buf),"  device_id    : %u\r\n",c->device_id);            Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  device_name  : %s\r\n",c->device_name);          Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  fw_version   : 0x%08lX\r\n",c->fw_version);     Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  write_count  : %lu\r\n",c->write_count);         Debug_Print(buf);

    Debug_Print("\r\n── Modbus ──────────────────────────────\r\n");
    snprintf(buf,sizeof(buf),"  baud         : %lu\r\n",c->modbus_baud);         Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  parity       : %s\r\n",
             c->modbus_parity==0?"None":c->modbus_parity==1?"Even":"Odd");       Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  stop_bits    : %u\r\n",c->modbus_stop_bits);     Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  timeout_ms   : %u\r\n",c->modbus_timeout_ms);    Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  poll_ms      : %u\r\n",c->poll_interval_ms);     Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  num_regs     : %u\r\n",c->num_regs);             Debug_Print(buf);

    Debug_Print("\r\n── Registers ───────────────────────────\r\n");
    Debug_Print("  IDX  EN  ALM  SLAVE  ADDR    FC  TYPE   SCALE    TAG\r\n");
    Debug_Print("  ---  --  ---  -----  ------  --  -----  -------  ----------\r\n");
    static const char *tn[] = {"U16","I16","U32","I32","F32"};
    for (uint8_t i = 0; i < c->num_regs; i++) {
        const RegDef_t *r = &c->regs[i];
        snprintf(buf,sizeof(buf),"  %-3u  %-2u  %-3s  0x%02X   0x%04X  0x%02X %-5s  %-7.2f  %s\r\n",
                 i,r->enabled,r->is_alarm?"Y":"N",r->slave_addr,r->reg_addr,r->func_code,
                 r->data_type<=DTYPE_FLOAT32?tn[r->data_type]:"?",
                 r->scale,r->tag);
        Debug_Print(buf);
    }

    Debug_Print("\r\n── Telemetry ───────────────────────────\r\n");
    snprintf(buf,sizeof(buf),"  publish_ms   : %lu\r\n",c->publish_interval_ms); Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  delta_mode   : %s\r\n",c->delta_mode?"ON":"OFF");Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  delta_thresh : %u\r\n",c->delta_threshold);      Debug_Print(buf);

    Debug_Print("\r\n── GSM / MQTT ──────────────────────────\r\n");
    snprintf(buf,sizeof(buf),"  modem_type   : %s (%u)\r\n",
             c->modem_type==MODEM_QUECTEL_EC200U?"Quectel EC200U":"SIMCom A7670C",
             c->modem_type);                                                      Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  apn          : %s\r\n",c->apn);                  Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  broker       : %s:%u\r\n",c->broker,c->broker_port); Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  client_id    : %s\r\n",c->mqtt_client_id);       Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  user         : %s\r\n",c->mqtt_user);            Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  topic        : %s\r\n",c->mqtt_topic);           Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  alarm_topic  : %s\r\n",c->mqtt_alarm_topic);    Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  qos          : %u\r\n",c->mqtt_qos);             Debug_Print(buf);

    Debug_Print("\r\n── Alarm Bits ──────────────────────────\r\n");
    if (c->num_alarm_bits == 0) {
        Debug_Print("  (none defined)\r\n");
    } else {
        Debug_Print("  IDX  REG  BIT  NAME\r\n");
        Debug_Print("  ---  ---  ---  -------\r\n");
        for (uint8_t i = 0; i < c->num_alarm_bits; i++) {
            const AlarmBitDef_t *a = &c->alarm_bits[i];
            snprintf(buf, sizeof(buf), "  %-3u  %-3u  %-3u  %s\r\n",
                     i, a->reg_idx, a->bit_pos, a->name);
            Debug_Print(buf);
        }
    }
    snprintf(buf, sizeof(buf), "  Total: %u/%u\r\n", c->num_alarm_bits, CONFIG_MAX_ALARM_BITS);
    Debug_Print(buf);

    Debug_Print("\r\n── OTA ─────────────────────────────────\r\n");
    snprintf(buf,sizeof(buf),"  ota_enabled  : %s\r\n",c->ota_enabled?"YES":"NO"); Debug_Print(buf);
    snprintf(buf,sizeof(buf),"  manifest_url : %s\r\n",c->ota_manifest_url);     Debug_Print(buf);
    Debug_Print("════════════════════════════════════════\r\n\r\n");
}
