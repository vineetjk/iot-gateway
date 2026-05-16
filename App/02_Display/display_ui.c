/**
 * display_ui.c
 * =========================================================================
 * Display state machine for SSD1306 128x32 OLED.
 * Boot screen → Normal (parameter cycling with scroll) → Alarm (flash).
 * =========================================================================
 */
#include "display_ui.h"
#include "ssd1306.h"
#include "config.h"
#include <string.h>
#include <stdio.h>

/* Forward declare large-string writer from ssd1306.c */
extern void SSD1306_WriteLargeString(int16_t x, uint8_t y, const char *s, SSD1306_Color col);

/* ── State ──────────────────────────────────────────────────────── */
static DispMode_t s_mode      = DISP_BOOT;
static uint8_t    s_cur_reg   = 0;       /* which register is displayed */
static uint32_t   s_rotate_t  = 0;       /* last rotation tick */
static int16_t    s_scroll_x  = 0;       /* horizontal scroll offset for tag */
static uint32_t   s_scroll_t  = 0;       /* scroll animation tick */
static uint8_t    s_alarm_flash = 0;     /* flash toggle */
static uint32_t   s_alarm_t   = 0;       /* alarm flash tick */
static bool       s_gsm_ok    = false;
static bool       s_mqtt_ok   = false;

/* Current displayed register data */
static char    s_tag[CONFIG_MAX_TAG_LEN + 1];
static char    s_value[16];              /* formatted value string */
static uint8_t s_num_regs  = 0;

/* Alarm state */
static bool    s_alarm_active = false;
static char    s_alarm_tag[CONFIG_MAX_TAG_LEN + 1];
static uint8_t s_alarm_idx = 0;

/* Boot screen state */
static uint8_t s_boot_line = 0;
static char    s_boot_items[6][12];      /* up to 6 boot items shown */
static uint8_t s_boot_ok[6];

/* Timing constants */
#define ROTATE_MS      3000U     /* cycle parameters every 3s */
#define SCROLL_MS      60U       /* scroll speed: 1px per 60ms */
#define SCROLL_PAUSE   1500U     /* pause at start before scrolling */
#define ALARM_FLASH_MS 500U      /* alarm flash period */

/* ── 16x16 alarm triangle bitmap ──────────────────────────────── */
static const uint8_t alarm_icon[] = {
    0x00,0x80, 0x00,0xC0, 0x00,0xE0, 0x00,0xB0,
    0x00,0x98, 0x00,0x8C, 0x00,0x86, 0x00,0x83,
    0x80,0x83, 0xC0,0x86, 0x60,0x8C, 0x30,0x98,
    0x18,0xB0, 0x0C,0xE0, 0xFE,0xFF, 0xFE,0xFF,
};

/* ── Boot Screen ────────────────────────────────────────────────── */
void Display_Init(void)
{
    s_mode = DISP_BOOT;
    s_boot_line = 0;
    memset(s_boot_items, 0, sizeof(s_boot_items));

    const GatewayConfig_t *cfg = Config_Get();

    SSD1306_Clear();
    /* Title bar */
    SSD1306_FillRect(0, 0, 128, 10, White);
    SSD1306_WriteStringAt(2, 1, "DELTA IoT GATEWAY", Font_6x8, Black);
    SSD1306_DrawHLine(0, 11, 128, White);

    /* Device ID */
    char id_str[22];
    snprintf(id_str, sizeof(id_str), "ID:0x%04X", cfg->device_id);
    SSD1306_WriteStringAt(4, 14, id_str, Font_6x8, White);
    SSD1306_WriteStringAt(4, 23, "Booting...", Font_6x8, White);
    SSD1306_UpdateScreen();
}

void Display_BootMsg(const char *label, bool ok)
{
    if (s_boot_line >= 6) return;
    strncpy(s_boot_items[s_boot_line], label, 11);
    s_boot_ok[s_boot_line] = ok ? 1 : 0;
    s_boot_line++;

    SSD1306_Clear();
    /* Title bar */
    SSD1306_FillRect(0, 0, 128, 10, White);
    SSD1306_WriteStringAt(2, 1, "DELTA IoT GATEWAY", Font_6x8, Black);
    SSD1306_DrawHLine(0, 11, 128, White);

    /* Show boot items in a compact row format */
    uint8_t x = 2, y = 14;
    for (uint8_t i = 0; i < s_boot_line; i++) {
        char item[20];
        snprintf(item, sizeof(item), "%s%s ",
                 s_boot_ok[i] ? "+" : "!", s_boot_items[i]);
        uint8_t len = strlen(item) * 6;
        if (x + len > 126) { x = 2; y += 9; }
        if (y > 24) break;
        SSD1306_WriteStringAt(x, y, item, Font_6x8, White);
        x += len;
    }
    SSD1306_UpdateScreen();
}

void Display_BootDone(void)
{
    s_mode = DISP_NORMAL;
    s_rotate_t = HAL_GetTick();
    s_scroll_x = 0;
    s_scroll_t = HAL_GetTick();
    s_cur_reg = 0;
}

/* ── Data Feed ──────────────────────────────────────────────────── */
void Display_SetRegValue(uint8_t idx, const char *tag, int32_t whole, uint32_t frac)
{
    (void)idx;
    strncpy(s_tag, tag, CONFIG_MAX_TAG_LEN);
    snprintf(s_value, sizeof(s_value), "%ld.%02lu", (long)whole, (unsigned long)frac);
}

void Display_SetGsmStatus(bool connected)  { s_gsm_ok = connected; }
void Display_SetMqttStatus(bool connected) { s_mqtt_ok = connected; }

/* ── Alarm ──────────────────────────────────────────────────────── */
void Display_SetAlarm(const char *alarm_tag, uint8_t alarm_idx)
{
    s_alarm_active = true;
    strncpy(s_alarm_tag, alarm_tag, CONFIG_MAX_TAG_LEN);
    s_alarm_idx = alarm_idx;
    s_alarm_flash = 0;
    s_alarm_t = HAL_GetTick();
    s_mode = DISP_ALARM;
}

void Display_ClearAlarm(void)
{
    s_alarm_active = false;
    s_mode = DISP_NORMAL;
    s_rotate_t = HAL_GetTick();
}

/* ── Draw status bar (bottom 6px) ───────────────────────────────── */
static void draw_status_bar(void)
{
    uint8_t y = 26;
    SSD1306_DrawHLine(0, y, 128, White);
    y += 1;

    /* GSM indicator */
    SSD1306_WriteStringAt(1, y, s_gsm_ok ? "4G" : "--", Font_6x8, White);

    /* MQTT dot */
    if (s_mqtt_ok) {
        SSD1306_DrawPixel(18, y + 2, White);
        SSD1306_DrawPixel(19, y + 2, White);
        SSD1306_DrawPixel(18, y + 3, White);
        SSD1306_DrawPixel(19, y + 3, White);
    }

    /* Register index */
    const GatewayConfig_t *cfg = Config_Get();
    uint8_t n_params = 0;
    for (uint8_t i = 0; i < cfg->num_regs; i++)
        if (!cfg->regs[i].is_alarm && cfg->regs[i].enabled) n_params++;

    if (n_params > 0) {
        char idx_str[12];
        snprintf(idx_str, sizeof(idx_str), "%u/%u", s_cur_reg + 1, n_params);
        uint8_t tw = strlen(idx_str) * 6;
        SSD1306_WriteStringAt(127 - tw, y, idx_str, Font_6x8, White);
    }
}

/* ── Normal Mode Render ──────────────────────────────────────────── */
static void render_normal(uint32_t now)
{
    const GatewayConfig_t *cfg = Config_Get();

    /* Count enabled non-alarm registers */
    uint8_t param_indices[CONFIG_MAX_REGS];
    uint8_t n_params = 0;
    for (uint8_t i = 0; i < cfg->num_regs; i++) {
        if (!cfg->regs[i].is_alarm && cfg->regs[i].enabled)
            param_indices[n_params++] = i;
    }

    if (n_params == 0) {
        /* No registers — show device name + connection status */
        SSD1306_Clear();
        SSD1306_FillRect(0, 0, 128, 10, White);
        char hdr[22];
        snprintf(hdr, sizeof(hdr), " %s", cfg->device_name);
        SSD1306_WriteStringAt(2, 1, hdr, Font_6x8, Black);
        SSD1306_DrawHLine(0, 11, 128, White);

        const char *status = s_mqtt_ok ? "Online" :
                             s_gsm_ok  ? "No MQTT" : "Offline";
        uint8_t sw = strlen(status) * 6;
        SSD1306_WriteStringAt((128 - sw) / 2, 15, status, Font_6x8, White);

        draw_status_bar();
        SSD1306_UpdateScreen();
        return;
    }

    /* Rotate to next register */
    if ((now - s_rotate_t) >= ROTATE_MS) {
        s_rotate_t = now;
        s_cur_reg = (s_cur_reg + 1) % n_params;
        s_scroll_x = 0;
        s_scroll_t = now;  /* start pause before scroll */
    }

    /* Get current register data */
    if (s_cur_reg >= n_params) s_cur_reg = 0;
    uint8_t ri = param_indices[s_cur_reg];
    const RegDef_t *r = &cfg->regs[ri];

    /* Get the extern modbus data */
    extern uint16_t modbus_data[];
    int32_t scaled = (int32_t)((float)modbus_data[ri] * r->scale * 100.0f);
    int32_t whole  = scaled / 100;
    uint32_t frac  = (uint32_t)((scaled < 0 ? -scaled : scaled) % 100);

    char val_str[16];
    snprintf(val_str, sizeof(val_str), "%ld.%02lu", (long)whole, (unsigned long)frac);

    /* Calculate tag text width for scrolling */
    uint8_t tag_w = strlen(r->tag) * 6;
    bool need_scroll = (tag_w > 124);

    /* Handle horizontal scroll with initial pause */
    if (need_scroll) {
        uint32_t elapsed = now - s_scroll_t;
        if (elapsed > SCROLL_PAUSE) {
            /* Scrolling active */
            uint32_t scroll_elapsed = elapsed - SCROLL_PAUSE;
            int16_t max_scroll = (int16_t)tag_w - 120;
            s_scroll_x = (int16_t)((scroll_elapsed / SCROLL_MS) % (max_scroll + 60));
            if (s_scroll_x > max_scroll + 30) {
                /* Reset: pause before next scroll cycle */
                s_scroll_x = 0;
                s_scroll_t = now;
            } else if (s_scroll_x > max_scroll) {
                s_scroll_x = max_scroll; /* hold at end briefly */
            }
        }
    } else {
        s_scroll_x = 0;
    }

    /* Render */
    SSD1306_Clear();

    /* Tag name row (y=0, height 8) with scroll */
    SSD1306_FillRect(0, 0, 128, 9, White);
    SSD1306_WriteStringAt(2 - s_scroll_x, 1, r->tag, Font_6x8, Black);

    /* Separator */
    SSD1306_DrawHLine(0, 10, 128, White);

    /* Value in large font (y=12, height 16) — centered */
    uint8_t val_w = strlen(val_str) * 12;
    int16_t val_x = (128 - val_w) / 2;
    if (val_x < 0) val_x = 0;
    SSD1306_WriteLargeString(val_x, 11, val_str, White);

    /* Status bar */
    draw_status_bar();

    SSD1306_UpdateScreen();
}

/* ── Alarm Mode Render ──────────────────────────────────────────── */
static void render_alarm(uint32_t now)
{
    /* Flash toggle */
    if ((now - s_alarm_t) >= ALARM_FLASH_MS) {
        s_alarm_t = now;
        s_alarm_flash = !s_alarm_flash;
    }

    SSD1306_Clear();

    if (s_alarm_flash) {
        /* Show alarm info */
        /* Draw alarm triangle at left */
        SSD1306_DrawBitmap(4, 8, 16, 16, alarm_icon, White);

        /* "ALARM" header */
        SSD1306_FillRect(0, 0, 128, 9, White);
        SSD1306_WriteStringAt(24, 1, "!! ALARM !!", Font_6x8, Black);

        /* Alarm name — centered */
        uint8_t nw = strlen(s_alarm_tag) * 6;
        int16_t nx = (128 - nw) / 2;
        if (nx < 24) nx = 24;
        SSD1306_WriteStringAt(nx, 15, s_alarm_tag, Font_6x8, White);
    } else {
        /* Inverted flash — mostly empty with border */
        SSD1306_DrawRect(0, 0, 128, 32, White);
        SSD1306_DrawRect(1, 1, 126, 30, White);
    }

    SSD1306_UpdateScreen();
}

/* ── Error code to short string ──────────────────────────────────── */
static const char *err_str(uint8_t code)
{
    switch (code) {
        case ERR_SIM_NOT_READY:  return "SIM FAIL";
        case ERR_NO_NETWORK:     return "NO NETWORK";
        case ERR_PDP_FAIL:       return "PDP FAIL";
        case ERR_MQTT_CONN:      return "MQTT CONN";
        case ERR_MQTT_SUB:       return "MQTT SUB";
        case ERR_MQTT_PUB:       return "MQTT PUB";
        case ERR_MQTT_DISC:      return "MQTT DISC";
        case ERR_GSM_NO_RESP:    return "GSM NO RSP";
        case ERR_GSM_TIMEOUT:    return "GSM TIMEOUT";
        case ERR_MODBUS_TIMEOUT: return "MB TIMEOUT";
        case ERR_MODBUS_CRC:     return "MB CRC";
        case ERR_FLASH_WRITE:    return "FLASH WR";
        case ERR_FLASH_READ:     return "FLASH RD";
        case ERR_OTA_DOWNLOAD:   return "OTA DL FAIL";
        case ERR_OTA_VERIFY:     return "OTA CRC";
        case ERR_OTA_NO_SERVER:  return "OTA SERVER";
        case ERR_CONFIG_SAVE:    return "CFG SAVE";
        default:                 return "UNKNOWN";
    }
}

/* ── Error state ────────────────────────────────────────────────── */
static uint8_t  s_err_code   = 0;
static uint32_t s_err_flash_t = 0;
static uint8_t  s_err_flash   = 0;

void Display_ShowError(uint8_t err_code)
{
    s_err_code = err_code;
    s_err_flash = 0;
    s_err_flash_t = HAL_GetTick();
    s_mode = DISP_ERROR;

    /* Draw immediately so caller sees it */
    SSD1306_Clear();
    SSD1306_FillRect(0, 0, 128, 10, White);
    SSD1306_WriteStringAt(2, 1, "!! ERROR !!", Font_6x8, Black);
    char code_str[12];
    snprintf(code_str, sizeof(code_str), "E:0x%02X", err_code);
    SSD1306_WriteStringAt(80, 1, code_str, Font_6x8, Black);
    SSD1306_DrawHLine(0, 11, 128, White);
    const char *msg = err_str(err_code);
    uint8_t mw = strlen(msg) * 6;
    SSD1306_WriteStringAt((128 - mw) / 2, 16, msg, Font_6x8, White);
    SSD1306_UpdateScreen();
}

void Display_ClearError(void)
{
    if (s_mode == DISP_ERROR) {
        s_err_code = 0;
        s_mode = DISP_NORMAL;
        s_rotate_t = HAL_GetTick();
    }
}

static void render_error(uint32_t now)
{
    if ((now - s_err_flash_t) >= 800U) {
        s_err_flash_t = now;
        s_err_flash = !s_err_flash;
    }

    SSD1306_Clear();

    if (s_err_flash) {
        SSD1306_FillRect(0, 0, 128, 10, White);
        SSD1306_WriteStringAt(2, 1, "!! ERROR !!", Font_6x8, Black);
        char code_str[12];
        snprintf(code_str, sizeof(code_str), "E:0x%02X", s_err_code);
        SSD1306_WriteStringAt(80, 1, code_str, Font_6x8, Black);
    } else {
        SSD1306_DrawRect(0, 0, 128, 10, White);
    }

    SSD1306_DrawHLine(0, 11, 128, White);
    const char *msg = err_str(s_err_code);
    uint8_t mw = strlen(msg) * 6;
    SSD1306_WriteStringAt((128 - mw) / 2, 16, msg, Font_6x8, White);
    SSD1306_UpdateScreen();
}

/* ── Updating screen ───────────────────────────────────────────── */
static uint32_t s_upd_done  = 0;
static uint32_t s_upd_total = 0;

void Display_ShowUpdating(uint32_t done, uint32_t total)
{
    s_upd_done  = done;
    s_upd_total = total;
    s_mode = DISP_UPDATING;

    SSD1306_Clear();
    SSD1306_FillRect(0, 0, 128, 10, White);
    SSD1306_WriteStringAt(2, 1, "FW UPDATE", Font_6x8, Black);
    SSD1306_DrawHLine(0, 11, 128, White);

    /* Progress text */
    char pct[16];
    uint8_t p = total ? (uint8_t)((done * 100UL) / total) : 0;
    snprintf(pct, sizeof(pct), "%u%%", p);
    uint8_t pw = strlen(pct) * 6;
    SSD1306_WriteStringAt((128 - pw) / 2, 13, pct, Font_6x8, White);

    /* Progress bar: y=23, height=6, 2px border */
    SSD1306_DrawRect(4, 23, 120, 7, White);
    if (total) {
        uint16_t fill = (uint16_t)((done * 116UL) / total);
        if (fill > 116) fill = 116;
        if (fill > 0) SSD1306_FillRect(6, 25, fill, 3, White);
    }

    SSD1306_UpdateScreen();
}

/* ── Main Update (call every loop iteration) ──────────────────── */
void Display_Update(uint32_t now)
{
    switch (s_mode) {
        case DISP_BOOT:
            /* Boot screen is driven by Display_BootMsg calls */
            break;
        case DISP_NORMAL:
            render_normal(now);
            break;
        case DISP_ALARM:
            render_alarm(now);
            break;
        case DISP_ERROR:
            render_error(now);
            break;
        case DISP_UPDATING:
            /* Updating screen is driven by Display_ShowUpdating calls */
            break;
    }
}
