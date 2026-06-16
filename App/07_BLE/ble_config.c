/**
 * ble_config.c
 * =========================================================================
 * BLE Configuration Mode — GATT server on EC200U module.
 *
 * State machine: IDLE → INIT → ADVERTISING → CONNECTED → IDLE
 *
 * Uses AT+QBT* commands over shared USART2.  GSM SM must be paused
 * before entering config mode.
 *
 * GATT Service UUID 0x6159 with 7 characteristics:
 *   0 (0xAA01) R+W  broker_ip:port
 *   1 (0xAA02) R+W  mqtt_user|mqtt_pass
 *   2 (0xAA03) R+W  device_id|device_name
 *   3 (0xAA04) R+W  modbus_baud|parity|stopbits|timeout|poll_ms
 *   4 (0xAA05) R+W  apn
 *   5 (0xAA06) R     fw_version|rssi|uptime|num_regs (status)
 *   6 (0xAA07) W     command: "save", "reboot", "reset_defaults", "exit"
 * =========================================================================
 */
#include "ble_config.h"
#include "modem_at.h"
#include "config.h"
#include "display_ui.h"
#include "rgb_led.h"
#include "debug_cli.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* ── BLE state machine ──────────────────────────────────────────── */
typedef enum {
    BLE_IDLE = 0,
    BLE_INIT_PWR_OFF,
    BLE_INIT_PWR_ON,
    BLE_INIT_NAME,
    BLE_INIT_ADV_PARAMS,
    BLE_INIT_ADV_DATA,
    BLE_INIT_RSP_DATA,
    BLE_INIT_SERVICE,
    BLE_INIT_CHARS,
    BLE_INIT_FINISH,
    BLE_INIT_START_ADV,
    BLE_ADVERTISING,
    BLE_CONNECTED,
    BLE_EXIT_ADV_OFF,
    BLE_EXIT_PWR_OFF,
    BLE_DONE,
} BleState_t;

static BleState_t s_ble_state = BLE_IDLE;
static uint32_t   s_ble_tick  = 0;
static uint32_t   s_ble_deadline = 0;
static bool       s_ble_cmd_sent = false;
static uint8_t    s_char_idx = 0;       /* current characteristic being set up */
static bool       s_config_changed = false;
static bool       s_done = false;

/* Button state */
static uint32_t s_btn_press_start = 0;
static bool     s_btn_was_pressed = false;
static bool     s_btn_seen_released = false;  /* must see HIGH before accepting hold */

/* ── Hex conversion helpers ─────────────────────────────────────── */
static const char hex_lut[] = "0123456789ABCDEF";

/* ASCII string → hex string (e.g. "Hi" → "4869"). Returns length of hex string. */
static uint16_t str_to_hex(const char *src, char *hex_out, uint16_t hex_max)
{
    uint16_t i = 0;
    while (*src && (i + 2) < hex_max) {
        uint8_t c = (uint8_t)*src++;
        hex_out[i++] = hex_lut[c >> 4];
        hex_out[i++] = hex_lut[c & 0x0F];
    }
    hex_out[i] = '\0';
    return i;
}

/* Hex string → ASCII string (e.g. "4869" → "Hi"). Returns byte count. */
static uint16_t hex_to_str(const char *hex, uint16_t hex_len, char *out, uint16_t out_max)
{
    uint16_t j = 0;
    for (uint16_t i = 0; i + 1 < hex_len && j < out_max - 1; i += 2) {
        uint8_t hi = hex[i], lo = hex[i + 1];
        hi = (hi >= 'A') ? (hi >= 'a' ? hi - 'a' + 10 : hi - 'A' + 10) : hi - '0';
        lo = (lo >= 'A') ? (lo >= 'a' ? lo - 'a' + 10 : lo - 'A' + 10) : lo - '0';
        out[j++] = (char)((hi << 4) | lo);
    }
    out[j] = '\0';
    return j;
}

/* ── Transition helper ──────────────────────────────────────────── */
static void ble_go(BleState_t ns)
{
    s_ble_state    = ns;
    s_ble_tick     = HAL_GetTick();
    s_ble_cmd_sent = false;
}

/* ── Build characteristic initial value as hex string ───────────── */
static uint16_t build_char_value(uint8_t char_id, char *hex_out, uint16_t hex_max)
{
    const GatewayConfig_t *cfg = Config_Get();
    char tmp[96];

    switch (char_id) {
    case 0: /* broker:port */
        snprintf(tmp, sizeof(tmp), "%s:%u", cfg->broker, cfg->broker_port);
        break;
    case 1: /* mqtt_user|mqtt_pass */
        snprintf(tmp, sizeof(tmp), "%s|%s", cfg->mqtt_user, cfg->mqtt_pass);
        break;
    case 2: /* device_id|device_name */
        snprintf(tmp, sizeof(tmp), "%u|%s", cfg->device_id, cfg->device_name);
        break;
    case 3: /* modbus_baud|parity|stopbits|timeout|poll_ms */
        snprintf(tmp, sizeof(tmp), "%lu|%u|%u|%u|%u",
                 cfg->modbus_baud, cfg->modbus_parity, cfg->modbus_stop_bits,
                 cfg->modbus_timeout_ms, cfg->poll_interval_ms);
        break;
    case 4: /* apn */
        snprintf(tmp, sizeof(tmp), "%s", cfg->apn);
        break;
    case 5: /* status: fw_version|rssi|uptime|num_regs */
        snprintf(tmp, sizeof(tmp), "%lu|0|%lu|%u",
                 cfg->fw_version, HAL_GetTick() / 1000UL, cfg->num_regs);
        break;
    case 6: /* command channel — empty initially */
        tmp[0] = '\0';
        break;
    default:
        tmp[0] = '\0';
        break;
    }

    return str_to_hex(tmp, hex_out, hex_max);
}

/* ── Characteristic properties and UUIDs ────────────────────────── */
/* Properties: Read=2, WriteNoResp=4, Write=8, Notify=16, Indicate=32 */
/* Chars 0-4: R+W (2+8=10), Char 5: R+Notify (2+16=18), Char 6: W (8) */
static const uint8_t  char_props[]  = { 10, 10, 10, 10, 10, 18, 8 };
static const uint16_t char_uuids[]  = { 43521, 43522, 43523, 43524, 43525, 43526, 43527 };
/* 0xAA01=43521, 0xAA02=43522, ... 0xAA07=43527 */

/* Permission: 1=read, 2=write, 3=read+write */
static const uint8_t  char_perms[]  = { 3, 3, 3, 3, 3, 1, 2 };
static const uint16_t char_maxlen[] = { 80, 64, 48, 32, 32, 48, 16 };

#define NUM_CHARS  7

/* ── Button GPIO init ──────────────────────────────────────────── */
void BLE_Config_InitButton(void)
{
    /* PA0: input with internal pull-up, button to GND */
    GPIO_InitTypeDef g = {0};
    g.Pin  = BLE_BTN_PIN;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BLE_BTN_PORT, &g);

    s_btn_press_start = 0;
    s_btn_was_pressed = false;
    s_btn_seen_released = false;
}

/* ── Check if button held for 3 seconds ────────────────────────── *
 * IMPORTANT: We require seeing the button RELEASED (HIGH) at least
 * once before accepting any hold. This prevents a floating/unconnected
 * PA0 pin from falsely triggering config mode on boot.
 * ────────────────────────────────────────────────────────────────── */
bool BLE_Config_ButtonHeld(void)
{
    bool pressed = (HAL_GPIO_ReadPin(BLE_BTN_PORT, BLE_BTN_PIN) == GPIO_PIN_RESET);
    uint32_t now = HAL_GetTick();

    if (!pressed) {
        /* Button released — mark that we've seen a valid HIGH state */
        s_btn_seen_released = true;
        s_btn_was_pressed = false;
        return false;
    }

    /* Button pressed, but ignore until we've seen it released first */
    if (!s_btn_seen_released) return false;

    if (!s_btn_was_pressed) {
        s_btn_was_pressed = true;
        s_btn_press_start = now;
    } else if ((now - s_btn_press_start) >= 3000U) {
        s_btn_was_pressed = false;  /* reset so it doesn't re-trigger */
        return true;
    }
    return false;
}

/* ── Enter config mode ─────────────────────────────────────────── */
void BLE_Config_Enter(void)
{
    Debug_Print("[BLE] Entering config mode\r\n");
    s_done = false;
    s_config_changed = false;
    s_char_idx = 0;

    Display_ShowConfigMode();
    RGB_CYAN();

    ble_go(BLE_INIT_PWR_OFF);
}

/* ── Check if config mode finished ─────────────────────────────── */
bool BLE_Config_IsDone(void)
{
    return s_done;
}

/* ── Parse a write to a characteristic and update config ────────── */
static void handle_char_write(uint8_t char_id, const char *ascii, uint16_t len)
{
    GatewayConfig_t *cfg = Config_GetMutable();
    (void)len;

    Debug_Printf("[BLE] Char %u write: %s\r\n", char_id, ascii);

    switch (char_id) {
    case 0: { /* broker:port */
        char *colon = strrchr(ascii, ':');
        if (colon) {
            *colon = '\0';
            strncpy(cfg->broker, ascii, CONFIG_MAX_BROKER_LEN - 1);
            cfg->broker[CONFIG_MAX_BROKER_LEN - 1] = '\0';
            cfg->broker_port = (uint16_t)atoi(colon + 1);
        } else {
            strncpy(cfg->broker, ascii, CONFIG_MAX_BROKER_LEN - 1);
            cfg->broker[CONFIG_MAX_BROKER_LEN - 1] = '\0';
        }
        s_config_changed = true;
        break;
    }
    case 1: { /* mqtt_user|mqtt_pass */
        char *pipe = strchr(ascii, '|');
        if (pipe) {
            *pipe = '\0';
            strncpy(cfg->mqtt_user, ascii, CONFIG_MAX_USER_LEN - 1);
            cfg->mqtt_user[CONFIG_MAX_USER_LEN - 1] = '\0';
            strncpy(cfg->mqtt_pass, pipe + 1, CONFIG_MAX_PASS_LEN - 1);
            cfg->mqtt_pass[CONFIG_MAX_PASS_LEN - 1] = '\0';
        }
        s_config_changed = true;
        break;
    }
    case 2: { /* device_id|device_name */
        char *pipe = strchr(ascii, '|');
        if (pipe) {
            *pipe = '\0';
            cfg->device_id = (uint16_t)atoi(ascii);
            strncpy(cfg->device_name, pipe + 1, sizeof(cfg->device_name) - 1);
            cfg->device_name[sizeof(cfg->device_name) - 1] = '\0';
        }
        s_config_changed = true;
        break;
    }
    case 3: { /* modbus_baud|parity|stopbits|timeout|poll_ms */
        /* Parse pipe-separated values */
        char buf[48];
        strncpy(buf, ascii, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
        char *fields[5];
        uint8_t n = 0;
        char *p = buf;
        fields[n++] = p;
        while (*p && n < 5) {
            if (*p == '|') { *p = '\0'; fields[n++] = p + 1; }
            p++;
        }
        if (n >= 1) cfg->modbus_baud       = (uint32_t)atol(fields[0]);
        if (n >= 2) cfg->modbus_parity      = (uint8_t)atoi(fields[1]);
        if (n >= 3) cfg->modbus_stop_bits   = (uint8_t)atoi(fields[2]);
        if (n >= 4) cfg->modbus_timeout_ms  = (uint16_t)atoi(fields[3]);
        if (n >= 5) cfg->poll_interval_ms   = (uint16_t)atoi(fields[4]);
        s_config_changed = true;
        break;
    }
    case 4: /* apn */
        strncpy(cfg->apn, ascii, CONFIG_MAX_APN_LEN - 1);
        cfg->apn[CONFIG_MAX_APN_LEN - 1] = '\0';
        s_config_changed = true;
        break;
    case 5: /* status — read only, ignore writes */
        break;
    case 6: { /* command */
        if (strcmp(ascii, "save") == 0) {
            HAL_StatusTypeDef r = Config_Save();
            Debug_Printf("[BLE] Config save: %s\r\n", r == HAL_OK ? "OK" : "FAIL");
            s_config_changed = false;
        } else if (strcmp(ascii, "reboot") == 0) {
            Debug_Print("[BLE] Reboot requested\r\n");
            if (s_config_changed) Config_Save();
            AT_Cmd("AT+QBTADV=0", "OK", 1000);
            AT_Cmd("AT+QBTPWR=0", "OK", 1000);
            HAL_Delay(100);
            NVIC_SystemReset();
        } else if (strcmp(ascii, "reset_defaults") == 0) {
            Debug_Print("[BLE] Reset to defaults\r\n");
            Config_LoadDefaults();
            s_config_changed = true;
        } else if (strcmp(ascii, "exit") == 0) {
            Debug_Print("[BLE] Exit requested\r\n");
            ble_go(BLE_EXIT_ADV_OFF);
        }
        break;
    }
    }
}

/* ── Parse +QBTLEVALDATA URC ───────────────────────────────────── */
static void parse_valdata_urc(const char *line)
{
    /* Format: +QBTLEVALDATA: <cid>,"<mac>",<len>,"<hex_data>" */
    const char *p = strstr(line, "+QBTLEVALDATA:");
    if (!p) return;
    p += 14; /* skip "+QBTLEVALDATA:" */
    while (*p == ' ') p++;

    /* Parse cid (characteristic ID) */
    uint8_t cid = (uint8_t)atoi(p);

    /* Skip to hex data — find the last quoted string */
    const char *last_quote = NULL;
    const char *q = p;
    while (*q) {
        if (*q == '"') last_quote = q;
        q++;
    }
    if (!last_quote) return;

    /* Find the opening quote of the last string */
    const char *open_q = last_quote - 1;
    while (open_q > p && *open_q != '"') open_q--;
    if (*open_q != '"') return;
    open_q++; /* skip opening quote */

    uint16_t hex_len = (uint16_t)(last_quote - open_q);
    if (hex_len == 0 || hex_len > 512) return;

    /* Decode hex → ASCII */
    char ascii[128];
    hex_to_str(open_q, hex_len, ascii, sizeof(ascii));

    handle_char_write(cid, ascii, (uint16_t)strlen(ascii));
}

/* ── Process — called every main loop tick in config mode ───────── */
void BLE_Config_Process(void)
{
    uint32_t now = HAL_GetTick();
    char *cmd = at_work;

    /* Cyan LED blink while in config mode */
    static uint32_t led_t = 0;
    static uint8_t led_on = 1;
    if ((now - led_t) >= 500U) {
        led_t = now;
        led_on = !led_on;
        if (led_on) RGB_CYAN(); else RGB_OFF();
    }

    switch (s_ble_state) {

    case BLE_IDLE:
        break;

    /* ── Power off (clean state) ── */
    case BLE_INIT_PWR_OFF:
        if (!s_ble_cmd_sent) {
            AT_RxFlush();
            AT_BeginCmd("AT+QBTPWR=0");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0) {
            /* OK or timeout/error — either way proceed */
            ble_go(BLE_INIT_PWR_ON);
        }
        break;

    /* ── Power on BLE GATT Server ── */
    case BLE_INIT_PWR_ON:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTPWR=1");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 3000U;
        }
        if (AT_Poll("OK", s_ble_deadline) == 1) {
            Debug_Print("[BLE] BLE power ON\r\n");
            ble_go(BLE_INIT_NAME);
        } else if ((int32_t)(now - s_ble_deadline) >= 0) {
            Debug_Print("[BLE] BLE power ON failed\r\n");
            ble_go(BLE_EXIT_PWR_OFF);
        }
        break;

    /* ── Set device name ── */
    case BLE_INIT_NAME: {
        if (!s_ble_cmd_sent) {
            const GatewayConfig_t *cfg = Config_Get();
            snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QBTNAME=0,\"DeltaGW-%04X\"",
                     cfg->device_id);
            AT_BeginCmd(cmd);
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0)
            ble_go(BLE_INIT_ADV_PARAMS);
        break;
    }

    /* ── Advertising parameters ── */
    case BLE_INIT_ADV_PARAMS:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTGATADV=1,128,160,0,0,7,0");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0)
            ble_go(BLE_INIT_ADV_DATA);
        break;

    /* ── Advertising data (flags) ── */
    case BLE_INIT_ADV_DATA:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTADVDATA=3,\"020106\"");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0)
            ble_go(BLE_INIT_RSP_DATA);
        break;

    /* ── Scan response data (device name) ── */
    case BLE_INIT_RSP_DATA: {
        if (!s_ble_cmd_sent) {
            const GatewayConfig_t *cfg = Config_Get();
            char name[32];
            snprintf(name, sizeof(name), "DeltaGW-%04X", cfg->device_id);
            uint8_t name_len = (uint8_t)strlen(name);

            /* Build scan response: AD length, AD type 0x09 (Complete Local Name), name hex */
            char name_hex[64];
            str_to_hex(name, name_hex, sizeof(name_hex));

            uint8_t ad_len = name_len + 1;  /* name + type byte */
            uint8_t total_len = ad_len + 1;  /* + ad_len byte itself */

            snprintf(cmd, AT_WORK_BUF_SIZE,
                     "AT+QBTADVRSPDATA=%u,\"%02X09%s\"",
                     total_len, ad_len, name_hex);
            AT_BeginCmd(cmd);
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0)
            ble_go(BLE_INIT_SERVICE);
        break;
    }

    /* ── Create GATT service ── */
    case BLE_INIT_SERVICE:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTGATSS=0,1,24921,1"); /* 0x6159 = 24921 decimal */
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0) {
            s_char_idx = 0;
            ble_go(BLE_INIT_CHARS);
        }
        break;

    /* ── Add characteristics one by one ── */
    case BLE_INIT_CHARS: {
        if (s_char_idx >= NUM_CHARS) {
            ble_go(BLE_INIT_FINISH);
            break;
        }

        /* Each characteristic needs two commands: SC (add) then SCV (set value) */
        /* We use s_ble_cmd_sent to track sub-steps: false=send SC, true=send SCV */
        static uint8_t char_sub = 0;  /* 0=SC, 1=SCV */

        if (!s_ble_cmd_sent) {
            char_sub = 0;
            /* AT+QBTGATSC=<servID>,<charID>,<properties>,<UUID_type>,<UUID_16> */
            snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QBTGATSC=0,%u,%u,1,%u",
                     s_char_idx, char_props[s_char_idx], char_uuids[s_char_idx]);
            AT_BeginCmd(cmd);
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }

        int r = AT_Poll("OK", s_ble_deadline);
        if (r != 0) {
            if (char_sub == 0) {
                /* SC done, now send SCV with initial value */
                char hex_val[200];
                uint16_t hlen = build_char_value(s_char_idx, hex_val, sizeof(hex_val));
                if (hlen == 0) { hex_val[0] = '0'; hex_val[1] = '0'; hex_val[2] = '\0'; }

                snprintf(cmd, AT_WORK_BUF_SIZE,
                         "AT+QBTGATSCV=0,%u,%u,1,%u,%u,\"%s\"",
                         s_char_idx, char_perms[s_char_idx],
                         char_uuids[s_char_idx], char_maxlen[s_char_idx], hex_val);
                AT_BeginCmd(cmd);
                s_ble_deadline = now + 2000U;
                char_sub = 1;
            } else {
                /* SCV done — move to next characteristic */
                Debug_Printf("[BLE] Char %u registered\r\n", s_char_idx);
                s_char_idx++;
                s_ble_cmd_sent = false;
            }
        }
        break;
    }

    /* ── Finish service registration ── */
    case BLE_INIT_FINISH:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTGATSSC=1,1");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 3000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0)
            ble_go(BLE_INIT_START_ADV);
        break;

    /* ── Start advertising ── */
    case BLE_INIT_START_ADV:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTADV=1");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0) {
            Debug_Print("[BLE] Advertising started\r\n");
            ble_go(BLE_ADVERTISING);
            s_ble_deadline = now + BLE_ADV_TIMEOUT_MS;
        }
        break;

    /* ── Waiting for phone connection ── */
    case BLE_ADVERTISING: {
        /* Check for connection URC: +QBTGATSCON: <connID>,"<mac>" */
        int r = AT_Poll("+QBTGATSCON:", s_ble_deadline);
        if (r == 1) {
            Debug_Print("[BLE] Phone connected!\r\n");
            /* Stop advertising once connected */
            AT_Cmd("AT+QBTADV=0", "OK", 1000);
            ble_go(BLE_CONNECTED);
        } else if (r == -1 || (int32_t)(now - s_ble_deadline) >= 0) {
            Debug_Print("[BLE] Advertising timeout — exiting config mode\r\n");
            ble_go(BLE_EXIT_ADV_OFF);
        }
        break;
    }

    /* ── Connected — process writes and wait for disconnect ── */
    case BLE_CONNECTED: {
        /* Peek ring buffer for URCs */
        char peek_buf[512];
        uint16_t peek_len = AT_RingPeek(peek_buf, sizeof(peek_buf) - 1);
        if (peek_len > 0) {
            peek_buf[peek_len] = '\0';

            /* Check for data write: +QBTLEVALDATA: */
            char *vd = strstr(peek_buf, "+QBTLEVALDATA:");
            if (vd) {
                /* Find end of this URC line */
                char *eol = strchr(vd, '\n');
                if (eol) {
                    uint16_t consume = (uint16_t)(eol - peek_buf + 1);
                    parse_valdata_urc(vd);
                    AT_RingConsume(consume);
                    break;  /* process one URC per tick */
                }
            }

            /* Check for disconnect: +QBTGATSDCON: */
            char *dc = strstr(peek_buf, "+QBTGATSDCON:");
            if (dc) {
                char *eol = strchr(dc, '\n');
                uint16_t consume = eol ? (uint16_t)(eol - peek_buf + 1) : peek_len;
                AT_RingConsume(consume);
                Debug_Print("[BLE] Phone disconnected\r\n");
                ble_go(BLE_EXIT_ADV_OFF);
                break;
            }

            /* Check for read request: +QBTGATRDDATAIND: — update char value */
            char *rd = strstr(peek_buf, "+QBTGATRDDATAIND:");
            if (rd) {
                char *eol = strchr(rd, '\n');
                uint16_t consume = eol ? (uint16_t)(eol - peek_buf + 1) : peek_len;
                AT_RingConsume(consume);
                /* Update status characteristic (char 5) on any read */
                char hex_val[100];
                uint16_t hlen = build_char_value(5, hex_val, sizeof(hex_val));
                if (hlen > 0) {
                    snprintf(cmd, AT_WORK_BUF_SIZE,
                             "AT+QBTGATCHSCV=0,5,%u,\"%s\"",
                             hlen / 2, hex_val);
                    AT_Cmd(cmd, "OK", 1000);
                }
                break;
            }

            /* Consume any other complete lines to keep buffer flowing */
            char *any_eol = strchr(peek_buf, '\n');
            if (any_eol) {
                uint16_t consume = (uint16_t)(any_eol - peek_buf + 1);
                AT_RingConsume(consume);
            }
        }
        break;
    }

    /* ── Exit: stop advertising ── */
    case BLE_EXIT_ADV_OFF:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTADV=0");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0)
            ble_go(BLE_EXIT_PWR_OFF);
        break;

    /* ── Exit: power off BLE ── */
    case BLE_EXIT_PWR_OFF:
        if (!s_ble_cmd_sent) {
            AT_BeginCmd("AT+QBTPWR=0");
            s_ble_cmd_sent = true;
            s_ble_deadline = now + 2000U;
        }
        if (AT_Poll("OK", s_ble_deadline) != 0) {
            /* Save if changed */
            if (s_config_changed) {
                HAL_StatusTypeDef r = Config_Save();
                Debug_Printf("[BLE] Auto-save on exit: %s\r\n",
                             r == HAL_OK ? "OK" : "FAIL");
            }
            Display_ExitConfigMode();
            RGB_OFF();
            Debug_Print("[BLE] Config mode exited\r\n");
            ble_go(BLE_DONE);
            s_done = true;
        }
        break;

    case BLE_DONE:
        break;
    }
}
