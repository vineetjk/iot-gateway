/**
 * main.c — Application (FLASH ORIGIN=0x08003000, bootloader at 0x08000000)
 * =========================================================================
 * STM32F103 IoT Gateway — complete integration.
 *
 * Peripheral map:
 *   USART1  PA9/PA10  Debug CLI (CH340 USB-C on board)
 *   USART2  PA2/PA3   A7670C GSM
 *   USART3  PB10/PB11 Modbus RTU RS-485  (PA4=DE/RE)
 *   I2C1    PB6/PB7   SSD1306 OLED 0.91"
 *   SPI1    PA5/PA6/PA7/PA8  W25Q64 (SCK/MISO/MOSI/CS)
 *   PB0     LED R  (270Ω to LED R-pin)
 *   PB1     LED G  (270Ω to LED G-pin)
 *   PB3     LED B  (270Ω to LED B-pin)
 *   PB4     A7670C RESET (active LOW pulse)
 *
 * Linker script: FLASH ORIGIN=0x08003000 LENGTH=108K
 * Add to SystemInit(): SCB->VTOR = 0x08003000;
 * =========================================================================
 */

#include "main.h"
#include "config.h"
#include "debug_cli.h"
#include "modbus.h"
#include "modem_hal.h"
#include "gsm_sm.h"
#include "ssd1306.h"
#include "display_ui.h"
#include "rgb_led.h"
#include "w25q_spi.h"
#include "ota_manager.h"
#include "telemetry.h"
#include "bootloader.h"
#include "ble_config.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

/* ── Peripheral handles ──────────────────────────────────────────── */
UART_HandleTypeDef huart1;   /* Debug CLI (CH340 USB-C) */
UART_HandleTypeDef huart2;   /* GSM     */
UART_HandleTypeDef huart3;   /* Modbus RS-485           */
I2C_HandleTypeDef  hi2c1;    /* OLED    */
SPI_HandleTypeDef  hspi1;    /* W25Q64  */

/* ── Application state ───────────────────────────────────────────── */
typedef enum {
    STATE_INIT = 0,
    STATE_IDLE,
    STATE_READING_MODBUS,
    STATE_SENDING_GSM,
    STATE_ERROR_MODBUS,
    STATE_ERROR_GSM,
    STATE_NO_NETWORK,
} SystemState_t;

static SystemState_t sysState = STATE_INIT;
uint16_t             modbus_data[CONFIG_MAX_REGS];
static uint16_t      last_data[CONFIG_MAX_REGS];
static uint16_t      last_alarm_data[CONFIG_MAX_REGS];
static bool          spi_flash_ok = false;

/* OTA check every 6 hours */
#define OTA_CHECK_INTERVAL_MS  (6UL * 3600UL * 1000UL)

static int ci_eq(const char *a, const char *b)
{
    while (*a && *b)
        if (tolower((unsigned char)*a++) != tolower((unsigned char)*b++)) return 0;
    return *a == *b;
}

/* ── CSV tokeniser: split "a,b,c" into arg[] array, return count ── */
#define CMD_MAX_ARGS 8
#define CMD_ARG_LEN  32
static char     s_args[CMD_MAX_ARGS][CMD_ARG_LEN];
static uint8_t  csv_split(const char *s)
{
    uint8_t n = 0;
    while (*s && n < CMD_MAX_ARGS) {
        uint8_t i = 0;
        while (*s && *s != ',' && i < CMD_ARG_LEN - 1) s_args[n][i++] = *s++;
        s_args[n][i] = '\0';
        while (*s && *s != ',') s++;   /* skip remainder of long fields */
        n++;
        if (*s == ',') s++;
    }
    return n;
}

/* ── MQTT command dispatcher ─────────────────────────────────────
 * CSV format:  cmd,arg1,arg2,...
 *   reg_add,<slave>,<addr>,<fc>,<type>,<scale>,<tag>[,<alarm>]
 *   reg_del,<index>
 *   set,<key>,<value>
 *   save
 *   reboot
 *   ota,<url>,<size>,<crc32>,<version>
 * ────────────────────────────────────────────────────────────────── */
static void App_HandleMqttCmd(const char *payload)
{
    memset(s_args, 0, sizeof(s_args));
    uint8_t argc = csv_split(payload);
    if (argc == 0) { Debug_Print("[CMD] empty\r\n"); return; }

    const char *cmd = s_args[0];
    GatewayConfig_t *c = Config_GetMutable();

    if (ci_eq(cmd, "ota")) {
        /* ota,<url>,<size>,<crc32>[,<version>]
         * URL can be >32 chars, extract from raw payload */
        if (!spi_flash_ok) { Debug_Print("[CMD] No SPI flash\r\n"); return; }
        if (argc < 4) { Debug_Print("[CMD] ota,url,size,crc32[,ver]\r\n"); return; }
        static char ota_url[128];
        const char *u = payload + 4; /* skip "ota," */
        uint8_t i = 0;
        while (*u && *u != ',' && i < 127) ota_url[i++] = *u++;
        ota_url[i] = '\0';
        OTA_HandleMqttCsv(ota_url, s_args[2], s_args[3],
                           argc >= 5 ? s_args[4] : "");

    } else if (ci_eq(cmd, "reg_add")) {
        /* reg_add,slave,addr,fc,type,scale,tag[,alarm] */
        if (argc < 7) { Debug_Print("[CMD] reg_add,slave,addr,fc,type,scale,tag[,alarm]\r\n"); return; }
        if (c->num_regs >= CONFIG_MAX_REGS) {
            Debug_Printf("[CMD] reg table full (%u)\r\n", CONFIG_MAX_REGS);
            return;
        }
        uint8_t fv = (uint8_t)atoi(s_args[3]);
        if (fv != 3 && fv != 4) { Debug_Print("[CMD] fc must be 3 or 4\r\n"); return; }

        RegDataType_t dt;
        if      (ci_eq(s_args[4],"u16")) dt = DTYPE_UINT16;
        else if (ci_eq(s_args[4],"i16")) dt = DTYPE_INT16;
        else if (ci_eq(s_args[4],"u32")) dt = DTYPE_UINT32;
        else if (ci_eq(s_args[4],"i32")) dt = DTYPE_INT32;
        else if (ci_eq(s_args[4],"f32")) dt = DTYPE_FLOAT32;
        else { Debug_Print("[CMD] type: u16 i16 u32 i32 f32\r\n"); return; }

        RegDef_t *r = &c->regs[c->num_regs];
        r->slave_addr = (uint8_t)strtol(s_args[1], NULL, 0);
        r->reg_addr   = (uint16_t)strtol(s_args[2], NULL, 0);
        r->func_code  = fv;
        r->data_type  = dt;
        r->scale      = strtof(s_args[5], NULL);
        strncpy(r->tag, s_args[6], CONFIG_MAX_TAG_LEN - 1);
        r->enabled    = 1;
        r->is_alarm   = (argc >= 8 && s_args[7][0] == '1') ? 1 : 0;
        c->num_regs++;
        Debug_Printf("[CMD] reg[%u] added: 0x%02X:0x%04X fc=%u %s scale=%s tag=%s\r\n",
            c->num_regs - 1, r->slave_addr, r->reg_addr, fv, s_args[4], s_args[5], r->tag);

    } else if (ci_eq(cmd, "reg_del")) {
        /* reg_del,index */
        if (argc < 2) { Debug_Print("[CMD] reg_del,<index>\r\n"); return; }
        uint8_t idx = (uint8_t)atoi(s_args[1]);
        if (idx >= c->num_regs) { Debug_Printf("[CMD] idx %u out of range\r\n", idx); return; }
        for (uint8_t i = idx; i < c->num_regs - 1; i++) c->regs[i] = c->regs[i + 1];
        memset(&c->regs[--c->num_regs], 0, sizeof(RegDef_t));
        Debug_Printf("[CMD] reg[%u] deleted. Total=%u\r\n", idx, c->num_regs);

    } else if (ci_eq(cmd, "set")) {
        /* set,key,value */
        if (argc < 3) { Debug_Print("[CMD] set,<key>,<value>\r\n"); return; }
        const char *key = s_args[1], *val = s_args[2];
        if      (ci_eq(key,"poll_ms"))        c->poll_interval_ms    = (uint16_t)atoi(val);
        else if (ci_eq(key,"pub_ms"))         c->publish_interval_ms = (uint32_t)atoi(val);
        else if (ci_eq(key,"delta_mode"))     c->delta_mode          = atoi(val) ? 1 : 0;
        else if (ci_eq(key,"delta_thresh"))   c->delta_threshold     = (uint16_t)atoi(val);
        else if (ci_eq(key,"modem_type"))     c->modem_type          = (uint8_t)atoi(val);
        else if (ci_eq(key,"apn"))            strncpy(c->apn, val, CONFIG_MAX_APN_LEN - 1);
        else if (ci_eq(key,"broker"))         strncpy(c->broker, val, CONFIG_MAX_BROKER_LEN - 1);
        else if (ci_eq(key,"broker_port"))    c->broker_port         = (uint16_t)atoi(val);
        else if (ci_eq(key,"mqtt_user"))      strncpy(c->mqtt_user, val, CONFIG_MAX_USER_LEN - 1);
        else if (ci_eq(key,"mqtt_pass"))      strncpy(c->mqtt_pass, val, CONFIG_MAX_PASS_LEN - 1);
        else if (ci_eq(key,"topic"))          strncpy(c->mqtt_topic, val, CONFIG_MAX_TOPIC_LEN - 1);
        else if (ci_eq(key,"alarm_topic"))    strncpy(c->mqtt_alarm_topic, val, CONFIG_MAX_TOPIC_LEN - 1);
        else if (ci_eq(key,"mqtt_qos"))       { uint8_t v=(uint8_t)atoi(val); if(v<=2) c->mqtt_qos=v; }
        else if (ci_eq(key,"ota_enabled"))    c->ota_enabled         = atoi(val) ? 1 : 0;
        else if (ci_eq(key,"ota_url"))        strncpy(c->ota_manifest_url, val, 127);
        else { Debug_Printf("[CMD] unknown key: %s\r\n", key); return; }
        Debug_Printf("[CMD] set %s=%s\r\n", key, val);

    } else if (ci_eq(cmd, "save")) {
        HAL_StatusTypeDef s = Config_Save();
        Debug_Printf("[CMD] Config save: %s\r\n", s == HAL_OK ? "OK" : "FAIL");
        if (s != HAL_OK) Display_ShowError(ERR_CONFIG_SAVE);

    } else if (ci_eq(cmd, "reboot")) {
        Debug_Print("[CMD] Rebooting...\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();

    } else {
        Debug_Printf("[CMD] Unknown: %s\r\n", cmd);
    }
}

/* ── Forward declarations ────────────────────────────────────────── */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_Init(void);
static void MX_USART2_Init(void);
static void MX_USART3_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void        App_UpdateLED(void);
static void        App_PollModbus(void);
static void        App_PublishData(void);
static void        App_PublishAlarms(void);
static GsmResult_t App_MqttPublish(const char *topic, const uint8_t *payload,
                                    uint16_t len, uint8_t qos);

/* ════════════════════════════════════════════════════════════════════
   MAIN
   ════════════════════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();

    /* Built-in LED on PC13 (active LOW) — heartbeat indicator */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef led = {.Pin=GPIO_PIN_13, .Mode=GPIO_MODE_OUTPUT_PP, .Speed=GPIO_SPEED_FREQ_LOW};
    HAL_GPIO_Init(GPIOC, &led);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* LED ON = code reached here */

    MX_USART1_Init();
    MX_USART2_Init();
    MX_USART3_Init();
    MX_I2C1_Init();
    MX_SPI1_Init();

    /* ── 1. CLI first so debug prints work ── */
    CLI_Init(&huart1);
    Debug_Print("[BOOT] UART OK\r\n");

    /* ── 2. Config from flash ── */
    Debug_Print("[BOOT] Config init...\r\n");
    Config_Init();
    const GatewayConfig_t *cfg = Config_Get();
    Debug_Print("[BOOT] Config OK\r\n");

    /* ── 3. Peripheral drivers ── */
    RGB_Init();
    Debug_Print("[BOOT] RGB OK\r\n");

    /* R-G-B boot sequence */
    RGB_RED();   HAL_Delay(200);
    RGB_GREEN(); HAL_Delay(200);
    RGB_BLUE();  HAL_Delay(200);
    RGB_OFF();

    Debug_Print("[BOOT] I2C OLED init...\r\n");
    SSD1306_Init(&hi2c1);
    Display_Init();
    Display_BootMsg("UART", true);
    Display_BootMsg("CFG", true);
    Display_BootMsg("RGB", true);
    Debug_Print("[BOOT] OLED OK\r\n");

    /* ── 4. W25Q64 + OTA (skip if flash absent) ── */
    OTA_Init(&hspi1, GPIOA, GPIO_PIN_8);
    spi_flash_ok = W25Q_Detect();
    Display_BootMsg("SPI", spi_flash_ok);
    if (spi_flash_ok) Telem_Init();
    else Debug_Print("[APP] W25Q64 not found — OTA/logging disabled\r\n");

    /* ── 5. Modbus ── */
    Modbus_Init(&huart3, GPIO_PIN_4, GPIOA);
    Display_BootMsg("MB", true);

    /* ── 5b. BLE config button (PA0) ── */
    BLE_Config_InitButton();

    /* ── 6. GSM — kick non-blocking state machine ── */
    Debug_Print("[APP] Starting GSM state machine...\r\n");
    Modem_Init(&huart2, GPIOB, GPIO_PIN_4);
    GSM_SM_Kick();
    sysState = STATE_NO_NETWORK;
    Display_BootMsg("GSM", true);

    /* ── 7. OTA first-boot confirmation — deferred to main loop ── */
    /* Checked once GSM_SM_IsConnected() returns true (see main loop) */

    HAL_Delay(1500);  /* show boot screen briefly */
    Display_BootDone();
    App_UpdateLED();

    Debug_Printf("[APP] %s (ID=0x%04X) started. %u registers.\r\n",
                 cfg->device_name, cfg->device_id, cfg->num_regs);
    Debug_Printf("[APP] MCU UID → 0x%04X  Topics: %s | %s\r\n",
                 Config_GetMcuId(), cfg->mqtt_topic, cfg->mqtt_alarm_topic);

    static bool startup_published  = false;
    static bool ota_boot_confirmed = false;
    static bool config_mode        = false;

    /* ── Main loop ── */
    uint32_t last_poll    = 0;
    uint32_t last_publish = 0;
    uint32_t last_ota     = HAL_GetTick();

    memset(last_data, 0, sizeof(last_data));
    memset(last_alarm_data, 0, sizeof(last_alarm_data));

    while (1)
    {
        uint32_t now = HAL_GetTick();

        CLI_Process();   /* non-blocking, always first */

        /* ── BLE Config Mode ── */
        if (config_mode) {
            BLE_Config_Process();
            if (BLE_Config_IsDone()) {
                config_mode = false;
                startup_published = false;  /* re-publish after reconnect */
                GSM_SM_Resume();
            }

            /* Display + heartbeat still run in config mode */
            static uint32_t disp_tick_cfg = 0;
            if ((now - disp_tick_cfg) >= 50) {
                disp_tick_cfg = now;
                Display_Update(now);
            }
            static uint32_t led_tick_cfg = 0;
            if ((now - led_tick_cfg) >= 500) {
                led_tick_cfg = now;
                HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
            }
            HAL_Delay(5);
            continue;  /* skip normal operation */
        }

        /* ── Check config button (3s hold) — EC200U only (has BLE) ── */
        if (cfg->modem_type == MODEM_QUECTEL_EC200U && BLE_Config_ButtonHeld()) {
            config_mode = true;
            GSM_SM_Pause();
            BLE_Config_Enter();
            continue;
        }

        /* ── GSM state machine (non-blocking) ── */
        Modem_MqttCheckURC();  /* async disconnect detection (modem-specific) */
        GsmSmStatus_t gsm_status = GSM_SM_Process();

        /* Update display state from SM (don't override transient modbus/publish states) */
        if (sysState == STATE_IDLE || sysState == STATE_NO_NETWORK ||
            sysState == STATE_ERROR_GSM) {
            SystemState_t prev = sysState;
            switch (gsm_status) {
                case GSM_SM_CONNECTED:  sysState = STATE_IDLE;       break;
                case GSM_SM_CONNECTING: sysState = STATE_NO_NETWORK; break;
                case GSM_SM_ERROR:      sysState = STATE_ERROR_GSM;  break;
                default: break;
            }
            /* Show/clear error on display at transitions */
            if (sysState == STATE_ERROR_GSM && prev != STATE_ERROR_GSM)
                Display_ShowError(ERR_GSM_TIMEOUT);
            else if (sysState != STATE_ERROR_GSM && prev == STATE_ERROR_GSM)
                Display_ClearError();
        }

        /* ── One-shot checks once GSM first connects ── */
        if (GSM_SM_IsConnected()) {
            /* OTA first-boot confirmation */
            if (!ota_boot_confirmed && spi_flash_ok && BL_IsFirstBootAfterOTA()) {
                BL_ConfirmBoot();
                Debug_Print("[APP] OTA boot confirmed\r\n");
                ota_boot_confirmed = true;
            }

            /* Startup "online" publish */
            if (!startup_published) {
                const char hello[] = "online";
                if (App_MqttPublish(cfg->mqtt_topic, (const uint8_t*)hello,
                                    sizeof(hello)-1, cfg->mqtt_qos) == GSM_OK) {
                    Debug_Print("[APP] Startup message published\r\n");
                    startup_published = true;
                } else {
                    GSM_SM_SetDisconnected();
                }
            }
        }

        /* ── Update display status ── */
        Display_SetGsmStatus(GSM_SM_IsConnected());
        Display_SetMqttStatus(GSM_SM_IsConnected() && startup_published);

        /* ── Modbus poll ── */
        if ((now - last_poll) >= cfg->poll_interval_ms) {
            last_poll = now;
            App_PollModbus();

            /* Check for alarm changes — publish immediately if changed */
            if (GSM_SM_IsConnected() && startup_published)
                App_PublishAlarms();

            App_UpdateLED();
        }

        /* ── Check incoming MQTT commands (BEFORE publish to avoid race) ── */
        /* Process any buffered RX even if connection just dropped — the
         * command was already received and sitting in the ring buffer. */
        {
            static char rx_topic[48], rx_payload[256];
            if (Modem_MqttCheckRx(rx_topic, sizeof(rx_topic),
                                  rx_payload, sizeof(rx_payload))) {
                Debug_Printf("[APP] MQTT RX: %s → %s\r\n", rx_topic, rx_payload);
                App_HandleMqttCmd(rx_payload);
            }
        }

        /* ── Telemetry data publish ── */
        if ((now - last_publish) >= cfg->publish_interval_ms) {
            last_publish = now;

            if (GSM_SM_IsConnected() && startup_published) {
                App_PublishData();

                /* Flush offline log if SPI flash present */
                if (spi_flash_ok) {
                    uint32_t pending = SpiLog_Pending();
                    if (pending > 0) {
                        Debug_Printf("[APP] Flushing %lu offline frames\r\n", pending);
                        uint8_t  fbuf[TELEM_FRAME_SIZE]; uint16_t flen = 0;
                        while (SpiLog_ReadNext(fbuf, &flen)) {
                            if (App_MqttPublish(cfg->mqtt_topic, fbuf, flen,
                                                cfg->mqtt_qos) != GSM_OK) {
                                GSM_SM_SetDisconnected();
                                break;
                            }
                            SpiLog_ConfirmRead();
                        }
                    }
                }

                App_UpdateLED();
            }
        }

        /* ── OTA check (periodic manifest poll) ── */
        if (spi_flash_ok && cfg->ota_enabled &&
            (now - last_ota) >= OTA_CHECK_INTERVAL_MS &&
            GSM_SM_IsConnected())
        {
            last_ota = now;
            Debug_Print("[APP] Checking OTA...\r\n");
            OtaResult_t r = OTA_CheckAndUpdate();
            if (r == OTA_RESULT_UP_TO_DATE) Debug_Print("[APP] FW up to date\r\n");
            else if (r != OTA_RESULT_UPDATED) Debug_Printf("[APP] OTA result=%d\r\n", (int)r);
        }

        /* ── RGB LED state update (every loop) ── */
        App_UpdateLED();

        /* Display update — drives animations, scrolling, rotation */
        static uint32_t disp_tick = 0;
        if ((now - disp_tick) >= 50) {  /* ~20 FPS */
            disp_tick = now;
            Display_Update(now);
        }

        /* Heartbeat blink PC13 every 500ms */
        static uint32_t led_tick = 0;
        if ((now - led_tick) >= 500) {
            led_tick = now;
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        }

        HAL_Delay(5);
    }
}

/* ════════════════════════════════════════════════════════════════════
   APP FUNCTIONS
   ════════════════════════════════════════════════════════════════════ */
static void App_PollModbus(void)
{
    const GatewayConfig_t *cfg = Config_Get();
    sysState = STATE_READING_MODBUS;

    for (uint8_t i = 0; i < cfg->num_regs; i++) {
        const RegDef_t *r = &cfg->regs[i];
        if (!r->enabled) continue;
        uint8_t qty = (r->data_type >= DTYPE_UINT32) ? 2 : 1;
        ModbusResult_t res = (r->func_code == 0x04)
            ? Modbus_ReadInputRegisters(r->slave_addr, r->reg_addr, qty,
                                        &modbus_data[i], cfg->modbus_timeout_ms)
            : Modbus_ReadHoldingRegisters(r->slave_addr, r->reg_addr, qty,
                                          &modbus_data[i], cfg->modbus_timeout_ms);
        if (res != MODBUS_OK) {
            Debug_Printf("[MB] reg[%u] error=%d\r\n", i, (int)res);
        }
    }
    sysState = STATE_IDLE;
}

/* ── Publish data (non-alarm) registers on timer ── */
static void App_PublishData(void)
{
    const GatewayConfig_t *cfg = Config_Get();
    sysState = STATE_SENDING_GSM;

    static uint32_t ts_base = 1700000000UL;
    uint32_t ts = ts_base + HAL_GetTick() / 1000UL;
    static uint16_t seq = 0;

    static char json[512];
    int pos = 0;

    /* In delta_mode: only publish if at least one data register changed */
    if (cfg->delta_mode && cfg->num_regs > 0) {
        bool any_changed = false;
        for (uint8_t i = 0; i < cfg->num_regs; i++) {
            if (cfg->regs[i].is_alarm) continue;
            uint16_t diff = (modbus_data[i] > last_data[i])
                          ? (modbus_data[i] - last_data[i])
                          : (last_data[i] - modbus_data[i]);
            if (diff >= cfg->delta_threshold) { any_changed = true; break; }
        }
        if (!any_changed) {
            Debug_Print("[APP] No data changes — skipping publish\r\n");
            sysState = STATE_IDLE;
            return;
        }
    }

    /* Header */
    pos += snprintf(json + pos, sizeof(json) - pos,
                    "{\"id\":%u,\"name\":\"%s\",\"ts\":%lu,\"seq\":%u",
                    cfg->device_id, cfg->device_name, ts, seq++);

    /* Only non-alarm register values */
    for (uint8_t i = 0; i < cfg->num_regs && pos < (int)sizeof(json) - 40; i++) {
        const RegDef_t *r = &cfg->regs[i];
        if (!r->enabled || r->is_alarm) continue;

        int32_t scaled = (int32_t)((float)modbus_data[i] * r->scale * 100.0f);
        int32_t whole  = scaled / 100;
        uint32_t frac  = (uint32_t)((scaled < 0 ? -scaled : scaled) % 100);

        if (r->tag[0])
            pos += snprintf(json + pos, sizeof(json) - pos,
                            ",\"%s\":%ld.%02lu", r->tag, (long)whole, (unsigned long)frac);
        else
            pos += snprintf(json + pos, sizeof(json) - pos,
                            ",\"r%u\":%ld.%02lu", i, (long)whole, (unsigned long)frac);
    }

    /* If no registers configured, add uptime */
    if (cfg->num_regs == 0) {
        pos += snprintf(json + pos, sizeof(json) - pos,
                        ",\"uptime_s\":%lu", HAL_GetTick() / 1000UL);
    }

    pos += snprintf(json + pos, sizeof(json) - pos, "}");

    GsmResult_t res = App_MqttPublish(cfg->mqtt_topic,
                                       (const uint8_t *)json, (uint16_t)pos,
                                       cfg->mqtt_qos);
    if (res == GSM_OK) {
        for (uint8_t i = 0; i < cfg->num_regs; i++)
            if (!cfg->regs[i].is_alarm) last_data[i] = modbus_data[i];
        Debug_Printf("[APP] Data published %u bytes\r\n", pos);
        sysState = STATE_IDLE;
    } else {
        Debug_Print("[APP] Data publish failed\r\n");
        GSM_SM_SetDisconnected();
        sysState = STATE_ERROR_GSM;
    }
}

/* ── Publish alarm registers only on change ── */
/* ── Publish alarm registers — bit-level decoding ── */
static void App_PublishAlarms(void)
{
    const GatewayConfig_t *cfg = Config_Get();
    if (cfg->num_alarm_bits == 0) return;

    /* Check if any alarm register changed */
    bool any_changed = false;
    for (uint8_t i = 0; i < cfg->num_regs; i++) {
        if (!cfg->regs[i].is_alarm || !cfg->regs[i].enabled) continue;
        if (modbus_data[i] != last_alarm_data[i]) { any_changed = true; break; }
    }
    if (!any_changed) return;

    sysState = STATE_SENDING_GSM;

    static uint32_t ts_base = 1700000000UL;
    uint32_t ts = ts_base + HAL_GetTick() / 1000UL;

    static char json[512];
    int pos = 0;

    pos += snprintf(json + pos, sizeof(json) - pos,
                    "{\"id\":%u,\"ts\":%lu", cfg->device_id, ts);

    /* Decode each alarm bit by name */
    const char *first_active_name = NULL;
    uint8_t     first_active_idx  = 0;
    bool        has_active = false;

    for (uint8_t i = 0; i < cfg->num_alarm_bits && pos < (int)sizeof(json) - 30; i++) {
        const AlarmBitDef_t *a = &cfg->alarm_bits[i];
        if (a->reg_idx >= cfg->num_regs) continue;

        uint16_t regval = modbus_data[a->reg_idx];
        uint8_t  bit_state = (regval >> a->bit_pos) & 1U;

        pos += snprintf(json + pos, sizeof(json) - pos,
                        ",\"%s\":%u", a->name, bit_state);

        /* Track first active alarm for display */
        if (bit_state && !has_active) {
            first_active_name = a->name;
            first_active_idx  = i;
            has_active = true;
        }
    }

    pos += snprintf(json + pos, sizeof(json) - pos, "}");

    /* Update display — show alarm or clear */
    if (has_active)
        Display_SetAlarm(first_active_name, first_active_idx);
    else
        Display_ClearAlarm();

    /* Publish to alarm topic */
    GsmResult_t res = App_MqttPublish(cfg->mqtt_alarm_topic,
                                       (const uint8_t *)json, (uint16_t)pos,
                                       cfg->mqtt_qos);
    if (res == GSM_OK) {
        for (uint8_t i = 0; i < cfg->num_regs; i++)
            if (cfg->regs[i].is_alarm) last_alarm_data[i] = modbus_data[i];
        Debug_Printf("[APP] Alarm published %u bytes\r\n", pos);
        sysState = STATE_IDLE;
    } else {
        Debug_Print("[APP] Alarm publish failed\r\n");
        GSM_SM_SetDisconnected();
        sysState = STATE_ERROR_GSM;
    }
}

/* Dispatch publish through modem HAL */
static GsmResult_t App_MqttPublish(const char *topic, const uint8_t *payload,
                                    uint16_t len, uint8_t qos)
{
    return Modem_MqttPublish(topic, payload, len, qos);
}

/* ════════════════════════════════════════════════════════════════════
   LED UPDATE — 4-pin RGB, non-blocking blink
   ════════════════════════════════════════════════════════════════════ */
static void App_UpdateLED(void)
{
    static uint32_t blink_tick = 0;
    static uint8_t  blink_on   = 0;
    uint32_t now = HAL_GetTick();

    switch (sysState) {
        case STATE_IDLE:
            RGB_GREEN();
            break;
        case STATE_READING_MODBUS:
            if ((now - blink_tick) >= 300) {
                blink_tick = now; blink_on = !blink_on;
                if (blink_on) RGB_CYAN(); else RGB_OFF();
            }
            break;
        case STATE_SENDING_GSM:
            if ((now - blink_tick) >= 120) {
                blink_tick = now; blink_on = !blink_on;
                if (blink_on) RGB_BLUE(); else RGB_OFF();
            }
            break;
        case STATE_ERROR_MODBUS:
            RGB_RED();
            break;
        case STATE_ERROR_GSM:
            if ((now - blink_tick) >= 600) {
                blink_tick = now; blink_on = !blink_on;
                if (blink_on) RGB_RED(); else RGB_OFF();
            }
            break;
        case STATE_NO_NETWORK:
            if ((now - blink_tick) >= 400) {
                blink_tick = now; blink_on = !blink_on;
                if (blink_on) RGB_PURPLE(); else RGB_OFF();
            }
            break;
        case STATE_INIT:
        default:
            RGB_BLUE();
            break;
    }
}

/* ════════════════════════════════════════════════════════════════════
   HAL CALLBACKS
   ════════════════════════════════════════════════════════════════════ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h)
{
    CLI_RxCallback(h);         /* debug CLI ring buffer feed */
    Modem_UART_RxCallback(h);  /* GSM ring buffer feed       */
}

/* ════════════════════════════════════════════════════════════════════
   PERIPHERAL INIT
   ════════════════════════════════════════════════════════════════════ */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState            = RCC_HSE_ON;
    osc.HSEPredivValue      = RCC_HSE_PREDIV_DIV1;
    osc.PLL.PLLState        = RCC_PLL_ON;
    osc.PLL.PLLSource       = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL          = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|
                         RCC_CLOCKTYPE_PCLK1 |RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;   /* 36 MHz */
    clk.APB2CLKDivider = RCC_HCLK_DIV1;   /* 72 MHz */
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB4 — GSM RESET — set HIGH FIRST to release module from reset ASAP.
     * During MCU boot PB4 floats LOW, holding the module in reset.
     * Add a 10K pull-up resistor on PB4 to 3.3V for hardware safety. */
    g.Pin=GPIO_PIN_4; g.Mode=GPIO_MODE_OUTPUT_PP; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB,&g);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET);  /* idle high = not in reset */

    /* PA4 — RS-485 DE/RE */
    g.Pin=GPIO_PIN_4; g.Mode=GPIO_MODE_OUTPUT_PP; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&g);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);

    /* PA8 — W25Q64 CS */
    g.Pin=GPIO_PIN_8; g.Mode=GPIO_MODE_OUTPUT_PP; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA,&g);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_SET);  /* deselect */
}

static void MX_USART1_Init(void)   /* CLI 115200 8N1 (CH340 USB-C) */
{
    huart1.Instance=USART1; huart1.Init.BaudRate=115200;
    huart1.Init.WordLength=UART_WORDLENGTH_8B; huart1.Init.StopBits=UART_STOPBITS_1;
    huart1.Init.Parity=UART_PARITY_NONE; huart1.Init.Mode=UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl=UART_HWCONTROL_NONE; huart1.Init.OverSampling=UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void MX_USART2_Init(void)   /* GSM 115200 8N1 */
{
    huart2.Instance=USART2; huart2.Init.BaudRate=115200;
    huart2.Init.WordLength=UART_WORDLENGTH_8B; huart2.Init.StopBits=UART_STOPBITS_1;
    huart2.Init.Parity=UART_PARITY_NONE; huart2.Init.Mode=UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl=UART_HWCONTROL_NONE; huart2.Init.OverSampling=UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}

static void MX_USART3_Init(void)   /* Modbus 9600 8N1 (RS-485) */
{
    huart3.Instance=USART3; huart3.Init.BaudRate=9600;
    huart3.Init.WordLength=UART_WORDLENGTH_8B; huart3.Init.StopBits=UART_STOPBITS_1;
    huart3.Init.Parity=UART_PARITY_NONE; huart3.Init.Mode=UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl=UART_HWCONTROL_NONE; huart3.Init.OverSampling=UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);
}

static void MX_I2C1_Init(void)     /* OLED 400 kHz */
{
    hi2c1.Instance=I2C1; hi2c1.Init.ClockSpeed=400000;
    hi2c1.Init.DutyCycle=I2C_DUTYCYCLE_2; hi2c1.Init.OwnAddress1=0;
    hi2c1.Init.AddressingMode=I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode=I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.GeneralCallMode=I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode=I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}

static void MX_SPI1_Init(void)     /* W25Q64 — CPOL=0 CPHA=0 mode 0 */
{
    hspi1.Instance=SPI1;
    hspi1.Init.Mode=SPI_MODE_MASTER; hspi1.Init.Direction=SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize=SPI_DATASIZE_8BIT; hspi1.Init.CLKPolarity=SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase=SPI_PHASE_1EDGE; hspi1.Init.NSS=SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_4;  /* 72/4=18 MHz */
    hspi1.Init.FirstBit=SPI_FIRSTBIT_MSB; hspi1.Init.TIMode=SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

/* Error handler */
void Error_Handler(void) { while(1) { RGB_RED(); HAL_Delay(200); RGB_OFF(); HAL_Delay(200); } }
