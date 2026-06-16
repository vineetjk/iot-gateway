/**
 * display_ui.h
 * =========================================================================
 * Display state machine for SSD1306 128x32 OLED.
 * Modes: BOOT → NORMAL (parameter cycling) → ALARM (full-screen flash).
 * =========================================================================
 */
#ifndef DISPLAY_UI_H
#define DISPLAY_UI_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    DISP_BOOT = 0,
    DISP_NORMAL,
    DISP_ALARM,
    DISP_ERROR,
    DISP_UPDATING,
    DISP_CONFIG,
} DispMode_t;

/* ── Error codes ─────────────────────────────────────────────────── */
#define ERR_NONE            0x00
#define ERR_SIM_NOT_READY   0x10
#define ERR_NO_NETWORK      0x11
#define ERR_PDP_FAIL        0x12
#define ERR_MQTT_CONN       0x13
#define ERR_MQTT_SUB        0x14
#define ERR_MQTT_PUB        0x15
#define ERR_MQTT_DISC       0x16
#define ERR_GSM_NO_RESP     0x20
#define ERR_GSM_TIMEOUT     0x21
#define ERR_MODBUS_TIMEOUT  0x30
#define ERR_MODBUS_CRC      0x31
#define ERR_FLASH_WRITE     0x40
#define ERR_FLASH_READ      0x41
#define ERR_OTA_DOWNLOAD    0x50
#define ERR_OTA_VERIFY      0x51
#define ERR_OTA_NO_SERVER   0x52
#define ERR_CONFIG_SAVE     0x60

/* Call once after SSD1306_Init */
void Display_Init(void);

/* Boot screen progress — call for each init step */
void Display_BootMsg(const char *label, bool ok);

/* Transition from boot to normal mode */
void Display_BootDone(void);

/* Call in main loop — drives animations, scrolling, rotation */
void Display_Update(uint32_t now);

/* Feed current register values for display */
void Display_SetRegValue(uint8_t idx, const char *tag, int32_t whole, uint32_t frac);

/* Trigger alarm mode (alarm_tag = NULL to clear) */
void Display_SetAlarm(const char *alarm_tag, uint8_t alarm_idx);
void Display_ClearAlarm(void);

/* Set status indicators */
void Display_SetGsmStatus(bool connected);
void Display_SetMqttStatus(bool connected);

/* Show error code on display (stays until cleared) */
void Display_ShowError(uint8_t err_code);
void Display_ClearError(void);

/* Show "Updating..." screen with progress */
void Display_ShowUpdating(uint32_t done, uint32_t total);

/* Config mode display — flashing "CONFIG MODE" */
void Display_ShowConfigMode(void);
void Display_ExitConfigMode(void);

#endif /* DISPLAY_UI_H */
