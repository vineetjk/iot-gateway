/**
 * config.h
 * =========================================================================
 * Gateway configuration — stored in internal flash with ping-pong
 * wear levelling across two 2 KB pages.
 * =========================================================================
 */
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Limits ──────────────────────────────────────────────────────── */
#define CONFIG_MAGIC            0xDE14A501UL
#define CONFIG_VERSION          4U
#define CONFIG_MAX_REGS         32U
#define CONFIG_MAX_TAG_LEN      20U
#define CONFIG_MAX_ALARM_BITS   24U
#define CONFIG_ALARM_NAME_LEN   8U
#define CONFIG_MAX_TOPIC_LEN    64U
#define CONFIG_MAX_APN_LEN      32U
#define CONFIG_MAX_BROKER_LEN   64U
#define CONFIG_MAX_CLIENT_LEN   32U
#define CONFIG_MAX_USER_LEN     32U
#define CONFIG_MAX_PASS_LEN     32U

/* ── Modem type ──────────────────────────────────────────────────── */
typedef enum {
    MODEM_SIMCOM_A7670C  = 0,
    MODEM_QUECTEL_EC200U = 1,
} ModemType_t;

/* ── Register data types ─────────────────────────────────────────── */
typedef enum {
    DTYPE_UINT16  = 0,
    DTYPE_INT16   = 1,
    DTYPE_UINT32  = 2,
    DTYPE_INT32   = 3,
    DTYPE_FLOAT32 = 4,
} RegDataType_t;

/* ── Single Modbus register descriptor ───────────────────────────── */
#pragma pack(1)
typedef struct {
    uint8_t       slave_addr;
    uint16_t      reg_addr;
    uint8_t       func_code;       /* 0x03=holding, 0x04=input */
    RegDataType_t data_type;
    float         scale;
    char          tag[CONFIG_MAX_TAG_LEN];
    uint8_t       enabled;
    uint8_t       is_alarm;            /* 0=parameter, 1=alarm */
} RegDef_t;
#pragma pack()

/* ── Alarm bit descriptor (maps register bit to a named alarm) ──── */
#pragma pack(1)
typedef struct {
    uint8_t  reg_idx;                    /* index into regs[] (alarm register) */
    uint8_t  bit_pos;                    /* 0-15 within the register */
    char     name[CONFIG_ALARM_NAME_LEN];/* short tag e.g. "OV","UC","OT" */
} AlarmBitDef_t;
#pragma pack()

/* ── Full gateway configuration ──────────────────────────────────── */
#pragma pack(1)
typedef struct {
    /* Header */
    uint32_t magic;
    uint8_t  cfg_version;
    uint32_t write_count;

    /* Device */
    uint16_t device_id;
    char     device_name[32];

    /* Modbus */
    uint32_t modbus_baud;
    uint8_t  modbus_parity;        /* 0=None 1=Even 2=Odd */
    uint8_t  modbus_stop_bits;
    uint16_t modbus_timeout_ms;
    uint16_t poll_interval_ms;
    uint8_t  num_regs;
    RegDef_t regs[CONFIG_MAX_REGS];

    /* Telemetry */
    uint32_t publish_interval_ms;
    uint8_t  delta_mode;
    uint16_t delta_threshold;

    /* GSM / MQTT */
    uint8_t  modem_type;           /* ModemType_t: 0=SIMCom A7670C, 1=Quectel EC200U */
    char     apn[CONFIG_MAX_APN_LEN];
    char     broker[CONFIG_MAX_BROKER_LEN];
    uint16_t broker_port;
    char     mqtt_client_id[CONFIG_MAX_CLIENT_LEN];
    char     mqtt_user[CONFIG_MAX_USER_LEN];
    char     mqtt_pass[CONFIG_MAX_PASS_LEN];
    char     mqtt_topic[CONFIG_MAX_TOPIC_LEN];
    char     mqtt_alarm_topic[CONFIG_MAX_TOPIC_LEN];
    uint8_t  mqtt_qos;

    /* OTA */
    char     ota_manifest_url[128];
    uint32_t fw_version;
    uint8_t  ota_enabled;

    /* Alarm bit definitions */
    uint8_t       num_alarm_bits;
    AlarmBitDef_t alarm_bits[CONFIG_MAX_ALARM_BITS];

    /* Footer */
    uint32_t crc32;
} GatewayConfig_t;
#pragma pack()

/* ── STM32 Unique ID → 16-bit device ID ─────────────────────────── */
#define STM32_UID_BASE  0x1FFFF7E8UL
uint16_t Config_GetMcuId(void);   /* derive 16-bit ID from 96-bit UID */

/* ── Public API ──────────────────────────────────────────────────── */
void                    Config_Init(void);
const GatewayConfig_t  *Config_Get(void);
GatewayConfig_t        *Config_GetMutable(void);
HAL_StatusTypeDef       Config_Save(void);
void                    Config_LoadDefaults(void);
bool                    Config_Validate(const GatewayConfig_t *cfg);
void                    Config_Print(void);

#endif /* CONFIG_H */
