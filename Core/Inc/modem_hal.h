/**
 * modem_hal.h
 * =========================================================================
 * Modem HAL — unified API for all cellular modems.
 *
 * To add a new modem:
 *   1. Create modem_<name>.c in App/01_Communication/
 *   2. Populate a const ModemOps_t with your implementations
 *   3. Add a case in Modem_SelectDriver()
 * =========================================================================
 */
#ifndef MODEM_HAL_H
#define MODEM_HAL_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Result codes ────────────────────────────────────────────────── */
typedef enum {
    GSM_OK = 0,
    GSM_ERR_TIMEOUT,
    GSM_ERR_NO_RESPONSE,
    GSM_ERR_SIM_NOT_READY,
    GSM_ERR_NO_NETWORK,
    GSM_ERR_PDP_FAIL,
    GSM_ERR_MQTT_FAIL,
    GSM_ERR_HTTP_FAIL,
} GsmResult_t;

/* Progress callback for OTA streaming download */
typedef void (*OtaProgressCb_t)(uint32_t done, uint32_t total);

/* ── Modem operations vtable ─────────────────────────────────────── */
typedef struct {
    /* ── Hardware ── */
    GPIO_PinState reset_assert_level;  /* A7670C: LOW, EC200U: HIGH */

    /* ── Network (data fields used by SM) ── */
    const char *apn_cmd_fmt;           /* e.g. "AT+CGDCONT=1,\"IP\",\"%s\"" */
    const char *pdp_act_cmd;           /* e.g. "AT+CGACT=1,1" */
    uint32_t    pdp_act_timeout_ms;
    const char *pdp_check_cmd;         /* e.g. "AT+CGACT?" */
    const char *pdp_check_expect;      /* e.g. "+CGACT: 1,1" */

    /* ── MQTT ── */
    GsmResult_t (*mqtt_connect)(const char *broker, uint16_t port,
                                const char *client_id,
                                const char *user, const char *pass);
    GsmResult_t (*mqtt_publish)(const char *topic,
                                const uint8_t *payload, uint16_t len,
                                uint8_t qos);
    GsmResult_t (*mqtt_disconnect)(void);
    GsmResult_t (*mqtt_subscribe)(const char *topic, uint8_t qos);
    bool        (*mqtt_check_rx)(char *topic_out, uint16_t topic_sz,
                                 char *payload_out, uint16_t payload_sz);
    void        (*mqtt_check_urc)(void);

    /* ── MQTT SM steps (non-blocking, for gsm_sm.c) ── */
    void (*sm_mqtt_cleanup_begin)(void);
    int  (*sm_mqtt_cleanup_poll)(uint32_t deadline);  /* 0=busy, 1=done, -1=err */
    void (*sm_mqtt_open_begin)(const char *broker, uint16_t port,
                                const char *client_id,
                                const char *user, const char *pass);
    int  (*sm_mqtt_open_poll)(uint32_t deadline);     /* 0=busy, 1=done, -1=err */

    /* ── HTTP ── */
    GsmResult_t (*http_get)(const char *url, char *out_buf,
                            uint16_t buf_len, uint16_t *out_len);
    GsmResult_t (*http_get_range)(const char *url,
                                  uint32_t range_start, uint32_t range_end,
                                  uint8_t *out_buf, uint16_t *out_len);
    GsmResult_t (*http_download_to_flash)(const char *url, uint32_t flash_addr,
                                          uint32_t expected_size,
                                          uint32_t *out_written,
                                          OtaProgressCb_t progress_cb);

    /* ── Misc ── */
    const char *reboot_cmd;
} ModemOps_t;

/* ── Compile-time modem selection ─────────────────────────────────── */
#include "modem_config.h"

#if MODEM_DRIVER == MODEM_DRV_QUECTEL
extern const ModemOps_t g_quectel_ops;
#define MODEM_OPS  (&g_quectel_ops)
#elif MODEM_DRIVER == MODEM_DRV_SIMCOM
extern const ModemOps_t g_simcom_ops;
#define MODEM_OPS  (&g_simcom_ops)
#else
#error "MODEM_DRIVER not set — edit modem_config.h"
#endif

/* ── Unified public API ──────────────────────────────────────────── */
void        Modem_Init(UART_HandleTypeDef *huart,
                       GPIO_TypeDef *rst_port, uint16_t rst_pin);
const ModemOps_t *Modem_GetOps(void);

/* Hardware */
void        Modem_ResetAssert(void);
void        Modem_ResetRelease(void);
GsmResult_t Modem_PowerOn(void);
GsmResult_t Modem_WaitReady(uint32_t timeout_ms);
void        Modem_Reboot(void);

/* Network */
GsmResult_t Modem_GetSignalQuality(int8_t *rssi_dbm);

/* MQTT */
GsmResult_t Modem_MqttPublish(const char *topic,
                               const uint8_t *payload, uint16_t len,
                               uint8_t qos);
GsmResult_t Modem_MqttDisconnect(void);
bool        Modem_MqttIsConnected(void);
GsmResult_t Modem_MqttSubscribe(const char *topic, uint8_t qos);
bool        Modem_MqttCheckRx(char *topic_out, uint16_t topic_sz,
                               char *payload_out, uint16_t payload_sz);
void        Modem_MqttCheckURC(void);
void        Modem_MqttSetUp(bool up);

/* HTTP */
GsmResult_t Modem_HttpGet(const char *url, char *out_buf,
                           uint16_t buf_len, uint16_t *out_len);
GsmResult_t Modem_HttpGetRange(const char *url,
                                uint32_t range_start, uint32_t range_end,
                                uint8_t *out_buf, uint16_t *out_len);
GsmResult_t Modem_HttpDownloadToFlash(const char *url, uint32_t flash_addr,
                                       uint32_t expected_size,
                                       uint32_t *out_written,
                                       OtaProgressCb_t progress_cb);

/* ISR callback */
void Modem_UART_RxCallback(UART_HandleTypeDef *huart);

#endif /* MODEM_HAL_H */
