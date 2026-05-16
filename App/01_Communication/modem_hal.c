/**
 * modem_hal.c
 * =========================================================================
 * Thin dispatch layer — routes all calls to the active modem driver.
 * =========================================================================
 */
#include "modem_hal.h"
#include "modem_at.h"
#include "config.h"
#include "debug_cli.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

/* ── State ───────────────────────────────────────────────────────── */
static const ModemOps_t *s_ops = MODEM_OPS;
static GPIO_TypeDef     *s_rst_port;
static uint16_t          s_rst_pin;
static bool              s_mqtt_up = false;

/* ── Init ─────────────────────────────────────────────────────────── */

void Modem_Init(UART_HandleTypeDef *huart,
                GPIO_TypeDef *rst_port, uint16_t rst_pin)
{
    s_rst_port = rst_port;
    s_rst_pin  = rst_pin;
    AT_Init(huart);
#if MODEM_DRIVER == MODEM_DRV_QUECTEL
    Debug_Print("[MODEM] Driver: Quectel EC200U\r\n");
#else
    Debug_Print("[MODEM] Driver: SIMCom A7670C\r\n");
#endif
}

const ModemOps_t *Modem_GetOps(void) { return s_ops; }

/* ── Hardware ────────────────────────────────────────────────────── */

void Modem_ResetAssert(void)
{
    HAL_GPIO_WritePin(s_rst_port, s_rst_pin, s_ops->reset_assert_level);
}

void Modem_ResetRelease(void)
{
    GPIO_PinState lvl = (s_ops->reset_assert_level == GPIO_PIN_RESET)
                        ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(s_rst_port, s_rst_pin, lvl);
}

GsmResult_t Modem_PowerOn(void)
{
    Debug_Print("[MODEM] Resetting module...\r\n");
    Modem_ResetAssert();
    HAL_Delay(200U);
    Modem_ResetRelease();
    return GSM_OK;
}

GsmResult_t Modem_WaitReady(uint32_t tmo)
{
    Debug_Print("[MODEM] Waiting for module...\r\n");
    uint32_t t = HAL_GetTick();

    if (AT_Wait("RDY", tmo > 5000U ? 5000U : tmo)) {
        Debug_Print("[MODEM] Got RDY\r\n");
        HAL_Delay(500U);
    }

    while ((HAL_GetTick() - t) < tmo) {
        if (AT_Cmd("AT", "OK", 1000U)) {
            AT_Cmd("ATE0",     "OK", 1000U);
            AT_Cmd("AT+CMEE=2","OK", 1000U);
            Debug_Print("[MODEM] Module ready\r\n");
            return GSM_OK;
        }
        HAL_Delay(500U);
    }
    Debug_Print("[MODEM] Module not responding!\r\n");
    return GSM_ERR_NO_RESPONSE;
}

void Modem_Reboot(void)
{
    AT_Send(s_ops->reboot_cmd);
    HAL_Delay(5000U);
}

/* ── Network ─────────────────────────────────────────────────────── */

GsmResult_t Modem_GetSignalQuality(int8_t *rssi_dbm)
{
    AT_RxFlush();
    AT_Send("AT+CSQ");
    char buf[64] = {0};
    uint16_t i = 0;
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 2000U) {
        while (at_tail != at_head && i < 63) {
            buf[i++] = (char)at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
            char *p = strstr(buf, "+CSQ:");
            if (p) {
                int r = atoi(p + 5);
                *rssi_dbm = (r == 99) ? -127 : (int8_t)(-113 + r * 2);
                return GSM_OK;
            }
        }
        HAL_Delay(5U);
    }
    return GSM_ERR_TIMEOUT;
}

/* ── MQTT dispatch ───────────────────────────────────────────────── */

GsmResult_t Modem_MqttPublish(const char *topic, const uint8_t *payload,
                               uint16_t len, uint8_t qos)
{
    if (!s_mqtt_up) return GSM_ERR_MQTT_FAIL;
    GsmResult_t r = s_ops->mqtt_publish(topic, payload, len, qos);
    if (r != GSM_OK) s_mqtt_up = false;
    return r;
}

GsmResult_t Modem_MqttDisconnect(void)
{
    GsmResult_t r = s_ops->mqtt_disconnect();
    s_mqtt_up = false;
    return r;
}

bool Modem_MqttIsConnected(void) { return s_mqtt_up; }

void Modem_MqttSetUp(bool up) { s_mqtt_up = up; }

GsmResult_t Modem_MqttSubscribe(const char *topic, uint8_t qos)
{
    if (!s_mqtt_up) return GSM_ERR_MQTT_FAIL;
    return s_ops->mqtt_subscribe(topic, qos);
}

bool Modem_MqttCheckRx(char *topic_out, uint16_t topic_sz,
                        char *payload_out, uint16_t payload_sz)
{
    return s_ops->mqtt_check_rx(topic_out, topic_sz, payload_out, payload_sz);
}

void Modem_MqttCheckURC(void)
{
    if (s_ops->mqtt_check_urc) s_ops->mqtt_check_urc();
}

/* ── HTTP dispatch ───────────────────────────────────────────────── */

GsmResult_t Modem_HttpGet(const char *url, char *out_buf,
                           uint16_t buf_len, uint16_t *out_len)
{
    return s_ops->http_get(url, out_buf, buf_len, out_len);
}

GsmResult_t Modem_HttpGetRange(const char *url,
                                uint32_t range_start, uint32_t range_end,
                                uint8_t *out_buf, uint16_t *out_len)
{
    return s_ops->http_get_range(url, range_start, range_end, out_buf, out_len);
}

GsmResult_t Modem_HttpDownloadToFlash(const char *url, uint32_t flash_addr,
                                       uint32_t expected_size,
                                       uint32_t *out_written,
                                       OtaProgressCb_t progress_cb)
{
    if (s_ops->http_download_to_flash)
        return s_ops->http_download_to_flash(url, flash_addr, expected_size,
                                             out_written, progress_cb);
    return GSM_ERR_HTTP_FAIL;
}

/* ── ISR callback ────────────────────────────────────────────────── */

void Modem_UART_RxCallback(UART_HandleTypeDef *huart)
{
    AT_UART_RxCallback(huart);
}
