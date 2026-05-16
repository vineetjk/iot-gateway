/**
 * modem_quectel.c
 * =========================================================================
 * Quectel EC200U modem driver.
 * AT+QMT* for MQTT, AT+QHTTP* for HTTP.
 * =========================================================================
 */
#include "modem_config.h"
#if MODEM_DRIVER == MODEM_DRV_QUECTEL

#include "modem_hal.h"
#include "modem_at.h"
#include "w25q_spi.h"
#include "debug_cli.h"
#include <string.h>
#include <stdio.h>

/* ════════════════════════════════════════════════════════════════════
   MQTT
   ════════════════════════════════════════════════════════════════════ */

static GsmResult_t quectel_mqtt_connect(const char *broker, uint16_t port,
                                         const char *client_id,
                                         const char *user, const char *pass)
{
    char *cmd = at_work;

    /* Cleanup any previous session */
    AT_Cmd("AT+QMTDISC=0",  "OK", 3000U);
    AT_Cmd("AT+QMTCLOSE=0", "OK", 3000U);
    HAL_Delay(500U);

    /* MQTT 3.1.1 */
    AT_Cmd("AT+QMTCFG=\"version\",0,4", "OK", 1000U);

    /* Open network connection to broker */
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QMTOPEN=0,\"%s\",%u", broker, port);
    if (!AT_Cmd(cmd, "+QMTOPEN: 0,0", 30000U)) return GSM_ERR_MQTT_FAIL;

    /* MQTT CONNECT */
    if (user && user[0])
        snprintf(cmd, AT_WORK_BUF_SIZE,
                 "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                 client_id, user, pass);
    else
        snprintf(cmd, AT_WORK_BUF_SIZE,
                 "AT+QMTCONN=0,\"%s\"", client_id);

    if (!AT_Cmd(cmd, "+QMTCONN: 0,0,0", 30000U)) return GSM_ERR_MQTT_FAIL;
    Debug_Print("[QEC] MQTT connected\r\n");
    return GSM_OK;
}

/* Fire-and-forget publish: payload + Ctrl+Z, no ACK wait.
 * +QMTPUB result and +QMTSTAT URCs handled by quectel_mqtt_check_urc(). */
static GsmResult_t quectel_mqtt_publish(const char *topic,
                                         const uint8_t *payload, uint16_t len,
                                         uint8_t qos)
{
    char cmd[160];
    (void)qos;

    /* AT+QMTPUB=0,0,0,0,"<topic>" — msgid=0 means no ACK expected */
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QMTPUB=0,0,0,0,\"%s\"", topic);
    AT_RxFlush();
    AT_Send(cmd);

    /* Wait for ">" data prompt */
    if (!AT_Wait(">", 500U)) {
        Debug_Print("[QEC] QMTPUB: no prompt\r\n");
        return GSM_ERR_MQTT_FAIL;
    }

    HAL_Delay(50U);
    AT_RawTx(payload, len);

    /* Ctrl+Z (0x1A) terminates the payload — triggers send */
    uint8_t ctrlz = 0x1AU;
    AT_RawTx(&ctrlz, 1U);

    return GSM_OK;
}

static GsmResult_t quectel_mqtt_disconnect(void)
{
    AT_Cmd("AT+QMTDISC=0",  "OK", 5000U);
    AT_Cmd("AT+QMTCLOSE=0", "OK", 3000U);
    return GSM_OK;
}

static GsmResult_t quectel_mqtt_subscribe(const char *topic, uint8_t qos)
{
    char cmd[128];
    static uint16_t sub_mid = 1;
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QMTSUB=0,%u,\"%s\",%u",
             sub_mid++, topic, qos);
    if (!AT_Cmd(cmd, "+QMTSUB: 0,", 10000U))
        return GSM_ERR_MQTT_FAIL;
    Debug_Printf("[QEC] Subscribed: %s\r\n", topic);
    return GSM_OK;
}

/* Parse +QMTRECV: 0,<msgid>,"<topic>","<payload>"
 * Payload can contain quotes (e.g. CSV with embedded commas/quotes),
 * so we find the LAST quote before EOL as the closing delimiter. */
static bool quectel_mqtt_check_rx(char *topic_out, uint16_t topic_sz,
                                    char *payload_out, uint16_t payload_sz)
{
    static char ubuf[384];
    AT_RingPeek(ubuf, sizeof(ubuf));

    topic_out[0]   = '\0';
    payload_out[0] = '\0';

    char *qr = strstr(ubuf, "+QMTRECV:");
    if (!qr) return false;

    /* Find end of line */
    char *eol = qr;
    while (*eol && *eol != '\n') eol++;
    if (!*eol) return false;  /* incomplete line, wait for more data */

    /* Parse: skip to first quote (topic) */
    char *q1 = strchr(qr, '"');
    if (q1) {
        q1++;
        uint16_t i = 0;
        while (*q1 && *q1 != '"' && i < topic_sz - 1U)
            topic_out[i++] = *q1++;
        topic_out[i] = '\0';
    }

    /* Skip to third quote (payload start) — after topic close quote and comma.
     * Find the LAST quote before eol as closing delimiter. */
    char *q3 = q1 ? strchr(q1 + 1, '"') : NULL;
    if (q3) {
        q3++;  /* points to payload content */
        char *qlast = eol;
        while (qlast > q3 && *qlast != '"') qlast--;
        uint16_t i = 0;
        while (q3 < qlast && i < payload_sz - 1U)
            payload_out[i++] = *q3++;
        payload_out[i] = '\0';
    }

    /* Consume up to and including the newline */
    if (*eol) eol++;
    AT_RingConsume((uint16_t)(eol - ubuf));
    return (topic_out[0] != '\0');
}

/* Check for +QMTSTAT URC (connection lost) — call from main loop */
static void quectel_mqtt_check_urc(void)
{
    static char peek[128];
    uint16_t pi = AT_RingPeek(peek, sizeof(peek));
    (void)pi;

    if (strstr(peek, "+QMTSTAT:")) {
        Debug_Print("[QEC] MQTT connection lost (QMTSTAT)\r\n");
        Modem_MqttSetUp(false);
        /* Consume the URC */
        char *end = strstr(peek, "+QMTSTAT:");
        while (*end && *end != '\n') end++;
        if (*end) end++;
        AT_RingConsume((uint16_t)(end - peek));
    }
}

/* ── Non-blocking SM sub-state machine for MQTT cleanup ──────────── */

static enum { QC_DISC, QC_CLOSE, QC_DONE } s_qc_step;
static bool     s_qc_sent;
static uint32_t s_qc_deadline;

static void quectel_sm_mqtt_cleanup_begin(void)
{
    s_qc_step = QC_DISC;
    s_qc_sent = false;
}

static int quectel_sm_mqtt_cleanup_poll(uint32_t deadline)
{
    (void)deadline;
    uint32_t now = HAL_GetTick();

    switch (s_qc_step) {
    case QC_DISC:
        if (!s_qc_sent) {
            AT_BeginCmd("AT+QMTDISC=0");
            s_qc_sent = true;
            s_qc_deadline = now + 5000U;
        }
        if (AT_Poll("OK", s_qc_deadline) != 0) {
            s_qc_step = QC_CLOSE; s_qc_sent = false;
        }
        break;
    case QC_CLOSE:
        if (!s_qc_sent) {
            AT_BeginCmd("AT+QMTCLOSE=0");
            s_qc_sent = true;
            s_qc_deadline = now + 3000U;
        }
        if (AT_Poll("OK", s_qc_deadline) != 0) {
            s_qc_step = QC_DONE;
            return 1;
        }
        break;
    case QC_DONE:
        return 1;
    }
    return 0;
}

/* ── Non-blocking SM sub-state machine for MQTT open+connect ─────── */

static enum { QO_CLOSE, QO_CFG, QO_OPEN, QO_CONN, QO_DONE } s_qo_step;
static bool     s_qo_sent;
static uint32_t s_qo_deadline;
static char     s_qo_broker[64];
static uint16_t s_qo_port;
static char     s_qo_client[32];
static char     s_qo_user[32];
static char     s_qo_pass[32];

static void quectel_sm_mqtt_open_begin(const char *broker, uint16_t port,
                                        const char *client_id,
                                        const char *user, const char *pass)
{
    strncpy(s_qo_broker, broker, 63);
    s_qo_port = port;
    strncpy(s_qo_client, client_id, 31);
    strncpy(s_qo_user, user ? user : "", 31);
    strncpy(s_qo_pass, pass ? pass : "", 31);
    s_qo_step = QO_CLOSE;
    s_qo_sent = false;
}

static int quectel_sm_mqtt_open_poll(uint32_t deadline)
{
    (void)deadline;
    uint32_t now = HAL_GetTick();
    char *cmd = at_work;
    int r;

    switch (s_qo_step) {
    case QO_CLOSE:
        if (!s_qo_sent) {
            AT_BeginCmd("AT+QMTCLOSE=0");
            s_qo_sent = true;
            s_qo_deadline = now + 3000U;
        }
        if (AT_Poll("OK", s_qo_deadline) != 0) {
            s_qo_step = QO_CFG; s_qo_sent = false;
        }
        break;
    case QO_CFG:
        if (!s_qo_sent) {
            AT_BeginCmd("AT+QMTCFG=\"version\",0,4");
            s_qo_sent = true;
            s_qo_deadline = now + 1000U;
        }
        if (AT_Poll("OK", s_qo_deadline) != 0) {
            s_qo_step = QO_OPEN; s_qo_sent = false;
        }
        break;
    case QO_OPEN:
        if (!s_qo_sent) {
            snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QMTOPEN=0,\"%s\",%u",
                     s_qo_broker, s_qo_port);
            AT_BeginCmd(cmd);
            s_qo_sent = true;
            s_qo_deadline = now + 30000U;
        }
        r = AT_Poll("+QMTOPEN: 0,0", s_qo_deadline);
        if (r == 1) { s_qo_step = QO_CONN; s_qo_sent = false; }
        else if (r == -1) {
            const char *acc = AT_GetAcc();
            if (strstr(acc, "+QMTOPEN: 0,") && !strstr(acc, "+QMTOPEN: 0,0"))
                return -1;  /* non-zero error */
            return -1;      /* timeout */
        }
        break;
    case QO_CONN:
        if (!s_qo_sent) {
            if (s_qo_user[0])
                snprintf(cmd, AT_WORK_BUF_SIZE,
                         "AT+QMTCONN=0,\"%s\",\"%s\",\"%s\"",
                         s_qo_client, s_qo_user, s_qo_pass);
            else
                snprintf(cmd, AT_WORK_BUF_SIZE,
                         "AT+QMTCONN=0,\"%s\"", s_qo_client);
            AT_BeginCmd(cmd);
            s_qo_sent = true;
            s_qo_deadline = now + 30000U;
        }
        r = AT_Poll("+QMTCONN: 0,0,0", s_qo_deadline);
        if (r == 1) { s_qo_step = QO_DONE; return 1; }
        if (r == -1) return -1;
        /* Check for rejection */
        {
            const char *acc = AT_GetAcc();
            if (strstr(acc, "+QMTCONN: 0,0,") &&
                !strstr(acc, "+QMTCONN: 0,0,0"))
                return -1;
        }
        break;
    case QO_DONE:
        return 1;
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════
   HTTP (AT+QHTTP*)
   ════════════════════════════════════════════════════════════════════ */

/* ── HTTP helpers ─────────────────────────────────────────────────── */

/* Set URL for HTTP request. Configures SSL if HTTPS. */
static bool q_set_url(const char *url)
{
    char *cmd = at_work;
    bool use_ssl = (strncmp(url, "https://", 8) == 0);

    /* Configure SSL */
    if (use_ssl) {
        AT_Cmd("AT+QHTTPCFG=\"sslctxid\",1", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"seclevel\",1,0", "OK", 2000U);
    } else {
        AT_Cmd("AT+QHTTPCFG=\"sslctxid\",0", "OK", 2000U);
    }

    uint16_t url_len = (uint16_t)strlen(url);
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+QHTTPURL=%u,80", url_len);
    AT_RxFlush();
    AT_Send(cmd);
    if (!AT_Wait("CONNECT", 5000U)) {
        Debug_Print("[QEC] URL no CONNECT\r\n");
        return false;
    }
    HAL_Delay(50);
    AT_RawTx((const uint8_t *)url, url_len);
    if (!AT_Wait("OK", 5000U)) {
        Debug_Print("[QEC] URL not accepted\r\n");
        return false;
    }
    Debug_Printf("[QEC] URL set (%u)\r\n", url_len);
    return true;
}

/* Parse +QHTTPGET: <err>,<status>,<datalen> (async URC) — waits up to 90s */
static uint32_t q_parse_httpget(void)
{
    char buf[128] = {0};
    uint16_t idx = 0;
    uint32_t t = HAL_GetTick();
    uint32_t last_log = 0;

    Debug_Print("[QEC] Waiting +QHTTPGET...\r\n");
    while ((HAL_GetTick() - t) < 90000U) {
        while (at_tail != at_head && idx < 127) {
            buf[idx++] = (char)at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
            char *p = strstr(buf, "+QHTTPGET:");
            if (p) {
                int err = 0, status = 0; uint32_t len = 0;
                sscanf(p, "+QHTTPGET: %d,%d,%lu", &err, &status, &len);
                Debug_Printf("[QEC] HTTPGET err=%d st=%d len=%lu\r\n", err, status, len);
                return (err == 0 && (status == 200 || status == 206)) ? len : 0;
            }
            if (strstr(buf, "ERROR")) {
                Debug_Printf("[QEC] HTTPGET ERROR: %s\r\n", buf);
                return 0;
            }
        }
        if ((HAL_GetTick() - t) / 10000U > last_log) {
            last_log = (HAL_GetTick() - t) / 10000U;
            Debug_Printf("[QEC] ...%lus (rx=%u)\r\n", (HAL_GetTick()-t)/1000U, idx);
        }
        HAL_Delay(5U);
    }
    Debug_Printf("[QEC] HTTPGET timeout buf: %.60s\r\n", buf);
    return 0;
}

/* Read body: AT+QHTTPREAD=80 → CONNECT → data → +QHTTPREAD: 0 */
static uint16_t q_read_body(uint8_t *out, uint16_t max_len)
{
    AT_RxFlush();
    AT_Send("AT+QHTTPREAD=80");
    if (!AT_Wait("CONNECT", 10000U)) return 0;

    uint16_t i = 0;
    uint32_t t = HAL_GetTick();
    while (i < max_len && (HAL_GetTick() - t) < 30000U) {
        while (at_tail != at_head && i < max_len) {
            out[i++] = at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
        }
        if (i >= max_len) break;
        HAL_Delay(2U);
    }
    AT_Wait("+QHTTPREAD:", 5000U);
    return i;
}

/* Simple HTTP GET — no custom headers, module handles SSL */
static GsmResult_t quectel_http_get(const char *url, char *out_buf,
                                     uint16_t buf_len, uint16_t *out_len)
{
    *out_len = 0;
    if (!q_set_url(url)) return GSM_ERR_HTTP_FAIL;

    AT_RxFlush();
    AT_Send("AT+QHTTPGET=60");

    uint32_t dlen = q_parse_httpget();
    if (!dlen) return GSM_ERR_HTTP_FAIL;

    uint16_t rlen = (dlen < (uint32_t)(buf_len - 1U)) ? (uint16_t)dlen : (buf_len - 1U);
    *out_len = q_read_body((uint8_t *)out_buf, rlen);
    out_buf[*out_len] = '\0';
    Debug_Printf("[QEC] GET done: %u bytes\r\n", *out_len);
    return GSM_OK;
}

/* Streaming OTA: download full file, then stream to SPI flash via QHTTPREAD */
static GsmResult_t quectel_http_download_to_flash(
    const char *url, uint32_t flash_addr, uint32_t expected_size,
    uint32_t *out_written, OtaProgressCb_t progress_cb)
{
    *out_written = 0;
    if (!q_set_url(url)) return GSM_ERR_HTTP_FAIL;

    /* Download full file — module buffers internally */
    AT_RxFlush();
    AT_Send("AT+QHTTPGET=90");
    uint32_t file_len = q_parse_httpget();
    if (!file_len) return GSM_ERR_HTTP_FAIL;

    Debug_Printf("[QEC] File ready: %lu bytes\r\n", file_len);

    /* Stream via QHTTPREAD → ring buffer → SPI flash */
    AT_RxFlush();
    AT_Send("AT+QHTTPREAD=60");
    if (!AT_Wait("CONNECT", 15000U)) {
        Debug_Print("[QEC] HTTPREAD no CONNECT\r\n");
        return GSM_ERR_HTTP_FAIL;
    }

    uint8_t page[256];
    uint16_t pi = 0;
    uint32_t written = 0;
    uint32_t t = HAL_GetTick();

    while (written < file_len && (HAL_GetTick() - t) < 120000U) {
        while (at_tail != at_head) {
            page[pi++] = at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;

            if (pi >= 256U) {
                /* Check if we'd write past file end */
                if (written + pi > file_len) {
                    pi = (uint16_t)(file_len - written);
                }
                W25Q_Write(flash_addr + written, page, pi);
                written += pi;
                pi = 0;
                t = HAL_GetTick();  /* reset timeout */

                if (progress_cb && (written % 4096U) < 256U)
                    progress_cb(written, file_len);

                if (written >= file_len) break;
            }
        }
        HAL_Delay(1U);
    }
    /* Flush remaining */
    if (pi > 0 && written < file_len) {
        uint16_t remain = (file_len - written < pi) ? (uint16_t)(file_len - written) : pi;
        W25Q_Write(flash_addr + written, page, remain);
        written += remain;
    }

    AT_Wait("+QHTTPREAD:", 5000U);
    *out_written = written;
    if (progress_cb) progress_cb(written, file_len);
    Debug_Printf("[QEC] Flash done: %lu/%lu\r\n", written, file_len);
    return (written >= file_len) ? GSM_OK : GSM_ERR_HTTP_FAIL;
}

/* Legacy range — fallback (not used for OTA) */
static GsmResult_t quectel_http_get_range(const char *url,
                                           uint32_t range_start, uint32_t range_end,
                                           uint8_t *out_buf, uint16_t *out_len)
{
    (void)range_start; (void)range_end;
    return quectel_http_get(url, (char*)out_buf,
                            (uint16_t)(range_end - range_start + 1U), out_len);
}

/* ════════════════════════════════════════════════════════════════════
   VTABLE
   ════════════════════════════════════════════════════════════════════ */

const ModemOps_t g_quectel_ops = {
    /* Hardware */
    .reset_assert_level   = GPIO_PIN_SET,       /* active-HIGH */

    /* Network */
    .apn_cmd_fmt          = "AT+QICSGP=1,1,\"%s\",\"\",\"\",1",
    .pdp_act_cmd          = "AT+QIACT=1",
    .pdp_act_timeout_ms   = 150000U,
    .pdp_check_cmd        = "AT+QIACT?",
    .pdp_check_expect     = "+QIACT: 1,1",

    /* MQTT */
    .mqtt_connect         = quectel_mqtt_connect,
    .mqtt_publish         = quectel_mqtt_publish,
    .mqtt_disconnect      = quectel_mqtt_disconnect,
    .mqtt_subscribe       = quectel_mqtt_subscribe,
    .mqtt_check_rx        = quectel_mqtt_check_rx,
    .mqtt_check_urc       = quectel_mqtt_check_urc,

    /* MQTT SM */
    .sm_mqtt_cleanup_begin = quectel_sm_mqtt_cleanup_begin,
    .sm_mqtt_cleanup_poll  = quectel_sm_mqtt_cleanup_poll,
    .sm_mqtt_open_begin    = quectel_sm_mqtt_open_begin,
    .sm_mqtt_open_poll     = quectel_sm_mqtt_open_poll,

    /* HTTP */
    .http_get               = quectel_http_get,
    .http_get_range         = quectel_http_get_range,
    .http_download_to_flash = quectel_http_download_to_flash,

    /* Misc */
    .reboot_cmd           = "AT+CFUN=1,1",
};

#endif /* MODEM_DRIVER == MODEM_DRV_QUECTEL */
