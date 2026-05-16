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

    /* Bind HTTP to PDP context 1 and ensure it's active */
    AT_Cmd("AT+QHTTPCFG=\"contextid\",1", "OK", 2000U);
    if (!AT_Cmd("AT+QIACT?", "+QIACT: 1", 2000U)) {
        Debug_Print("[QEC] PDP down, reactivating...\r\n");
        AT_Cmd("AT+QIACT=1", "OK", 30000U);
    }

    /* Configure SSL */
    if (use_ssl) {
        AT_Cmd("AT+QHTTPCFG=\"sslctxid\",1", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"sslversion\",1,4", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"ciphersuite\",1,0xFFFF", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"seclevel\",1,0", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"sni\",1,1", "OK", 2000U);
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

/* ── Raw TCP/SSL socket OTA download ──────────────────────────────────
 * Bypasses AT+QHTTP* (which can't do HTTPS to GitHub/CDNs).
 * Opens a raw TCP or SSL socket, sends HTTP/1.1 GET manually,
 * parses Content-Length from response headers, streams body to flash. */

/* Parse URL into host, port, path, ssl flag */
static bool q_parse_url(const char *url, char *host, uint16_t host_sz,
                         uint16_t *port, const char **path, bool *ssl)
{
    *ssl = false; *port = 80;
    if (strncmp(url, "https://", 8) == 0) { url += 8; *ssl = true; *port = 443; }
    else if (strncmp(url, "http://", 7) == 0) { url += 7; }

    const char *slash = strchr(url, '/');
    const char *colon = strchr(url, ':');
    uint16_t hlen;

    if (colon && (!slash || colon < slash)) {
        hlen = (uint16_t)(colon - url);
        *port = (uint16_t)atoi(colon + 1);
    } else {
        hlen = slash ? (uint16_t)(slash - url) : (uint16_t)strlen(url);
    }
    if (hlen >= host_sz) hlen = host_sz - 1;
    memcpy(host, url, hlen); host[hlen] = '\0';
    *path = slash ? slash : "/";
    return (hlen > 0);
}

static GsmResult_t quectel_http_download_to_flash(
    const char *url, uint32_t flash_addr, uint32_t expected_size,
    uint32_t *out_written, OtaProgressCb_t progress_cb)
{
    char *cmd = at_work;
    char host[64];
    uint16_t port;
    const char *path;
    bool ssl;
    static char redir_url[192];  /* for redirect following */
    const char *cur_url = url;
    uint8_t redirects = 0;
    *out_written = 0;

redirect_retry:
    if (redirects > 3) {
        Debug_Print("[QEC] Too many redirects\r\n");
        return GSM_ERR_HTTP_FAIL;
    }

    if (!q_parse_url(cur_url, host, sizeof(host), &port, &path, &ssl)) {
        Debug_Print("[QEC] Bad URL\r\n");
        return GSM_ERR_HTTP_FAIL;
    }
    Debug_Printf("[QEC] Host=%s Port=%u SSL=%d\r\n", host, port, ssl);
    Debug_Printf("[QEC] Path=%s\r\n", path);

    /* Close any previous socket */
    AT_Cmd("AT+QICLOSE=0", "OK", 5000U);
    HAL_Delay(500);

    /* Configure SSL if needed */
    if (ssl) {
        AT_Cmd("AT+QSSLCFG=\"sslversion\",0,4", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"ciphersuite\",0,0xFFFF", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"seclevel\",0,0", "OK", 2000U);
        AT_Cmd("AT+QSSLCFG=\"sni\",0,1", "OK", 2000U);
        /* Open SSL socket */
        snprintf(cmd, AT_WORK_BUF_SIZE,
                 "AT+QSSLOPEN=1,0,0,\"%s\",%u,0", host, port);
    } else {
        /* Open plain TCP socket (buffer access mode = 0) */
        snprintf(cmd, AT_WORK_BUF_SIZE,
                 "AT+QIOPEN=1,0,\"TCP\",\"%s\",%u,0,0", host, port);
    }

    AT_RxFlush();
    AT_Send(cmd);

    /* Wait for connection: +QIOPEN: 0,0 or +QSSLOPEN: 0,0 */
    const char *expect = ssl ? "+QSSLOPEN: 0,0" : "+QIOPEN: 0,0";
    if (!AT_Wait(expect, 30000U)) {
        Debug_Print("[QEC] Socket open failed\r\n");
        /* Print what we got */
        char ebuf[80] = {0}; uint16_t ei = 0;
        while (at_tail != at_head && ei < 79) {
            ebuf[ei++] = (char)at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
        }
        Debug_Printf("[QEC] Got: %s\r\n", ebuf);
        return GSM_ERR_HTTP_FAIL;
    }
    Debug_Print("[QEC] Socket connected\r\n");

    /* Send HTTP GET request */
    int req_len = snprintf(cmd, AT_WORK_BUF_SIZE,
        "GET %s HTTP/1.1\r\n"
        "Host: %s\r\n"
        "Connection: close\r\n"
        "User-Agent: STM32-OTA/1.0\r\n"
        "\r\n", path, host);

    /* AT+QISEND=0,<len> or AT+QSSLSEND=0,<len> */
    {
        char scmd[32];
        snprintf(scmd, sizeof(scmd), ssl ? "AT+QSSLSEND=0,%d" : "AT+QISEND=0,%d", req_len);
        AT_RxFlush();
        AT_Send(scmd);
        if (!AT_Wait(">", 5000U)) {
            Debug_Print("[QEC] No send prompt\r\n");
            AT_Cmd(ssl ? "AT+QSSLCLOSE=0" : "AT+QICLOSE=0", "OK", 5000U);
            return GSM_ERR_HTTP_FAIL;
        }
        AT_RawTx((const uint8_t *)cmd, (uint16_t)req_len);
        if (!AT_Wait("SEND OK", 10000U)) {
            Debug_Print("[QEC] Send failed\r\n");
            AT_Cmd(ssl ? "AT+QSSLCLOSE=0" : "AT+QICLOSE=0", "OK", 5000U);
            return GSM_ERR_HTTP_FAIL;
        }
    }
    Debug_Print("[QEC] GET sent, reading response...\r\n");

    /* Buffer access mode: wait for recv notification.
     * TCP: +QIURC: "recv",0  |  SSL: +QSSLURC: "recv",0 */
    if (!AT_Wait("recv", 30000U)) {
        Debug_Print("[QEC] No data received\r\n");
        AT_Cmd(ssl ? "AT+QSSLCLOSE=0" : "AT+QICLOSE=0", "OK", 5000U);
        return GSM_ERR_HTTP_FAIL;
    }
    Debug_Print("[QEC] Data available\r\n");
    HAL_Delay(500);
    AT_RxFlush();  /* Discard the URC line remnants */

    /* Read headers using AT+QIRD */
    uint32_t content_length = expected_size;
    uint8_t page[256];
    uint16_t pi = 0;
    uint32_t written = 0;
    uint32_t t = HAL_GetTick();
    bool in_body = false;
    char hdr_buf[512] = {0};
    uint16_t hi = 0;
    bool got_length = false;
    const char *close_cmd = ssl ? "AT+QSSLCLOSE=0" : "AT+QICLOSE=0";
    const char *read_cmd = ssl ? "AT+QSSLRECV=0,512" : "AT+QIRD=0,512";

    /* Read data in chunks via AT+QIRD / AT+QSSLRECV */
    const char *resp_prefix = ssl ? "+QSSLRECV:" : "+QIRD:";
    uint8_t prefix_len = ssl ? 10 : 6;

    while ((HAL_GetTick() - t) < 120000U) {
        AT_RxFlush();
        AT_Send(read_cmd);
        HAL_Delay(100);

        /* Parse response: +QIRD: <len>\r\n<data> or +QSSLRECV: <len>\r\n<data> */
        char rbuf[40] = {0};
        uint16_t ri = 0;
        uint32_t rt = HAL_GetTick();
        while ((HAL_GetTick() - rt) < 3000U && ri < 39) {
            if (at_tail != at_head) {
                rbuf[ri++] = (char)at_ring[at_tail];
                at_tail = (at_tail + 1U) % AT_RX_SIZE;
                if (strstr(rbuf, resp_prefix) && strchr(rbuf, '\n')) break;
            }
            HAL_Delay(1);
        }

        char *qird = strstr(rbuf, resp_prefix);
        uint16_t chunk_len = qird ? (uint16_t)atoi(qird + prefix_len) : 0;

        if (chunk_len == 0) {
            /* No more data — check if connection closed */
            HAL_Delay(1000);
            if (in_body && written > 0) break;  /* Done */
            if ((HAL_GetTick() - t) > 30000U) break;  /* Timeout */
            continue;
        }

        /* Read chunk_len data bytes from ring buffer (follows +QIRD: <len>\r\n) */
        uint16_t got = 0;
        rt = HAL_GetTick();
        while (got < chunk_len && (HAL_GetTick() - rt) < 5000U) {
            if (at_tail != at_head) {
                uint8_t byte = at_ring[at_tail];
                at_tail = (at_tail + 1U) % AT_RX_SIZE;
                got++;

                if (!in_body) {
                    if (hi < sizeof(hdr_buf) - 1) hdr_buf[hi++] = (char)byte;
                    if (hi >= 4 && strstr(hdr_buf + (hi > 20 ? hi - 20 : 0), "\r\n\r\n")) {
                        in_body = true;
                        hdr_buf[hi] = '\0';
                        Debug_Printf("[QEC] HDR: %.150s\r\n", hdr_buf);
                        /* Parse Content-Length */
                        char *cl = strstr(hdr_buf, "Content-Length:");
                        if (!cl) cl = strstr(hdr_buf, "content-length:");
                        if (cl) { content_length = (uint32_t)atol(cl + 15); got_length = true; }
                        /* Parse HTTP status */
                        int status = 0;
                        char *http_s = strstr(hdr_buf, "HTTP/");
                        if (http_s) { char *sp = strchr(http_s, ' '); if (sp) status = atoi(sp+1); }
                        Debug_Printf("[QEC] HTTP %d len=%lu\r\n", status, content_length);
                        /* Redirect? */
                        if (status == 301 || status == 302) {
                            char *loc = strstr(hdr_buf, "Location:");
                            if (!loc) loc = strstr(hdr_buf, "location:");
                            if (loc) {
                                loc += 9; while (*loc == ' ') loc++;
                                char *end = strstr(loc, "\r\n");
                                if (end) *end = '\0';
                                Debug_Printf("[QEC] → %s\r\n", loc);
                                strncpy(redir_url, loc, sizeof(redir_url)-1);
                                AT_Cmd(close_cmd, "OK", 5000U);
                                cur_url = redir_url;
                                redirects++;
                                goto redirect_retry;
                            }
                            AT_Cmd(close_cmd, "OK", 5000U);
                            return GSM_ERR_HTTP_FAIL;
                        }
                        if (status != 200 && status != 206) {
                            Debug_Printf("[QEC] Bad status %d\r\n", status);
                            AT_Cmd(close_cmd, "OK", 5000U);
                            return GSM_ERR_HTTP_FAIL;
                        }
                    }
                } else {
                    /* Body byte → flash page */
                    page[pi++] = byte;
                    if (pi >= 256U) {
                        W25Q_Write(flash_addr + written, page, pi);
                        written += pi;
                        pi = 0;
                        t = HAL_GetTick();
                        if (progress_cb && (written % 4096U) < 256U)
                            progress_cb(written, content_length);
                    }
                }
            } else {
                HAL_Delay(1);
            }
        }
        /* Drain trailing OK from QIRD */
        AT_Wait("OK", 2000U);

        if (got_length && written >= content_length) break;
    }

    /* Flush partial page */
    if (pi > 0) {
        W25Q_Write(flash_addr + written, page, pi);
        written += pi;
    }

    AT_Cmd(close_cmd, "OK", 5000U);
    *out_written = written;
    if (progress_cb) progress_cb(written, got_length ? content_length : written);
    Debug_Printf("[QEC] Done: %lu bytes\r\n", written);
    return (written > 0 && (!got_length || written >= content_length))
           ? GSM_OK : GSM_ERR_HTTP_FAIL;
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
