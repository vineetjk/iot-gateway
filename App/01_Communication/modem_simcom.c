/**
 * modem_simcom.c
 * =========================================================================
 * SIMCom A7670C modem driver.
 * AT+CMQTT* for MQTT, AT+HTTP* for HTTP.
 * =========================================================================
 */
#include "modem_config.h"
#if MODEM_DRIVER == MODEM_DRV_SIMCOM

#include "modem_hal.h"
#include "modem_at.h"
#include "debug_cli.h"
#include <string.h>
#include <stdio.h>

/* ════════════════════════════════════════════════════════════════════
   MQTT
   ════════════════════════════════════════════════════════════════════ */

static GsmResult_t simcom_mqtt_connect(const char *broker, uint16_t port,
                                        const char *client_id,
                                        const char *user, const char *pass)
{
    char *cmd = at_work;
    AT_Cmd("AT+CMQTTDISC=0,60", "OK", 3000U);
    AT_Cmd("AT+CMQTTREL=0",     "OK", 3000U);
    AT_Cmd("AT+CMQTTSTOP",      "OK", 3000U);
    HAL_Delay(500U);

    if (!AT_Cmd("AT+CMQTTSTART", "OK", 10000U)) return GSM_ERR_MQTT_FAIL;
    HAL_Delay(500U);

    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+CMQTTACCQ=0,\"%s\",0", client_id);
    if (!AT_Cmd(cmd, "OK", 5000U)) return GSM_ERR_MQTT_FAIL;

    if (user && user[0])
        snprintf(cmd, AT_WORK_BUF_SIZE,
                 "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1,\"%s\",\"%s\"",
                 broker, port, user, pass);
    else
        snprintf(cmd, AT_WORK_BUF_SIZE,
                 "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1", broker, port);

    if (!AT_Cmd(cmd, "+CMQTTCONNECT: 0,0", 20000U)) return GSM_ERR_MQTT_FAIL;
    Debug_Print("[SIM] MQTT connected\r\n");
    return GSM_OK;
}

static GsmResult_t simcom_mqtt_publish(const char *topic,
                                        const uint8_t *payload, uint16_t len,
                                        uint8_t qos)
{
    char cmd[128];
    uint16_t tlen = (uint16_t)strlen(topic);

    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+CMQTTTOPIC=0,%u", tlen);
    AT_RxFlush(); AT_Send(cmd);
    if (!AT_SendAfterPrompt((const uint8_t*)topic, tlen, "OK", 5000U))
        return GSM_ERR_MQTT_FAIL;

    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+CMQTTPAYLOAD=0,%u", len);
    AT_RxFlush(); AT_Send(cmd);
    if (!AT_SendAfterPrompt(payload, len, "OK", 5000U))
        return GSM_ERR_MQTT_FAIL;

    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+CMQTTPUB=0,%u,60", qos);
    if (!AT_Cmd(cmd, "+CMQTTPUB: 0,0", 10000U))
        return GSM_ERR_MQTT_FAIL;

    return GSM_OK;
}

static GsmResult_t simcom_mqtt_disconnect(void)
{
    AT_Cmd("AT+CMQTTDISC=0,120", "+CMQTTDISC: 0,0", 15000U);
    AT_Cmd("AT+CMQTTREL=0", "OK", 3000U);
    AT_Cmd("AT+CMQTTSTOP",  "OK", 5000U);
    return GSM_OK;
}

static GsmResult_t simcom_mqtt_subscribe(const char *topic, uint8_t qos)
{
    char cmd[128];
    uint16_t tlen = (uint16_t)strlen(topic);
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+CMQTTSUB=0,%u,%u", tlen, qos);
    AT_RxFlush(); AT_Send(cmd);
    if (!AT_SendAfterPrompt((const uint8_t*)topic, tlen,
                             "+CMQTTSUB: 0,0", 10000U))
        return GSM_ERR_MQTT_FAIL;
    Debug_Printf("[SIM] Subscribed: %s\r\n", topic);
    return GSM_OK;
}

static bool simcom_mqtt_check_rx(char *topic_out, uint16_t topic_sz,
                                  char *payload_out, uint16_t payload_sz)
{
    static char ubuf[384];
    uint16_t ui = AT_RingPeek(ubuf, sizeof(ubuf));
    (void)ui;

    topic_out[0] = '\0';
    payload_out[0] = '\0';

    char *start = strstr(ubuf, "+CMQTTRXSTART:");
    char *end   = strstr(ubuf, "+CMQTTRXEND:");
    if (!start || !end || end <= start) return false;

    char *tp = strstr(start, "+CMQTTRXTOPIC:");
    if (tp) {
        char *nl = strchr(tp, '\n');
        if (nl) {
            nl++;
            uint16_t i = 0;
            while (*nl && *nl != '\r' && *nl != '\n' && i < topic_sz - 1U)
                topic_out[i++] = *nl++;
            topic_out[i] = '\0';
        }
    }

    char *pp = strstr(start, "+CMQTTRXPAYLOAD:");
    if (pp) {
        char *nl = strchr(pp, '\n');
        if (nl) {
            nl++;
            uint16_t i = 0;
            while (*nl && i < payload_sz - 1U) {
                if (nl[0] == '+' && strncmp(nl, "+CMQTTRXEND", 11) == 0) break;
                if (*nl != '\r' && *nl != '\n')
                    payload_out[i++] = *nl;
                nl++;
            }
            payload_out[i] = '\0';
        }
    }

    char *consume_end = end;
    while (*consume_end && *consume_end != '\n') consume_end++;
    if (*consume_end) consume_end++;
    AT_RingConsume((uint16_t)(consume_end - ubuf));
    return true;
}

/* SIMCom doesn't need async URC check like Quectel */
static void simcom_mqtt_check_urc(void) { }

/* ── Non-blocking SM sub-state machine for MQTT cleanup ──────────── */

static enum { SC_DISC, SC_REL, SC_STOP, SC_DONE } s_sc_step;
static bool     s_sc_sent;
static uint32_t s_sc_deadline;

static void simcom_sm_mqtt_cleanup_begin(void)
{
    s_sc_step = SC_DISC;
    s_sc_sent = false;
}

static int simcom_sm_mqtt_cleanup_poll(uint32_t deadline)
{
    (void)deadline;
    uint32_t now = HAL_GetTick();

    switch (s_sc_step) {
    case SC_DISC:
        if (!s_sc_sent) {
            AT_BeginCmd("AT+CMQTTDISC=0,60");
            s_sc_sent = true;
            s_sc_deadline = now + 5000U;
        }
        if (AT_Poll("OK", s_sc_deadline) != 0) {
            s_sc_step = SC_REL; s_sc_sent = false;
        }
        break;
    case SC_REL:
        if (!s_sc_sent) {
            AT_BeginCmd("AT+CMQTTREL=0");
            s_sc_sent = true;
            s_sc_deadline = now + 3000U;
        }
        if (AT_Poll("OK", s_sc_deadline) != 0) {
            s_sc_step = SC_STOP; s_sc_sent = false;
        }
        break;
    case SC_STOP:
        if (!s_sc_sent) {
            AT_BeginCmd("AT+CMQTTSTOP");
            s_sc_sent = true;
            s_sc_deadline = now + 5000U;
        }
        if (AT_Poll("OK", s_sc_deadline) != 0) {
            s_sc_step = SC_DONE;
            return 1;
        }
        break;
    case SC_DONE:
        return 1;
    }
    return 0;
}

/* ── Non-blocking SM sub-state machine for MQTT open+connect ─────── */

static enum { SO_SETTLE, SO_START, SO_ACCQ, SO_CONN, SO_DONE } s_so_step;
static bool     s_so_sent;
static uint32_t s_so_deadline;
static char     s_so_broker[64];
static uint16_t s_so_port;
static char     s_so_client[32];
static char     s_so_user[32];
static char     s_so_pass[32];

static void simcom_sm_mqtt_open_begin(const char *broker, uint16_t port,
                                       const char *client_id,
                                       const char *user, const char *pass)
{
    strncpy(s_so_broker, broker, 63);
    s_so_port = port;
    strncpy(s_so_client, client_id, 31);
    strncpy(s_so_user, user ? user : "", 31);
    strncpy(s_so_pass, pass ? pass : "", 31);
    s_so_step = SO_SETTLE;
    s_so_sent = false;
}

static int simcom_sm_mqtt_open_poll(uint32_t deadline)
{
    (void)deadline;
    uint32_t now = HAL_GetTick();
    char *cmd = at_work;
    int r;

    switch (s_so_step) {
    case SO_SETTLE:
        if (!s_so_sent) { s_so_sent = true; s_so_deadline = now + 500U; }
        if ((int32_t)(now - s_so_deadline) >= 0) {
            s_so_step = SO_START; s_so_sent = false;
        }
        break;
    case SO_START:
        if (!s_so_sent) {
            AT_BeginCmd("AT+CMQTTSTART");
            s_so_sent = true;
            s_so_deadline = now + 10000U;
        }
        r = AT_Poll("OK", s_so_deadline);
        if (r == 1) { s_so_step = SO_ACCQ; s_so_sent = false; }
        else if (r == -1) return -1;
        break;
    case SO_ACCQ:
        if (!s_so_sent) {
            snprintf(cmd, AT_WORK_BUF_SIZE, "AT+CMQTTACCQ=0,\"%s\",0", s_so_client);
            AT_BeginCmd(cmd);
            s_so_sent = true;
            s_so_deadline = now + 5000U;
        }
        r = AT_Poll("OK", s_so_deadline);
        if (r == 1) { s_so_step = SO_CONN; s_so_sent = false; }
        else if (r == -1) return -1;
        break;
    case SO_CONN:
        if (!s_so_sent) {
            if (s_so_user[0])
                snprintf(cmd, AT_WORK_BUF_SIZE,
                         "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1,\"%s\",\"%s\"",
                         s_so_broker, s_so_port, s_so_user, s_so_pass);
            else
                snprintf(cmd, AT_WORK_BUF_SIZE,
                         "AT+CMQTTCONNECT=0,\"tcp://%s:%u\",60,1",
                         s_so_broker, s_so_port);
            AT_BeginCmd(cmd);
            s_so_sent = true;
            s_so_deadline = now + 30000U;
        }
        r = AT_Poll("+CMQTTCONNECT: 0,0", s_so_deadline);
        if (r == 1) { s_so_step = SO_DONE; return 1; }
        if (r == -1) return -1;
        /* Check for rejection while still polling */
        {
            const char *acc = AT_GetAcc();
            if (strstr(acc, "+CMQTTCONNECT: 0,") &&
                !strstr(acc, "+CMQTTCONNECT: 0,0"))
                return -1;
        }
        break;
    case SO_DONE:
        return 1;
    }
    return 0;
}

/* ════════════════════════════════════════════════════════════════════
   HTTP (AT+HTTP*)
   ════════════════════════════════════════════════════════════════════ */

static uint32_t parse_http_action(void)
{
    char buf[128] = {0};
    uint16_t idx = 0;
    uint32_t t = HAL_GetTick();
    while ((HAL_GetTick() - t) < 30000U) {
        while (at_tail != at_head && idx < 127) {
            buf[idx++] = (char)at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
            char *p = strstr(buf, "+HTTPACTION:");
            if (p) {
                int meth = 0, status = 0; uint32_t len = 0;
                sscanf(p, "+HTTPACTION: %d,%d,%lu", &meth, &status, &len);
                Debug_Printf("[SIM] HTTP status=%d len=%lu\r\n", status, len);
                return (status == 200 || status == 206) ? len : 0;
            }
        }
        HAL_Delay(5U);
    }
    return 0;
}

static uint16_t read_http_body(uint8_t *out, uint16_t max_len)
{
    HAL_Delay(100U);
    uint16_t i = 0;
    bool in_data = false;
    uint8_t skip_nl = 0;
    uint32_t t = HAL_GetTick();
    while (i < max_len && (HAL_GetTick() - t) < 15000U) {
        while (at_tail != at_head && i < max_len) {
            uint8_t b = at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
            if (!in_data) { if (b == '\n' && ++skip_nl >= 1) in_data = true; }
            else out[i++] = b;
        }
        HAL_Delay(2U);
    }
    return i;
}

static GsmResult_t simcom_http_get(const char *url, char *out_buf,
                                    uint16_t buf_len, uint16_t *out_len)
{
    char *cmd = at_work;
    *out_len = 0;
    AT_Cmd("AT+HTTPTERM", "OK", 2000U); HAL_Delay(200U);
    if (!AT_Cmd("AT+HTTPINIT", "OK", 5000U)) return GSM_ERR_HTTP_FAIL;
    AT_Cmd("AT+HTTPPARA=\"CID\",1",       "OK", 2000U);
    AT_Cmd("AT+HTTPPARA=\"CONNECTTO\",30", "OK", 2000U);
    AT_Cmd("AT+HTTPPARA=\"RECVTO\",30",   "OK", 2000U);
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (!AT_Cmd(cmd, "OK", 3000U)) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    AT_RxFlush(); AT_Send("AT+HTTPACTION=0");
    uint32_t dlen = parse_http_action();
    if (!dlen) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    uint16_t rlen = (dlen < (uint32_t)(buf_len-1U)) ? (uint16_t)dlen : (buf_len-1U);
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+HTTPREAD=0,%u", rlen);
    AT_RxFlush(); AT_Send(cmd);
    if (!AT_Wait("+HTTPREAD:", 5000U)) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    *out_len = read_http_body((uint8_t*)out_buf, rlen);
    out_buf[*out_len] = '\0';
    AT_Cmd("AT+HTTPTERM", "OK", 2000U);
    Debug_Printf("[SIM] HTTP GET done: %u bytes\r\n", *out_len);
    return GSM_OK;
}

static GsmResult_t simcom_http_get_range(const char *url,
                                          uint32_t range_start, uint32_t range_end,
                                          uint8_t *out_buf, uint16_t *out_len)
{
    char *cmd = at_work;
    *out_len = 0;
    uint16_t want = (uint16_t)(range_end - range_start + 1U);

    AT_Cmd("AT+HTTPTERM", "OK", 2000U); HAL_Delay(200U);
    if (!AT_Cmd("AT+HTTPINIT", "OK", 5000U)) return GSM_ERR_HTTP_FAIL;
    AT_Cmd("AT+HTTPPARA=\"CID\",1",        "OK", 2000U);
    AT_Cmd("AT+HTTPPARA=\"CONNECTTO\",60",  "OK", 2000U);
    AT_Cmd("AT+HTTPPARA=\"RECVTO\",60",    "OK", 2000U);
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+HTTPPARA=\"URL\",\"%s\"", url);
    if (!AT_Cmd(cmd, "OK", 3000U)) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+HTTPPARA=\"USERDATA\",\"Range: bytes=%lu-%lu\"",
             range_start, range_end);
    if (!AT_Cmd(cmd, "OK", 3000U)) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    AT_RxFlush(); AT_Send("AT+HTTPACTION=0");
    uint32_t dlen = parse_http_action();
    if (!dlen) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    if ((uint32_t)want > dlen) want = (uint16_t)dlen;
    snprintf(cmd, AT_WORK_BUF_SIZE, "AT+HTTPREAD=0,%u", want);
    AT_RxFlush(); AT_Send(cmd);
    if (!AT_Wait("+HTTPREAD:", 5000U)) { AT_Cmd("AT+HTTPTERM","OK",2000U); return GSM_ERR_HTTP_FAIL; }

    *out_len = read_http_body(out_buf, want);
    AT_Cmd("AT+HTTPTERM", "OK", 2000U);
    return (*out_len == want) ? GSM_OK : GSM_ERR_HTTP_FAIL;
}

/* ════════════════════════════════════════════════════════════════════
   VTABLE
   ════════════════════════════════════════════════════════════════════ */

const ModemOps_t g_simcom_ops = {
    /* Hardware */
    .reset_assert_level   = GPIO_PIN_RESET,    /* active-LOW */

    /* Network */
    .apn_cmd_fmt          = "AT+CGDCONT=1,\"IP\",\"%s\"",
    .pdp_act_cmd          = "AT+CGACT=1,1",
    .pdp_act_timeout_ms   = 15000U,
    .pdp_check_cmd        = "AT+CGACT?",
    .pdp_check_expect     = "+CGACT: 1,1",

    /* MQTT */
    .mqtt_connect         = simcom_mqtt_connect,
    .mqtt_publish         = simcom_mqtt_publish,
    .mqtt_disconnect      = simcom_mqtt_disconnect,
    .mqtt_subscribe       = simcom_mqtt_subscribe,
    .mqtt_check_rx        = simcom_mqtt_check_rx,
    .mqtt_check_urc       = simcom_mqtt_check_urc,

    /* MQTT SM */
    .sm_mqtt_cleanup_begin = simcom_sm_mqtt_cleanup_begin,
    .sm_mqtt_cleanup_poll  = simcom_sm_mqtt_cleanup_poll,
    .sm_mqtt_open_begin    = simcom_sm_mqtt_open_begin,
    .sm_mqtt_open_poll     = simcom_sm_mqtt_open_poll,

    /* HTTP */
    .http_get             = simcom_http_get,
    .http_get_range       = simcom_http_get_range,

    /* Misc */
    .reboot_cmd           = "AT+CRESET",
};

#endif /* MODEM_DRIVER == MODEM_DRV_SIMCOM */
