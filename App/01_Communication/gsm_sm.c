/**
 * gsm_sm.c
 * =========================================================================
 * Non-blocking GSM connection state machine.
 *
 * State sequence:
 *   RESET → RESET_HOLD (200ms) → BOOT_WAIT (RDY, 6s) →
 *   AT_PROBE (retry 10s) → ECHO_OFF → CMEE →
 *   SIM_CHECK → NET_SEND/NET_RECV/NET_PAUSE (90s total) →
 *   APN_SET → PDP_ACT → [PDP_CHECK] →
 *   MQTT_CLEANUP → MQTT_OPEN → MQTT_SUB →
 *   CONNECTED
 *
 * Modem-specific MQTT sequences are delegated to vtable sub-state
 * machines (sm_mqtt_cleanup_* / sm_mqtt_open_*), keeping this file
 * modem-agnostic for the MQTT portion.
 *
 * On any hard failure: ERROR state with timed retry.
 * =========================================================================
 */
#include "gsm_sm.h"
#include "modem_hal.h"
#include "modem_at.h"
#include "config.h"
#include "debug_cli.h"
#include <string.h>
#include <stdio.h>

/* ── Internal states ────────────────────────────────────────────── */
typedef enum {
    ST_IDLE = 0,
    ST_RESET,
    ST_RESET_HOLD,    /* hold RESET low 200ms */
    ST_BOOT_WAIT,     /* wait for "RDY" URC, up to 6s */
    ST_AT_PROBE,      /* AT → OK, retry every 1s for 10s */
    ST_ECHO_OFF,      /* ATE0 */
    ST_CMEE,          /* AT+CMEE=2 */
    ST_SIM_CHECK,     /* AT+CPIN? → READY */
    ST_NET_SEND,      /* send AT+CEREG? */
    ST_NET_RECV,      /* wait for +CEREG: with stat=1 or 5 */
    ST_NET_PAUSE,     /* 3s between CEREG retries */
    ST_APN_SET,       /* AT+CGDCONT / AT+QICSGP */
    ST_PDP_ACT,       /* AT+CGACT=1,1 / AT+QIACT=1 */
    ST_PDP_CHECK,     /* fallback if PDP_ACT errors */
    ST_MQTT_CLEANUP,  /* vtable: sm_mqtt_cleanup_begin/poll */
    ST_MQTT_OPEN,     /* vtable: sm_mqtt_open_begin/poll */
    ST_MQTT_SUB,      /* Subscribe to command topic */
    ST_CONNECTED,
    ST_ERROR,         /* waiting for retry timer */
} SmState_t;

/* ── State variables ────────────────────────────────────────────── */
static SmState_t s_state      = ST_IDLE;
static uint32_t  s_tick       = 0;   /* HAL_GetTick() at last state entry  */
static uint32_t  s_deadline   = 0;   /* absolute tick deadline for current wait */
static uint32_t  s_net_start  = 0;   /* tick when LTE reg sequence started  */
static bool      s_cmd_sent   = false;
static bool      s_pdp_retry  = false;
static uint8_t   s_sim_retries = 0;  /* SIM not-ready retry counter */

/* Transition to a new state (resets cmd_sent and state tick) */
static void go(SmState_t ns)
{
    s_state    = ns;
    s_tick     = HAL_GetTick();
    s_cmd_sent = false;
}

/* ── Public API ─────────────────────────────────────────────────── */

GsmSmStatus_t GSM_SM_GetStatus(void)
{
    switch (s_state) {
        case ST_IDLE:      return GSM_SM_IDLE;
        case ST_CONNECTED: return GSM_SM_CONNECTED;
        case ST_ERROR:     return GSM_SM_ERROR;
        default:           return GSM_SM_CONNECTING;
    }
}

bool GSM_SM_IsConnected(void) { return s_state == ST_CONNECTED; }

void GSM_SM_Kick(void)
{
    Debug_Print("[SM] GSM state machine starting\r\n");
    s_pdp_retry  = false;
    s_sim_retries = 0;
    Modem_MqttSetUp(false);
    go(ST_RESET);
}

void GSM_SM_SetDisconnected(void)
{
    if (s_state == ST_CONNECTED) {
        Debug_Print("[SM] Publish failed — reconnecting in 60s\r\n");
        Modem_MqttSetUp(false);
        go(ST_ERROR);
        s_deadline = HAL_GetTick() + 60000U;
    }
}

/* ── State machine ─────────────────────────────────────────────── */

GsmSmStatus_t GSM_SM_Process(void)
{
    const GatewayConfig_t *cfg = Config_Get();
    const ModemOps_t *ops = Modem_GetOps();
    uint32_t now = HAL_GetTick();
    int      r;
    char *cmd = at_work;

    switch (s_state) {

    /* ── Idle — waiting for Kick() ── */
    case ST_IDLE:
        break;

    /* ── Assert RESET ── */
    case ST_RESET:
        Modem_ResetAssert();
        s_deadline = now + 200U;
        go(ST_RESET_HOLD);
        break;

    /* ── Hold RESET for 200ms ── */
    case ST_RESET_HOLD:
        if ((int32_t)(now - s_deadline) >= 0) {
            Modem_ResetRelease();
            Debug_Print("[SM] Reset pulse done\r\n");
            AT_FlushAcc();
            s_deadline = now + 6000U;
            go(ST_BOOT_WAIT);
        }
        break;

    /* ── Wait for "RDY" URC (non-critical, 6s) ── */
    case ST_BOOT_WAIT:
        r = AT_Poll("RDY", s_deadline);
        if (r == 1) {
            Debug_Print("[SM] Got RDY from module\r\n");
        } else if (r != 0) {
            Debug_Print("[SM] No RDY — probing anyway\r\n");
        } else {
            break;  /* still waiting */
        }
        go(ST_AT_PROBE);   /* proceed either way */
        break;

    /* ── AT probe — retry every 1s, 500ms settle on first, 10s total ── */
    case ST_AT_PROBE:
        /* Small settle before first probe */
        if (!s_cmd_sent && (now - s_tick) < 500U) break;

        if (!s_cmd_sent) {
            AT_BeginCmd("AT");
            s_cmd_sent = true;
            s_deadline = now + 1000U;
        }
        r = AT_Poll("OK", s_deadline);
        if (r == 1) {
            Debug_Print("[SM] Module responding\r\n");
            go(ST_ECHO_OFF);
        } else if (r == -1) {
            if ((now - s_tick) < 10000U) {
                s_cmd_sent = false;  /* retry */
            } else {
                Debug_Print("[SM] Module not responding\r\n");
                go(ST_ERROR);
                s_deadline = now + 30000U;
            }
        }
        break;

    /* ── ATE0 (failure OK — just continue) ── */
    case ST_ECHO_OFF:
        if (!s_cmd_sent) {
            AT_BeginCmd("ATE0");
            s_cmd_sent = true;
            s_deadline = now + 1000U;
        }
        if (AT_Poll("OK", s_deadline) != 0) go(ST_CMEE);
        break;

    /* ── AT+CMEE=2 (failure OK) ── */
    case ST_CMEE:
        if (!s_cmd_sent) {
            AT_BeginCmd("AT+CMEE=2");
            s_cmd_sent = true;
            s_deadline = now + 1000U;
        }
        if (AT_Poll("OK", s_deadline) != 0) go(ST_SIM_CHECK);
        break;

    /* ── AT+CPIN? → wait for READY, retry up to 5x before giving up ── */
    case ST_SIM_CHECK:
        if (!s_cmd_sent) {
            AT_BeginCmd("AT+CPIN?");
            s_cmd_sent = true;
            s_deadline = now + 5000U;
        }
        r = AT_Poll("READY", s_deadline);
        if (r == 1) {
            Debug_Print("[SM] SIM ready\r\n");
            s_sim_retries = 0;
            s_net_start = now;
            s_state = ST_NET_SEND;
            s_tick  = now;
        } else if (r == -1) {
            if (s_sim_retries < 5) {
                s_sim_retries++;
                Debug_Printf("[SM] SIM not ready, retry %u/5\r\n", s_sim_retries);
                s_cmd_sent = false;  /* retry CPIN? without resetting module */
            } else {
                s_sim_retries = 0;
                Debug_Print("[SM] SIM failed — hard reset\r\n");
                Modem_MqttSetUp(false);
                go(ST_ERROR);
                s_deadline = now + 15000U;
            }
        }
        break;

    /* ── Send AT+CEREG? (or AT+CREG? as fallback) ── */
    case ST_NET_SEND:
        /* Try CEREG (LTE) for first 30s, then alternate with CREG (2G/3G) */
        if ((now - s_net_start) < 30000U)
            AT_BeginCmd("AT+CEREG?");
        else
            AT_BeginCmd(((now / 3000U) & 1U) ? "AT+CREG?" : "AT+CEREG?");
        s_deadline = now + 2000U;
        s_state    = ST_NET_RECV;
        s_tick     = now;
        s_cmd_sent = true;
        break;

    /* ── Wait for +CEREG: or +CREG: and check registration status ── */
    case ST_NET_RECV: {
        /* Accept either +CEREG: or +CREG: response */
        const char *acc = AT_GetAcc();
        /* Drain ring into accumulator */
        r = AT_Poll("+CEREG:", s_deadline);
        if (r != 1) r = AT_Poll("+CREG:", s_deadline);

        if (r == 1) {
            acc = AT_GetAcc();
            const char *col = strstr(acc, "REG:");
            if (col) {
                const char *comma = strchr(col, ',');
                if (comma) {
                    char stat = comma[1];
                    if (stat == '1' || stat == '5') {
                        Debug_Printf("[SM] Network registered (stat=%c)\r\n", stat);
                        go(ST_APN_SET);
                        break;
                    }
                }
            }
        }
        if (r != 0) {
            if ((now - s_net_start) < 90000U) {  /* 90s total */
                s_deadline = now + 3000U;
                s_state    = ST_NET_PAUSE;
                s_tick     = now;
                s_cmd_sent = false;
            } else {
                Debug_Print("[SM] Network registration timeout\r\n");
                go(ST_ERROR);
                s_deadline = now + 60000U;
            }
        }
        break;
    }

    /* ── 3s pause between registration retries ── */
    case ST_NET_PAUSE:
        if ((int32_t)(now - s_deadline) >= 0) {
            s_state    = ST_NET_SEND;
            s_tick     = now;
            s_cmd_sent = false;
        }
        break;

    /* ── APN set (modem-specific command from vtable) ── */
    case ST_APN_SET:
        if (!s_cmd_sent) {
            snprintf(cmd, AT_WORK_BUF_SIZE, ops->apn_cmd_fmt, cfg->apn);
            AT_BeginCmd(cmd);
            s_cmd_sent = true;
            s_deadline = now + 3000U;
        }
        if (AT_Poll("OK", s_deadline) != 0) go(ST_PDP_ACT);
        break;

    /* ── PDP activate (modem-specific timeout from vtable) ── */
    case ST_PDP_ACT:
        if (!s_cmd_sent) {
            AT_BeginCmd(ops->pdp_act_cmd);
            s_cmd_sent = true;
            s_deadline = now + ops->pdp_act_timeout_ms;
        }
        r = AT_Poll("OK", s_deadline);
        if (r == 1) {
            Debug_Print("[SM] PDP context active\r\n");
            s_pdp_retry = false;
            go(ST_MQTT_CLEANUP);
        } else if (r == -1) {
            if (!s_pdp_retry) {
                s_pdp_retry = true;
                go(ST_PDP_CHECK);
            } else {
                Debug_Print("[SM] PDP activation failed\r\n");
                go(ST_ERROR);
                s_deadline = now + 60000U;
            }
        }
        break;

    /* ── PDP verify (modem-specific expected response from vtable) ── */
    case ST_PDP_CHECK:
        if (!s_cmd_sent) {
            AT_BeginCmd(ops->pdp_check_cmd);
            s_cmd_sent = true;
            s_deadline = now + 3000U;
        }
        r = AT_Poll(ops->pdp_check_expect, s_deadline);
        if (r == 1) {
            Debug_Print("[SM] PDP already active\r\n");
            go(ST_MQTT_CLEANUP);
        } else if (r == -1) {
            Debug_Print("[SM] PDP check failed\r\n");
            go(ST_ERROR);
            s_deadline = now + 60000U;
        }
        break;

    /* ── MQTT cleanup — delegated to vtable sub-state machine ── */
    case ST_MQTT_CLEANUP:
        if (!s_cmd_sent) {
            ops->sm_mqtt_cleanup_begin();
            s_cmd_sent = true;
            s_deadline = now + 30000U;
        }
        r = ops->sm_mqtt_cleanup_poll(s_deadline);
        if (r == 1) {
            Debug_Print("[SM] MQTT cleanup done\r\n");
            go(ST_MQTT_OPEN);
        } else if (r == -1) {
            /* Cleanup errors are non-fatal, proceed anyway */
            Debug_Print("[SM] MQTT cleanup error (non-fatal)\r\n");
            go(ST_MQTT_OPEN);
        }
        break;

    /* ── MQTT open+connect — delegated to vtable sub-state machine ── */
    case ST_MQTT_OPEN:
        if (!s_cmd_sent) {
            ops->sm_mqtt_open_begin(cfg->broker, cfg->broker_port,
                                    cfg->mqtt_client_id,
                                    cfg->mqtt_user, cfg->mqtt_pass);
            s_cmd_sent = true;
            s_deadline = now + 90000U;
        }
        r = ops->sm_mqtt_open_poll(s_deadline);
        if (r == 1) {
            Debug_Print("[SM] MQTT connected!\r\n");
            Modem_MqttSetUp(true);
            go(ST_MQTT_SUB);
        } else if (r == -1) {
            Debug_Print("[SM] MQTT connect failed\r\n");
            go(ST_ERROR);
            s_deadline = now + 60000U;
        }
        break;

    /* ── Subscribe to command topic ─────── */
    case ST_MQTT_SUB: {
        char sub_topic[64];
        snprintf(sub_topic, sizeof(sub_topic),
                 "delta/gw/%04X/cmd", cfg->device_id);
        GsmResult_t sr = Modem_MqttSubscribe(sub_topic, 1);
        if (sr == GSM_OK)
            Debug_Printf("[SM] Subscribed: %s\r\n", sub_topic);
        else
            Debug_Print("[SM] Subscribe failed (non-fatal)\r\n");
        go(ST_CONNECTED);
        break;
    }

    /* ── Connected — nothing to do, publish happens in main loop ── */
    case ST_CONNECTED:
        break;

    /* ── Error — wait, then restart full sequence ── */
    case ST_ERROR:
        if ((int32_t)(now - s_deadline) >= 0) {
            Debug_Print("[SM] Retrying GSM connection\r\n");
            s_pdp_retry   = false;
            s_sim_retries = 0;
            Modem_MqttSetUp(false);
            go(ST_RESET);
        }
        break;
    }

    return GSM_SM_GetStatus();
}
