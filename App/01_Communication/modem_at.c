/**
 * modem_at.c
 * =========================================================================
 * Shared UART ring buffer and AT command primitives.
 * Used by all modem driver files (modem_simcom.c, modem_quectel.c, etc.)
 * =========================================================================
 */
#include "modem_at.h"
#include "debug_cli.h"
#include <string.h>
#include <stdio.h>

/* ── UART and ring buffer ────────────────────────────────────────── */
static UART_HandleTypeDef *s_huart;
static uint8_t             s_rxb;

volatile uint8_t  at_ring[AT_RX_SIZE];
volatile uint16_t at_head = 0, at_tail = 0;

char at_work[AT_WORK_BUF_SIZE];

void AT_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    HAL_UART_Receive_IT(s_huart, &s_rxb, 1U);
}

void AT_UART_RxCallback(UART_HandleTypeDef *h)
{
    if (h->Instance == s_huart->Instance) {
        uint16_t n = (at_head + 1U) % AT_RX_SIZE;
        if (n != at_tail) { at_ring[at_head] = s_rxb; at_head = n; }
        HAL_UART_Receive_IT(s_huart, &s_rxb, 1U);
    }
}

/* ── Blocking primitives ─────────────────────────────────────────── */

void AT_RxFlush(void) { at_tail = at_head; }

void AT_Send(const char *cmd)
{
    HAL_UART_Transmit(s_huart, (uint8_t*)cmd, strlen(cmd), 1000);
    HAL_UART_Transmit(s_huart, (uint8_t*)"\r\n", 2, 100);
    Debug_Printf("[A7>] %s\r\n", cmd);
}

void AT_RawTx(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(s_huart, (uint8_t*)data, len, 3000U);
}

bool AT_Wait(const char *exp, uint32_t tmo)
{
    static char buf[AT_RX_SIZE];
    uint16_t idx = 0;
    uint32_t t   = HAL_GetTick();
    memset(buf, 0, sizeof(buf));

    while ((HAL_GetTick() - t) < tmo) {
        while (at_tail != at_head) {
            uint8_t b = at_ring[at_tail];
            at_tail = (at_tail + 1U) % AT_RX_SIZE;
            if (idx < sizeof(buf) - 1U) buf[idx++] = (char)b;
            if (strstr(buf, exp))           { Debug_Printf("[A7<] found: %s\r\n", exp); return true; }
            if (strstr(buf, "\r\nERROR\r\n") ||
                strstr(buf, "+CME ERROR")    ||
                strstr(buf, "+CMS ERROR"))   { Debug_Print("[A7<] ERROR\r\n"); return false; }
        }
        HAL_Delay(2U);
    }
    Debug_Printf("[A7] TIMEOUT: %s\r\n", exp);
    return false;
}

bool AT_Cmd(const char *cmd, const char *exp, uint32_t tmo)
{
    AT_RxFlush();
    AT_Send(cmd);
    return AT_Wait(exp, tmo);
}

bool AT_SendAfterPrompt(const uint8_t *data, uint16_t len,
                          const char *expect, uint32_t tmo)
{
    if (!AT_Wait(">", 5000U)) { Debug_Print("[AT] No > prompt\r\n"); return false; }
    HAL_Delay(50);
    AT_RawTx(data, len);
    HAL_Delay(50);
    return AT_Wait(expect, tmo);
}

/* ── Ring buffer peek/consume (for URC scanning) ─────────────────── */

uint16_t AT_RingPeek(char *buf, uint16_t buf_sz)
{
    uint16_t i = 0;
    uint16_t t = at_tail;
    while (t != at_head && i < buf_sz - 1U) {
        buf[i++] = (char)at_ring[t];
        t = (t + 1U) % AT_RX_SIZE;
    }
    buf[i] = '\0';
    return i;
}

void AT_RingConsume(uint16_t count)
{
    at_tail = (at_tail + count) % AT_RX_SIZE;
}

/* ── Non-blocking accumulator (for gsm_sm.c) ────────────────────── */

static char     s_acc[512];
static uint16_t s_acc_idx = 0;

void AT_FlushAcc(void)
{
    at_tail   = at_head;
    s_acc_idx = 0;
    s_acc[0]  = '\0';
}

void AT_BeginCmd(const char *cmd)
{
    AT_FlushAcc();
    AT_Send(cmd);
}

int AT_Poll(const char *expect, uint32_t deadline_tick)
{
    while (at_tail != at_head) {
        uint8_t b = at_ring[at_tail];
        at_tail = (at_tail + 1U) % AT_RX_SIZE;
        if (s_acc_idx < sizeof(s_acc) - 1U) {
            s_acc[s_acc_idx++] = (char)b;
            s_acc[s_acc_idx]   = '\0';
        }
    }

    if (strstr(s_acc, expect))
        return 1;
    if (strstr(s_acc, "\r\nERROR\r\n") ||
        strstr(s_acc, "+CME ERROR")    ||
        strstr(s_acc, "+CMS ERROR"))
        return -1;
    if ((int32_t)(HAL_GetTick() - deadline_tick) >= 0)
        return -1;
    return 0;
}

const char *AT_GetAcc(void) { return s_acc; }
