/**
 * modem_at.h
 * =========================================================================
 * Shared AT primitives and UART ring buffer for all modem drivers.
 * =========================================================================
 */
#ifndef MODEM_AT_H
#define MODEM_AT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ── Init / ISR ──────────────────────────────────────────────────── */
void AT_Init(UART_HandleTypeDef *huart);
void AT_UART_RxCallback(UART_HandleTypeDef *huart);

/* ── Blocking AT primitives ──────────────────────────────────────── */
void AT_RxFlush(void);
void AT_Send(const char *cmd);                          /* send + \r\n */
void AT_RawTx(const uint8_t *data, uint16_t len);       /* raw bytes   */
bool AT_Wait(const char *expect, uint32_t timeout_ms);
bool AT_Cmd(const char *cmd, const char *expect, uint32_t timeout_ms);
bool AT_SendAfterPrompt(const uint8_t *data, uint16_t len,
                         const char *expect, uint32_t timeout_ms);

/* ── Non-blocking AT primitives (for gsm_sm.c) ──────────────────── */
void        AT_FlushAcc(void);
void        AT_BeginCmd(const char *cmd);
int         AT_Poll(const char *expect, uint32_t deadline_tick);
const char *AT_GetAcc(void);

/* ── Ring buffer access (for URC scanning in modem drivers) ─────── */
uint16_t AT_RingPeek(char *buf, uint16_t buf_sz);  /* peek without consuming */
void     AT_RingConsume(uint16_t count);             /* advance tail */

/* ── Direct ring access for low-level drivers ──────────────────── */
#define AT_RX_SIZE 2048U
extern volatile uint8_t  at_ring[];
extern volatile uint16_t at_head, at_tail;

/* ── Shared work buffer (single-threaded, reusable by all drivers) ── */
#define AT_WORK_BUF_SIZE 256U
extern char at_work[];   /* shared cmd/hdr scratch buffer */

#endif /* MODEM_AT_H */
