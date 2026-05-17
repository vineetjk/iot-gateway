/**
 * debug_cli.h
 * =========================================================================
 * Serial CLI on USART3 (PB10=TX, PB11=RX) at 115200 8N1.
 *
 * Wire USART3 to a USB-TTL adapter. Use PuTTY/minicom with local echo ON.
 *
 * Call order:
 *   CLI_Init(&huart3);          — once, before Config_Init()
 *   CLI_RxCallback(&huart);     — from HAL_UART_RxCpltCallback()
 *   CLI_Process();              — every main loop iteration
 * =========================================================================
 */
#ifndef DEBUG_CLI_H
#define DEBUG_CLI_H

#include "stm32f1xx_hal.h"
#include "build_config.h"

#define CLI_RX_BUF_SIZE  256U
#define CLI_CMD_BUF_SIZE 128U

void CLI_Init(UART_HandleTypeDef *huart);
void CLI_Process(void);
void CLI_RxCallback(UART_HandleTypeDef *huart);

/* Log control — logs OFF by default, enable with "log on" CLI command */
extern volatile uint8_t g_log_enabled;

#if DEBUG_LOG_ENABLED
  void Debug_Print(const char *msg);
  void Debug_Printf(const char *fmt, ...);
#else
  /* Strip all debug prints in production — saves ~13KB flash (format strings) */
  #define Debug_Print(msg)       ((void)0)
  #define Debug_Printf(fmt, ...) ((void)0)
#endif

#endif /* DEBUG_CLI_H */
