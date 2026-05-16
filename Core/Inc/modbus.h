/**
 * modbus.h
 * =========================================================================
 * Modbus RTU Master — USART1 (PA9/PA10), RS-485 via MAX485/SN75HVD.
 * DE/RE control pin: PA4.
 * =========================================================================
 */
#ifndef MODBUS_H
#define MODBUS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERR_TIMEOUT,
    MODBUS_ERR_CRC,
    MODBUS_ERR_EXCEPTION,
    MODBUS_ERR_LENGTH,
} ModbusResult_t;

void           Modbus_Init(UART_HandleTypeDef *huart,
                            uint16_t de_pin, GPIO_TypeDef *de_port);
ModbusResult_t Modbus_ReadHoldingRegisters(uint8_t  slave,
                                            uint16_t start_reg,
                                            uint16_t num_regs,
                                            uint16_t *out,
                                            uint32_t timeout_ms);
ModbusResult_t Modbus_ReadInputRegisters(uint8_t  slave,
                                          uint16_t start_reg,
                                          uint16_t num_regs,
                                          uint16_t *out,
                                          uint32_t timeout_ms);
#endif /* MODBUS_H */
