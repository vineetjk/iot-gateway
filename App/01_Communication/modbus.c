/**
 * modbus.c
 * =========================================================================
 * Modbus RTU Master implementation.
 * =========================================================================
 */
#include "modbus.h"
#include "debug_cli.h"
#include <string.h>

static UART_HandleTypeDef *s_huart;
static uint16_t            s_de_pin;
static GPIO_TypeDef       *s_de_port;

static uint16_t crc16(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= *buf++;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

/* RTS/DE: HIGH = transmit enable (standard MAX485 logic) */
/* If your board inverts this, swap SET/RESET below */
static void rs485_tx(void) { HAL_GPIO_WritePin(s_de_port,s_de_pin,GPIO_PIN_SET); }
static void rs485_rx(void) { HAL_GPIO_WritePin(s_de_port,s_de_pin,GPIO_PIN_RESET); }

void Modbus_Init(UART_HandleTypeDef *h, uint16_t de_pin, GPIO_TypeDef *de_port)
{
    s_huart   = h;
    s_de_pin  = de_pin;
    s_de_port = de_port;
    rs485_rx();
}

static ModbusResult_t modbus_rw(uint8_t slave, uint8_t fc,
                                 uint16_t start, uint16_t qty,
                                 uint16_t *out, uint32_t tmo)
{
    uint8_t req[8];
    req[0]=(uint8_t)slave; req[1]=fc;
    req[2]=(start>>8)&0xFF; req[3]=start&0xFF;
    req[4]=(qty>>8)&0xFF;   req[5]=qty&0xFF;
    uint16_t c=crc16(req,6);
    req[6]=c&0xFF; req[7]=(c>>8)&0xFF;

    rs485_tx();
    HAL_Delay(1);  /* DE settle time */
    HAL_StatusTypeDef tx_st = HAL_UART_Transmit(s_huart, req, 8, tmo);
    HAL_Delay(2);  /* guard time for last byte to shift out */
    rs485_rx();

    Debug_Printf("[MB] TX[%02X %02X %02X %02X %02X %02X %02X %02X] st=%d\r\n",
                 req[0],req[1],req[2],req[3],req[4],req[5],req[6],req[7],(int)tx_st);

    uint16_t expected = 5U + qty * 2U;
    uint8_t  resp[256];
    memset(resp, 0, sizeof(resp));

    if (HAL_UART_Receive(s_huart, resp, expected, tmo) == HAL_TIMEOUT)
        return MODBUS_ERR_TIMEOUT;

    if (resp[1] == (fc | 0x80)) return MODBUS_ERR_EXCEPTION;
    if (resp[2] != (uint8_t)(qty * 2)) return MODBUS_ERR_LENGTH;

    uint16_t rc = (uint16_t)(resp[expected-1]<<8) | resp[expected-2];
    if (rc != crc16(resp, expected-2)) return MODBUS_ERR_CRC;

    for (uint16_t i = 0; i < qty; i++)
        out[i] = ((uint16_t)resp[3+i*2]<<8) | resp[4+i*2];

    return MODBUS_OK;
}

ModbusResult_t Modbus_ReadHoldingRegisters(uint8_t slave, uint16_t start,
                                            uint16_t qty, uint16_t *out,
                                            uint32_t tmo)
{ return modbus_rw(slave, 0x03, start, qty, out, tmo); }

ModbusResult_t Modbus_ReadInputRegisters(uint8_t slave, uint16_t start,
                                          uint16_t qty, uint16_t *out,
                                          uint32_t tmo)
{ return modbus_rw(slave, 0x04, start, qty, out, tmo); }
