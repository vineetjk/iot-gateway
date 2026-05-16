/**
 * w25q_spi.c
 * =========================================================================
 * W25Q64 (8MB) SPI flash driver implementation.
 * =========================================================================
 */
#include "w25q_spi.h"
#include <string.h>

#define CMD_WREN    0x06
#define CMD_READ    0x03
#define CMD_PP      0x02   /* page program   */
#define CMD_SE      0x20   /* sector erase   */
#define CMD_JEDEC   0x9F
#define CMD_RDSR1   0x05
#define CMD_RELPD   0xAB   /* release power-down */
#define STATUS_WIP  0x01

static SPI_HandleTypeDef *s_spi;
static GPIO_TypeDef       *s_cs_port;
static uint16_t            s_cs_pin;

static void cs_low(void)  { HAL_GPIO_WritePin(s_cs_port,s_cs_pin,GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(s_cs_port,s_cs_pin,GPIO_PIN_SET); }
static void spi_tx(const uint8_t *d, uint16_t n) { HAL_SPI_Transmit(s_spi,(uint8_t*)d,n,100); }
static void spi_rx(uint8_t *d, uint16_t n)       { HAL_SPI_Receive(s_spi,d,n,100); }

void W25Q_Init(SPI_HandleTypeDef *h, GPIO_TypeDef *cp, uint16_t cn)
{
    s_spi=h; s_cs_port=cp; s_cs_pin=cn;
    cs_high(); HAL_Delay(5);
    cs_low(); uint8_t c=CMD_RELPD; spi_tx(&c,1); cs_high(); HAL_Delay(1);
}

void W25Q_ReadID(uint32_t *id)
{
    uint8_t c=CMD_JEDEC, b[3]={0};
    cs_low(); spi_tx(&c,1); spi_rx(b,3); cs_high();
    *id=((uint32_t)b[0]<<16)|((uint32_t)b[1]<<8)|b[2];
}

bool W25Q_Detect(void)
{
    uint32_t id; W25Q_ReadID(&id);
    uint8_t mfr = (id >> 16) & 0xFF;
    uint8_t cap = id & 0xFF;
    /* Accept Winbond (0xEF), XMC (0x20), GigaDevice (0xC8) — all W25Q64-compatible */
    return (mfr == 0xEF || mfr == 0x20 || mfr == 0xC8) && cap >= 0x16;
}

void W25Q_WaitBusy(void)
{
    uint8_t c=CMD_RDSR1,s;
    do { cs_low(); spi_tx(&c,1); spi_rx(&s,1); cs_high(); HAL_Delay(1); }
    while (s & STATUS_WIP);
}

static void wren(void) { uint8_t c=CMD_WREN; cs_low(); spi_tx(&c,1); cs_high(); }

void W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t c[4]={CMD_READ,(addr>>16)&0xFF,(addr>>8)&0xFF,addr&0xFF};
    cs_low(); spi_tx(c,4); spi_rx(buf,(uint16_t)len); cs_high();
}

HAL_StatusTypeDef W25Q_EraseSector(uint32_t addr)
{
    W25Q_WaitBusy(); wren();
    uint8_t c[4]={CMD_SE,(addr>>16)&0xFF,(addr>>8)&0xFF,addr&0xFF};
    cs_low(); spi_tx(c,4); cs_high();
    W25Q_WaitBusy();
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_EraseRange(uint32_t addr, uint32_t len)
{
    uint32_t end=addr+len;
    uint32_t s=addr&~(W25Q_SECTOR_SIZE-1U);
    while(s<end){ if(W25Q_EraseSector(s)!=HAL_OK) return HAL_ERROR; s+=W25Q_SECTOR_SIZE; }
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_WritePage(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    W25Q_WaitBusy(); wren();
    uint8_t c[4]={CMD_PP,(addr>>16)&0xFF,(addr>>8)&0xFF,addr&0xFF};
    cs_low(); spi_tx(c,4); spi_tx(buf,len); cs_high();
    W25Q_WaitBusy();
    return HAL_OK;
}

HAL_StatusTypeDef W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len)
{
    uint32_t written=0;
    while(written<len){
        uint32_t off=( addr+written)%W25Q_PAGE_SIZE;
        uint32_t chunk=W25Q_PAGE_SIZE-off;
        if(chunk>(len-written)) chunk=len-written;
        if(W25Q_WritePage(addr+written,buf+written,(uint16_t)chunk)!=HAL_OK)
            return HAL_ERROR;
        written+=chunk;
    }
    return HAL_OK;
}
