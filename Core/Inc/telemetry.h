/**
 * telemetry.h
 * =========================================================================
 * Binary telemetry frame builder + W25Q64 offline ring-buffer logger.
 *
 * Frame format (little-endian, #pragma pack(1)):
 *   [magic:1][version:1][device_id:2][timestamp:4][seq:2][num_regs:1]
 *   [regs: num_regs × 2 bytes][crc16:2]
 *   Fixed for 100 regs: 1+1+2+4+2+1+200+2 = 213 bytes
 *
 * Delta frame format:
 *   [0xAC:1][device_id:2][timestamp:4][seq:2][count:1]
 *   [count × (reg_index:1 + value:2)][crc16:2]
 *
 * Log region: W25Q64 0x045000–0x800000 (~7 MB)
 *   Stores frames in a ring buffer. Flushed to MQTT when GSM is up.
 * =========================================================================
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "config.h"
#include "bootloader.h"
#include "w25q_spi.h"
#include <stdint.h>
#include <stdbool.h>

#define TELEM_MAGIC      0xAB
#define TELEM_DELTA_MAGIC 0xAC
#define TELEM_VERSION    0x01
#define TELEM_FRAME_SIZE 213U   /* fixed for 100 uint16 registers */

/* ── Log region ── */
#define LOG_REGION_START SPI_FLASH_LOG_ADDR           /* 0x045000 */
#define LOG_REGION_SIZE  (7U*1024U*1024U - 0x45000U)
#define LOG_MAX_FRAMES   (LOG_REGION_SIZE / TELEM_FRAME_SIZE)
#define LOG_META_MAGIC   0x106600UL

#pragma pack(1)
typedef struct {
    uint8_t  magic;
    uint8_t  version;
    uint16_t device_id;
    uint32_t timestamp;
    uint16_t seq;
    uint8_t  num_regs;
    uint16_t regs[CONFIG_MAX_REGS];
    uint16_t crc16;
} TelemFrame_t;
#pragma pack()

void     Telem_Init(void);
void     Telem_BuildFrame(TelemFrame_t *f, const uint16_t *regs,
                           uint8_t num_regs, uint32_t timestamp);
uint16_t Telem_BuildDeltaFrame(uint8_t *out_buf, uint16_t out_size,
                                const uint16_t *current, const uint16_t *last,
                                uint8_t num_regs, uint32_t timestamp);

/* Offline log */
void     SpiLog_Write(const uint8_t *frame, uint16_t len);
uint32_t SpiLog_Pending(void);
bool     SpiLog_ReadNext(uint8_t *frame_buf, uint16_t *len);
void     SpiLog_ConfirmRead(void);
void     SpiLog_Clear(void);

#endif /* TELEMETRY_H */
