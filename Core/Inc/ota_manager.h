/**
 * ota_manager.h
 * =========================================================================
 * OTA manager — runs inside the APPLICATION.
 * Downloads firmware via A7670C HTTP into W25Q64 slot A,
 * writes metadata, sets SRAM flag, reboots into bootloader.
 * =========================================================================
 */
#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include "bootloader.h"
#include "w25q_spi.h"
#include "config.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    OTA_RESULT_UP_TO_DATE = 0,
    OTA_RESULT_UPDATED,
    OTA_RESULT_NO_SERVER,
    OTA_RESULT_DOWNLOAD_FAIL,
    OTA_RESULT_VERIFY_FAIL,
    OTA_RESULT_WRITE_FAIL,
} OtaResult_t;

void        OTA_Init(SPI_HandleTypeDef *hspi,
                     GPIO_TypeDef *cs_port, uint16_t cs_pin);
OtaResult_t OTA_CheckAndUpdate(void);
bool        OTA_GetServerVersion(uint32_t *version_out);
OtaResult_t OTA_HandleMqttCommand(const char *payload);
OtaResult_t OTA_HandleMqttCsv(const char *url, const char *size_s,
                                const char *crc_s, const char *ver_s);

#endif /* OTA_MANAGER_H */
