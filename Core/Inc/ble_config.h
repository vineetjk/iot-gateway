/**
 * ble_config.h
 * =========================================================================
 * BLE Configuration Mode — allows field configuration via BLE GATT
 * using the EC200U module's built-in BLE stack (AT+QBT* commands).
 *
 * A technician holds a button (PA0) for 3 seconds to enter config mode.
 * A mobile app (nRF Connect) reads/writes gateway parameters via GATT.
 * =========================================================================
 */
#ifndef BLE_CONFIG_H
#define BLE_CONFIG_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/* Button pin */
#define BLE_BTN_PORT    GPIOA
#define BLE_BTN_PIN     GPIO_PIN_0

/* Timeout: exit config mode if no phone connects within 120s */
#define BLE_ADV_TIMEOUT_MS   120000U

/* ── Public API ──────────────────────────────────────────────────── */

/* Call once during init to set up button GPIO */
void BLE_Config_InitButton(void);

/* Enter config mode — pauses GSM, starts BLE GATT server */
void BLE_Config_Enter(void);

/* Call every main loop tick while in config mode */
void BLE_Config_Process(void);

/* Returns true when config mode is finished (timeout/disconnect/exit cmd) */
bool BLE_Config_IsDone(void);

/* Check button state — returns true if held >= 3 seconds */
bool BLE_Config_ButtonHeld(void);

#endif /* BLE_CONFIG_H */
