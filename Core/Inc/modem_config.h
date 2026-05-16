/**
 * modem_config.h
 * =========================================================================
 * Compile-time modem selection.
 * Only ONE modem driver is compiled into the firmware, saving ~2-4KB flash
 * and ~350 bytes RAM.
 *
 * Change MODEM_DRIVER to switch modems, then rebuild.
 * =========================================================================
 */
#ifndef MODEM_CONFIG_H
#define MODEM_CONFIG_H

/* Modem driver IDs */
#define MODEM_DRV_SIMCOM   0   /* SIMCom A7670C  — AT+CMQTT, AT+HTTP */
#define MODEM_DRV_QUECTEL  1   /* Quectel EC200U — AT+QMT, AT+QHTTP  */

/* ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
 * SELECT YOUR MODEM HERE
 * ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ */
#define MODEM_DRIVER  MODEM_DRV_QUECTEL

#endif /* MODEM_CONFIG_H */
