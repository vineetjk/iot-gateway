/**
 * gsm_sm.h
 * =========================================================================
 * Non-blocking GSM connection state machine.
 *
 * Drives the full sequence (reset → boot → AT probe → SIM check →
 * LTE registration → PDP activation → MQTT connect) without using
 * HAL_Delay or any blocking loops.
 *
 * Call GSM_SM_Kick() once to start, then call GSM_SM_Process() every
 * main-loop iteration. Read GSM_SM_IsConnected() to know when MQTT is up.
 * On publish failure, call GSM_SM_SetDisconnected() to trigger reconnect.
 * =========================================================================
 */
#ifndef GSM_SM_H
#define GSM_SM_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>

typedef enum {
    GSM_SM_IDLE = 0,    /* not started */
    GSM_SM_CONNECTING,  /* working through init sequence */
    GSM_SM_CONNECTED,   /* MQTT up, ready to publish */
    GSM_SM_ERROR,       /* failed, waiting for retry timer */
} GsmSmStatus_t;

void          GSM_SM_Kick(void);             /* start / restart connection */
GsmSmStatus_t GSM_SM_Process(void);          /* call every main loop tick   */
bool          GSM_SM_IsConnected(void);
GsmSmStatus_t GSM_SM_GetStatus(void);
void          GSM_SM_SetDisconnected(void);  /* call when publish fails     */

#endif /* GSM_SM_H */
