/**
 * rgb_led.h
 * =========================================================================
 * 4-pin common anode RGB LED driver — PWM dimming via TIM3/TIM2.
 *
 * Wiring:
 *   LED R pin → 270R → PB0  (TIM3_CH3)
 *   LED G pin → 270R → PB1  (TIM3_CH4)
 *   LED B pin → 270R → PB3  (TIM2_CH2, requires JTAG partial disable)
 *   LED VCC   → 3.3V  (common anode)
 * =========================================================================
 */
#ifndef RGB_LED_H
#define RGB_LED_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* Pin assignments */
#define LED_R_PORT   GPIOB
#define LED_R_PIN    GPIO_PIN_0

#define LED_G_PORT   GPIOB
#define LED_G_PIN    GPIO_PIN_1

#define LED_B_PORT   GPIOB
#define LED_B_PIN    GPIO_PIN_3

/* Default brightness (0-255). Lower = dimmer. */
#define RGB_BRIGHTNESS_DEFAULT  10U

void RGB_Init(void);
void RGB_Set(uint8_t r, uint8_t g, uint8_t b);         /* 0=off, 1=on (uses current brightness) */
void RGB_SetPWM(uint8_t r, uint8_t g, uint8_t b);      /* 0-255 per channel */
void RGB_SetBrightness(uint8_t level);                  /* 0-255 */
uint8_t RGB_GetBrightness(void);

/* Colour helpers */
#define RGB_OFF()     RGB_Set(0,0,0)
#define RGB_RED()     RGB_Set(1,0,0)
#define RGB_GREEN()   RGB_Set(0,1,0)
#define RGB_BLUE()    RGB_Set(0,0,1)
#define RGB_CYAN()    RGB_Set(0,1,1)
#define RGB_PURPLE()  RGB_Set(1,0,1)
#define RGB_YELLOW()  RGB_Set(1,1,0)
#define RGB_WHITE()   RGB_Set(1,1,1)

#endif /* RGB_LED_H */
