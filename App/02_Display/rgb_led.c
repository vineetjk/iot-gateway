/**
 * rgb_led.c
 * =========================================================================
 * RGB LED driver with PWM brightness control.
 *   PB0 = TIM3_CH3 (Red)
 *   PB1 = TIM3_CH4 (Green)
 *   PB3 = TIM2_CH2 (Blue, needs JTAG partial remap)
 *
 * Common anode: PWM=255 → fully OFF, PWM=0 → fully ON (inverted).
 * =========================================================================
 */
#include "rgb_led.h"

static TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim2;
static uint8_t s_brightness = RGB_BRIGHTNESS_DEFAULT;

void RGB_Init(void)
{
    /* Enable clocks */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* PB3 is JTDO by default — disable JTAG (keep SWD) to use as TIM2_CH2 */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* PB0 (TIM3_CH3), PB1 (TIM3_CH4) — AF push-pull */
    GPIO_InitTypeDef g = {0};
    g.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &g);

    /* PB3 (TIM2_CH2) — needs partial remap */
    __HAL_AFIO_REMAP_TIM2_PARTIAL_1();  /* CH1=PA15, CH2=PB3, CH3=PA2, CH4=PA3 */
    g.Pin = GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB, &g);

    /* TIM3: PB0=CH3, PB1=CH4
     * 72MHz / 72 = 1MHz tick, period=255 → ~3.9kHz PWM (flicker-free) */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler     = 71;     /* 72MHz / (71+1) = 1MHz */
    htim3.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim3.Init.Period         = 255;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = 255;              /* start OFF (common anode: 255=off) */
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* TIM2: PB3=CH2 (partial remap 1) */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler     = 71;
    htim2.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim2.Init.Period         = 255;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    oc.Pulse = 255;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    RGB_OFF();
}

void RGB_SetPWM(uint8_t r, uint8_t g, uint8_t b)
{
    /* Common anode: invert — 0=full brightness, 255=off */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 255U - r);  /* Red */
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 255U - g);  /* Green */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 255U - b);  /* Blue */
}

void RGB_Set(uint8_t r, uint8_t g, uint8_t b)
{
    RGB_SetPWM(r ? s_brightness : 0,
               g ? s_brightness : 0,
               b ? s_brightness : 0);
}

void RGB_SetBrightness(uint8_t level) { s_brightness = level; }
uint8_t RGB_GetBrightness(void)       { return s_brightness; }
