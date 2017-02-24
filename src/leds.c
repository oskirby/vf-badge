/* Driver for the PWM dimmable LEDs */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <vf-badge.h>

#define LED_TIMER_RCC   RCC_TIM3
#define LED_TIMER       TIM3

#define PORT_LED    GPIOA
#define PIN_LED_L   GPIO7
#define PIN_LED_R   GPIO6

void
led_setup(void)
{
    /* Configure LED pins for timer output. */
    gpio_mode_setup(PORT_LED, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_LED_L | PIN_LED_R);
    gpio_set_af(PORT_LED, GPIO_AF1, PIN_LED_L | PIN_LED_R);

    /* Enable the timer clocks */
    rcc_periph_clock_enable(LED_TIMER_RCC);
    timer_reset(LED_TIMER);
    timer_set_mode(LED_TIMER, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_CENTER_1, TIM_CR1_DIR_UP);
    timer_set_period(LED_TIMER, UINT8_MAX+1);

    /* Configure channel 1 for center-aligned PWM */
    timer_set_oc_mode(LED_TIMER, TIM_OC1, TIM_OCM_PWM2);
    timer_set_oc_polarity_high(LED_TIMER, TIM_OC1);
    timer_enable_oc_output(LED_TIMER, TIM_OC1);
    timer_set_oc_value(LED_TIMER, TIM_OC1, 0);
    timer_enable_counter(LED_TIMER);

    /* Configure channel 2 for center-aligned PWM */
    timer_set_oc_mode(LED_TIMER, TIM_OC2, TIM_OCM_PWM2);
    timer_set_oc_polarity_high(LED_TIMER, TIM_OC2);
    timer_enable_oc_output(LED_TIMER, TIM_OC2);
    timer_set_oc_value(LED_TIMER, TIM_OC2, 0);

    /* Start the timer. */
    timer_enable_counter(LED_TIMER);
} /* led_setup */

void
led_set_right(uint8_t value)
{
    timer_set_oc_value(LED_TIMER, TIM_OC1, value);
} /* led_set_right */

void
led_set_left(uint8_t value)
{
    timer_set_oc_value(LED_TIMER, TIM_OC2, value);
} /* led_set_left */

