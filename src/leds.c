/* Driver for the PWM dimmable LEDs */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <vf-badge.h>

#define LED_TIMER_RCC   RCC_LPTIM1
#define LED_TIMER       LPTIM1_BASE

#define PORT_LED    GPIOA
#define PIN_LED_L   GPIO7
#define PIN_LED_R   GPIO6

void
led_setup(void)
{
    uint32_t ccipr;

    /* Configure the right LED as a GPIO output. */
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED_R);
    gpio_set(PORT_LED, PIN_LED_R);

    /* Configure the left LED as a low-power timer for PWM. */
    gpio_mode_setup(PORT_LED, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_LED_L);
    gpio_set_af(PORT_LED, GPIO_AF1, PIN_LED_L);

    /* Switch the low-power timer to use the LSI oscillator */
    ccipr = RCC_CCIPR & ~(RCC_CCIPR_LPTIM1SEL_MASK << RCC_CCIPR_LPTIM1SEL_SHIFT);
    RCC_CCIPR = ccipr | (RCC_CCIPR_LPTIM1SEL_LSI << RCC_CCIPR_LPTIM1SEL_SHIFT);

    /* Setup the low-power timer for PWM output. */
    rcc_periph_clock_enable(LED_TIMER_RCC);
    LPTIM_IER(LED_TIMER) = 0;
    LPTIM_CR(LED_TIMER) = LPTIM_CR_ENABLE;
    LPTIM_ARR(LED_TIMER) = UINT8_MAX;
    LPTIM_CMP(LED_TIMER) = 1;
    LPTIM_CFGR(LED_TIMER) = LPTIM_CFGR_CKPOL_RISING | LPTIM_CFGR_WAVPOL | LPTIM_CFGR_PRESC_1 | LPTIM_CFGR_TRIGEN_SW;
    LPTIM_CR(LED_TIMER) = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
#if 0
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
#endif
} /* led_setup */

void
led_set_right(uint8_t value)
{
    if (value >= 0x80) {
        gpio_clear(PORT_LED, PIN_LED_R);
    } else {
        gpio_set(PORT_LED, PIN_LED_R);
    }
    //timer_set_oc_value(LED_TIMER, TIM_OC1, value);
} /* led_set_right */

void
led_set_left(uint8_t value)
{
    if (value) {
        LPTIM_CMP(LED_TIMER) = UINT8_MAX - value;
        LPTIM_CR(LED_TIMER) = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
    } else {
        LPTIM_CMP(LED_TIMER) = UINT8_MAX;
        LPTIM_CR(LED_TIMER) = LPTIM_CR_ENABLE | LPTIM_CR_SNGSTRT;
    }
} /* led_set_left */

