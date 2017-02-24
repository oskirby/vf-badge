/* Driver for the APA102 serial controllable LED */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "vf-badge.h"

#define APA102_USART_RCC    RCC_USART1
#define APA102_USART        USART1
#define APA102_PORT         GPIOA
#define APA102_PIN_VREG     GPIO0
#define APA102_PIN_TX       GPIO2
#define APA102_PIN_CLK      GPIO4

/* Bits used for fixed-point math */
#define FIXED_BITS  8
#define FIXED_ONE   (1u << FIXED_BITS)
#define FIXED_MAX   (FIXED_ONE - 1)

/* Color calibration constants */
/* Blue need some serious help since we're way under voltage spec.' */
#define CAL_RED     (FIXED_ONE / 8)
#define CAL_GREEN   (FIXED_ONE / 8)
#define CAL_BLUE    (FIXED_ONE)

void
apa102_setup(void)
{
    /* Enable clocks for USART1. */
    rcc_periph_clock_enable(APA102_USART_RCC);

    /* Enable the 5V regulator */
    gpio_mode_setup(APA102_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, APA102_PIN_VREG);
    gpio_set(APA102_PORT, APA102_PIN_VREG);

    /* Setup GPIO pins for USART1 */
    gpio_mode_setup(APA102_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, APA102_PIN_TX | APA102_PIN_CLK);
    gpio_set_af(APA102_PORT, GPIO_AF1, APA102_PIN_TX | APA102_PIN_CLK);

    /* Setup USART1 for synchronous transmit */
    usart_set_baudrate(APA102_USART, 400000);
    USART_CR2(APA102_USART) = USART_CR2_MSBFIRST | USART_CR2_LBCL | USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA;
    USART_CR1(APA102_USART) = USART_CR1_TE | USART_CR1_UE;
} /* apa102_setup */

void
apa102_write_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t value)
{
    /* Send the start frame. */
    usart_send_blocking(APA102_USART, 0);
    usart_send_blocking(APA102_USART, 0);
    usart_send_blocking(APA102_USART, 0);
    usart_send_blocking(APA102_USART, 0);

    /* Send the RGB value */
    usart_send_blocking(APA102_USART, (value >> 3) | 0xE0);
#if 1
    usart_send_blocking(APA102_USART, ((unsigned int)blue * CAL_BLUE) >> FIXED_BITS);
    usart_send_blocking(APA102_USART, ((unsigned int)green * CAL_GREEN) >> FIXED_BITS);
    usart_send_blocking(APA102_USART, ((unsigned int)red * CAL_RED) >> FIXED_BITS);
#else
    usart_send_blocking(APA102_USART, blue);
    usart_send_blocking(APA102_USART, green);
    usart_send_blocking(APA102_USART, red);
#endif

    /* Send the end frame. */
    usart_send_blocking(APA102_USART, 0xff);
    usart_send_blocking(APA102_USART, 0xff);
    usart_send_blocking(APA102_USART, 0xff);
    usart_send_blocking(APA102_USART, 0xff);
} /* apa102_write_rgb */


void
apa102_write_hsv(unsigned int hue, uint8_t saturation, uint8_t value)
{
    /* Get sector and fraction of sector. */
    unsigned int sector = hue / 60;
    unsigned int fraction = ((hue % 60) << FIXED_BITS) / 60;
    unsigned int p, q, t;
    unsigned int red, blue, green;

    /* Math stuff */
    p = FIXED_ONE - saturation;
    q = FIXED_ONE - (((unsigned int)saturation * fraction) >> FIXED_BITS);
    t = FIXED_ONE - (((unsigned int)saturation * (FIXED_ONE - fraction)) >> FIXED_BITS);

    /* Color sector determines the final result. */
    switch (sector) {
        case 0:
            red = FIXED_MAX;
            green = t;
            blue = p;
            break;
        case 1:
            red = q;
            green = FIXED_MAX;
            blue = p;
            break;
        case 2:
            red = p;
            green = FIXED_MAX;
            blue = t;
            break;
        case 3:
            red = p;
            green = q;
            blue = FIXED_MAX;
            break;
        case 4:
            red = t;
            green  = p;
            blue = FIXED_MAX;
            break;
        case 5:
        default:
            red = FIXED_MAX;
            green = p;
            blue = q;
            break;
    } /* switch */

    /* Output the RGB value to the LED. */
    if (red > FIXED_MAX) red = FIXED_MAX;
    if (green > FIXED_MAX) green  = FIXED_MAX;
    if (blue > FIXED_MAX) blue = FIXED_MAX;
    apa102_write_rgb(red, green, blue, value);
} /* apa102_write_hsv */

