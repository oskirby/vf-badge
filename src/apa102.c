/* Driver for the APA102 serial controllable LED */
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include "vf-badge.h"

#define APA102_PORT         GPIOA
#define APA102_PIN_VREG     GPIO0
#define APA102_PIN_TX       GPIO2
#define APA102_PIN_CLK      GPIO4

/* Synchronous USART is not available on the TSSOP-20 package. */
//#define APA102_USART_RCC    RCC_USART2
//#define APA102_USART        USART2

/* Color calibration constants */
#define CAL_RED     (FIXED_ONE / 3)
#define CAL_GREEN   (FIXED_ONE / 4)
#define CAL_BLUE    (FIXED_ONE)

void
apa102_setup(void)
{
    /* Enable the 5V regulator */
    gpio_mode_setup(APA102_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, APA102_PIN_VREG);
    gpio_set(APA102_PORT, APA102_PIN_VREG);

#ifdef APA102_USART
    /* Enable clocks for the USART. */
    rcc_periph_clock_enable(APA102_USART_RCC);

    /* Setup GPIO pins for USART control */
    gpio_mode_setup(APA102_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, APA102_PIN_TX | APA102_PIN_CLK);
    gpio_set_af(APA102_PORT, GPIO_AF4, APA102_PIN_TX | APA102_PIN_CLK);

    /* Setup the USART for synchronous transmit */
    usart_set_baudrate(APA102_USART, 100000);
    USART_CR2(APA102_USART) = USART_CR2_MSBFIRST | USART_CR2_LBCL | USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA;
    USART_CR1(APA102_USART) = USART_CR1_TE | USART_CR1_UE;
#else
    /* Setup the clock and data pins as GPIO outputs for bitbashing the APA102. */
    gpio_mode_setup(APA102_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, APA102_PIN_TX | APA102_PIN_CLK);
    gpio_set(APA102_PORT, APA102_PIN_TX | APA102_PIN_CLK);
#endif
} /* apa102_setup */

#define APA102_NOPS() __asm volatile("nop")

#define APA102_BITBANG(_bit_) \
do { GPIO_BSRR(APA102_PORT) = (APA102_PIN_CLK << 16) | (APA102_PIN_TX << ((_bit_) ? 0 : 16)); \
    APA102_NOPS();\
    GPIO_BSRR(APA102_PORT) =  (APA102_PIN_CLK << 0); \
} while(0)

static void
apa102_write(const uint8_t *data, unsigned int len)
{
#ifdef APA102_USART
    while (len--) {
        usart_send_blocking(APA102_USART, *data++);
    } /* while */
#else
    while (len--) {
        uint8_t byte = *data++;
        APA102_BITBANG(byte & 0x80);
        APA102_BITBANG(byte & 0x40);
        APA102_BITBANG(byte & 0x20);
        APA102_BITBANG(byte & 0x10);
        APA102_BITBANG(byte & 0x08);
        APA102_BITBANG(byte & 0x04);
        APA102_BITBANG(byte & 0x02);
        APA102_BITBANG(byte & 0x01);
    } /* while */
    GPIO_BSRR(APA102_PORT) =  (APA102_PIN_TX << 16);
#endif
} /* apa102_write */

void
apa102_write_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t value)
{
    uint8_t data[] = {
        0x00, 0x00, 0x00, 0x00, 0x00, /* start frame */
        (value >> 3) | 0xE0,    /* pixel brightness */
#ifdef CAL_BLUE
        FIXED_MUL(blue, CAL_BLUE),
#else
        blue,
#endif
#ifdef CAL_GREEN
        FIXED_MUL(green, CAL_GREEN),
#else
        green,
#endif
#ifdef CAL_RED
        FIXED_MUL(red, CAL_RED),
#else
        red,
#endif
        0xff, /* end frame. */
    };
    if (value) {
        gpio_set(APA102_PORT, APA102_PIN_VREG);
    } else {
        gpio_clear(APA102_PORT, APA102_PIN_VREG);
    }
    apa102_write(data, sizeof(data));
} /* apa102_write_rgb */


void
apa102_write_hsv(unsigned int hue, uint8_t saturation, uint8_t value)
{
    /* Get sector and fraction of sector. */
    unsigned int sector = hue / 60;
    unsigned int fraction = (hue % 60) * (FIXED_ONE / 60);
    unsigned int p, q, t;
    unsigned int red, blue, green;

    /* Math stuff */
    p = FIXED_ONE - saturation;
    q = FIXED_ONE - FIXED_MUL(saturation, fraction);
    //t = FIXED_ONE - FIXED_MUL(saturation, (FIXED_ONE - fraction));
    t = FIXED_ONE + FIXED_ONE - saturation - q;

    /* Color sector determines the final result. */
    if (sector >= 6) {
        sector %= 6;
    }
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
