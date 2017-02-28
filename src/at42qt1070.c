/* Driver for the AT42QT1070 Capacative touch controller. */
#include <string.h>
#include <stdlib.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

#include "vf-badge.h"

/*---------------------------------------------------------
 * AT42QT1070 Capacative Touch Controller Definitions
 *---------------------------------------------------------
 */
#define AT42_NUM_KEYS               7
#define AT42_I2C_ADDR               0x1b
#define AT42_I2C_SPEED              400000

/* AT42QT1070 Register map */
#define AT42_REG_CHIP_ID            0
#define AT42_REG_FIRMWARE_VER       1
#define AT42_REG_DETECT_STATUS      2
#define AT42_REG_KEY_STATUS         3
#define AT42_REG_KEY_SIGNAL(_x_)    (4 + 2*(_x_))
#define AT42_REG_KEY_SIGNAL_MSB(_x_) (AT42_REG_KEY_SIGNAL(_x_)+0)
#define AT42_REG_KEY_SIGNAL_LSB(_x_) (AT42_REG_KEY_SIGNAL(_x_)+1)
#define AT42_REG_REF_DATA(_x_)      (18 + 2*(_x_))
#define AT42_REG_REF_DATA_MSB(_x_)  (AT42_REG_REF_DATA(_x_)+0)
#define AT42_REG_REF_DATA_LSB(_x_)  (AT42_REG_REF_DATA(_x_)+1)
#define AT42_REG_NTHR_KEY(_x_)      (32 + (_x_))
#define AT42_REG_AVE_ASK_KEY(_x_)   (39 + (_x_))
#define AT42_REG_DI_KEY(_x_)        (46 + (_x_))
#define AT42_REG_CAL_GUARD          53
#define AT42_REG_LOW_POWER          54
#define AT42_REG_DURATION           55
#define AT42_REG_CALIBRATE          56
#define AT42_REG_RESET              57

/* Address 0: Chip ID Register */
#define AT42QT1070_CHIP_ID          0x2e

/* Address 2: Detection Status */
#define AT42_DETECT_STATUS_CALIBRATE    (1<<7)
#define AT42_DETECT_STATUS_OVERFLOW     (1<<6)
#define AT42_DETECT_STATUS_TOUCH        (1<<0)

/* Address 3: Key Status */
#define AT42_KEY_STATUS_KEY(_x_)        (1<<(_x_))
#define AT42_KEY_STATUS_KEY_MASK        ((1<<AT42_NUM_KEYS))-1)

/* Address 39-45: Averaging Factor/Adjacent Key Suppression */
#define AT42_AVE_SHIFT                  4
#define AT42_AVE_MASK                   (0xf << AT42_AVE_SHIFT)
#define AT42_AVE_VALUE(_x_)             ((_x_) << AT42_AVE_SHIFT)
#define AT42_AVE_DEFAULT                AT42_AVE_VALUE(8)
#define AT42_AKS_SHIFT                  0
#define AT42_AKS_MASK                   (0xf << AT42_AKS_SHIFT)
#define AT42_ASK_VALUE(_x_)             ((_x_) << AT42_AKS_SHIFT)

/* Address 53: FastOutDI/Max Cal/Guard Channel */
#define AT42_FAST_OUT                   (1 << 5)
#define AT42_MAX_CAL                    (1 << 4)
#define AT42_GUARD_CHANNEL_SHIFT        0
#define AT42_GUARD_CHANNEL_MASK         (0xf << AT42_GUARD_CHANNEL_SHIFT)
#define AT42_GUARD_CHANNEL_VALUE(_ch_)  ((_ch_) << AT42_GUARD_CHANNEL_SHIFT)
#define AT42_GUARD_CHANNEL_DISABLE      (AT42_NUM_KEYS << AT42_GUARD_CHANNEL_SHIFT)

/*---------------------------------------------------------
 * Vancoufur Badge Setup and Calibration
 *---------------------------------------------------------
 */
#define AT42_PORT_I2C   GPIOA
#define AT42_I2C        I2C1_BASE
#define AT42_PIN_SCL    GPIO9
#define AT42_PIN_SDA    GPIO10

#define AT42_PORT_CHANGE    GPIOB
#define AT42_PIN_CHANGE     GPIO1

/* Keep a cache of the first four registers. */
static uint8_t at42_regs[4] = {0, 0, 0, 0};

static int
at42_writeb(uint8_t reg, uint8_t val)
{
    return write_i2c(AT42_I2C, AT42_I2C_ADDR, reg, 1, &val);
}

static int
at42_write(uint8_t reg, const uint8_t *data, unsigned int len)
{
    return write_i2c(AT42_I2C, AT42_I2C_ADDR, reg, len, (uint8_t *)data);
} /* at42_write */

static int
at42_read(uint8_t reg, uint8_t *data, unsigned int len)
{
    return read_i2c(AT42_I2C, AT42_I2C_ADDR, reg, len, data);
} /* at42_read */

void
at42_setup(void)
{
    uint32_t ccipr;

    /* Configure I2C pins. */
    gpio_mode_setup(AT42_PORT_I2C, GPIO_MODE_AF, GPIO_PUPD_PULLUP, AT42_PIN_SCL | AT42_PIN_SDA);
    gpio_set_output_options(AT42_PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, AT42_PIN_SCL | AT42_PIN_SDA);
    gpio_set_af(AT42_PORT_I2C, GPIO_AF1, AT42_PIN_SCL | AT42_PIN_SDA);

    /* Setup the I2C clocks. */
    rcc_periph_clock_enable(RCC_I2C1);
    ccipr = RCC_CCIPR & ~(RCC_CCIPR_I2C1SEL_MASK << RCC_CCIPR_I2C1SEL_SHIFT);
    RCC_CCIPR = ccipr | (RCC_CCIPR_I2C1SEL_SYS << RCC_CCIPR_I2C1SEL_SHIFT);

    i2c_reset(AT42_I2C);

    /* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
    i2c_peripheral_disable(AT42_I2C);
    i2c_enable_analog_filter(AT42_I2C);
    i2c_set_digital_filter(AT42_I2C, I2C_CR1_DNF_DISABLED);

    //Configure PRESC[3:0] SDADEL[3:0] SCLDEL[3:0] SCLH[7:0] SCLL[7:0]
    // in TIMINGR

    /* TODO: Figure out the timing for 400kHz bus speed. */
    i2c_set_prescaler(AT42_I2C, 0);
    i2c_set_scl_low_period(AT42_I2C, 0x4);
    i2c_set_scl_high_period(AT42_I2C, 0x2);
    i2c_set_data_hold_time(AT42_I2C, 0x0);
    i2c_set_data_setup_time(AT42_I2C, 0x1);

    /* configure No-Stretch CR1 (only relevant in slave mode) */
    i2c_enable_stretching(AT42_I2C);
    i2c_set_7bit_addr_mode(AT42_I2C);
    i2c_peripheral_enable(AT42_I2C);

    /* Delay for the peripheral boot and calibration time then read the registers. */
    delay_nsec(250000000);
    if (at42_read(0, at42_regs, sizeof(at42_regs)) != sizeof(at42_regs)) {
        memset(at42_regs, 0, sizeof(at42_regs));
    }

    /* Setup the initial calibration. */
    if (at42_regs[AT42_REG_CHIP_ID] == AT42QT1070_CHIP_ID) {
        /* Assign KEY0 to a secondary group for multitouch. */
        at42_writeb(AT42_REG_AVE_ASK_KEY(5), AT42_ASK_VALUE(1) | AT42_AVE_DEFAULT);

        at42_writeb(AT42_REG_CAL_GUARD, 5); /* Set KEY0 as the guard channel. */
        at42_writeb(AT42_REG_LOW_POWER, 4); /* Set update time to 32ms. */
        at42_writeb(AT42_REG_CALIBRATE, 0xff);  /* (Re)start calibration. */

        /* Enable the change detect pin. */
        rcc_periph_clock_enable(RCC_GPIOB);
        gpio_mode_setup(AT42_PORT_CHANGE, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, AT42_PIN_CHANGE);
    }
    else {
        /* Switch off the I2C clocks and pins to save power */
        rcc_periph_clock_disable(RCC_I2C1);
        gpio_mode_setup(AT42_PORT_I2C, GPIO_MODE_INPUT, GPIO_PUPD_NONE, AT42_PIN_SCL | AT42_PIN_SDA);
    }
} /* at42_setup */

uint8_t
at42_status(void)
{
    if ((at42_regs[AT42_REG_CHIP_ID] == AT42QT1070_CHIP_ID) && (gpio_get(AT42_PORT_CHANGE, AT42_PIN_CHANGE) == 0)) {
        at42_read(AT42_REG_DETECT_STATUS, &at42_regs[AT42_REG_DETECT_STATUS], 2);
    }
    return at42_regs[AT42_REG_KEY_STATUS];
} /* at42_status */

uint16_t
at42_channel(uint8_t key)
{
    if (at42_regs[AT42_REG_CHIP_ID] == AT42QT1070_CHIP_ID) {
        uint8_t data[2];
        at42_read(AT42_REG_KEY_SIGNAL(key), data, sizeof(data));
        return (data[0] << 8) | data[1];
    }
    return 0;
} /* at42_channel */

