/* Driver for the AT42QT1070 Capacative touch controller. */
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
#define AT42_PIN_SCL    GPIO9
#define AT42_PIN_SDA    GPIO10

#define AT42_PORT_CHANGE    GPIOB
#define AT42_PIN_CHANGE     GPIO1

void
at42_setup(void)
{
    /* Configure I2C pins. */
    gpio_mode_setup(AT42_PORT_I2C, GPIO_MODE_AF, GPIO_PUPD_NONE, AT42_PIN_SCL | AT42_PIN_SDA);
    gpio_set_af(AT42_PORT_I2C, GPIO_AF4, AT42_PIN_SCL | AT42_PIN_SDA);

    /* Configure the GPIO input for change detection */
    gpio_mode_setup(AT42_PORT_CHANGE, GPIO_MODE_INPUT, GPIO_PUPD_NONE, AT42_PIN_CHANGE);
} /* at42_setup */

uint8_t
at42_status(void)
{
    /* TODO: Implement Me! */
    return 0;
} /* at42_status */

uint16_t
at42_channel(uint8_t key)
{
    /* TODO: Implement Me! */
    return 0;
} /* at42_channel */

