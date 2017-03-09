#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/cortex.h>

#include "vf-badge.h"

/* Very gross and I feel sad...  */
void
delay_nsec(unsigned long nsec)
{
    /* Cortex-M series should execute at 1IPC (+1 on a predicted branch). */
    unsigned long long cycles = ((unsigned long long)nsec * (unsigned long long)rcc_ahb_frequency) >> 2;
    __asm volatile(
        "delay_nsec_loop:           \n"
        "sub %[cylo], %[billion]    \n"
        "sbc %[cyhi], %[zero]       \n"
        "bpl delay_nsec_loop        \n"
        :: [cyhi]"r"(cycles >> 32), [cylo]"r"(cycles & 0xffffffff), [billion]"rI"(1000000000), [zero]"r"(0));
}

/*---------------------------------------------------------
 * Right LED state machine
 *---------------------------------------------------------
 */
const char *hello_world = "Zebra";

#define MORSE_DOT_MSEC  120

enum led_right_states {
    /* Normal blinking states. */
    LED_RIGHT_PAUSE = 0,
    LED_RIGHT_BLINK0,
    LED_RIGHT_BLINK1,
    LED_RIGHT_BLINK2,
    /* Morse coding states. */
    LED_MORSE_DOT,
    LED_MORSE_DASH,
    LED_MORSE_CHAR_SPACE,
    LED_MORSE_WORD_SPACE,
    LED_MORSE_NEXTBIT,
};

static void
led_right_blink(struct schedule *sched, const struct state *state, unsigned long uptime)
{
    led_set_right(((uptime - sched->start) < state->value) ? 0xff : 0);
} /* led_right_blink */

/* The morse string to output */
static const char *morse_string = "";

static void
led_right_morse(struct schedule *sched, const struct state *state, unsigned long uptime)
{
    static struct morse code;

    /* Output morse code bits. */
    if (code.len) {
        int bit = morse_next_bit(&code);
        sched_goto(sched,  bit ? LED_MORSE_DASH : LED_MORSE_DOT, uptime);
        return;
    }

    /* Get the next character to determine the upcoming state. */
    if (*morse_string  == '\0') {
        sched_goto(sched, LED_RIGHT_PAUSE, uptime);
    }
    else if (*morse_string == ' ') {
        morse_string++;
        sched_goto(sched, LED_MORSE_WORD_SPACE, uptime);
    }
    else {
        code = morse_encode(*morse_string++);
        sched_goto(sched, LED_MORSE_CHAR_SPACE, uptime);
    }
} /* led_right_morse */

/* Tripple blink and pause. */
static const struct state led_right_sm[] = {
    [LED_RIGHT_PAUSE] = {.next = LED_RIGHT_BLINK0, .duration = 4500, .value = 0, .tick = led_right_blink},
    [LED_RIGHT_BLINK0] = {.next = LED_RIGHT_BLINK1, .duration = 250, .value = 100, .tick = led_right_blink},
    [LED_RIGHT_BLINK1] = {.next = LED_RIGHT_PAUSE, .duration = 250, .value = 100, .tick = led_right_blink},
    [LED_RIGHT_BLINK2] = {.next = LED_RIGHT_PAUSE, .duration = 200, .value = 80, .tick = led_right_blink},
    /* Morse coding states. */
    [LED_MORSE_DOT] = {.next = LED_MORSE_NEXTBIT, .duration = MORSE_DOT_MSEC*2, .value = MORSE_DOT_MSEC, .tick = led_right_blink},
    [LED_MORSE_DASH] = {.next = LED_MORSE_NEXTBIT, .duration = MORSE_DOT_MSEC*4, .value = MORSE_DOT_MSEC*3, .tick = led_right_blink},
    [LED_MORSE_CHAR_SPACE] = {.next = LED_MORSE_NEXTBIT, .duration = MORSE_DOT_MSEC*3, .value = 0, .tick = led_right_blink},
    [LED_MORSE_WORD_SPACE] = {.next = LED_MORSE_NEXTBIT, .duration = MORSE_DOT_MSEC*7, .value = 0, .tick = led_right_blink},
    [LED_MORSE_NEXTBIT] = {.next = LED_RIGHT_PAUSE, .duration = 0, .value = 0, .entry = led_right_morse},
};
static struct schedule led_right_schedule = {
    .sm = led_right_sm,
};

/* Jump to the morse coding state to begin output. */
static void
led_start_morse(const char *str, unsigned long uptime)
{
    morse_string = str;
    sched_goto(&led_right_schedule, LED_MORSE_WORD_SPACE, uptime);
} /* led_start_morse */

/*---------------------------------------------------------
 * Left LED state machine
 *---------------------------------------------------------
 */
enum led_left_states {
    LED_LEFT_OFF = 0,
    LED_LEFT_PULSE,
};

static void
led_left_entry(struct schedule *sched, const struct state *state, unsigned long uptime)
{
    led_set_left(state->value);
} /* led_right_entry */

static void
led_left_pulse(struct schedule *sched, const struct state *state, unsigned long uptime)
{
    unsigned int x = FIXED_MUL(state->value, (uptime - sched->start));
    if (x <= 0xff) {
        led_set_left(x);
    }
    else if (x <= 0x1ff) {
        led_set_left(0xff - (x & 0xff));
    }
    else {
        sched_goto(sched, LED_LEFT_OFF, uptime);
    }
} /* led_left_pulse */

/* Pulse the left LED on with a soft ramp up/down. */
static const struct state led_left_sm[] = {
    [LED_LEFT_PULSE] = {.next = LED_LEFT_OFF, .duration = 0, .value = FIXED_ONE / 5, .tick = led_left_pulse},
    [LED_LEFT_OFF] = {.next = LED_LEFT_PULSE, .duration = 2000, .value = 0, .entry = led_left_entry},
};
static struct schedule led_left_schedule = {
    .sm = led_left_sm,
};

/*---------------------------------------------------------
 * RGB LED state machine
 *---------------------------------------------------------
 */
#define RGB_IDLE_VALUE  0x30
#define RGB_SATURATION  UINT8_MAX
#define RGB_PULSE_RATE  (FIXED_ONE / 4)

/* TODO: It would be nice to have a soft transition back to the
 * idle state after releasing a key.
 */
enum led_rgb_states {
    RGB_STATE_IDLE = 0,
    RGB_STATE_KEY0,
    RGB_STATE_KEY1,
    RGB_STATE_KEY2,
    RGB_STATE_KEY3,
    RGB_STATE_KEY4,
};

static unsigned int hue = 0;

static void
rgb_sweep_tick(struct schedule *sched, const struct state *state, unsigned long uptime)
{
    hue += FIXED_MUL(state->value, uptime - sched->prevtick);
    if (hue >= (360 << FIXED_BITS)) {
        hue -= (360 << FIXED_BITS);
    }
    apa102_write_hsv(hue >> FIXED_BITS, RGB_SATURATION, RGB_IDLE_VALUE);
} /* rgb_sweep_tick */

static void
rgb_pulse_tick(struct schedule *sched, const struct state *state, unsigned long uptime)
{
    unsigned int x = FIXED_MUL(RGB_PULSE_RATE, (uptime - sched->start));
    if (x & 0x100) {
        apa102_write_hsv(state->value, RGB_SATURATION, 0xff - (x & 0xff));
    } else {
        apa102_write_hsv(state->value, RGB_SATURATION, (x & 0xff));
    }
    /* Reset the hue target so the HSV sweep resumes from here. */
    hue = state->value << FIXED_BITS;
} /* rgb_pulse_tick */

static const struct state led_rgb_sm[] = {
    /* Idle behaviour is to sweep the hue through its range. */
    [RGB_STATE_IDLE] = {.next = 0, .duration = 0, .value = FIXED_ONE * 5, .tick = rgb_sweep_tick},
    [RGB_STATE_KEY0] = {.next = 0, .duration = 0, .value = FIXED_ONE * 70, .tick = rgb_sweep_tick},
    /* States for pulsing the LED on and off at a fixed hue while keys are set. */
    [RGB_STATE_KEY1] = {.next = 0, .duration = 0, .value = 0, .tick = rgb_pulse_tick},
    [RGB_STATE_KEY2] = {.next = 0, .duration = 0, .value = 90, .tick = rgb_pulse_tick},
    [RGB_STATE_KEY3] = {.next = 0, .duration = 0, .value = 180, .tick = rgb_pulse_tick},
    [RGB_STATE_KEY4] = {.next = 0, .duration = 0, .value = 270, .tick = rgb_pulse_tick},
};
static struct schedule led_rgb_schedule = {
    .sm = led_rgb_sm,
};

/*---------------------------------------------------------
 * Realtime Clock and Application Logic
 *---------------------------------------------------------
 */

/* RTC Wakeup period */
#define RTC_PERIOD_MSEC         20

static unsigned long uptime = 0;
static unsigned long at42_timestamp = 0;

static void
rtc_wakeup(void)
{
    /* NOTE: the backup domain protection must be *DISABLED* for this to work. */
    rtc_clear_wakeup_flag();
    exti_reset_request(EXTI20);

    /* Update our timekeeping */
	uptime += RTC_PERIOD_MSEC;

    /* Handle changes in the capacative touch state. */
    if (at42_change()) {
        uint8_t keys = at42_status();
        at42_timestamp = uptime;

        /* Jump states based on which keys are now set. */
        if (keys & (1<<0)) {
            sched_goto(&led_rgb_schedule, RGB_STATE_KEY0, uptime);
        }
        else if (keys & (1<<1)) {
            sched_goto(&led_rgb_schedule, RGB_STATE_KEY1, uptime);
        }
        else if (keys & (1<<2)) {
            sched_goto(&led_rgb_schedule, RGB_STATE_KEY2, uptime);
        }
        else if (keys & (1<<3)) {
            sched_goto(&led_rgb_schedule, RGB_STATE_KEY3, uptime);
        }
        else if (keys & (1<<4)) {
            sched_goto(&led_rgb_schedule, RGB_STATE_KEY4, uptime);
        }
        else {
            sched_goto(&led_rgb_schedule, RGB_STATE_IDLE, uptime);
        }
    }
    /*
     * If more than 5 minutes elapses without a key change, then
     * recalibrate the AT42 touch controller, and start blinking out
     * some morse.
     */
    else if ((uptime - at42_timestamp) > (5 * 60 * 1000)) {
        at42_calibrate();
        at42_timestamp = uptime;
        led_start_morse(hello_world, uptime);
    }

    /* Run the LED state machines */
    sched_tick(&led_right_schedule, uptime);
    sched_tick(&led_left_schedule, uptime);
    sched_tick(&led_rgb_schedule, uptime);
} /* rtc_wakeup */

/* TODO: Can improve latency by moving this into the main() loop
 * and running without the interrupts stacking/unstacking delay.
 */
void
rtc_isr(void)
{
    rtc_wakeup();
} /* rtc_isr */

static void
rtc_setup(void)
{
    /* From the datasheet, but it's not particularly well tuned. */
    const unsigned int rtc_frequency = 37000;

    /* Unlock the backup domain. */
    rcc_periph_clock_enable(RCC_PWR);
    pwr_disable_backup_domain_write_protect();

    RCC_CSR |= RCC_CSR_RTCRST;
    RCC_CSR &= ~RCC_CSR_RTCRST;

    /* Enable the low-speed oscillator. */
    rcc_osc_on(RCC_LSI);
    rcc_wait_for_osc_ready(RCC_LSI);
    rcc_rtc_select_clock(RCC_CSR_RTCSEL_LSI);
    RCC_CSR |= RCC_CSR_RTCEN;

    /* Setup the realtime clock. */
    rtc_unlock();
    rtc_set_wakeup_time(rtc_frequency * RTC_PERIOD_MSEC / 2000, RTC_CR_WUCLKSEL_RTC_DIV2);
    RTC_CR |= RTC_CR_WUTIE;
    rtc_clear_wakeup_flag();
    rtc_lock();

    /* Enable the RTC wakeup event. */
    nvic_enable_irq(NVIC_RTC_IRQ);
	exti_set_trigger(EXTI20, EXTI_TRIGGER_RISING);
    exti_enable_request(EXTI20);
} /* rtc_setup */

int main(void)
{
    /* The fastest we can go at the lowest core voltage. */
    const struct rcc_clock_scale rcc_config = {
        .hpre = RCC_CFGR_HPRE_SYSCLK_NODIV,
        .ppre1 = RCC_CFGR_PPRE1_HCLK_NODIV,
        .ppre2 = RCC_CFGR_PPRE2_HCLK_NODIV,
        .voltage_scale = PWR_SCALE3,
        .flash_config = FLASH_ACR_LATENCY_0WS,
        .ahb_frequency	= 4194000,
        .apb1_frequency = 4194000,
        .apb2_frequency = 4194000,
        .msi_range = RCC_ICSCR_MSIRANGE_4MHZ,
    };

    /* Ensure NRST gets asserted to ensure all peripherals get reset. */
    if ((RCC_CSR & RCC_CSR_SFTRSTF) == 0) {
        scb_reset_system();
    }

    rcc_clock_setup_msi(&rcc_config);
    rcc_periph_clock_enable(RCC_GPIOA);

    led_setup();
    apa102_setup();
    at42_setup();

    /* Start the blink state machines. */
    led_start_morse(hello_world, uptime);
    sched_goto(&led_left_schedule, LED_LEFT_PULSE, uptime);
    sched_goto(&led_rgb_schedule, RGB_STATE_IDLE, uptime);

    /* Start the RTC wakeup timer. */
    cm_enable_interrupts();
    rtc_setup();

    while (1) {
        /* Don't enter stop mode for the first 10 seconds of operation. */
        if (uptime > 10000) {
            SCB_SCR |= SCB_SCR_SLEEPDEEP;
            PWR_CR |=  PWR_CR_LPSDSR;
            pwr_set_stop_mode();
        }
        __asm__("wfi");
    }

    return 0;
} /* main */
