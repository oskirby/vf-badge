/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

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
    /* Should execute at 1IPC +1 on a predicted branch. */
    unsigned long long cycles = ((unsigned long long)nsec * (unsigned long long)rcc_ahb_frequency) >> 2;
    __asm volatile(
        "delay_nsec_loop:           \n"
        "sub %[cylo], %[billion]    \n"
        "sbc %[cyhi], %[zero]       \n"
        "bpl delay_nsec_loop        \n"
        :: [cyhi]"r"(cycles >> 32), [cylo]"r"(cycles & 0xffffffff), [billion]"rI"(1000000000), [zero]"r"(0));
}


static void
led_right_entry(struct schedule *sched, const struct state *state, unsigned long uptime, unsigned int value)
{
    led_set_right(value);
} /* led_right_entry */

static void
led_left_entry(struct schedule *sched, const struct state *state, unsigned long uptime, unsigned int value)
{
    led_set_left(value);
} /* led_right_entry */

static void
led_left_rampup(struct schedule *sched, const struct state *state, unsigned long uptime, unsigned int value)
{
    unsigned int x = FIXED_MUL(value, (uptime - sched->start));
    if (x > 0xff) {
        sched_goto(sched, 2, uptime);
    }
    else {
        led_set_left(x);
    }
} /* led_left_rampup */

static void
led_left_rampdown(struct schedule *sched, const struct state *state, unsigned long uptime, unsigned int value)
{
    unsigned int x = FIXED_MUL(value, (uptime - sched->start));
    if (x > 0xff) {
        sched_goto(sched, 0, uptime);
    }
    else {
        led_set_left(0xff - x);
    }
} /* led_left_rampdown */

/* Tripple blink and pause. */
static const struct state led_right_sm[] = {
    {.next = 1, .duration = 50, .value = 0xff, .entry = led_right_entry},
    {.next = 2, .duration = 70, .value = 0x00, .entry = led_right_entry},
    {.next = 3, .duration = 50, .value = 0xff, .entry = led_right_entry},
    {.next = 4, .duration = 70, .value = 0x00, .entry = led_right_entry},
    {.next = 5, .duration = 50, .value = 0xff, .entry = led_right_entry},
    {.next = 0, .duration = 1500, .value = 0x00, .entry = led_right_entry}
};
static struct schedule led_right_schedule = {
    .sm = led_right_sm,
};

/* Pulse the left LED on with a soft ramp up/down. */
static const struct state led_left_sm[] = {
    {.next = 1, .duration = 2000, .value = 0, .entry = led_left_entry},
    {.next = 0, .duration = 0, .value = FIXED_ONE / 5, .tick = led_left_rampup},
    {.next = 0, .duration = 0, .value = FIXED_ONE / 5, .tick = led_left_rampdown},
};
static struct schedule led_left_schedule = {
    .sm = led_left_sm,
};


/* The fastest we can go at the lowest core voltage. */
static const struct rcc_clock_scale rcc_config = {
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

/* RTC Wakeup period */
#define RTC_PERIOD_MSEC         20

/* Hue rate change per millisecond (in degrees) */
#define HUE_DEGREES_PER_MSEC    (FIXED_ONE * 10)

static unsigned long uptime = 0;
static unsigned int hue = 0;

static void
rtc_wakeup(void)
{
    uint8_t keys;

    /* NOTE: the backup domain protection must be *DISABLED* for this to work. */
    rtc_clear_wakeup_flag();
    exti_reset_request(EXTI20);

    /* Update our timekeeping */
	uptime += RTC_PERIOD_MSEC;
    keys = at42_status();

    /* LED state machine */
    sched_tick(&led_right_schedule, uptime);
    sched_tick(&led_left_schedule, uptime);

    /* Update the HSV sweep - or override by key touch */
    if (keys & (1<<0)) {
        /* Treat the paw-pad separately. */
        apa102_write_rgb(0xff, 0xff, 0xff, 0x40);
        return;
    }
    else if (keys & (1<<1)) {
        hue = (0 << FIXED_BITS);
    }
    else if (keys & (1<<2)) {
        hue = (90 << FIXED_BITS);
    }
    else if (keys & (1<<3)) {
        hue = (180 << FIXED_BITS);
    }
    else if (keys & (1<<4)) {
        hue = (270 << FIXED_BITS);
    }
    else {
        hue += FIXED_MUL(HUE_DEGREES_PER_MSEC, RTC_PERIOD_MSEC);
        if (hue >= (360 << FIXED_BITS)) {
            hue -= (360 << FIXED_BITS);
        }
    }
    apa102_write_hsv(hue >> FIXED_BITS, 0xff, 0x40);
} /* rtc_wakeup */

void
rtc_isr(void)
{
    rtc_wakeup();
}

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
    sched_goto(&led_right_schedule, 0, uptime);
    sched_goto(&led_left_schedule, 0, uptime);

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

