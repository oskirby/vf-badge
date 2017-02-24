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
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "vf-badge.h"

#define PORT_LED GPIOA

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
    value *= (uptime - sched->start);
    if (value > 0xff) {
        sched_goto(sched, 2, uptime);
    }
    else {
        led_set_left(value);
    }
} /* led_left_rampup */

static void
led_left_rampdown(struct schedule *sched, const struct state *state, unsigned long uptime, unsigned int value)
{
    value *= (uptime - sched->start);
    if (value > 0xff) {
        sched_goto(sched, 0, uptime);
    }
    else {
        led_set_left(0xff - value);
    }
} /* led_left_rampdown */

/* Tripple blink and pause. */
static const struct state led_right_sm[] = {
    {.next = 1, .duration = 5, .value = 0xff, .entry = led_right_entry},
    {.next = 2, .duration = 7, .value = 0x00, .entry = led_right_entry},
    {.next = 3, .duration = 5, .value = 0xff, .entry = led_right_entry},
    {.next = 4, .duration = 7, .value = 0x00, .entry = led_right_entry},
    {.next = 5, .duration = 5, .value = 0xff, .entry = led_right_entry},
    {.next = 0, .duration = 150, .value = 0x00, .entry = led_right_entry}
};
static struct schedule led_right_schedule = {
    .sm = led_right_sm,
};

/* Pulse the left LED on with a soft ramp up/down. */
static const struct state led_left_sm[] = {
    {.next = 1, .duration = 200, .value = 0, .entry = led_left_entry},
    {.next = 0, .duration = 0, .value = 2, .tick = led_left_rampup},
    {.next = 0, .duration = 0, .value = 2, .tick = led_left_rampdown},
};
static struct schedule led_left_schedule = {
    .sm = led_left_sm,
};

static unsigned long upticks = 0;
static unsigned int hue = 0;

/* Called when systick fires */
void
sys_tick_handler(void)
{
    /* Timekeeping */
    upticks++;

    /* LED state machine */
    sched_tick(&led_right_schedule, upticks);
    sched_tick(&led_left_schedule, upticks);

    /* APA102 Hue sweep */
    hue++;
    if (hue >= 360) {
        hue -= 360;
    }
    apa102_write_hsv(hue, 0xff, 0x80);
} /* sys_tick_handler */

static void
sys_tick_setup(void)
{
    systick_set_clocksource(STK_CSR_CLKSOURCE_EXT);
    STK_CVR = 0;

    systick_set_reload(rcc_ahb_frequency / 800);
    systick_counter_enable();
    systick_interrupt_enable();
} /* sys_tick_setup */


int main(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);

    led_setup();
    apa102_setup();

    /* Start the blink state machines. */
    sched_goto(&led_right_schedule, 0, upticks);
    sched_goto(&led_left_schedule, 0, upticks);

    /* Start the systick */
    sys_tick_setup();

    while (1) {
        __asm__("wfi");
    }

    return 0;
} /* main */

