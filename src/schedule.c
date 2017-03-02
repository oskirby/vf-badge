/* Scheduling helper library */
#include <vf-badge.h>

/* Update the state machine after a change in uptime. */
void
sched_tick(struct schedule *sched, unsigned long uptime)
{
    const struct state *state = &sched->sm[sched->state];

    /* Check for state timeouts. */
    if (state->duration && (uptime >= (sched->start + state->duration))) {
        sched_goto(sched, state->next, uptime);
    }
    /* Otherwise, tick the state. */
    else if (state->tick) {
        state->tick(sched, state, uptime);
    }
    sched->prevtick = uptime;
} /* sched_tick */

/* Transition to a new state. */
void
sched_goto(struct schedule *sched, unsigned int state, unsigned long uptime)
{
    sched->state = state;
    sched->start = uptime;
    sched->prevtick = uptime;
    if (sched->sm[state].entry) {
        sched->sm[state].entry(sched, &sched->sm[state], uptime);
    }
} /* sched_goto */

