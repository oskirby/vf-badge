/* Project header for the VF badge */
#include <stdint.h>

void apa102_setup(void);
void apa102_write_rgb(uint8_t red, uint8_t green, uint8_t blue, uint8_t value);
void apa102_write_hsv(unsigned int hue, uint8_t saturation, uint8_t value);

void led_setup(void);
void led_set_right(uint8_t value);
void led_set_left(uint8_t value);

/* Bits used for fixed-point math */
#define FIXED_BITS          8
#define FIXED_ONE           (1u << FIXED_BITS)
#define FIXED_MAX           (FIXED_ONE - 1)
#define FIXED_MUL(_a_, _b_) (((unsigned long)(_a_) * (unsigned long)(_b_)) >> FIXED_BITS)

struct schedule;

struct state {
    unsigned int next;      /* Next state number on timeout. */
    unsigned int duration;  /* Current state duration, in ticks, or zero for infinity.  */
    unsigned int value;     /* Argument passed to the entry/tick callbacks. */
	void (*entry)(struct schedule *, const struct state *state, unsigned long uptime);
	void (*tick)(struct schedule *, const struct state *state, unsigned long uptime);
};

struct schedule {
    unsigned long start;    /* Current state start time. */
    unsigned long prevtick; /* Uptime of previous tick. */
    unsigned int state;     /* Current state number. */
    const struct state *sm; /* Array of state structures. */
};

#define sched_init(_sched_) memset(_sched_, 0, sizeof(struct schedule))
void sched_tick(struct schedule *sched, unsigned long uptime);
void sched_goto(struct schedule *sched, unsigned int state, unsigned long uptime);

void delay_nsec(unsigned long nsec);

void at42_setup(void);
void at42_calibrate(void);
uint8_t at42_change(void);
uint8_t at42_status(void);
uint16_t at42_channel(uint8_t key);

/* Morse encoding */
struct morse {
    uint8_t len;    /* Length of the code in bits. */
    uint8_t code;   /* Bitmask of symbols (0=dot 1=dash). */
};
struct morse morse_encode(char c);

/* Get the next symbol of morse code (dash or dot) and decrement the symbol length. */
#define morse_next_bit(_m_) \
    (((_m_)->code >> ((_m_)->len ? --(_m_)->len : (_m_)->len)) & 0x1)
