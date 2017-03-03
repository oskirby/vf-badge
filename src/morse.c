#include <vf-badge.h>

#define DOT           0
#define DASH          1

#define MORSE_INDEX(_c_)  ((_c_) & ~0x20)

/* Variadic macro voodoo for morse encoding. */
#define MORSE_ENCODE1(n0)                       {.len = 1, .code = n0<<0}
#define MORSE_ENCODE2(n0, n1)                   {.len = 2, .code = n0<<1 | n1<<0}
#define MORSE_ENCODE3(n0, n1, n2)               {.len = 3, .code = n0<<2 | n1<<1 | n2<<0}
#define MORSE_ENCODE4(n0, n1, n2, n3)           {.len = 4, .code = n0<<3 | n1<<2 | n2<<1 | n3<<0}
#define MORSE_ENCODE5(n0, n1, n2, n3, n4)       {.len = 5, .code = n0<<4 | n1<<3 | n2<<2 | n3<<1 | n4<<0}
#define MORSE_ENCODE6(n0, n1, n2, n3, n4, n5)   {.len = 6, .code = n0<<5 | n1<<4 | n2<<3 | n3<<2 | n4<<1 | n5<<0}
#define MORSE_ENCODEX(n0, n1, n2, n3, n4, n5, num, ...)  MORSE_ENCODE##num
#define MORSE_ENCODE(...) MORSE_ENCODEX(__VA_ARGS__,6,5,4,3,2,1,...)(__VA_ARGS__)

/* Morse encoding tables */
static const struct morse morse_code_table[] = {
    /* Numeric Digits. */
    [MORSE_INDEX('0')] = MORSE_ENCODE(DASH, DASH, DASH, DASH, DASH),
    [MORSE_INDEX('1')] = MORSE_ENCODE(DOT, DASH, DASH, DASH, DASH),
    [MORSE_INDEX('2')] = MORSE_ENCODE(DOT, DOT, DASH, DASH, DASH),
    [MORSE_INDEX('3')] = MORSE_ENCODE(DOT, DOT, DOT, DASH, DASH),
    [MORSE_INDEX('4')] = MORSE_ENCODE(DOT, DOT, DOT, DOT, DASH),
    [MORSE_INDEX('5')] = MORSE_ENCODE(DOT, DOT, DOT, DOT, DOT),
    [MORSE_INDEX('6')] = MORSE_ENCODE(DASH, DOT, DOT, DOT, DOT),
    [MORSE_INDEX('7')] = MORSE_ENCODE(DASH, DASH, DOT, DOT, DOT),
    [MORSE_INDEX('8')] = MORSE_ENCODE(DASH, DASH, DASH, DOT, DOT),
    [MORSE_INDEX('9')] = MORSE_ENCODE(DASH, DASH, DASH, DASH, DOT),
    /* Alphabetic Characters */
    [MORSE_INDEX('A')] = MORSE_ENCODE(DOT, DASH),
    [MORSE_INDEX('B')] = MORSE_ENCODE(DASH, DOT, DOT, DOT),
    [MORSE_INDEX('C')] = MORSE_ENCODE(DASH, DOT, DASH, DOT),
    [MORSE_INDEX('D')] = MORSE_ENCODE(DASH, DOT, DOT),
    [MORSE_INDEX('E')] = MORSE_ENCODE(DOT),
    [MORSE_INDEX('F')] = MORSE_ENCODE(DOT, DOT, DASH, DOT),
    [MORSE_INDEX('G')] = MORSE_ENCODE(DASH, DASH, DOT),
    [MORSE_INDEX('H')] = MORSE_ENCODE(DOT, DOT, DOT, DOT),
    [MORSE_INDEX('I')] = MORSE_ENCODE(DOT, DOT),
    [MORSE_INDEX('J')] = MORSE_ENCODE(DOT, DASH, DASH, DASH),
    [MORSE_INDEX('K')] = MORSE_ENCODE(DASH, DOT, DASH),
    [MORSE_INDEX('L')] = MORSE_ENCODE(DOT, DASH, DOT, DOT),
    [MORSE_INDEX('M')] = MORSE_ENCODE(DASH, DASH),
    [MORSE_INDEX('N')] = MORSE_ENCODE(DASH, DOT),
    [MORSE_INDEX('O')] = MORSE_ENCODE(DASH, DASH, DASH),
    [MORSE_INDEX('P')] = MORSE_ENCODE(DOT, DASH, DASH, DOT),
    [MORSE_INDEX('Q')] = MORSE_ENCODE(DASH, DASH, DOT, DASH),
    [MORSE_INDEX('R')] = MORSE_ENCODE(DOT, DASH, DOT),
    [MORSE_INDEX('S')] = MORSE_ENCODE(DOT, DOT, DOT),
    [MORSE_INDEX('T')] = MORSE_ENCODE(DASH),
    [MORSE_INDEX('U')] = MORSE_ENCODE(DOT, DOT, DASH),
    [MORSE_INDEX('V')] = MORSE_ENCODE(DOT, DOT, DOT, DASH),
    [MORSE_INDEX('W')] = MORSE_ENCODE(DOT, DASH, DASH),
    [MORSE_INDEX('X')] = MORSE_ENCODE(DASH, DOT, DOT, DASH),
    [MORSE_INDEX('Y')] = MORSE_ENCODE(DASH, DOT, DASH, DASH),
    [MORSE_INDEX('Z')] = MORSE_ENCODE(DASH, DASH, DOT, DOT),
    /* Punctuation */
    [MORSE_INDEX('.')] = MORSE_ENCODE(DOT, DASH, DOT, DASH, DOT, DASH),
    [MORSE_INDEX(',')] = MORSE_ENCODE(DASH, DASH, DOT, DOT, DASH, DASH),
    [MORSE_INDEX(':')] = MORSE_ENCODE(DASH, DASH, DASH, DOT, DOT, DOT),
    [MORSE_INDEX('?')] = MORSE_ENCODE(DOT, DOT, DASH, DASH, DOT, DOT),
    [MORSE_INDEX('\'')] = MORSE_ENCODE(DOT, DASH, DASH, DASH, DASH, DOT),
    [MORSE_INDEX('-')] = MORSE_ENCODE(DASH, DOT, DOT, DOT, DOT, DASH),
    [MORSE_INDEX('/')] = MORSE_ENCODE(DASH, DOT, DOT, DASH, DOT),
    [MORSE_INDEX('(')] = MORSE_ENCODE(DASH, DOT, DASH, DASH, DOT, DASH),
    [MORSE_INDEX(')')] = MORSE_ENCODE(DASH, DOT, DASH, DASH, DOT, DASH),
    [MORSE_INDEX('"')] = MORSE_ENCODE(DOT, DASH, DOT, DOT, DASH, DOT),
    [MORSE_INDEX('@')] = MORSE_ENCODE(DOT, DASH, DASH, DOT, DASH, DOT),
    [MORSE_INDEX('=')] = MORSE_ENCODE(DASH, DOT, DOT, DOT, DASH),
};

struct morse
morse_encode(char c)
{
    unsigned idx = MORSE_INDEX(c);
    if (idx > sizeof(morse_code_table)/sizeof(struct morse)) {
        struct morse err = {0, 0};
        return err;
    } else {
        return morse_code_table[idx];
    }
} /* morse_encode */

