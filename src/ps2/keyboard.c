#include "keyboard.h"
#include "rpi.h"
#include "rpi-interrupts.h"

extern ps2_key_t const ps2_keys[];

static unsigned clock_gpio;
static unsigned data_gpio;

static keyboard_modifiers_t current_modifiers;

/* ps2_keys[] contains entries for scan codes 0x00..0x83 */
#define PS2_KEYS_COUNT 0x84

#define SCANCODE_Q_SIZE 128
#define EVENT_Q_SIZE 64

static volatile unsigned char scancode_q[SCANCODE_Q_SIZE];
static volatile unsigned sc_head, sc_tail;

static volatile key_event_t event_q[EVENT_Q_SIZE];
static volatile unsigned ev_head, ev_tail;

/* ISR PS/2 frame assembly state */
static volatile unsigned ps2_bit_count;
static volatile unsigned char ps2_data_byte;
static volatile unsigned ps2_parity_bit;

static int is_letter(unsigned char ch) {
    return ch >= 'a' && ch <= 'z';
}

static int is_modifier_key(unsigned char keycode) {
    return keycode == PS2_KEY_SHIFT ||
           keycode == PS2_KEY_ALT ||
           keycode == PS2_KEY_CTRL ||
           keycode == PS2_KEY_CAPS_LOCK ||
           keycode == PS2_KEY_NUM_LOCK ||
           keycode == PS2_KEY_SCROLL_LOCK;
}

static void set_modifier(keyboard_modifiers_t mod) {
    current_modifiers |= mod;
}

static void clear_modifier(keyboard_modifiers_t mod) {
    current_modifiers &= ~mod;
}

static void update_modifier_state(key_action_t action) {
    switch (action.keycode) {
    case PS2_KEY_SHIFT:
        if (action.what == KEY_PRESS)
            set_modifier(KEYBOARD_MOD_SHIFT);
        else
            clear_modifier(KEYBOARD_MOD_SHIFT);
        break;
    case PS2_KEY_ALT:
        if (action.what == KEY_PRESS)
            set_modifier(KEYBOARD_MOD_ALT);
        else
            clear_modifier(KEYBOARD_MOD_ALT);
        break;
    case PS2_KEY_CTRL:
        if (action.what == KEY_PRESS)
            set_modifier(KEYBOARD_MOD_CTRL);
        else
            clear_modifier(KEYBOARD_MOD_CTRL);
        break;
    case PS2_KEY_CAPS_LOCK:
        if (action.what == KEY_PRESS) {
            if (current_modifiers & KEYBOARD_MOD_CAPS_LOCK)
                clear_modifier(KEYBOARD_MOD_CAPS_LOCK);
            else
                set_modifier(KEYBOARD_MOD_CAPS_LOCK);
        }
        break;
    case PS2_KEY_NUM_LOCK:
        if (action.what == KEY_PRESS) {
            if (current_modifiers & KEYBOARD_MOD_NUM_LOCK)
                clear_modifier(KEYBOARD_MOD_NUM_LOCK);
            else
                set_modifier(KEYBOARD_MOD_NUM_LOCK);
        }
        break;
    case PS2_KEY_SCROLL_LOCK:
        if (action.what == KEY_PRESS) {
            if (current_modifiers & KEYBOARD_MOD_SCROLL_LOCK)
                clear_modifier(KEYBOARD_MOD_SCROLL_LOCK);
            else
                set_modifier(KEYBOARD_MOD_SCROLL_LOCK);
        }
        break;
    default:
        break;
    }
}

static int scancode_q_empty(void) {
    return sc_head == sc_tail;
}

static int scancode_q_full(void) {
    return ((sc_head + 1) % SCANCODE_Q_SIZE) == sc_tail;
}

static void scancode_q_push(unsigned char code) {
    if (scancode_q_full())
        return;
    scancode_q[sc_head] = code;
    sc_head = (sc_head + 1) % SCANCODE_Q_SIZE;
}

static unsigned char scancode_q_pop_blocking(void) {
    while (scancode_q_empty()) {
    }
    unsigned char code = scancode_q[sc_tail];
    sc_tail = (sc_tail + 1) % SCANCODE_Q_SIZE;
    return code;
}

static int event_q_empty(void) {
    return ev_head == ev_tail;
}

static int event_q_full(void) {
    return ((ev_head + 1) % EVENT_Q_SIZE) == ev_tail;
}

static void event_q_push(key_event_t ev) {
    if (event_q_full())
        return;
    event_q[ev_head] = ev;
    ev_head = (ev_head + 1) % EVENT_Q_SIZE;
}

static key_event_t event_q_pop(void) {
    key_event_t ev = event_q[ev_tail];
    ev_tail = (ev_tail + 1) % EVENT_Q_SIZE;
    return ev;
}

/*
 * Called on each falling edge of PS/2 clock.
 * Samples one bit from data line.
 */
static void keyboard_ps2_isr_bit(void) {
    unsigned bit = gpio_read(data_gpio) ? 1 : 0;

    if (ps2_bit_count == 0) {
        /* start bit must be 0 */
        if (bit == 0) {
            ps2_bit_count = 1;
            ps2_data_byte = 0;
            ps2_parity_bit = 0;
        }
        return;
    }

    if (ps2_bit_count >= 1 && ps2_bit_count <= 8) {
        unsigned i = ps2_bit_count - 1;
        if (bit)
            ps2_data_byte |= (1u << i);
        ps2_bit_count++;
        return;
    }

    if (ps2_bit_count == 9) {
        ps2_parity_bit = bit;
        ps2_bit_count++;
        return;
    }

    if (ps2_bit_count == 10) {
        unsigned stop_ok = (bit == 1);

        unsigned ones = 0;
        for (int i = 0; i < 8; i++)
            ones += (ps2_data_byte >> i) & 1u;
        ones += ps2_parity_bit;

        /* PS/2 uses odd parity */
        unsigned parity_ok = (ones & 1u) == 1u;

        if (stop_ok && parity_ok)
            scancode_q_push(ps2_data_byte);

        ps2_bit_count = 0;
        ps2_data_byte = 0;
        ps2_parity_bit = 0;
        return;
    }

    ps2_bit_count = 0;
}

void keyboard_init(unsigned int clock_pin, unsigned int data_pin) {
    clock_gpio = clock_pin;
    data_gpio = data_pin;
    current_modifiers = 0;

    sc_head = sc_tail = 0;
    ev_head = ev_tail = 0;

    ps2_bit_count = 0;
    ps2_data_byte = 0;
    ps2_parity_bit = 0;

    gpio_set_input(clock_gpio);
    gpio_set_input(data_gpio);

    gpio_set_pullup(clock_gpio);
    gpio_set_pullup(data_gpio);
    printk("dskjlfabjdlsbfklsjdf\n");
}

unsigned char keyboard_read_scancode(void) {
    return scancode_q_pop_blocking();
}

key_action_t keyboard_read_sequence(void) {
    key_action_t action;
    unsigned char sc = keyboard_read_scancode();

    if (sc == 0xF0) {
        action.what = KEY_RELEASE;
        action.keycode = keyboard_read_scancode();
        return action;
    }

    if (sc == 0xE0) {
        unsigned char next = keyboard_read_scancode();
        if (next == 0xF0) {
            action.what = KEY_RELEASE;
            action.keycode = keyboard_read_scancode();
        } else {
            action.what = KEY_PRESS;
            action.keycode = next;
        }
        return action;
    }

    action.what = KEY_PRESS;
    action.keycode = sc;
    return action;
}

/*
 * Note: this can still block if only a prefix byte (E0/F0) is present.
 * Good enough for now.
 */
static int keyboard_decode_one_event_nonblocking(void) {
    if (scancode_q_empty())
        return 0;

    key_action_t action = keyboard_read_sequence();

    if (action.keycode >= PS2_KEYS_COUNT)
        return 1;

    ps2_key_t key = ps2_keys[action.keycode];

    if (key.ch == PS2_KEY_NONE)
        return 1;

    if (is_modifier_key(key.ch)) {
        update_modifier_state(action);
        return 1;
    }

    key_event_t ev;
    ev.action = action;
    ev.key = key;
    ev.modifiers = current_modifiers;
    event_q_push(ev);
    return 1;
}

int keyboard_event_available(void) {
    if (!event_q_empty())
        return 1;

    while (!scancode_q_empty() && event_q_empty()) {
        if (!keyboard_decode_one_event_nonblocking())
            break;
    }

    return !event_q_empty();
}

key_event_t keyboard_read_event(void) {
    while (!keyboard_event_available()) {
    }
    return event_q_pop();
}

unsigned char keyboard_read_next(void) {
    for (;;) {
        key_event_t ev = keyboard_read_event();

        if (ev.action.what != KEY_PRESS)
            continue;

        unsigned char ch;

        if (is_letter(ev.key.ch)) {
            int shift = (ev.modifiers & KEYBOARD_MOD_SHIFT) != 0;
            int caps  = (ev.modifiers & KEYBOARD_MOD_CAPS_LOCK) != 0;

            if (shift ^ caps)
                ch = ev.key.other_ch;
            else
                ch = ev.key.ch;
        } else {
            if (ev.modifiers & KEYBOARD_MOD_SHIFT)
                ch = ev.key.other_ch;
            else
                ch = ev.key.ch;
        }

        return ch;
    }
}

void keyboard_use_interrupts(void) {
    gpio_event_clear(clock_gpio);
    dev_barrier();

    gpio_int_falling_edge(clock_gpio);
    dev_barrier();
}

/*
 * Call from the global IRQ handler.
 */
void keyboard_check_and_handle_interrupt(void) {
    if (!gpio_has_interrupt()) {
        return;
    }

    if (!gpio_event_detected(clock_gpio)) {
        return;
    }

    gpio_event_clear(clock_gpio);
    dev_barrier();

    keyboard_ps2_isr_bit();
}