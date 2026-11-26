/*

   █         █    █            ▄▄▄▀▀▀▀▀▀▄▄▄        █▀▀▀▀▀▀▀▀▀▀█
   █        █     █          ▄▀            ▀▄      █          █
   █       █      █        ▄▀                ▀▄    █          █
   █      █       █        █                  █    █          █
   █     █        █       █                    █   █          █
   █    █         █       █                    █   █▄▄▄▄▄▄▄▄▄▄█
   █   █ █        █       █                    █   █      █
   █  █   █       █        █                  █    █       █
   █ █     █      █        ▀▄                ▄▀    █        █
   ██       █     █          ▀▄            ▄▀      █         █
   █         █    █▄▄▄▄▄▄▄▄    ▀▀▀▄▄▄▄▄▄▀▀▀        █          █

   ▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀
   D E F A U L T

*/
#include QMK_KEYBOARD_H
#include <stdio.h>
#include <string.h>
#include "klor.h"
#include "i2c_master.h"
#include "print.h"
#include "pointing_device.h"

static bool gen6_read(uint32_t addr, uint8_t *data,
        uint16_t len) {
    uint8_t cmd[8] = {
        0x01, 0x09,
        addr & 0xFF,
        (addr >> 8) & 0xFF,
        (addr >> 16) & 0xFF,
        (addr >> 24) & 0xFF,
        len & 0xFF,
        (len >> 8) & 0xFF
    };

    uint8_t buf[len + 3];

    if (i2c_transmit(0x58, cmd, sizeof(cmd), 100)
            != I2C_STATUS_SUCCESS) {
        return false;
    }

    if (i2c_receive(0x58, buf, sizeof(buf), 100)
            != I2C_STATUS_SUCCESS) {
        return false;
    }

    uint8_t checksum = 0;

    for (uint16_t i = 0; i < len + 2; i++) {
        checksum += buf[i];
    }

    if (checksum != buf[len + 2]) {
        return false;
    }

    if ((buf[0] | (buf[1] << 8)) != len + 3) {
        return false;
    }

    memcpy(data, buf + 2, len);
    wait_us(150);

    return true;
}
static bool gen6_write(uint32_t addr,
        const uint8_t *data,
        uint16_t len) {
    uint8_t buf[len + 9];

    buf[0] = 0x00;
    buf[1] = 0x09;
    buf[2] = addr & 0xFF;
    buf[3] = (addr >> 8) & 0xFF;
    buf[4] = (addr >> 16) & 0xFF;
    buf[5] = (addr >> 24) & 0xFF;
    buf[6] = len & 0xFF;
    buf[7] = (len >> 8) & 0xFF;

    memcpy(buf + 8, data, len);

    uint8_t checksum = 0;

    for (uint16_t i = 0; i < len + 8; i++) {
        checksum += buf[i];
    }

    buf[len + 8] = checksum;

    if (i2c_transmit(0x58, buf, sizeof(buf), 100)
            != I2C_STATUS_SUCCESS) {
        return false;
    }

    wait_ms(1);
    return true;
}
#define GEN6_FEED_CONFIG4 0x200E000B

static bool gen6_set_ptp(bool enabled) {
    uint8_t value;

    if (!gen6_read(GEN6_FEED_CONFIG4, &value, 1)) {
        return false;
    }

    value &= 0xF3;

    if (enabled) {
        value |= 0x04;
    }

    if (!gen6_write(GEN6_FEED_CONFIG4, &value, 1)) {
        return false;
    }

    uint8_t verify;

    if (!gen6_read(GEN6_FEED_CONFIG4, &verify, 1)) {
        return false;
    }

    return (verify & 0x0C) == (enabled ? 0x04 : 0);
}

static bool gen6_initialized = false;

void pointing_device_driver_init(void) {
    i2c_init();

    wait_ms(500);

    uint8_t discard[17];

    for (uint8_t i = 0; i < 5; i++) {
        i2c_receive(0x58, discard, 17, 20);
        wait_ms(1);
    }

    wait_ms(50);

    gen6_initialized = gen6_set_ptp(true);
}
uint16_t pointing_device_driver_get_cpi(void) {
    return 0;
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
    (void)cpi;
}
static int8_t gen6_extract_delta(float *accum) {
    int32_t value = (int32_t)*accum;

    if (value > 127) value = 127;
    if (value < -127) value = -127;

    *accum -= value;

    return (int8_t)value;
}

static void gen6_rotate(int16_t *x, int16_t *y) {
    int16_t tx = *x;
    int16_t ty = *y;

#if GEN6_ROTATION == 90
    *x = ty;
    *y = -tx;
#elif GEN6_ROTATION == 180
    *x = -tx;
    *y = -ty;
#elif GEN6_ROTATION == 270
    *x = -ty;
    *y = tx;
#endif
}
report_mouse_t pointing_device_driver_get_report(
        report_mouse_t report
        ) {
    static uint32_t last_valid = 0;

    static uint8_t prev_count = 0;
    static uint8_t prev_id1 = 0;
    static uint8_t prev_id2 = 0;

    static int16_t prev_x = 0;
    static int16_t prev_y = 0;

    static float accum_x = 0;
    static float accum_y = 0;
    static float accum_h = 0;
    static float accum_v = 0;

    static uint16_t last_scan = 0;
    static bool have_scan = false;

    static uint32_t tap_start = 0;
    static int16_t tap_x = 0;
    static int16_t tap_y = 0;
    static uint8_t tap_fingers = 0;
    static bool tap_moved = false;
    static bool tap_pending_release = false;
    static bool tap_physical_click = false;
    static uint32_t tap_click_time = 0;
    static uint8_t tap_click_button = 0;

    if (tap_pending_release) {
        if (timer_elapsed32(tap_click_time) < 30) {
            report.buttons = tap_click_button;
            return report;
        }

        tap_pending_release = false;
        report.buttons = 0;
        return report;
    }
    if (!gen6_initialized) {
        return report;
    }


    uint8_t p[17] = {0};

    // Read directly, without the motion-ready query.
    if (i2c_receive(0x58, p, sizeof(p), 10)
            != I2C_STATUS_SUCCESS) {
        if (timer_elapsed32(last_valid) > 50) {
            prev_count = 0;
            have_scan = false;
            report.buttons = 0;
        }

        return report;
    }

    wait_us(100);

    uint16_t len = p[0] | (p[1] << 8);

    if (p[2] != 0x01 || len != 17) {
        if (timer_elapsed32(last_valid) > 25) {
            report.buttons = 0;

            if (prev_count > 0) {
                uint32_t duration =
                    timer_elapsed32(tap_start);

                bool accepted =
                    !tap_moved &&
                    !tap_physical_click &&
                    duration <= GEN6_TAP_TERM_MS;


                if (accepted) {
                    report.buttons =
                        tap_fingers == 2 ? 0x02 : 0x01;

                    tap_pending_release = true;
                    tap_click_time = timer_read32();
                    tap_click_button = report.buttons;
                }
            }

            prev_count = 0;
            have_scan = false;
        }

        return report;
    }

    uint16_t scan = p[13] | (p[14] << 8);

    // Ignore duplicate sensor frames.
    if (have_scan && scan == last_scan) {
        return report;
    }

    last_scan = scan;
    have_scan = true;
    last_valid = timer_read32();

    bool tip1 = (p[3] & 0x02) != 0;
    bool tip2 = (p[8] & 0x02) != 0;

    uint8_t id1 = p[3] >> 2;
    uint8_t id2 = p[8] >> 2;

    int16_t x1 = p[4] | (p[5] << 8);
    int16_t y1 = p[6] | (p[7] << 8);

    int16_t x2 = p[9] | (p[10] << 8);
    int16_t y2 = p[11] | (p[12] << 8);

    uint8_t count = tip1 + tip2;

    report.buttons = p[16] & 0x01;

    int16_t x = 0;
    int16_t y = 0;

    if (count == 1) {
        x = tip1 ? x1 : x2;
        y = tip1 ? y1 : y2;
        id1 = tip1 ? id1 : id2;
        id2 = 0;
    } else if (count == 2) {
        // Track the midpoint of both fingers.
        x = (x1 + x2) / 2;
        y = (y1 + y2) / 2;

        // Normalize contact order.
        if (id1 > id2) {
            uint8_t tmp = id1;
            id1 = id2;
            id2 = tmp;
        }
    }

    // Start tracking a potential tap.
    if (prev_count == 0 && count > 0) {
        tap_start = timer_read32();
        tap_x = x;
        tap_y = y;
        tap_fingers = count;
        tap_moved = false;
        tap_physical_click = false;
    }

    // Track the maximum number of fingers.
    if (count > tap_fingers) {
        tap_fingers = count;
        tap_x = x;
        tap_y = y;
    }

    // Cancel tap when movement exceeds the threshold.
    if (count > 0 && count == prev_count) {
        int32_t tx = (int32_t)x - tap_x;
        int32_t ty = (int32_t)y - tap_y;

        if (tx * tx + ty * ty >
                GEN6_TAP_THRESHOLD * GEN6_TAP_THRESHOLD) {
            tap_moved = true;
        }
    }

    // Prevent a physical click from generating a second click.
    if (report.buttons & 0x01) {
        tap_physical_click = true;
    }

    // Detect a tap on lift-off.
    if (count == 0 && prev_count > 0) {
        if (!tap_moved &&
                !tap_physical_click &&
                timer_elapsed32(tap_start) <= GEN6_TAP_TERM_MS) {

            report.buttons = tap_fingers == 2 ? 0x02 : 0x01;
            tap_pending_release = true;
            tap_click_time = timer_read32();
            tap_click_button = report.buttons;
        }
    }
    // Reset tracking when fingers are added,
    // removed or replaced.
    if (count != prev_count ||
            id1 != prev_id1 ||
            id2 != prev_id2 ||
            count == 0) {

        prev_count = count;
        prev_id1 = id1;
        prev_id2 = id2;

        prev_x = x;
        prev_y = y;

        accum_x = 0;
        accum_y = 0;
        accum_h = 0;
        accum_v = 0;

        return report;
    }

    int16_t dx = x - prev_x;
    int16_t dy = y - prev_y;

    prev_x = x;
    prev_y = y;

    gen6_rotate(&dx, &dy);

    // Reject implausibly large coordinate jumps.
    if (dx > 250 || dx < -250 ||
            dy > 250 || dy < -250) {
        return report;
    }
    if (count == 1) {
        if (tap_moved ||
                tap_physical_click ||
                timer_elapsed32(tap_start) > GEN6_TAP_TERM_MS) {

            accum_x += dx * GEN6_CURSOR_SENSITIVITY;
            accum_y += dy * GEN6_CURSOR_SENSITIVITY;

            report.x = gen6_extract_delta(&accum_x);
            report.y = gen6_extract_delta(&accum_y);
        }
    }

    if (count == 2) {
#if GEN6_NATURAL_SCROLL
        accum_h -= dx * GEN6_SCROLL_SENSITIVITY;
        accum_v += dy * GEN6_SCROLL_SENSITIVITY;
#else
        accum_h += dx * GEN6_SCROLL_SENSITIVITY;
        accum_v -= dy * GEN6_SCROLL_SENSITIVITY;
#endif

        report.h = gen6_extract_delta(&accum_h);
        report.v = gen6_extract_delta(&accum_v);
    }

    return report;
}
void keyboard_post_init_user(void) {
    i2c_init();
    wait_ms(1000);
}


// ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ D E F I N I T I O N S                                                                                                                      │
// └────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
// ▝▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▘

// ┌───────────────────────────────────────────────────────────┐
// │ d e f i n e   l a y e r s                                 │
// └───────────────────────────────────────────────────────────┘

enum klor_layers {
    /* _M_XYZ = Mac Os, _W_XYZ = Win/Linux */
    _QWERTY,
    _LOWER,
    _RAISE,
    _ADJUST,
};

// ┌───────────────────────────────────────────────────────────┐
// │ d e f i n e   k e y c o d e s                             │
// └───────────────────────────────────────────────────────────┘

enum custom_keycodes {
    QWERTY = SAFE_RANGE,
    LOWER,
    RAISE,
    ADJUST,
    OS_SWAP,
    MAKE_H,
};

// ┌───────────────────────────────────────────────────────────┐
// │ d e f i n e   m a c r o n a m e s                         │
// └───────────────────────────────────────────────────────────┘

// LEFT HAND HOME ROW MODS ├───────────────────────────────────┐

#define GUI_A MT(MOD_LGUI, KC_A)
#define ALT_R MT(MOD_LALT, KC_R)
#define CTL_S MT(MOD_LCTL, KC_S)
#define SHT_T MT(MOD_LSFT, KC_T)

// RIGHT HAND HOME ROW MODS ├───────────────────────────────────┐

#define SHT_N MT(MOD_RSFT, KC_N)
#define CTL_E MT(MOD_LCTL, KC_E)
#define ALT_I MT(MOD_LALT, KC_I)
#define GUI_O MT(MOD_LGUI, KC_O)


// ┌───────────────────────────────────────────────────────────┐
// │ d e f i n e   s o u n d s                                 │
// └───────────────────────────────────────────────────────────┘

#ifdef AUDIO_ENABLE
#define WINXP_SOUND W__NOTE(_DS6), Q__NOTE(_DS5), H__NOTE(_AS5), H__NOTE(_GS5), H__NOTE(_DS5), H__NOTE(_DS6), H__NOTE(_AS5)
#define MAC_SOUND S__NOTE(_CS5), B__NOTE(_C5)

float winxp_song[][2] = SONG(WINXP_SOUND);
float mac_song[][2] = SONG(MAC_SOUND);
#endif // AUDIO_ENABLE

// ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ K E Y M A P S                                                                                                                              │
// └────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
// ▝▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▘

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

    /*
       ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

       ┌───────────────────────────────────────────────────────────┐
       │ q w e r t y                                               │
       └───────────────────────────────────────────────────────────┘
       ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
       │    Q    │    W    │    E    │    R    │    T    │                    │    Y    │    U    │    I    │    O    │    P    │
       ┌─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤                    ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┐
       │   TAB   │    A    │    S    │    D    │    F    │    G    │                    │    H    │    J    │    K    │    L    │    ;    │    '    │
       ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
       │   CTRL  │    Z    │    X    │    C    │    V    │    B    ││  MUTE  ││PLY/PSE ││    N    │    M    │    ,    │    .    │    /    │    \    │
       └─────────┴─────────┴─────────┼─────────┼─────────┼─────────┼╰────────╯╰────────╯┼─────────┼─────────┼─────────┼─────────┴─────────┴─────────┘
       │  LALT   │  LOWER  │  RALT   │  SPACE  ││  BSPCE  │  ENTER  │  RAISE  │    DEL  │
       └─────────┴─────────┴─────────┴─────────┘└─────────┴─────────┴─────────┴─────────┘ */

    [_QWERTY] = LAYOUT_polydactyl(
            //╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷
            KC_Q,    KC_W,    KC_E,     KC_R,      KC_T,                          KC_Y,    KC_U,      KC_I,     KC_O,    KC_P,
            LSFT_T(KC_TAB),  KC_A,    KC_S,    KC_D,   LSFT_T(KC_F),KC_G,                          KC_H, RSFT_T(KC_J), KC_K,     KC_L,    KC_SCLN,  KC_QUOT,
            KC_LCTL, KC_Z,    KC_X,    KC_C,     KC_V,      KC_B,     KC_MUTE,   KC_MPLY,  KC_N,    KC_M,      KC_COMM,  KC_DOT,  KC_SLSH,  KC_BSLS,
            KC_LALT,  LOWER,     KC_RALT,  KC_SPC,    KC_BSPC,  KC_ENT,  RAISE,     KC_DEL
            ),

    /*
       ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

       ┌───────────────────────────────────────────────────────────┐
       │ l o w e r                                                 │
       └───────────────────────────────────────────────────────────┘
       ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
       │   F1    │   F2    │   F3    │   F4    │   F5    │                    │   F6    │   F7    │   F8    │   F9    │   F10   │
       ┌─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤                    ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┐
       │    -    │    !    │    @    │    #    │    $    │    %    │                    │    ^    │    &    │    `    │    ~    │    =    │    +    │
       ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
       │    _    │    1    │    2    │    3    │    4    │    5    ││  MUTE  ││PLY/PASE││    6    │    7    │    8    │    9    │    0    │    *    │
       └─────────┴─────────┴─────────┼─────────┼─────────┼─────────┼╰────────╯╰────────╯┼─────────┼─────────┼─────────┼─────────┴─────────┴─────────┘
       │         │         │         │  LSHIFT ││  RSHIFT │         │         │         │
       └─────────┴─────────┴─────────┴─────────┘└─────────┴─────────┴─────────┴─────────┘ */

    [_LOWER] = LAYOUT_polydactyl(
            //╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷
            KC_F1,   KC_F2,    KC_F3,    KC_F4,    KC_F5,                         KC_F6,    KC_F7,    KC_F8,    KC_F9,    KC_F10,
            KC_MINS,  KC_EXLM, KC_AT,    KC_HASH,  KC_DLR,   KC_PERC,                       KC_CIRC,  KC_AMPR,  KC_GRV,   KC_TILD,  KC_EQL,   KC_PLUS,
            KC_UNDS,  KC_1,    KC_2,     KC_3,     KC_4,     KC_5,    KC_MUTE,   KC_MPLY,   KC_6,     KC_7,     KC_8,     KC_9,     KC_0,     KC_ASTR,
            KC_TRNS,  KC_TRNS,  KC_TRNS, KC_LSFT,   KC_RSFT,   KC_TRNS,  KC_TRNS,  KC_TRNS
            ),
    /*
       ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

       ┌───────────────────────────────────────────────────────────┐
       │ r a i s e                                                 │
       └───────────────────────────────────────────────────────────┘
       ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
       │ RGB_SPI │ RGB_HUI │         │         │    {    │                    │    }    │   LEFT  │   UP    │  RIGHT  │   HOME  │
       ┌─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤                    ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┐
       │ RGB_BRI │ RGB_EFF │         │         │         │    [    │                    │    ]    │         │  DOWN   │         │  PSCRN  │  P UP   │
       ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
       │ RGB_BRD │ RGB_SPD │ RGB_HUD │         │         │    (    ││  MUTE  ││PLY/PASE││    )    │         │         │         │   END   │ P DOWN  │
       └─────────┴─────────┴─────────┼─────────┼─────────┼─────────┼╰────────╯╰────────╯┼─────────┼─────────┼─────────┼─────────┴─────────┴─────────┘
       │         │  ADJUST │         │  LSHIFT ││  RSHIFT │         │         │  ADJUST │
       └─────────┴─────────┴─────────┴─────────┘└─────────┴─────────┴─────────┴─────────┘ */

    [_RAISE] = LAYOUT_polydactyl(
            //╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷
            RGB_SPI,  RGB_HUI,  HF_NEXT,  CK_UP,    KC_LCBR,                       KC_RCBR,  KC_LEFT,  KC_UP,    KC_RGHT,  KC_HOME,
            RGB_VAI,  RGB_MOD,  RGB_M_B,  HF_TOGG,  CK_TOGG,  KC_LBRC,                       KC_RBRC,  KC_TRNS,  KC_DOWN,  KC_TRNS,  KC_PSCR,  KC_PGUP,
            RGB_VAD,  RGB_SPD,  RGB_HUD,  HF_PREV,  CK_DOWN,  KC_LPRN,  KC_MUTE,   KC_MPLY,  KC_RPRN,  KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_END,   KC_PGDN,
            KC_TRNS,  KC_TRNS,  KC_TRNS,  KC_LSFT,   KC_RSFT,  KC_TRNS,  KC_TRNS,  ADJUST
            ),
    /*
       ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

       ┌───────────────────────────────────────────────────────────┐
       │ a d j u s t                                               │
       └───────────────────────────────────────────────────────────┘
       ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
       │ AUDIO   │ HAPTIC  │ RGB HUE │ RGB MOD │         │ ╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮ │         │   F7    │   F8    │   F9    │   F14   │
       ┌─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤ │╰╯╰╯╰╯╰╯╰╯╰╯╰╯╰╯│ ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┐
       │  RESET  │ DEBUG   │ QWERTY  │ RGB SAT │         │         ├─╯                ╰─┤         │   F4    │   F5    │   F6    │   F12   │   F13   │
       ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
       │  MAKE   │ OS SWAP │         │ RGB VAL │         │         ││  MUTE  ││PLY/PSE ││         │   F1    │   F2    │   F3    │   F10   │   F11   │
       └─────────┴─────────┴─────────┼─────────┼─────────┼─────────┼╰────────╯╰────────╯┼─────────┼─────────┼─────────┼─────────┴─────────┴─────────┘
       │    ▼    │    ▼    │    ▼    │    ▼    ││    ▼    │    ▼    │    ▼    │    ▼    │
       └─────────┴─────────┴─────────┴─────────┘└─────────┴─────────┴─────────┴─────────┘ */

    [_ADJUST] = LAYOUT_polydactyl(
            //╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷
            AU_TOGG,   HF_TOGG,  RGB_HUI,  RGB_MOD,  XXXXXXX,                       XXXXXXX,  KC_F7,    KC_F8,    KC_F9,    KC_F14,
            QK_BOOT,  DB_TOGG,   QWERTY,   RGB_SAI,  XXXXXXX,  XXXXXXX,                       XXXXXXX,  KC_F4,    KC_F5,    KC_F6,    KC_F12,   KC_F13,
            MAKE_H,   OS_SWAP,   KC_TRNS,  RGB_VAI,  XXXXXXX,  XXXXXXX,  KC_MUTE,   KC_MPLY,  XXXXXXX,  KC_F1,    KC_F2,    KC_F3,    KC_F10,   KC_F11,
            _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______
            ),

    /*
       ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

       ┌───────────────────────────────────────────────────────────┐
       │ t e m p l a t e   p o l y d a c t y l                     │
       └───────────────────────────────────────────────────────────┘
       ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
       │         │         │         │         │         │ ╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮ │         │         │         │         │         │
       ┌─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤ │╰╯╰╯╰╯╰╯╰╯╰╯╰╯╰╯│ ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┐
       │         │         │         │         │         │         ├─╯                ╰─┤         │         │         │         │         │         │
       ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
       │         │         │         │         │         │         ││        ││        ││         │         │         │         │         │         │
       └─────────┴─────────┴─────────┼─────────┼─────────┼─────────┼╰────────╯╰────────╯┼─────────┼─────────┼─────────┼─────────┴─────────┴─────────┘
       │         │         │         │         ││         │         │         │         │
       └─────────┴─────────┴─────────┴─────────┘└─────────┴─────────┴─────────┴─────────┘

       [_TEMPLATE] = LAYOUT_polydactyl(
    //╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷
    _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______
    )

    ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

    ┌───────────────────────────────────────────────────────────┐
    │ t e m p l a t e   k o n r a d                             │
    └───────────────────────────────────────────────────────────┘
    ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
    │         │         │         │         │         │ ╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮ │         │         │         │         │         │
    ┌─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤ │╰╯╰╯╰╯╰╯╰╯╰╯╰╯╰╯│ ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┐
    │         │         │         │         │         │         ├─╯                ╰─┤         │         │         │         │         │         │
    ├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┼─────────┤
    │         │         │         │         │         │         ││        ││        ││         │         │         │         │         │         │
    └─────────┴─────────┴─────────┼─────────┼─────────┼─────────┤╰────────╯╰────────╯├─────────┼─────────┼─────────┼─────────┴─────────┴─────────┘
    │         │         │         │                    │         │         │         │
    └─────────┴─────────┴─────────┘                    └─────────┴─────────┴─────────┘

    [_TEMPLATE] = LAYOUT_konrad(
    //╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷         ╷
    _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,                       _______,  _______,  _______
    )

    ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

    ┌─────────────────────────────────────────────────┐
    │ t e m p l a t e   y u b i t s u m e             │
    └─────────────────────────────────────────────────┘
    ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
    │         │         │         │         │         │ ╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮ │         │         │         │         │         │
    ├─────────┼─────────┼─────────┼─────────┼─────────┤ │╰╯╰╯╰╯╰╯╰╯╰╯╰╯╰╯│ ├─────────┼─────────┼─────────┼─────────┼─────────┤
    │         │         │         │         │         ├─╯                ╰─┤         │         │         │         │         │
    ├─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┤
    │         │         │         │         │         ││        ││        ││         │         │         │         │         │
    └─────────┴─────────┼─────────┼─────────┼─────────┼╰────────╯╰────────╯┼─────────┼─────────┼─────────┼─────────┴─────────┘
    │         │         │         │         ││         │         │         │         │
    └─────────┴─────────┴─────────┴─────────┘└─────────┴─────────┴─────────┴─────────┘

    [_TEMPLATE] = LAYOUT_yubitsume(
    //╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷
    _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______,  _______,  _______,
    _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______
    )


    ╺━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸

        ┌─────────────────────────────────────────────────┐
        │ t e m p l a t e   s a e g e w e r k             │
        └─────────────────────────────────────────────────┘
        ┌─────────┬─────────┬─────────┬─────────┬─────────┐                    ┌─────────┬─────────┬─────────┬─────────┬─────────┐
        │         │         │         │         │         │ ╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮╭╮ │         │         │         │         │         │
        ├─────────┼─────────┼─────────┼─────────┼─────────┤ │╰╯╰╯╰╯╰╯╰╯╰╯╰╯╰╯│ ├─────────┼─────────┼─────────┼─────────┼─────────┤
        │         │         │         │         │         ├─╯                ╰─┤         │         │         │         │         │
        ├─────────┼─────────┼─────────┼─────────┼─────────┤╭────────╮╭────────╮├─────────┼─────────┼─────────┼─────────┼─────────┤
        │         │         │         │         │         ││        ││        ││         │         │         │         │         │
        └─────────┴─────────┼─────────┼─────────┼─────────┤╰────────╯╰────────╯├─────────┼─────────┼─────────┼─────────┴─────────┘
        │         │         │         │                    │         │         │         │
        └─────────┴─────────┴─────────┘                    └─────────┴─────────┴─────────┘

        [_TEMPLATE] = LAYOUT_saegewerk(
                //╷         ╷         ╷         ╷         ╷         ╷         ╷╷         ╷         ╷         ╷         ╷         ╷         ╷
                _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,
                _______,  _______,  _______,  _______,  _______,                       _______,  _______,  _______,  _______,  _______,
                _______,  _______,  _______,  _______,  _______,  _______,   _______,  _______,  _______,  _______,  _______,  _______,
                _______,  _______,  _______,                       _______,  _______,  _______
                )

        */
};



// ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ L A Y E R   I N D I C A T O R                                                                                                                                    │
// └────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
// ▝▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▘

#ifdef RGB_MATRIX_ENABLE

bool rgb_matrix_indicators_advanced_user(uint8_t led_min, uint8_t led_max) {
    for (uint8_t i = led_min; i < led_max; i++) {
        switch(get_highest_layer(layer_state|default_layer_state)) {
            case 3:
                rgb_matrix_set_color(i, 80, 0, 0);
                break;
            case 2:
                rgb_matrix_set_color(i, 0, 80, 0);
                break;
            case 1:
                rgb_matrix_set_color(i, 0, 0, 80);
                break;
            default:
                break;
        }
    }
    return false;
}

#endif

// ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ O L E D                                                                                                                                    │
// └────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
// ▝▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▘

#ifdef OLED_ENABLE

// ┌───────────────────────────────────────────────────────────┐
// │ d y n a m i c   m a c r o                                 │
// └───────────────────────────────────────────────────────────┘

char layer_state_str[24];
char o_text[24] = "";
int dmacro_num = 0;

#ifdef DYNAMIC_MACRO_ENABLE
char dmacro_text[4][24] = { "", "RECORDING", "STOP RECORDING",  "PLAY RECORDING"};
static uint16_t dmacro_timer;
const char PROGMEM rec_ico[] = {0xD1, 0xE1, 0};
const char PROGMEM stop_ico[] = {0xD3, 0xE1, 0};
const char PROGMEM play_ico[] = {0xD2, 0xE1, 0};

bool dynamic_macro_record_start_user(int8_t direction) {
    dmacro_num = 1;
    return true;
}

bool dynamic_macro_record_end_user(int8_t direction) {
    dmacro_num = 2;
    dmacro_timer = timer_read();
    return true;
}

bool dynamic_macro_play_user(int8_t direction) {
    dmacro_num = 3;
    dmacro_timer = timer_read();
    return true;
}

#endif //DYNAMIC_MACRO_ENABLE


void matrix_scan_user(void) {
#ifdef DYNAMIC_MACRO_ENABLE
    // DynMacroTimer
    if(dmacro_num > 0){
        if (timer_elapsed(dmacro_timer) < 3000) {
            strcpy ( o_text, dmacro_text[dmacro_num] );
        }
        else {
            if (dmacro_num == 1) {
                strcpy ( o_text, dmacro_text[1] );
            }
            else {
                strcpy ( o_text, layer_state_str );
                dmacro_num = 0;
            }
        }
    }
#endif //DYNAMIC_MACRO_ENABLE
}


// ┌───────────────────────────────────────────────────────────┐
// │ o l e d   g r a p h i c s                                 │
// └───────────────────────────────────────────────────────────┘

void render_os_lock_status(void) {
    static const char PROGMEM sep_v[] = {0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0};
    static const char PROGMEM sep_h1[] = {0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0};
    static const char PROGMEM sep_h2[] = {0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0xE1, 0};
    static const char PROGMEM face_1[] = {0x85, 0x86, 0x87, 0x88, 0x89, 0x8A, 0x8B, 0x8C, 0x8D, 0x8E, 0xE1, 0};
    static const char PROGMEM face_2[] = {0xA5, 0xA6, 0xA7, 0xA8, 0xA9, 0xAA, 0xAB, 0xAC, 0xAD, 0xAE, 0xE1, 0};
    static const char PROGMEM face_3[] = {0xE1, 0xE1, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xE1, 0};
    static const char PROGMEM os_m_1[] = {0x95, 0x96, 0};
    static const char PROGMEM os_m_2[] = {0xB5, 0xB6, 0};
    static const char PROGMEM os_w_1[] = {0x97, 0x98, 0};
    static const char PROGMEM os_w_2[] = {0xB7, 0xB8, 0};
    static const char PROGMEM s_lock[] = {0x8F, 0x90, 0};
    static const char PROGMEM n_lock[] = {0x91, 0x92, 0};
    static const char PROGMEM c_lock[] = {0x93, 0x94, 0};
    static const char PROGMEM b_lock[] = {0xE1, 0xE1, 0};
#ifdef AUDIO_ENABLE
    static const char PROGMEM aud_en[] = {0xAF, 0xB0, 0};
    static const char PROGMEM aud_di[] = {0xCF, 0xD0, 0};
#endif
#ifdef HAPTIC_ENABLE
    static const char PROGMEM hap_en[] = {0xB1, 0xB2, 0};
    static const char PROGMEM hap_di[] = {0xCF, 0xD0, 0};
#endif

    // os mode status ────────────────────────────────────────┐

    oled_write_ln_P(sep_v, false);

    if (keymap_config.swap_lctl_lgui) {
        oled_write_P(os_m_1, false); // ──── MAC
    } else {
        oled_write_P(os_w_1, false); // ──── WIN
    }

    oled_write_P(sep_h1, false);
    oled_write_P(face_1, false);

    if (keymap_config.swap_lctl_lgui) {
        oled_write_P(os_m_2, false); // ──── MAC
    } else {
        oled_write_P(os_w_2, false); // ──── WIN
    }

    oled_write_P(sep_h1, false);
    oled_write_P(face_2, false);
    oled_write_P(sep_h1, false);
    oled_write_P(face_3, false);


    // lock key layer status ─────────────────────────────────┐

    led_t led_usb_state = host_keyboard_led_state();

    if (led_usb_state.num_lock) {
        oled_write_P(n_lock, false); // ──── NUMLOCK
    } else {
        oled_write_P(b_lock, false);
    }
    if (led_usb_state.caps_lock) {
        oled_write_P(c_lock, false); // ─── CAPSLOCK
    } else {
        oled_write_P(b_lock, false);
    }
    if (led_usb_state.scroll_lock) { // ─ SCROLLLOCK
        oled_write_P(s_lock, false);
    } else {
        oled_write_P(b_lock, false);
    }

    // hardware feature status ──────────────────────────────┐

    oled_write_P(sep_h2, false);

#ifndef AUDIO_ENABLE
    oled_write_P(b_lock, false);
#endif
#ifndef HAPTIC_ENABLE
    oled_write_P(b_lock, false);
#endif

#ifdef AUDIO_ENABLE // ────────────────── AUDIO
    if (is_audio_on()) {
        oled_write_P(aud_en, false);
    } else {
        oled_write_P(aud_di, false);
    }
#endif // AUDIO ENABLE

#ifdef HAPTIC_ENABLE // ─────────────── HAPTIC
                     //        oled_write_P(hap_en, false);
    if (haptic_get_enable()) {
        oled_write_P(hap_en, false);
    } else {
        oled_write_P(hap_di, false);
    }

#endif // HAPTIC ENABLE
}


// layer status ──────────────────────────────────────────┐

int layerstate = 0;

layer_state_t layer_state_set_kb(layer_state_t state) {
    switch (get_highest_layer(layer_state | default_layer_state)) {
        case 0:
            strcpy ( layer_state_str, "BASE QWERTY");
            break;
        case 1:
            strcpy ( layer_state_str, "LOWER");
            break;
        case 2:
            strcpy ( layer_state_str, "RAISE");
            break;
        case 3:
            strcpy ( layer_state_str, "ADJUST");
            break;
        default:
            strcpy ( layer_state_str, "XXXXXX");
    }
    if (dmacro_num < 1) {
        strcpy ( o_text, layer_state_str );
    }
    //return state;
    return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}


// ┌───────────────────────────────────────────────────────────┐
// │ w r i t e   t o   o l e d                                 │
// └───────────────────────────────────────────────────────────┘

bool oled_task_kb(void) {
    if (!oled_task_user()) {
        return false;
    }
    if (is_keyboard_master()) {  // ────────────────────────── PRIMARY SIDE

        // layer status ──────────────────────────────────────────────────┐
#ifdef DYNAMIC_MACRO_ENABLE
        if(dmacro_num == 1){ oled_write_P(rec_ico, false); }
        if(dmacro_num == 2){ oled_write_P(stop_ico, false); }
        if(dmacro_num == 3){ oled_write_P(play_ico, false); }
#endif //DYNAMIC_MACRO_ENABLE

        oled_write_ln(o_text, false);
        render_os_lock_status();

    } else {  // ─────────────────────────────────────────── SECONDARY SIDE

        // KLOR face ─────────────────────────────────────────────────────┐

        static const char PROGMEM klor_face[] = {
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x80, 0x60, 0x30, 0x18, 0x88, 0xf0, 0x20, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
            0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0x60, 0x30, 0x0c, 0xc6,
            0xe3, 0xfa, 0xdc, 0x80, 0x40, 0x40, 0x40, 0x20, 0x20, 0xa0, 0xa0, 0xe0, 0xc0, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xc0, 0x20,
            0x18, 0x8c, 0xc3, 0xf1, 0xf8, 0xfe, 0xff, 0xff, 0xf7, 0xf2, 0xf2, 0xfb, 0xf9, 0xf9, 0xfd, 0xfc,
            0xfc, 0xff, 0x7f, 0x06, 0x00, 0x00, 0x20, 0x70, 0x88, 0x86, 0xc3, 0xf0, 0xf8, 0xfe, 0xff, 0xff,
            0xff, 0x7f, 0xfc, 0xfe, 0xfe, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x07, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x04,
            0x0e, 0x07, 0x07, 0x07, 0x07, 0x87, 0xe3, 0x9b, 0xef, 0xf3, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0x83, 0x80, 0x80, 0xc0, 0x40, 0x40, 0x40, 0x40, 0x41, 0x41, 0x41, 0x40, 0x40, 0x20, 0x38,
            0xfc, 0x73, 0x7c, 0xbf, 0x3f, 0x3f, 0x3f, 0xbf, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xfe, 0xfe, 0xfe, 0xfa, 0xfa, 0xf3,
            0xe3, 0xe3, 0xc3, 0xc1, 0x81, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x18, 0x0c, 0x04, 0x06, 0x06, 0x06, 0x86, 0xc6, 0x3c, 0x1c, 0x8e, 0xc3,
            0xf1, 0xf8, 0xfe, 0x97, 0x90, 0x98, 0xc8, 0xcf, 0xcf, 0xe7, 0xe4, 0xe4, 0xf8, 0x78, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0,
            0xe0, 0xe0, 0xf0, 0xf0, 0xf8, 0xf8, 0xf8, 0xfc, 0xfc, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xf8, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0,
            0xc0, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0c, 0x12, 0x21, 0x70, 0x3c, 0x3e, 0x3f, 0x3f,
            0x3f, 0x1f, 0x9f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f, 0x0f, 0x07, 0x01, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
            0x04, 0x09, 0x19, 0x13, 0xb3, 0xa7, 0xcf, 0xcf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xdf, 0xbf, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xfe, 0xfc, 0xfc, 0xf8, 0xf0, 0xf0, 0xe0, 0xe0, 0xc0, 0x80,
            0xf0, 0xfe, 0xff, 0xff, 0xff, 0x7f, 0x0f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x1f, 0x11, 0x19, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x08, 0x08, 0x08, 0x08,
            0x08, 0x08, 0x05, 0x00, 0x0f, 0x1f, 0x33, 0x27, 0x47, 0xcf, 0x9f, 0x9f, 0x3f, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0xfd, 0xfd, 0xfd, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
            0xff, 0xff, 0xff, 0x1f, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x2c, 0x24, 0x26, 0x27, 0x27, 0x27, 0x27, 0x27,
            0x27, 0x27, 0x27, 0x23, 0x23, 0x33, 0x13, 0x13, 0x1f, 0x03, 0x07, 0x0f, 0x0f, 0x1f, 0x1f, 0x3f,
            0x1f, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
        };
        oled_write_raw_P(klor_face, sizeof(klor_face));
    }
    return false;
}
#endif // OLED_ENABLE

// ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ M A C R O S                                                                                                                                │
// └────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
// ▝▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▘

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {

        case OS_SWAP:
            if (record->event.pressed) {
                if (!keymap_config.swap_lctl_lgui) {
                    keymap_config.swap_lctl_lgui = true;  // ─── MAC
#ifdef AUDIO_ENABLE
                    PLAY_SONG(mac_song);
#endif // AUDIO_ENABLE
                }
                else {
                    keymap_config.swap_lctl_lgui = false; // ─── WIN
#ifdef AUDIO_ENABLE
                    PLAY_SONG(winxp_song);
#endif // AUDIO_ENABLE
                }
                //              #ifdef HAPTIC_ENABLE
                //                drv2605l_pulse(52);
                //              #endif // HAPTIC_ENABLE
                eeconfig_update_keymap(&keymap_config);
                clear_keyboard();  // ──── clear to prevent stuck keys
                return false;
            }


            // ┌───────────────────────────────────────────────────────────┐
            // │ l a y e r                                                 │
            // └───────────────────────────────────────────────────────────┘
        case QWERTY:
            if (record->event.pressed) {
                set_single_persistent_default_layer(_QWERTY);
                //                #ifdef HAPTIC_ENABLE
                //                  drv2605l_pulse(64);
                //                #endif // HAPTIC_ENABLE
            }
            return false;
        case LOWER:
            if (record->event.pressed) {
                layer_on(_LOWER);
                //                #ifdef HAPTIC_ENABLE
                //                  drv2605l_pulse(17);
                //                #endif // HAPTIC_ENABLE
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            } else {
                layer_off(_LOWER);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            }
            return false;
        case RAISE:
            if (record->event.pressed) {
                layer_on(_RAISE);
                //                #ifdef HAPTIC_ENABLE
                //                  drv2605l_pulse(17);
                //                #endif // HAPTIC_ENABLE
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            } else {
                layer_off(_RAISE);
                update_tri_layer(_LOWER, _RAISE, _ADJUST);
            }
            return false;
        case ADJUST:
            if (record->event.pressed) {
                layer_on(_ADJUST);
                //                #ifdef HAPTIC_ENABLE
                //                  drv2605l_pulse(17);
                //                #endif // HAPTIC_ENABLE
            } else {
                layer_off(_ADJUST);
            }
            return false;

            // ┌───────────────────────────────────────────────────────────┐
            // │ q m k                                                     │
            // └───────────────────────────────────────────────────────────┘

        case MAKE_H:
            if (record->event.pressed) {
#ifdef KEYBOARD_klor_kb2040
                SEND_STRING ("qmk compile -kb klor/2040 -km default");
#else
                SEND_STRING ("qmk compile -kb klor -km default");
#endif
                tap_code(KC_ENTER);
            }
            break;

            // ┌───────────────────────────────────────────────────────────┐
            // │ p r o d u c t i v i t y                                   │
            // └───────────────────────────────────────────────────────────┘

        case KC_MPLY:
            if (record->event.pressed) {
                //          #ifdef HAPTIC_ENABLE
                //                  drv2605l_pulse(4);
                //          #endif // HAPTIC_ENABLE
            }
            break;
    }
    return true;
}


// ┌────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┐
// │ E N C O D E R                                                                                                                              │
// └────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────────┘
// ▝▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▘

#if defined(ENCODER_MAP_ENABLE)
const uint16_t PROGMEM encoder_map[][NUM_ENCODERS][NUM_DIRECTIONS] = {
    [0] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU),           ENCODER_CCW_CW(KC_WBAK, KC_WFWD) },
    [1] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU),           ENCODER_CCW_CW(KC_MPRV, KC_MNXT) },
    [2] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU),           ENCODER_CCW_CW(KC_MPRV, KC_MNXT) },
    [3] = { ENCODER_CCW_CW(KC_VOLD, KC_VOLU),           ENCODER_CCW_CW(KC_MPRV, KC_MNXT) },
};
#endif

/*

   ▐█    ▟▛ ▐█     ▄▆▀▀▀▀▀▀▆▄  ▐█▀▀▀▀▀█▌
   ▐█   ▟▛  ▐█    ▟▛        ▜▙ ▐█     █▌
   ▐█  ▟▛   ▐█   ▐█          █▋▐█     █▌
   ▐█ ▟█▙   ▐█   ▐█          █▋▐█▀▀▜█▀▀▘
   ▐█▟▛ ▜▙  ▐█    ▜▙        ▟▛ ▐█   ▜▙
   ▐█▛   ▜▙ ▐█▄▄▄▄ ▀▜▆▄▄▄▄▆▛▀  ▐█    ▜▙

   ▄██████████████▄
   ████████████████
   ▄██████▀  ▀████▀  ▀██████▄
   ███████▄  ▄████▄  ▄███████
   ███████████▀▀▀▀███████████
   ▀█████████▀ ▄▄ ▀█████████▀
   ████▀ ▄██▄ ▀████
   ████▄▄████▄▄████

*/



