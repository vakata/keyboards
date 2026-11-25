/* Copyright 2022 James White <jamesmnw@gmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#define EE_HANDS

#define VIAL_KEYBOARD_UID {0xF8, 0x9B, 0xFD, 0x00, 0xEE, 0xC4, 0x9F, 0xE0}
#define VIAL_COMBO_ENTRIES 35
#define VIAL_TAP_DANCE_ENTRIES 20
#define LAYER_STATE_16BIT
#define DYNAMIC_KEYMAP_LAYER_COUNT 3
#define DYNAMIC_KEYMAP_MACRO_COUNT 0

#define TAPPING_TERM 200
#define TAP_HOLD_CAPS_DELAY 80
#define ONESHOT_TAP_TOGGLE 3
#define ONESHOT_TIMEOUT 5000
#define AUTO_SHIFT_TIMEOUT 180
#define AUTO_SHIFT_REPEAT

// OLED driver
#ifdef OLED_DRIVER_ENABLE
  #define OLED_DISPLAY_128X32
  #define OLED_TIMEOUT 30000
#endif
