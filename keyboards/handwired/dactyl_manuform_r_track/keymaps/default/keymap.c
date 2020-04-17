/* Copyright 2020 Qurn
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
#include QMK_KEYBOARD_H

#define _COLEMAK 0
#define _LOWER 1
#define _RAISE 2

#define RAISE MO(_RAISE)
#define LOWER MO(_LOWER)

#include "pointing_device.h"
#include "../../pmw3360/pmw3360.h"
#define SCROLL_DIVIDER 12
#define CPI_1 2000
#define CPI_2 4000
#define CPI_3 8000
#define CLAMP_HID(value) value < -127 ? -127 : value > 127 ? 127 : value

// tapdance keycodes
enum td_keycodes {
  ALT_TM, // 
  SFT_TM, // 
  CTL_TM, // 
  RAISE_M3 // 
};

// define a type containing as many tapdance states as you need
typedef enum {
  SINGLE_TAP,
  SINGLE_HOLD
} td_state_t;

// create a global instance of the tapdance state type
static td_state_t td_state;

// declare your tapdance functions:

// function to determine the current tapdance state
int cur_dance (qk_tap_dance_state_t *state);

// `finished` and `reset` functions for each tapdance keycode
void alttm_finished (qk_tap_dance_state_t *state, void *user_data);
void alttm_reset (qk_tap_dance_state_t *state, void *user_data);
void sfttm_finished (qk_tap_dance_state_t *state, void *user_data);
void sfttm_reset (qk_tap_dance_state_t *state, void *user_data);
void ctltm_finished (qk_tap_dance_state_t *state, void *user_data);
void ctltm_reset (qk_tap_dance_state_t *state, void *user_data);
void raisem3_finished (qk_tap_dance_state_t *state, void *user_data);
void raisem3_reset (qk_tap_dance_state_t *state, void *user_data);

enum custom_keycodes {
    KC_INTE = SAFE_RANGE,
    KC_CURS,
    KC_SCRL,
    KC_CHRT,
    KC_SCLN_INV,
	KC_QUOT_NOT_US_INT,
    KC_CPI_1,
    KC_CPI_2,
    KC_CPI_3
};

typedef union {
  uint32_t raw;
  struct {
    uint16_t cpi;
  };
} config_dmrt_t;


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

  [_COLEMAK] = LAYOUT_5x6(
     KC_EQL,  KC_1  , KC_2  , KC_3  , KC_4  , KC_5  ,                        KC_6  , KC_7  , KC_8  , KC_9  , KC_0        ,KC_QUOT_NOT_US_INT,
	 KC_TAB,  KC_Q  , KC_W  , KC_F  , KC_P  , KC_G  ,                        KC_J  , KC_L  , KC_U  , KC_Y  , KC_SCLN_INV ,KC_MINS,
     KC_ESC,  KC_A  , KC_R  , KC_S  , KC_T  , KC_D  ,                        KC_H  , KC_N  , KC_E  , KC_I  , KC_O        ,TD(RAISE_M3),
     KC_TILD, KC_Z  , KC_X  , KC_C  , KC_V  , KC_B  ,                        KC_K  , KC_M  ,KC_COMM, KC_DOT, KC_SLSH     ,KC_BSLASH,
                       KC_MS_BTN1, KC_MS_BTN2,                                             KC_MS_BTN3, KC_LGUI,
                                      TD(SFT_TM),  KC_ENT,                   _______,  KC_SPC,
                                      TD(CTL_TM),  TD(RAISE_M3),             _______,  KC_LGUI,
                                      TD(ALT_TM),  LOWER,                    KC_BSPC,  KC_DEL
  ),

  [_LOWER] = LAYOUT_5x6(

     KC_TILD,KC_EXLM   , KC_AT    ,KC_HASH   , KC_DLR ,KC_PERC,                        KC_CIRC, KC_AMPR       , KC_ASTR    , KC_LPRN  , KC_RPRN   , KC_DEL,
     _______,_______   ,_______   ,_______   ,_______ ,_______,                        _______, _______       , RALT(KC_Y) , _______  , _______   , KC_PLUS,
     _______,RALT(KC_Q),RALT(KC_S),RALT(KC_S), KC_RBRC,_______,                        _______, LSFT(KC_LBRC) , RALT(KC_5) , _______  , RALT(KC_P), KC_MUTE,
     _______,KC_HOME   ,KC_PGUP   ,KC_PGDN   , KC_END ,_______,                        _______, _______       , _______    , KC_P6    , KC_MINS   , KC_PIPE,
                                  _______,_______,                                _______,_______,
                                             _______,_______,            _______,_______,
                                             _______,_______,            _______,_______,
                                             _______,_______,            _______,_______

  ),

  [_RAISE] = LAYOUT_5x6(
       _______,KC_LEFT   , KC_U     , KC_DOWN , KC_RGHT,KC_LPRN,                  KC_RPRN, KC_MPRV      , KC_MPLY      , KC_MNXT  , _______   , KC_VOLU,
       _______, _______  , _______  , _______ , _______,_______,                  KC_TILD, KC_MS_BTN3   , RALT(KC_Y)   ,_______   , _______   , KC_VOLD,
       _______,KC_LPRN   ,KC_RPRN   , KC_LBRC , KC_RBRC,KC_LBRC,                  KC_RBRC, LSFT(KC_LBRC), LSFT(KC_RBRC),RALT(KC_P), RALT(KC_P), KC_MS_BTN3,
       KC_F12 ,KC_F1     , KC_F2    , KC_F3   , KC_F4  , KC_F5 ,                  KC_F6  , KC_F7        , KC_F8        , KC_F9    , KC_F10    , KC_F11 ,
                                    _______,_______,                                   _______,_______,
                                               _______,_______,            _______,_______,
                                               _______,_______,            _______,_______,
                                               _______,_______,            _______,_______
  ),
};

uint8_t trackMode = 0; // 0 Mousecursor; 1 arrowkeys/carret; 2 scrollwheel
bool integrationMode = false;
int16_t cumi_x = 0;
int16_t cumi_y = 0;
bool is_sft_active = false;

// determine the tapdance state to return
int cur_dance (qk_tap_dance_state_t *state) {
  if (state->count == 1) {
    //if (state->interrupted || !state->pressed) { return SINGLE_TAP; }
    if (!state->pressed) { return SINGLE_TAP; }
    else { return SINGLE_HOLD; }
  }
  else { return 2; } // any number higher than the maximum state value you return above
}

// handle the possible states for each tapdance keycode you define:
void alttm_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case SINGLE_TAP:
	  if (trackMode == 2) {
		//tap_mods(MOD_BIT(KC_LALT));
	  } else {
		trackMode = 2;
	  }
      break;
    case SINGLE_HOLD:
      register_mods(MOD_BIT(KC_LALT));
  }
}
void alttm_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
      unregister_mods(MOD_BIT(KC_LALT));
  }
}
void sfttm_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case SINGLE_TAP:
	  if (trackMode == 1) {
		//tap_mods(MOD_BIT(KC_LSFT));
	  } else {
		trackMode = 1;
	  }
      break;
    case SINGLE_HOLD:
      register_mods(MOD_BIT(KC_LSFT));
	  is_sft_active = true;
  }
}
void sfttm_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
      unregister_mods(MOD_BIT(KC_LSFT));
	  is_sft_active = false;
  }
}
void ctltm_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case SINGLE_TAP:
	  if (trackMode == 0) {
		//tap_mods(MOD_BIT(KC_LSFT));
	  } else {
		trackMode = 0;
	  }
      break;
    case SINGLE_HOLD:
      register_mods(MOD_BIT(KC_LCTL));
  }
}
void ctltm_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
      unregister_mods(MOD_BIT(KC_LCTL));
  }
}

void raisem3_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
	  layer_on(_RAISE);
	  cumi_x = 0;
	  cumi_y = 0;
	  integrationMode = true;
  }
}
void raisem3_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
	  layer_off(_RAISE);
	  integrationMode = false;
  }
}

// define `ACTION_TAP_DANCE_FN_ADVANCED()` for each tapdance keycode, passing in `finished` and `reset` functions
qk_tap_dance_action_t tap_dance_actions[] = {
  [SFT_TM] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, sfttm_finished, sfttm_reset),
  [CTL_TM] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, ctltm_finished, ctltm_reset),
  [ALT_TM] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, alttm_finished, alttm_reset),
  [RAISE_M3] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, raisem3_finished, raisem3_reset)
};

void on_mouse_button(uint8_t mouse_button, bool pressed) {
    report_mouse_t report = pointing_device_get_report();

    if(pressed)
        report.buttons |= mouse_button;
    else
        report.buttons &= ~mouse_button;
    pointing_device_set_report(report);
	pointing_device_send();
}

void on_cpi_button(int16_t cpi) {

    // read cpi first to prevent unnecessary writes to EEPROM
    if(pmw_get_config().cpi == cpi)
        return;

    pmw_set_config((config_pmw_t){ cpi });

    config_dmrt_t kb_config;
    kb_config.cpi = cpi;
    eeconfig_update_kb(kb_config.raw);
}

void pointing_device_init(void){
    if(!is_keyboard_master())
        return;

    pmw_init();
    // read config from EEPROM and update if needed

    config_dmrt_t kb_config;
    kb_config.raw = eeconfig_read_kb();

    if(!kb_config.cpi) {
        kb_config.cpi = CPI_2;
        eeconfig_update_kb(kb_config.raw);
    }

    pmw_set_config((config_pmw_t){ kb_config.cpi });
}

int16_t cum_x = 0;
int16_t cum_y = 0;

// Triggers help to move only horizontal or vertical. When accumulated distance triggeres, only move one discrete value in direction with bigger delta.
uint8_t carret_trigger = 80;
uint8_t scroll_trigger = 32;

float cursor_multiplier = 0.4; // adjust cursor speed
uint8_t integration_divisor = 50; // slow down every mode in integration mode

void tap_tb(int16_t delta, uint16_t keycode0, uint16_t keycode1) {
	if(delta > 0) {
		tap_code(keycode0);
	} else if(delta < 0) {
		tap_code(keycode1);
	}
}

int16_t clamped_x;
int16_t clamped_y;
int sign(int x) {
    return (x > 0) - (x < 0);
}
void pointing_device_task(void){
    if(!is_keyboard_master())
        return;

    report_mouse_t mouse_report = pointing_device_get_report();
    report_pmw_t pmw_report = pmw_get_report();

	if (integrationMode) {
		cumi_x += pmw_report.x;
		cumi_y += pmw_report.y;
		if (trackMode == 1) { //cursor
			clamped_x = CLAMP_HID(cumi_x / (integration_divisor / 2));
			clamped_y = CLAMP_HID(cumi_y / (integration_divisor / 2));
		} else {
			clamped_x = CLAMP_HID(cumi_x / integration_divisor);
			clamped_y = CLAMP_HID(cumi_y / integration_divisor);
		}
	} else {
		clamped_x = CLAMP_HID(pmw_report.x);
		clamped_y = CLAMP_HID(pmw_report.y);
	}
    if(trackMode == 2) {//scroll
        // accumulate scroll untill triggered
		cum_x = CLAMP_HID(cum_x + clamped_x);
		cum_y = CLAMP_HID(cum_y + clamped_y);
		if(abs(cum_x) + abs(cum_y) >= scroll_trigger){
			if(abs(cum_x) > abs(cum_y)) {
				mouse_report.h = sign(cum_x);
			} else {
				mouse_report.v = sign(cum_y);
			}
			cum_x = 0;
			cum_y = 0;
		}
    }
	else if (trackMode == 0) { //cursor
        mouse_report.x = (int)(clamped_x * cursor_multiplier);
        mouse_report.y = (int)(-clamped_y * cursor_multiplier);
    } else { //carret
		cum_x = CLAMP_HID(cum_x + clamped_x);
		cum_y = CLAMP_HID(cum_y + clamped_y);
		if(abs(cum_x) + abs(cum_y) >= carret_trigger){
			if(abs(cum_x) > abs(cum_y)) {
				tap_tb(cum_x, KC_RIGHT, KC_LEFT);
			} else {
				tap_tb(cum_y,  KC_UP, KC_DOWN  );
			}
			cum_x = 0;
			cum_y = 0;
		}
	}
    pointing_device_set_report(mouse_report);
    pointing_device_send();
}

bool process_record_kb(uint16_t keycode, keyrecord_t *record) {
    if(!process_record_user(keycode, record)) {
        return false;
	}

    // handle mouse drag and scroll
    switch (keycode) {
		case KC_SCLN_INV:
    	  if (record->event.pressed) {
			if (is_sft_active) {
			  unregister_mods(MOD_BIT(KC_LSFT));
			  register_code(KC_SCLN);
			} else {
			  register_mods(MOD_BIT(KC_LSFT));
			  register_code(KC_SCLN);
			}
		  } else {
			if (is_sft_active) {
			  unregister_code(KC_SCLN);
			  register_mods(MOD_BIT(KC_LSFT));
			} else {
			  unregister_code(KC_SCLN);
			  unregister_mods(MOD_BIT(KC_LSFT));
			}
		  }
          return false;

		case KC_QUOT_NOT_US_INT:
    	  if (record->event.pressed) {
			  tap_code(KC_QUOT);
			  tap_code(KC_SPC);
		  }
		  return false;

        case KC_BTN1:
            on_mouse_button(MOUSE_BTN1, record->event.pressed);
            return false;

        case KC_BTN2:
            on_mouse_button(MOUSE_BTN2, record->event.pressed);
            return false;

        case KC_BTN3:
            on_mouse_button(MOUSE_BTN3, record->event.pressed);
            return false;

        case KC_BTN4:
            on_mouse_button(MOUSE_BTN4, record->event.pressed);
            return false;

        case KC_BTN5:
            on_mouse_button(MOUSE_BTN5, record->event.pressed);
            return false;

		default:
    	  return true;
  }
}

#ifndef POLLING
	ISR(INT2_vect) {
	    motion_time = timer_read32() + 50;
	}
#endif

