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
#define CLAMP_HID(value) value < -127 ? -127 : value > 127 ? 127 : value

// tapdance keycodes
enum td_keycodes {
  ALT_TM,
  SFT_TM,
  CTL_TM,
  GUI_TM
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
void raisetm_finished (qk_tap_dance_state_t *state, void *user_data);
void raisetm_reset (qk_tap_dance_state_t *state, void *user_data);
void lowertm_finished (qk_tap_dance_state_t *state, void *user_data);
void lowertm_reset (qk_tap_dance_state_t *state, void *user_data);

enum custom_keycodes {
    KC_INTE = SAFE_RANGE,
    KC_RAISE,
    KC_LOWER,
    KC_SCLN_INV,
	KC_QUOT_NOT_US_INT,
	KC_TILD_NOT_US_INT,
	KC_BSPC_LCTL,
	KC_DEL_ALT,
	KC_ENT_LGUI,
	KC_SPC_LSFT,
    KC_CPI_1,
    KC_CPI_2,
    KC_CPI_3
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {

[_COLEMAK] = LAYOUT_5x6(
//KC_EQL,  KC_1  , KC_2  , KC_3  , KC_4  , KC_5  ,                      KC_6  , KC_7  , KC_8  , KC_9  , KC_0        ,KC_QUOT_NOT_US_INT,
KC_CPI_1,           KC_CPI_2, KC_CPI_3, KC_3 , KC_4 , KC_5  ,            KC_CPI_1, KC_CPI_2  , KC_CPI_3   , KC_9  , KC_0        ,KC_QUOT_NOT_US_INT,
KC_TAB,             KC_Q ,    KC_W  ,   KC_F , KC_P , KC_G  ,            KC_J  ,   KC_L  ,     KC_U       , KC_Y  , KC_SCLN_INV ,KC_MINS,
KC_ESC,             KC_A ,    KC_R  ,   KC_S , KC_T , KC_D  ,            KC_H  ,   KC_N  ,     KC_E       , KC_I  , KC_O        ,KC_RAISE,
KC_TILD_NOT_US_INT, KC_Z ,    KC_X  ,   KC_C , KC_V , KC_B  ,            KC_K  ,   KC_M  ,     KC_COMM    , KC_DOT, KC_SLSH     ,KC_BSLASH,
                  KC_MS_BTN1, KC_MS_BTN2,                                       KC_MS_BTN3, KC_LGUI,
                                 TD(SFT_TM),  KC_RAISE,        _______,  KC_SPC_LSFT,
                                 TD(CTL_TM),  TD(GUI_TM),          _______,  KC_ENT_LGUI,
                                 TD(ALT_TM),  KC_LOWER,        KC_BSPC_LCTL, KC_DEL_ALT
),

[_LOWER] = LAYOUT_5x6(

KC_TILD,KC_EXLM   , KC_AT     , KC_HASH   , KC_DLR ,KC_PERC,          KC_CIRC, KC_AMPR       , KC_ASTR    , KC_LPRN  , KC_RPRN   , KC_DEL,
_______,KC_HOME   , KC_PGUP   , KC_PGDN   , KC_END ,_______,          KC_HOME, KC_END        , RALT(KC_Y) , KC_PGUP  , RALT(LSFT(KC_SCLN)) , KC_PLUS,
_______,RALT(KC_Q),RALT(KC_S) ,RALT(KC_S) , KC_RBRC,_______,          _______, LSFT(KC_LBRC) , RALT(KC_5) , KC_PGDN  , RALT(KC_P), KC_MUTE,
KC_F12 ,KC_F1     , KC_F2     , KC_F3     , KC_F4  , KC_F5 ,          KC_F6  , KC_F7         , KC_F8      , KC_F9    , KC_F10    , KC_F11 ,
                              _______,_______,                           _______,_______,
                                        _______,_______,           _______,_______,
                                        _______,_______,           _______,_______,
                                        _______,_______,           _______,_______
),

[_RAISE] = LAYOUT_5x6(
_______,KC_LEFT   , KC_U     , KC_DOWN , KC_RGHT,KC_LPRN,            KC_RPRN, KC_MPRV      , KC_MPLY      , KC_MNXT  , _______   , KC_VOLU,
KC_EQL,  KC_1     , KC_2     , KC_3    , KC_4   , KC_5  ,            KC_6   , KC_7         , KC_8         , KC_9     , KC_0      ,KC_QUOT_NOT_US_INT,
_______,KC_LPRN   , KC_RPRN  , KC_LBRC , KC_RBRC,KC_LBRC,            KC_RBRC, LSFT(KC_LBRC),LSFT(KC_RBRC) ,LSFT(KC_COMM),LSFT(KC_DOT),KC_MS_BTN3,
KC_TILD,KC_EXLM   , KC_AT     ,KC_HASH , KC_DLR ,KC_PERC,            KC_CIRC, KC_AMPR      , KC_ASTR      , KC_PLUS  , KC_EQL   , KC_DEL,
                             _______,_______,                              _______,_______,
                                        _______,_______,           _______,_______,
                                        _______,_______,           _______,_______,
                                        _______,_______,           _______,_______
),
};

uint8_t trackMode = 0; // 0 Mousecursor; 1 arrowkeys/carret; 2 scrollwheel; 3 sound/brightness
uint8_t prev_trackMode = 0;
bool integrationMode = false;
int16_t cumi_x = 0;
int16_t cumi_y = 0;
int16_t cum_x = 0;
int16_t cum_y = 0;
int16_t clamped_x;
int16_t clamped_y;

//identify keycombinations
bool is_sft_active = false;
bool is_ctl_active = false;
bool is_alt_active = false;
bool is_gui_active = false;
bool is_low_active = false;
bool is_rai_active = false;

bool is_spc_active = false;
bool is_del_active = false;
bool is_ent_active = false;
bool is_bsp_active = false;

// Triggers help to move only horizontal or vertical. When accumulated distance triggeres, only move one discrete value in direction with bigger delta.
uint8_t  carret_trigger = 24;       // higher means slower
uint16_t carret_trigger_inte = 340; // in integration mode higher trigger
uint8_t  scroll_trigger = 8;
uint16_t scroll_trigger_inte = 1000;
uint8_t integration_divisor = 100; // slow down every thing in integration mode
float cursor_multiplier = 200;    // adjust cursor speed
float cursor_multiplier_inte = 20;

// determine the tapdance state to return
int cur_dance (qk_tap_dance_state_t *state) {
  if (state->count == 1) {
    //if (state->interrupted || !state->pressed) { return SINGLE_TAP; } //interrupted sends SINGLE_TAP
    if (!state->pressed) { return SINGLE_TAP; } //interrupted sends SINGLE_HOLD
    else { return SINGLE_HOLD; }
  }
  else { return 2; } // any number higher than the maximum state value you return above
}

void alttm_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case SINGLE_TAP:
	  trackMode = 2;
      break;
    case SINGLE_HOLD:
      register_mods(MOD_BIT(KC_LALT));
	  is_alt_active = true;
  }
}
void alttm_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
      unregister_mods(MOD_BIT(KC_LALT));
	  is_alt_active = false;
  }
}
void sfttm_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  switch (td_state) {
    case SINGLE_TAP:
	  trackMode = 1;
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
	  trackMode = 0;
      break;
    case SINGLE_HOLD:
      register_mods(MOD_BIT(KC_LCTL));
	  is_ctl_active = true;
  }
}
void ctltm_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (td_state) {
    case SINGLE_TAP:
      break;
    case SINGLE_HOLD:
      unregister_mods(MOD_BIT(KC_LCTL));
	  is_ctl_active = false;
  }
}
void guitm_finished (qk_tap_dance_state_t *state, void *user_data) {
  td_state = cur_dance(state);
  register_mods(MOD_BIT(KC_LGUI));
  is_gui_active = true;
}
void guitm_reset (qk_tap_dance_state_t *state, void *user_data) {
  unregister_mods(MOD_BIT(KC_LGUI));
  is_gui_active = false;
}

// define `ACTION_TAP_DANCE_FN_ADVANCED()` for each tapdance keycode, passing in `finished` and `reset` functions
qk_tap_dance_action_t tap_dance_actions[] = {
  [SFT_TM]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL, sfttm_finished  , sfttm_reset),
  [CTL_TM]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL, ctltm_finished  , ctltm_reset),
  [ALT_TM]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL, alttm_finished  , alttm_reset),
  [GUI_TM]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL, guitm_finished  , guitm_reset),
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

void pointing_device_init(void){
    if(!is_keyboard_master())
        return;
    pmw_init();
}

int sign(int x) {
    return (x > 0) - (x < 0);
}
int max(int num1, int num2)
{
    return (num1 > num2 ) ? num1 : num2;
}
int min(int num1, int num2) 
{
    return (num1 > num2 ) ? num2 : num1;
}

int16_t factor;

void tap_tb(uint16_t keycode0, uint16_t keycode1, uint16_t keycode2, uint16_t keycode3) {
	if(abs(cum_x) + abs(cum_y) >= factor){
		if(abs(cum_x) > abs(cum_y)) {
			if(cum_x > 0) {
				for (int i = 0; i <= (abs(cum_x) + abs(cum_y)) / factor; i++) {
					tap_code(keycode0);
					cum_x = max(cum_x - factor, 0);
				}
				cum_y = 0;
			} else {
				for (int i = 0; i <= (abs(cum_x) + abs(cum_y)) / factor; i++) {
					tap_code(keycode1);
					cum_x = min(cum_x + factor, 0);
				}
				cum_y = 0;
			}
		} else {
			if(cum_y > 0) {
				for (int i = 0; i <= (abs(cum_x) + abs(cum_y)) / factor; i++) {
					tap_code(keycode2);
					cum_y = max(cum_y - factor, 0);
					}
				cum_x = 0;
			} else {
				for (int i = 0; i <= (abs(cum_x) + abs(cum_y)) / factor; i++) {
					tap_code(keycode3);
					cum_y = min(cum_y + factor, 0);
				}
				cum_x = 0;
			}
		}
	}
}

void pointing_device_task(void){
    if(!is_keyboard_master())
        return;

    report_mouse_t mouse_report = pointing_device_get_report();
    report_pmw_t pmw_report = pmw_get_report();

	if (integrationMode) {
		cumi_x += pmw_report.x;
		cumi_y += pmw_report.y;
		clamped_x = CLAMP_HID(cumi_x);
		clamped_y = CLAMP_HID(cumi_y);
	} else {
		clamped_x = CLAMP_HID(pmw_report.x);
		clamped_y = CLAMP_HID(pmw_report.y);
	}

    if (trackMode != 0) {
        // accumulate movement until triggered
		cum_x = cum_x + clamped_x;
		cum_y = cum_y + clamped_y;
	}
    if (trackMode == 0) { //cursor (0)
		if (integrationMode)
			factor = cursor_multiplier_inte;
		else
			factor = cursor_multiplier;
        mouse_report.x = ( clamped_x * factor) / 100;
        mouse_report.y = (-clamped_y * factor) / 100;
    } else if (trackMode == 1) { //carret (1)
		if (integrationMode)
			factor = carret_trigger_inte;
		else
			factor = carret_trigger;
		tap_tb(KC_RIGHT, KC_LEFT, KC_UP, KC_DOWN);
	} else if(trackMode == 2) { //scroll
		if (integrationMode)
			factor = scroll_trigger_inte;
		else
			factor = scroll_trigger;
		if(abs(cum_x) + abs(cum_y) >= factor){
			if(abs(cum_x) > abs(cum_y)) {
				mouse_report.h = sign(cum_x) * (abs(cum_x) + abs(cum_y)) / factor;
			} else {
				mouse_report.v = sign(cum_y) * (abs(cum_x) + abs(cum_y)) / factor;
			}
			cum_x = 0;
			cum_y = 0;
		}
	} else { // sound vol/brightness (3)
		factor = carret_trigger;
		tap_tb(KC_BRIGHTNESS_UP, KC_BRIGHTNESS_DOWN, KC_AUDIO_VOL_UP, KC_AUDIO_VOL_DOWN);
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

		case KC_LOWER:
    	  if (record->event.pressed) {
			layer_on(_LOWER);
	  		is_low_active = true;
	  		prev_trackMode = trackMode;
	  		trackMode = 3;
		  } else {
			layer_off(_LOWER);
	  		is_low_active = false;
	  		trackMode = prev_trackMode;
		  }
          return false;

		case KC_RAISE:
    	  if (record->event.pressed) {
			layer_on(_RAISE);
	  		cumi_x = 0;
	  		cumi_y = 0;
	  		integrationMode = true;
	  		is_rai_active = true;
		  } else {
			layer_off(_RAISE);
	  		cumi_x = 0;
	  		cumi_y = 0;
  		    integrationMode = false;
  		    is_rai_active = false;
		  }
          return false;

		case KC_SPC_LSFT:
    	  if (record->event.pressed) {
			if (is_ctl_active || is_alt_active || is_gui_active || is_rai_active || is_low_active) {
			  register_mods(MOD_BIT(KC_LSFT));
			  is_spc_active = true;
			} else {
			  register_code(KC_SPC);
			}
		  } else {
			if (is_spc_active) {
			  unregister_mods(MOD_BIT(KC_LSFT));
			  is_spc_active = false;
			} else {
			  unregister_code(KC_SPC);
			}
		  }
          return false;

		case KC_ENT_LGUI:
    	  if (record->event.pressed) {
			if (is_ctl_active || is_alt_active || is_sft_active || is_rai_active || is_low_active) {
			  register_mods(MOD_BIT(KC_LGUI));
			  is_ent_active = true;
			} else {
			  register_code(KC_ENT);
			}
		  } else {
			if (is_ent_active) {
			  unregister_mods(MOD_BIT(KC_LGUI));
			  is_ent_active = false;
			} else {
			  unregister_code(KC_ENT);
			}
		  }
          return false;

		case KC_DEL_ALT:
    	  if (record->event.pressed) {
			if (is_ctl_active || is_sft_active || is_gui_active || is_rai_active || is_low_active) {
			  register_mods(MOD_BIT(KC_LALT));
			  is_del_active = true;
			} else {
			  register_code(KC_DEL);
			}
		  } else {
			if (is_del_active) {
			  unregister_mods(MOD_BIT(KC_LALT));
			  is_del_active = false;
			} else {
			  unregister_code(KC_DEL);
			}
		  }
          return false;

		case KC_BSPC_LCTL:
    	  if (record->event.pressed) {
			if (is_sft_active || is_alt_active || is_gui_active || is_rai_active || is_low_active) {
			  register_mods(MOD_BIT(KC_LCTL));
			  is_bsp_active = true;
			} else {
			  register_code(KC_BSPC);
			}
		  } else {
			if (is_bsp_active) {
			  unregister_mods(MOD_BIT(KC_LCTL));
			  is_bsp_active = false;
			} else {
			  unregister_code(KC_BSPC);
			}
		  }
          return false;

		// no repetitive ::: with holding
		case KC_SCLN_INV:
    	  if (record->event.pressed) {
			if (is_sft_active) {
			  unregister_mods(MOD_BIT(KC_LSFT));
			  tap_code(KC_SCLN);
			  register_mods(MOD_BIT(KC_LSFT));
			} else {
			  register_mods(MOD_BIT(KC_LSFT));
			  tap_code(KC_SCLN);
			  unregister_mods(MOD_BIT(KC_LSFT));
			}
		  }
          return false;

		// ; and : gets confused in some corner cases
		//case KC_SCLN_INV:
    	//  if (record->event.pressed) {
		//	if (is_sft_active) {
		//	  unregister_mods(MOD_BIT(KC_LSFT));
		//	  register_code(KC_SCLN);
		//	} else {
		//	  register_mods(MOD_BIT(KC_LSFT));
		//	  register_code(KC_SCLN);
		//	}
		//  } else {
		//	if (is_sft_active) {
		//	  unregister_code(KC_SCLN);
		//	  register_mods(MOD_BIT(KC_LSFT));
		//	} else {
		//	  unregister_code(KC_SCLN);
		//	  unregister_mods(MOD_BIT(KC_LSFT));
		//	}
		//  }
        //  return false;

		case KC_TILD_NOT_US_INT:
    	  if (record->event.pressed) {
			  tap_code16(KC_TILD);
			  tap_code(KC_SPC);
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

        case KC_CPI_1:
			if (cursor_multiplier > 20)
				cursor_multiplier = cursor_multiplier - 20;
            return false;

        case KC_CPI_2:
			cursor_multiplier = 160;
            return false;

        case KC_CPI_3:
			cursor_multiplier = cursor_multiplier + 20;
            return false;

		default:
    	  return true;
  }
}

//#ifndef POLLING
//	ISR(INT2_vect) {
//	    motion_time = timer_read32() + 50;
//	}
//#endif
