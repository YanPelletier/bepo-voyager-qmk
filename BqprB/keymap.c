#include QMK_KEYBOARD_H
#include "version.h"
#include "i18n.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  HSV_0_236_141,
  HSV_90_248_81,
  HSV_163_245_128,
  HSV_31_218_204,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
  ST_MACRO_3,
  ST_MACRO_4,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
  DANCE_3,
  DANCE_4,
  DANCE_5,
  DANCE_6,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    KC_ESCAPE,      BP_DQOT,        BP_LGIL,        BP_RGIL,        BP_LPRN,        BP_RPRN,                                        BP_AT,          BP_PLUS,        BP_MINS,        BP_SLSH,        BP_ASTR,        BP_EQL,         
    TD(DANCE_0),    BP_B,           BP_ECUT,        BP_P,           BP_O,           BP_EGRV,                                        BP_DCRC,        TD(DANCE_3),    TD(DANCE_4),    BP_L,           BP_J,           TD(DANCE_5),    
    TD(DANCE_1),    MT(MOD_LGUI, BP_A),MT(MOD_LALT, BP_U),MT(MOD_LCTL, BP_I),MT(MOD_LSFT, BP_E),BP_COMM,                                        TD(DANCE_6),    MT(MOD_RSFT, BP_T),MT(MOD_RCTL, BP_S),MT(MOD_LALT, BP_R),MT(MOD_RGUI, BP_N),BP_M,           
    BP_DLR,         LT(2,BP_AGRV),  BP_Y,           TD(DANCE_2),    BP_DOT,         BP_K,                                           BP_APOS,        BP_Q,           BP_G,           BP_H,           LT(2,BP_F),     BP_W,           
                                                    KC_BSPC,        LT(1,KC_ENTER),                                 OSM(MOD_RALT),  MEH_T(KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_PSCR,        KC_TRANSPARENT, KC_TRANSPARENT,                                 LCTL(KC_LEFT),  LALT(KC_LEFT),  KC_UP,          LALT(KC_RIGHT), LCTL(KC_RIGHT), KC_PAGE_UP,     
    KC_TRANSPARENT, KC_LEFT_GUI,    KC_LEFT_ALT,    KC_LEFT_CTRL,   KC_LEFT_SHIFT,  KC_TRANSPARENT,                                 KC_HOME,        KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_END,         KC_PGDN,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_F12,         
    KC_TRANSPARENT, KC_LEFT_GUI,    MT(MOD_LALT, KC_AUDIO_VOL_DOWN),MT(MOD_LCTL, KC_AUDIO_MUTE),MT(MOD_LSFT, KC_AUDIO_VOL_UP),KC_TRANSPARENT,                                 KC_TRANSPARENT, MT(MOD_RSFT, KC_MEDIA_PREV_TRACK),MT(MOD_RCTL, KC_MEDIA_STOP),MT(MOD_LALT, KC_MEDIA_PLAY_PAUSE),MT(MOD_RGUI, KC_MEDIA_NEXT_TRACK),KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    TO(0),          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_KP_SLASH,    KC_NO,          KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_KP_7,        KC_KP_8,        KC_KP_9,        KC_KP_ASTERISK, KC_NO,          KC_NO,          
    KC_NO,          ST_MACRO_0,     KC_NO,          KC_NO,          ST_MACRO_1,     ST_MACRO_2,                                     KC_KP_4,        KC_KP_5,        KC_KP_6,        KC_KP_MINUS,    KC_NO,          KC_NO,          
    KC_NO,          ST_MACRO_3,     KC_NO,          KC_NO,          ST_MACRO_4,     KC_NO,                                          KC_KP_1,        KC_KP_2,        KC_KP_3,        KC_KP_PLUS,     KC_NO,          KC_NO,          
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_KP_0,        KC_KP_DOT
  ),
  [4] = LAYOUT_voyager(
    TO(0),          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          
    KC_NO,          RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        KC_NO,                                          KC_NO,          RGB_VAD,        RGB_VAI,        RGB_SPD,        RGB_SPI,        KC_NO,          
    KC_NO,          HSV_0_236_141,  HSV_90_248_81,  HSV_163_245_128,HSV_31_218_204, KC_NO,                                          KC_NO,          RGB_HUD,        RGB_HUI,        RGB_SAD,        RGB_SAD,        KC_NO,          
                                                    KC_NO,          KC_NO,                                          KC_NO,          KC_NO
  ),
  [5] = LAYOUT_voyager(
    TO(0),          KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,       
    KC_TAB,         KC_B,           FRCA_16,        KC_P,           KC_O,           FRCA_15,                                        KC_EXLM,        KC_V,           KC_D,           KC_L,           KC_J,           KC_Z,           
    FRCA_29,        MT(MOD_LGUI, KC_A),MT(MOD_LALT, KC_U),MT(MOD_LCTL, KC_I),MT(MOD_LSFT, KC_E),KC_COMMA,                                       KC_C,           MT(MOD_RSFT, KC_T),MT(MOD_RCTL, KC_S),MT(MOD_RALT, KC_R),MT(MOD_RGUI, KC_N),KC_M,           
    KC_DLR,         KC_NO,          KC_Y,           KC_X,           KC_DOT,         KC_K,                                           KC_QUOTE,       KC_Q,           KC_G,           KC_H,           KC_F,           KC_W,           
                                                    KC_BSPC,        KC_ENTER,                                       OSM(MOD_RALT),  MEH_T(KC_SPACE)
  ),
};

const uint16_t PROGMEM combo0[] = { MT(MOD_LSFT, BP_E), MT(MOD_RSFT, BP_T), MT(MOD_RCTL, BP_S), MT(MOD_LCTL, BP_I), COMBO_END};
const uint16_t PROGMEM combo1[] = { BP_EQL, BP_ASTR, BP_SLSH, COMBO_END};
const uint16_t PROGMEM combo2[] = { MEH_T(KC_SPACE), MT(MOD_RSFT, BP_T), MT(MOD_RCTL, BP_S), MT(MOD_LALT, BP_R), MT(MOD_RGUI, BP_N), COMBO_END};
const uint16_t PROGMEM combo3[] = { MT(MOD_RSFT, BP_T), MT(MOD_RCTL, BP_S), MEH_T(KC_SPACE), COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, KC_CAPS),
    COMBO(combo1, TO(5)),
    COMBO(combo2, KC_F24),
    COMBO(combo3, KC_F23),
};


extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {38,255,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,236,141}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,236,141}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {38,255,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,236,141}, {0,0,255}, {38,255,255}, {0,0,255} },

    [1] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {163,246,143}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {44,233,204}, {44,233,204}, {86,238,143}, {44,233,204}, {44,233,204}, {0,0,140}, {18,233,186}, {86,238,143}, {86,238,143}, {86,238,143}, {18,233,186}, {0,0,140}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,255}, {0,0,0}, {0,0,0} },

    [2] = { {0,0,0}, {218,215,148}, {218,215,148}, {218,215,148}, {218,215,148}, {218,215,148}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {154,231,143}, {255,243,150}, {174,231,143}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {218,215,148}, {218,215,148}, {218,215,148}, {218,215,148}, {218,215,148}, {218,215,148}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {218,215,148}, {0,0,0}, {46,201,237}, {255,243,150}, {89,245,103}, {46,201,237}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [3] = { {163,246,143}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,255,132}, {0,0,0}, {0,0,0}, {83,252,85}, {83,252,85}, {0,0,0}, {0,255,132}, {0,0,0}, {0,0,0}, {83,252,85}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {38,180,246}, {0,0,0}, {0,0,0}, {0,0,76}, {0,0,76}, {0,0,76}, {199,174,219}, {0,0,0}, {0,0,0}, {0,0,76}, {0,0,255}, {0,0,76}, {19,221,85}, {0,0,0}, {0,0,0}, {0,0,76}, {0,0,76}, {0,0,76}, {19,251,197}, {0,0,0}, {0,0,0}, {0,0,76}, {0,0,255} },

    [4] = { {163,246,143}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {31,218,204}, {31,218,204}, {31,218,204}, {31,218,204}, {0,0,0}, {0,0,0}, {0,236,141}, {90,248,81}, {163,245,128}, {31,218,204}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {31,218,204}, {31,218,204}, {31,218,204}, {31,218,204}, {0,0,0}, {0,0,0}, {31,218,204}, {31,218,204}, {31,218,204}, {31,218,204}, {0,0,0}, {0,0,0}, {0,0,0} },

    [5] = { {163,246,143}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255}, {0,255,255} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 3:
      set_layer_color(3);
      break;
    case 4:
      set_layer_color(4);
      break;
    case 5:
      set_layer_color(5);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_R) SS_DELAY(100) SS_TAP(X_M) SS_DELAY(100) SS_TAP(X_1));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_P) SS_DELAY(100) SS_TAP(X_M) SS_DELAY(100) SS_TAP(X_1));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_T) SS_DELAY(100) SS_TAP(X_O) SS_DELAY(100) SS_TAP(X_P));
    }
    break;
    case ST_MACRO_3:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_R) SS_DELAY(100) SS_TAP(X_M) SS_DELAY(100) SS_TAP(X_2));
    }
    break;
    case ST_MACRO_4:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_P) SS_DELAY(100) SS_TAP(X_M) SS_DELAY(100) SS_TAP(X_2));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_0_236_141:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(0,236,141);
      }
      return false;
    case HSV_90_248_81:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(90,248,81);
      }
      return false;
    case HSV_163_245_128:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(163,245,128);
      }
      return false;
    case HSV_31_218_204:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(31,218,204);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[7];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case SINGLE_HOLD: register_code16(RSFT(KC_TAB)); break;
        case DOUBLE_TAP: register_code16(KC_TAB); register_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case SINGLE_HOLD: unregister_code16(RSFT(KC_TAB)); break;
        case DOUBLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(BP_CCED);
        tap_code16(BP_CCED);
        tap_code16(BP_CCED);
    }
    if(state->count > 3) {
        tap_code16(BP_CCED);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(BP_CCED); break;
        case SINGLE_HOLD: layer_move(3); break;
        case DOUBLE_TAP: register_code16(BP_CCED); register_code16(BP_CCED); break;
        case DOUBLE_SINGLE_TAP: tap_code16(BP_CCED); register_code16(BP_CCED);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(BP_CCED); break;
        case DOUBLE_TAP: unregister_code16(BP_CCED); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(BP_CCED); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(BP_X);
        tap_code16(BP_X);
        tap_code16(BP_X);
    }
    if(state->count > 3) {
        tap_code16(BP_X);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(BP_X); break;
        case SINGLE_HOLD: register_code16(LCTL(BP_X)); break;
        case DOUBLE_TAP: register_code16(BP_X); register_code16(BP_X); break;
        case DOUBLE_SINGLE_TAP: tap_code16(BP_X); register_code16(BP_X);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(BP_X); break;
        case SINGLE_HOLD: unregister_code16(LCTL(BP_X)); break;
        case DOUBLE_TAP: unregister_code16(BP_X); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(BP_X); break;
    }
    dance_state[2].step = 0;
}
void on_dance_3(tap_dance_state_t *state, void *user_data);
void dance_3_finished(tap_dance_state_t *state, void *user_data);
void dance_3_reset(tap_dance_state_t *state, void *user_data);

void on_dance_3(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(BP_V);
        tap_code16(BP_V);
        tap_code16(BP_V);
    }
    if(state->count > 3) {
        tap_code16(BP_V);
    }
}

void dance_3_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[3].step = dance_step(state);
    switch (dance_state[3].step) {
        case SINGLE_TAP: register_code16(BP_V); break;
        case SINGLE_HOLD: register_code16(LCTL(BP_V)); break;
        case DOUBLE_TAP: register_code16(BP_V); register_code16(BP_V); break;
        case DOUBLE_SINGLE_TAP: tap_code16(BP_V); register_code16(BP_V);
    }
}

void dance_3_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[3].step) {
        case SINGLE_TAP: unregister_code16(BP_V); break;
        case SINGLE_HOLD: unregister_code16(LCTL(BP_V)); break;
        case DOUBLE_TAP: unregister_code16(BP_V); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(BP_V); break;
    }
    dance_state[3].step = 0;
}
void on_dance_4(tap_dance_state_t *state, void *user_data);
void dance_4_finished(tap_dance_state_t *state, void *user_data);
void dance_4_reset(tap_dance_state_t *state, void *user_data);

void on_dance_4(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(BP_D);
        tap_code16(BP_D);
        tap_code16(BP_D);
    }
    if(state->count > 3) {
        tap_code16(BP_D);
    }
}

void dance_4_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[4].step = dance_step(state);
    switch (dance_state[4].step) {
        case SINGLE_TAP: register_code16(BP_D); break;
        case SINGLE_HOLD: layer_move(4); break;
        case DOUBLE_TAP: register_code16(BP_D); register_code16(BP_D); break;
        case DOUBLE_SINGLE_TAP: tap_code16(BP_D); register_code16(BP_D);
    }
}

void dance_4_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[4].step) {
        case SINGLE_TAP: unregister_code16(BP_D); break;
        case DOUBLE_TAP: unregister_code16(BP_D); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(BP_D); break;
    }
    dance_state[4].step = 0;
}
void on_dance_5(tap_dance_state_t *state, void *user_data);
void dance_5_finished(tap_dance_state_t *state, void *user_data);
void dance_5_reset(tap_dance_state_t *state, void *user_data);

void on_dance_5(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(BP_Z);
        tap_code16(BP_Z);
        tap_code16(BP_Z);
    }
    if(state->count > 3) {
        tap_code16(BP_Z);
    }
}

void dance_5_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[5].step = dance_step(state);
    switch (dance_state[5].step) {
        case SINGLE_TAP: register_code16(BP_Z); break;
        case SINGLE_HOLD: register_code16(BP_PERC); break;
        case DOUBLE_TAP: register_code16(BP_Z); register_code16(BP_Z); break;
        case DOUBLE_SINGLE_TAP: tap_code16(BP_Z); register_code16(BP_Z);
    }
}

void dance_5_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[5].step) {
        case SINGLE_TAP: unregister_code16(BP_Z); break;
        case SINGLE_HOLD: unregister_code16(BP_PERC); break;
        case DOUBLE_TAP: unregister_code16(BP_Z); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(BP_Z); break;
    }
    dance_state[5].step = 0;
}
void on_dance_6(tap_dance_state_t *state, void *user_data);
void dance_6_finished(tap_dance_state_t *state, void *user_data);
void dance_6_reset(tap_dance_state_t *state, void *user_data);

void on_dance_6(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(BP_C);
        tap_code16(BP_C);
        tap_code16(BP_C);
    }
    if(state->count > 3) {
        tap_code16(BP_C);
    }
}

void dance_6_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[6].step = dance_step(state);
    switch (dance_state[6].step) {
        case SINGLE_TAP: register_code16(BP_C); break;
        case SINGLE_HOLD: register_code16(LCTL(BP_C)); break;
        case DOUBLE_TAP: register_code16(BP_C); register_code16(BP_C); break;
        case DOUBLE_SINGLE_TAP: tap_code16(BP_C); register_code16(BP_C);
    }
}

void dance_6_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[6].step) {
        case SINGLE_TAP: unregister_code16(BP_C); break;
        case SINGLE_HOLD: unregister_code16(LCTL(BP_C)); break;
        case DOUBLE_TAP: unregister_code16(BP_C); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(BP_C); break;
    }
    dance_state[6].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
        [DANCE_3] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_3, dance_3_finished, dance_3_reset),
        [DANCE_4] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_4, dance_4_finished, dance_4_reset),
        [DANCE_5] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_5, dance_5_finished, dance_5_reset),
        [DANCE_6] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_6, dance_6_finished, dance_6_reset),
};




// Custom QMK here

// SHIFT-BACKSPACE = DELETE
const key_override_t delete_key_override = ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, KC_DEL);

// This globally defines all key overrides to be used
const key_override_t **key_overrides = (const key_override_t *[]){
	&delete_key_override
};