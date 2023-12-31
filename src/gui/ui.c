// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: VNC_CONNECT

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////


// SCREEN: ui_Index
void ui_Index_screen_init(void);
lv_obj_t *ui_Index;
lv_obj_t *ui_Panel5;
lv_obj_t *ui_Panel1;
lv_obj_t *ui_Label1;
void ui_event_host( lv_event_t * e);
lv_obj_t *ui_host;
lv_obj_t *ui_Panel2;
lv_obj_t *ui_Panel3;
lv_obj_t *ui_Label3;
void ui_event_port( lv_event_t * e);
lv_obj_t *ui_port;
lv_obj_t *ui_Panel4;
lv_obj_t *ui_Label2;
void ui_event_password( lv_event_t * e);
lv_obj_t *ui_password;
void ui_event_connect_btn( lv_event_t * e);
lv_obj_t *ui_connect_btn;
lv_obj_t *ui_Label4;
lv_obj_t *ui_loading;
void ui_event_keyboard( lv_event_t * e);
lv_obj_t *ui_keyboard;
lv_obj_t *ui____initial_actions0;

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_host( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_FOCUSED) {
      _ui_flag_modify( ui_keyboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      _ui_keyboard_set_target(ui_keyboard,  ui_host);
}
}
void ui_event_port( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_FOCUSED) {
      _ui_flag_modify( ui_keyboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      _ui_keyboard_set_target(ui_keyboard,  ui_port);
}
}
void ui_event_password( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_FOCUSED) {
      _ui_flag_modify( ui_keyboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
      _ui_keyboard_set_target(ui_keyboard,  ui_password);
}
}
void ui_event_connect_btn( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_VALUE_CHANGED &&  !lv_obj_has_state(target,LV_STATE_CHECKED)  ) {
      _ui_flag_modify( ui_loading, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
}
}
void ui_event_keyboard( lv_event_t * e) {
    lv_event_code_t event_code = lv_event_get_code(e);lv_obj_t * target = lv_event_get_target(e);
if ( event_code == LV_EVENT_READY) {
      _ui_flag_modify( ui_keyboard, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
}
}

///////////////////// SCREENS ////////////////////

void ui_init( void )
{
lv_disp_t *dispp = lv_disp_get_default();
lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), false, LV_FONT_DEFAULT);
lv_disp_set_theme(dispp, theme);
ui_Index_screen_init();
ui____initial_actions0 = lv_obj_create(NULL);
lv_disp_load_scr( ui_Index);
}
