// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.2
// LVGL version: 8.3.6
// Project name: VNC_CONNECT

#include "../ui.h"

void ui_Index_screen_init(void)
{
ui_Index = lv_obj_create(NULL);
lv_obj_clear_flag( ui_Index, LV_OBJ_FLAG_SCROLLABLE );    /// Flags

ui_Panel5 = lv_obj_create(ui_Index);
lv_obj_set_width( ui_Panel5, lv_pct(100));
lv_obj_set_height( ui_Panel5, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel5, LV_ALIGN_TOP_MID );
lv_obj_set_flex_flow(ui_Panel5,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel5, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_END, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel5, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_bg_color(ui_Panel5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT );
lv_obj_set_style_bg_opa(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_width(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel5, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel5, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel5, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel5, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel5, 10, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel5, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel1 = lv_obj_create(ui_Panel5);
lv_obj_set_width( ui_Panel1, lv_pct(100));
lv_obj_set_height( ui_Panel1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel1, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel1,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel1, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel1, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_radius(ui_Panel1, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label1 = lv_label_create(ui_Panel1);
lv_obj_set_width( ui_Label1, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label1, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label1, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label1,"IP / Hostname:");

ui_host = lv_textarea_create(ui_Panel1);
lv_obj_set_width( ui_host, lv_pct(100));
lv_obj_set_height( ui_host, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_host, LV_ALIGN_CENTER );
lv_textarea_set_text(ui_host,"10.10.20.57");
lv_textarea_set_one_line(ui_host,true);



ui_Panel2 = lv_obj_create(ui_Panel1);
lv_obj_set_width( ui_Panel2, lv_pct(100));
lv_obj_set_height( ui_Panel2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Panel2, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel2,LV_FLEX_FLOW_ROW);
lv_obj_set_flex_align(ui_Panel2, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel2, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_border_width(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel2, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel2, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel2, 20, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Panel3 = lv_obj_create(ui_Panel2);
lv_obj_set_height( ui_Panel3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_flex_grow( ui_Panel3, 1);
lv_obj_set_align( ui_Panel3, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel3,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel3, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel3, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_border_width(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel3, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel3, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel3, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label3 = lv_label_create(ui_Panel3);
lv_obj_set_width( ui_Label3, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label3, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label3, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label3,"Port:");

ui_port = lv_textarea_create(ui_Panel3);
lv_obj_set_width( ui_port, lv_pct(100));
lv_obj_set_height( ui_port, LV_SIZE_CONTENT);   /// 70
lv_obj_set_align( ui_port, LV_ALIGN_CENTER );
if ("0123456789"=="") lv_textarea_set_accepted_chars(ui_port, NULL);
else lv_textarea_set_accepted_chars(ui_port, "0123456789");
lv_textarea_set_max_length(ui_port,4);
lv_textarea_set_text(ui_port,"5900");
lv_textarea_set_placeholder_text(ui_port,"5900");
lv_textarea_set_one_line(ui_port,true);



ui_Panel4 = lv_obj_create(ui_Panel2);
lv_obj_set_height( ui_Panel4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_flex_grow( ui_Panel4, 2);
lv_obj_set_x( ui_Panel4, 1 );
lv_obj_set_y( ui_Panel4, 0 );
lv_obj_set_align( ui_Panel4, LV_ALIGN_CENTER );
lv_obj_set_flex_flow(ui_Panel4,LV_FLEX_FLOW_COLUMN);
lv_obj_set_flex_align(ui_Panel4, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
lv_obj_clear_flag( ui_Panel4, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_border_width(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_border_side(ui_Panel4, LV_BORDER_SIDE_NONE, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_left(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_right(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_top(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_bottom(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_row(ui_Panel4, 5, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_pad_column(ui_Panel4, 0, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_Label2 = lv_label_create(ui_Panel4);
lv_obj_set_width( ui_Label2, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label2, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label2, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label2,"Password:");

ui_password = lv_textarea_create(ui_Panel4);
lv_obj_set_width( ui_password, lv_pct(100));
lv_obj_set_height( ui_password, LV_SIZE_CONTENT);   /// 70
lv_obj_set_align( ui_password, LV_ALIGN_CENTER );
lv_textarea_set_text(ui_password,"123456");
lv_textarea_set_one_line(ui_password,true);



ui_connect_btn = lv_btn_create(ui_Panel5);
lv_obj_set_width( ui_connect_btn, 116);
lv_obj_set_height( ui_connect_btn, 50);
lv_obj_set_align( ui_connect_btn, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_connect_btn, LV_OBJ_FLAG_CHECKABLE | LV_OBJ_FLAG_SCROLL_ON_FOCUS );   /// Flags
lv_obj_clear_flag( ui_connect_btn, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
lv_obj_set_style_shadow_width(ui_connect_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_shadow_spread(ui_connect_btn, 0, LV_PART_MAIN| LV_STATE_DEFAULT);
lv_obj_set_style_bg_color(ui_connect_btn, lv_color_hex(0xD6D6D6), LV_PART_MAIN | LV_STATE_CHECKED );
lv_obj_set_style_bg_opa(ui_connect_btn, 255, LV_PART_MAIN| LV_STATE_CHECKED);
lv_obj_set_style_bg_color(ui_connect_btn, lv_color_hex(0xCCCCCC), LV_PART_MAIN | LV_STATE_DISABLED );
lv_obj_set_style_bg_opa(ui_connect_btn, 255, LV_PART_MAIN| LV_STATE_DISABLED);

ui_Label4 = lv_label_create(ui_connect_btn);
lv_obj_set_width( ui_Label4, LV_SIZE_CONTENT);  /// 1
lv_obj_set_height( ui_Label4, LV_SIZE_CONTENT);   /// 1
lv_obj_set_align( ui_Label4, LV_ALIGN_CENTER );
lv_label_set_text(ui_Label4,"Connect");
lv_obj_set_style_text_font(ui_Label4, &lv_font_montserrat_16, LV_PART_MAIN| LV_STATE_DEFAULT);

ui_loading = lv_spinner_create(ui_connect_btn,1000,90);
lv_obj_set_width( ui_loading, 30);
lv_obj_set_height( ui_loading, 30);
lv_obj_set_align( ui_loading, LV_ALIGN_CENTER );
lv_obj_add_flag( ui_loading, LV_OBJ_FLAG_HIDDEN );   /// Flags
lv_obj_clear_flag( ui_loading, LV_OBJ_FLAG_CLICKABLE );    /// Flags
lv_obj_set_style_arc_width(ui_loading, 4, LV_PART_MAIN| LV_STATE_DEFAULT);

lv_obj_set_style_arc_width(ui_loading, 4, LV_PART_INDICATOR| LV_STATE_DEFAULT);

ui_keyboard = lv_keyboard_create(ui_Index);
lv_keyboard_set_mode(ui_keyboard,LV_KEYBOARD_MODE_NUMBER);
lv_obj_set_height( ui_keyboard, 146);
lv_obj_set_width( ui_keyboard, lv_pct(100));
lv_obj_set_align( ui_keyboard, LV_ALIGN_BOTTOM_MID );
lv_obj_add_flag( ui_keyboard, LV_OBJ_FLAG_HIDDEN );   /// Flags

lv_obj_add_event_cb(ui_host, ui_event_host, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_port, ui_event_port, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_password, ui_event_password, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_connect_btn, ui_event_connect_btn, LV_EVENT_ALL, NULL);
lv_obj_add_event_cb(ui_keyboard, ui_event_keyboard, LV_EVENT_ALL, NULL);

}