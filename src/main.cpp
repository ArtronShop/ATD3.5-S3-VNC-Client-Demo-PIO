#include <Arduino.h>
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <Wire.h>
#include <FT6336.h>
#include <TFT_eSPI.h>
#include <VNC_TFT_eSPI.h>
#include <VNC.h>

#include "gui/ui.h"

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

#define LCD_BL_PIN (3)

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * 10 ];

#define I2C_SDA_PIN (15)
#define I2C_SCL_PIN (16)

VNC_TFT_eSPI tft = VNC_TFT_eSPI();
arduinoVNC vnc = arduinoVNC(&tft);


/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t *indev_driver, lv_indev_data_t * data) {
  uint8_t touchPoint = touch.read((uint16_t*)(&data->point.x), (uint16_t*)(&data->point.y));
  if (touchPoint > 0) {
    // Serial.printf("X: %d\tY: %d\n", data->point.x, data->point.y);
  }
  data->state = touchPoint > 0 ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
}

volatile bool vnc_use = false;


const char* ssid = "Artron@Kit";
const char* password = "Kit_Artron";

void setup() {
  Serial.begin( 115200 ); /* prepare for possible serial debug */
  
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println();
  Serial.println();

  WiFi.begin(ssid, password);

  lv_init();

  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */
    // tft.invertDisplay(false);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400E3);
  touch.init();

  lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * 10 );

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);
    
  ui_init();

  lv_obj_clear_flag(ui_connect_btn, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_add_event_cb(ui_connect_btn, [](lv_event_t * e) {
    lv_obj_add_state(ui_connect_btn, LV_STATE_CHECKED);
    lv_obj_clear_flag(ui_loading, LV_OBJ_FLAG_HIDDEN);

    const char * host = lv_textarea_get_text(ui_host);
    int port = atoi(lv_textarea_get_text(ui_port));
    const char * password = lv_textarea_get_text(ui_password);
    vnc.begin(host, port);
    if (strlen(password) > 0) {
      vnc.setPassword(password);
    }
    vnc_use = true;
  }, LV_EVENT_CLICKED, NULL);

  pinMode(LCD_BL_PIN, OUTPUT);
  digitalWrite(LCD_BL_PIN, HIGH);
}

void loop() {
  if (!vnc_use) {
    lv_timer_handler(); /* let the GUI do its work */
    delay(5);
  } else {
    if(WiFi.status() != WL_CONNECTED) {
        vnc.reconnect();
        delay(100);
    } else {
        if(vnc.connected()) {
            // Touch
            static bool last_pressed = false;
            static uint16_t last_x = 0;
            static uint16_t last_y = 0;
            uint16_t x = 0, y = 0;
            uint8_t touch_point = touch.read(&x, &y);
            if (touch_point > 0) {
                if ((last_x != x) || (last_y != y)) {
                    vnc.mouseEvent(x, y, 0b0001);
                    Serial.printf("Point: %d, x: %d, y: %d\n", touch_point, x, y);
                    last_x = x;
                    last_y = y;
                }
                last_pressed = true;
            } else {
                if (last_pressed) {
                    vnc.mouseEvent(last_x, last_y, 0b0000);
                    last_pressed = false;
                }
            }
        }
        vnc.loop();
        if(!vnc.connected()) {
            delay(5000);
        }
    }
  }
}
