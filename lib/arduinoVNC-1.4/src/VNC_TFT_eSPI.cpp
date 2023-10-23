/*
 * @file VNC_ST7789.cpp
 * @date 17.01.2021
 * @author Markus Sattler
 *
 * based on the work of modi12jin
 * 
 * Copyright (c) 2021 Markus Sattler. All rights reserved.
 * This file is part of the VNC client for Arduino.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, a copy can be downloaded from
 * http://www.gnu.org/licenses/gpl.html, or obtained by writing to the
 * Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 *
 *
 */

#include "VNC_config.h"

#ifdef VNC_TFT_eSPI_ENABLE

#include "VNC.h"

#include <SPI.h>
#include <TFT_eSPI.h>

#include "VNC_TFT_eSPI.h"

VNC_TFT_eSPI::VNC_TFT_eSPI() {
    TFT_eSPI();
}

bool VNC_TFT_eSPI::hasCopyRect(void) {
    return false;
}

uint32_t VNC_TFT_eSPI::getHeight(void) {
    return 320;
}

uint32_t VNC_TFT_eSPI::getWidth(void) {
    return 480;
}

void VNC_TFT_eSPI::draw_area(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint8_t * data) {
    TFT_eSPI::startWrite();
    TFT_eSPI::setAddrWindow(x, y, w, h);
    TFT_eSPI::pushColors((uint16_t *)data, w * h, true);
    TFT_eSPI::endWrite();
}

void VNC_TFT_eSPI::draw_rect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t color) {
    TFT_eSPI::fillRect(x, y, w, h, ((((color)&0xff) << 8) | (((color) >> 8))));
}

void VNC_TFT_eSPI::copy_rect(uint32_t src_x, uint32_t src_y, uint32_t dest_x, uint32_t dest_y, uint32_t w, uint32_t h) {
}

void VNC_TFT_eSPI::area_update_start(uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    TFT_eSPI::startWrite();
    TFT_eSPI::setAddrWindow(x, y, w, h);
}

void VNC_TFT_eSPI::area_update_data(char * data, uint32_t pixel) {
    TFT_eSPI::pushColors((uint16_t *)data, pixel, true);
}

void VNC_TFT_eSPI::area_update_end(void) {
    TFT_eSPI::endWrite();
}

void VNC_TFT_eSPI::vnc_options_override(dfb_vnc_options * opt) {
    // TODO: may need to be swaped for ESP8266
    // opt->client.bigendian = 1;
}
#endif