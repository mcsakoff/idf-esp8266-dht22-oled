#include "ssd1306.hpp"
#include "font8x8_basic.h"

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_err.h>

static const char TAG[] = "ssd1306";

#define SSD1306_BUFFER_LENGTH SSD1306_LCD_WIDTH * SSD1306_LCD_HEIGHT / 8

static uint8_t buffer[SSD1306_BUFFER_LENGTH] = {
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xe0, 0xf8, 0xfe,
        0xff, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x9f, 0xdf, 0xff, 0x7f, 0x3f, 0xbf, 0x9f, 0x9f, 0xdf, 0xdf,
        0xdf, 0xdf, 0x9f, 0x9f, 0x9f, 0x9f, 0xbf, 0xbf, 0xbf, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xe0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0xff, 0xff, 0x7f,
        0xbb, 0xbd, 0xdc, 0xce, 0x66, 0x37, 0x1f, 0x9f, 0x9f, 0xa3, 0xf3, 0xf3, 0xc1, 0x93, 0x91, 0x01,
        0x00, 0x00, 0x00, 0xe0, 0xc1, 0xe0, 0xe0, 0xe1, 0xe1, 0x61, 0x23, 0x67, 0x4f, 0x8f, 0x3f, 0x7f,
        0xff, 0x7f, 0x7f, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x24, 0x3f, 0x1e, 0xfd,
        0x3e, 0xff, 0x03, 0x00, 0x00, 0x03, 0x05, 0x18, 0x12, 0x57, 0x56, 0x31, 0x13, 0x0b, 0x07, 0x00,
        0x00, 0x00, 0x00, 0x03, 0x07, 0x33, 0x51, 0x37, 0x1f, 0x17, 0x13, 0x06, 0x0c, 0x00, 0x00, 0x00,
        0x01, 0xfe, 0x38, 0xc1, 0xbf, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
        0x0f, 0x0f, 0x3c, 0xc0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x90, 0xc8, 0xde, 0xf0, 0xf0, 0xf0, 0xe0,
        0xc0, 0xe0, 0xf0, 0xf0, 0xe0, 0xe6, 0xd8, 0x30, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xf8,
        0xd7, 0x98, 0x0f, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x80, 0xc0, 0x60, 0x60, 0x30, 0xb0, 0x30, 0x98, 0xe8, 0xec, 0xfe, 0xfa, 0xfc,
        0xfe, 0xfe, 0xff, 0xff, 0xff, 0x1f, 0x1c, 0xf0, 0xe7, 0xe3, 0xc0, 0x80, 0x82, 0x02, 0x12, 0x72,
        0x72, 0x72, 0x32, 0x02, 0x02, 0x01, 0xc1, 0xc7, 0xe0, 0xc0, 0x78, 0xbc, 0x0e, 0x03, 0xe1, 0xff,
        0xff, 0xff, 0xff, 0xf8, 0xf0, 0xd0, 0xe0, 0xa0, 0xe0, 0x40, 0x40, 0x80, 0x80, 0x00, 0x00, 0x00,
        0xe0, 0xf8, 0xfa, 0xfd, 0xfe, 0xfe, 0x7f, 0xef, 0xf9, 0xfe, 0x7f, 0xff, 0x7f, 0x7f, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0x00, 0x7e, 0x7f, 0x7f, 0x7d, 0xf3, 0xc3, 0x87, 0x0f, 0x07, 0x0f,
        0x0f, 0x0f, 0x0f, 0x0f, 0x8f, 0xc7, 0xe7, 0x33, 0x19, 0x0c, 0x82, 0xc0, 0xf0, 0xfc, 0xff, 0xff,
        0xff, 0xff, 0xff, 0xff, 0xff, 0xc1, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xfd, 0xfa, 0xe4
};

SSD1306::SSD1306(gpio_num_t SDA, gpio_num_t SCL) {
    vTaskDelay(100 / portTICK_RATE_MS);
    this->init_i2c(SDA, SCL);
    this->init_ssd1306();

    this->flush();
    this->enable(true);
    vTaskDelay(4000 / portTICK_RATE_MS);

    this->clear();
    this->invert(false);
    put_line(0, 0, "WEMOS.cc");
    put_line(0, 1, "--------");
    put_line(0, 2, "2020 (c)");
    put_line(0, 3, " TheBodu");
    put_line(0, 4, "--------");
    put_line(0, 5, "ver 1.02");
    this->flush();

    ESP_LOGI(TAG, "display is ready");
}

void SSD1306::init_i2c(gpio_num_t SDA, gpio_num_t SCL) {
    ESP_LOGD(TAG, "init I2C bus...");
    i2c_port_t i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA;
    conf.scl_io_num = SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
}

void SSD1306::init_ssd1306() {
    ESP_LOGD(TAG, "init display...");
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_CTRL_CMD_STREAM, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_DISPLAY_OFF, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_DISPLAY_CLK_DIV, true);
    i2c_master_write_byte(cmd, 0x80, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_MUX_RATIO, true);
    i2c_master_write_byte(cmd, SSD1306_LCD_HEIGHT - 1, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_DISPLAY_OFFSET, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_DISPLAY_START_LINE, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_CHARGE_PUMP, true);
    i2c_master_write_byte(cmd, 0x14, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_MEMORY_ADDR_MODE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_SEGMENT_REMAP, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_COM_SCAN_MODE, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_COM_PIN_MAP, true);
    i2c_master_write_byte(cmd, 0x12, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_CONTRAST, true);
    i2c_master_write_byte(cmd, 0x7f, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_PRECHARGE, true);
    i2c_master_write_byte(cmd, 0xf1, true);   // 0x1f ???
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_VCOMH_DESELCT, true);
    i2c_master_write_byte(cmd, 0x40, true);   // 0x30 ???
    i2c_master_write_byte(cmd, SSD1306_CMD_DISPLAY_RAM, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_DISPLAY_INVERTED, true);
    //i2c_master_write_byte(cmd, SSD1306_CMD_SET_NO_SCROLL, true);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

void SSD1306::flush() {
    // - Send commands
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_CTRL_CMD_STREAM, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_COLUMN_RANGE, true);
    // for 64x48
    i2c_master_write_byte(cmd, 32, true);
    i2c_master_write_byte(cmd, 32 + SSD1306_LCD_WIDTH - 1, true);
    /* // --- other
     * i2c_master_write_byte(cmd, 0, true);
     * i2c_master_write_byte(cmd, SSD1306_LCD_WIDTH - 1, true);
     */
    i2c_master_write_byte(cmd, SSD1306_CMD_SET_PAGE_RANGE, true);
    i2c_master_write_byte(cmd, 0, true);
    i2c_master_write_byte(cmd, SSD1306_LCD_HEIGHT / 8 - 1, true);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);

    // - Send buffer data
    for (int i = 0; i < SSD1306_BUFFER_LENGTH; i += 16) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, SSD1306_CTRL_DATA_STREAM, true);
        for (int j = 0; j < 16; j++) {
            i2c_master_write_byte(cmd, buffer[i + j], true);
        }
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS));
        i2c_cmd_link_delete(cmd);
    }
}

void SSD1306::enable(bool) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_CTRL_CMD_SINGLE, true);
    i2c_master_write_byte(cmd, SSD1306_CMD_DISPLAY_ON, true);
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

void SSD1306::clear() {
    memset(buffer, 0, SSD1306_BUFFER_LENGTH);
}

void SSD1306::invert(bool inverted) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD1306_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, SSD1306_CTRL_CMD_SINGLE, true);
    if (inverted) {
        i2c_master_write_byte(cmd, SSD1306_CMD_DISPLAY_INVERTED, true);
    } else {
        i2c_master_write_byte(cmd, SSD1306_CMD_DISPLAY_NORMAL, true);
    }
    i2c_master_stop(cmd);
    ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

void SSD1306::put_char(int x, int y, char c) {
    if (x < 0 || x > SSD1306_LCD_WIDTH / 8 || y < 0 || y > SSD1306_LCD_HEIGHT / 8) return;
    if (c >= (char) 128) c = '?';
    int i = (x * 8) + (y * SSD1306_LCD_WIDTH);
    memcpy(&buffer[i], font8x8_basic_tr[(int) c], 8);
}

void SSD1306::put_line(int x, int y, const char *text) {
    while (x < 0) {
        x += SSD1306_LCD_WIDTH / 8;
    }
    while (y < 0) {
        y += SSD1306_LCD_HEIGHT / 8;
    }

    size_t len = strlen(text);
    for (int i = 0; i < len; i++) {
        put_char(x + i, y, text[i]);
    }
}
