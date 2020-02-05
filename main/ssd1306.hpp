/*
 *  OLED Display (SSD1306) driver
 */
#pragma once

#include <stdint.h>
#include <driver/i2c.h>

#define SSD1306_I2C_ADDRESS                0x3C     // SLA (0x3C) + WRITE_MODE (0x00)
#define SSD1306_LCD_WIDTH                  64
#define SSD1306_LCD_HEIGHT                 48

// See: http://robotcantalk.blogspot.com/2015/03/interfacing-arduino-with-ssd1306-driven.html

                                                    //   Co  == 0 - following stream will contain data (not control) bytes only
                                                    //   D/C == 0 - following byte is command
// Control byte                                     //   Co  D/C
#define SSD1306_CTRL_CMD_STREAM            0x00     //    0   0   000000
#define SSD1306_CTRL_DATA_STREAM           0x40     //    0   1   000000
#define SSD1306_CTRL_CMD_SINGLE            0x80     //    1   0   000000
#define SSD1306_CTRL_DATA_SINGLE           0xc0     //    1   1   000000

#define SSD1306_CMD_SET_CONTRAST           0x81     // follow with 0x7F
#define SSD1306_CMD_DISPLAY_RAM            0xA4
#define SSD1306_CMD_DISPLAY_ALLON          0xA5
#define SSD1306_CMD_DISPLAY_NORMAL         0xA6
#define SSD1306_CMD_DISPLAY_INVERTED       0xA7
#define SSD1306_CMD_DISPLAY_OFF            0xAE
#define SSD1306_CMD_DISPLAY_ON             0xAF

#define SSD1306_CMD_SET_MEMORY_ADDR_MODE   0x20     // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define SSD1306_CMD_SET_COLUMN_RANGE       0x21     // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define SSD1306_CMD_SET_PAGE_RANGE         0x22     // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7
#define SSD1306_CMD_SET_NO_SCROLL          0x2e

#define SSD1306_CMD_SET_DISPLAY_START_LINE 0x40
#define SSD1306_CMD_SET_SEGMENT_REMAP      0xA1
#define SSD1306_CMD_SET_MUX_RATIO          0xA8     // follow with (SSD1306_LCD_HEIGHT - 1)
#define SSD1306_CMD_SET_COM_SCAN_MODE      0xC8
#define SSD1306_CMD_SET_DISPLAY_OFFSET     0xD3     // follow with 0x00 - no offset
#define SSD1306_CMD_SET_COM_PIN_MAP        0xDA     // follow with 0x12
#define SSD1306_CMD_NOP                    0xE3     // NOP

#define SSD1306_CMD_SET_DISPLAY_CLK_DIV    0xD5     // follow with 0x80
#define SSD1306_CMD_SET_PRECHARGE          0xD9     // follow with 0xF1
#define SSD1306_CMD_SET_VCOMH_DESELCT      0xDB     // follow with 0x30

#define SSD1306_CMD_SET_CHARGE_PUMP        0x8D     // follow with 0x14


class SSD1306 {
public:
    SSD1306(gpio_num_t SDA, gpio_num_t SCL);

    void flush();
    void clear();
    void invert(bool);
    void enable(bool);

    void put_char(int x, int y, char c);
    void put_line(int x, int y, const char *text);

private:
    void init_i2c(gpio_num_t SDA, gpio_num_t SCL);
    void init_ssd1306();
};
