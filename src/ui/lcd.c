#include <string.h>

#include "asserts.h"
#include "lang_utils.h"

#include "lcd_hw.h"
#include "lcd.h"

static void lcd_write(const hd44780_register_t target_register,
    const uint8_t data) {

    _assert(VERIFY_ENUM(target_register, hd44780_register_t));

    uint8_t i2c_command = 0b00000000;
    i2c_command |= (target_register & 0x01) << HD44780_PINS_RS_POS;
    i2c_command |= (0x01) << HD44780_PINS_E_POS;
    i2c_command |= (0x01) << HD44780_PINS_K_POS;
    i2c_command |= (data & 0b1111) << HD44780_PINS_D4_POS;

    lcd_write_data(&i2c_command, sizeof(i2c_command));
    i2c_command &= (uint8_t)(~((0x01) << HD44780_PINS_E_POS));
    lcd_write_data(&i2c_command, sizeof(i2c_command));
}

static void lcd_write_4bit_mode(const hd44780_register_t target_register, const uint8_t data) {
    _assert(VERIFY_ENUM(target_register, hd44780_register_t));

    lcd_write(target_register, (data >> 4) & 0b1111);
    lcd_write(target_register, (data) & 0b1111);
}

static void lcd_set_4bit_mode() {
    lcd_write(HD44780_REGISTER_COMMAND, 0b0011);
    lcd_write(HD44780_REGISTER_COMMAND, 0b0011);
    lcd_write(HD44780_REGISTER_COMMAND, 0b0011);
    lcd_write(HD44780_REGISTER_COMMAND, 0b0010);
}

void lcd_clear_display() {
    lcd_write_4bit_mode(HD44780_REGISTER_COMMAND, 0b00000001);
}


static void lcd_set_entry_mode(const lcd_cursor_move_direction_t cursor_direction,
    const lcd_shift_display_t shift_display) {

    _assert(VERIFY_ENUM(cursor_direction, lcd_cursor_move_direction_t));
    _assert(VERIFY_ENUM(shift_display, lcd_shift_display_t));

    const uint8_t command = 0b00000100
        | (cursor_direction << 1)
        | (shift_display << 0);

    lcd_write_4bit_mode(HD44780_REGISTER_COMMAND, command);
}

static void lcd_set_display_mode(const lcd_display_state_t display_state,
    const lcd_cursor_state_t cursor_state, const lcd_cursor_blink_t cursor_blink) {

    _assert(VERIFY_ENUM(display_state, lcd_display_state_t));
    _assert(VERIFY_ENUM(cursor_state, lcd_cursor_state_t));
    _assert(VERIFY_ENUM(cursor_blink, lcd_cursor_blink_t));

    const uint8_t command = 0b00001000
        | (display_state << 2)
        | (cursor_state << 1)
        | (cursor_blink << 0);

    lcd_write_4bit_mode(HD44780_REGISTER_COMMAND, command);
}

static void lcd_set_cursor(uint8_t row, uint8_t column) {
    _assert(lcd_is_row_correct(row));
    _assert(lcd_is_column_correct(column));

    uint8_t target_possition = (uint8_t)(row * 64 + column);
    uint8_t command = 0x80 | target_possition;

    lcd_write_4bit_mode(HD44780_REGISTER_COMMAND, command);
}

static void lcd_write_char(const char c) {
    lcd_write_4bit_mode(HD44780_REGISTER_DATA, c);
}

void lcd_write_string(uint8_t row, uint8_t column, const char * const msg) {
    _assert(lcd_is_row_correct(row));
    _assert(lcd_is_column_correct(column));
    assert_pointer(msg);

    lcd_set_cursor(row, column);
    for(size_t i = 0; i < MIN(strlen(msg), LCD_LINE_LEN); i++) {
        lcd_write_char(msg[i]);
    }
}

void lcd_init() {
    lcd_init_hw();
    lcd_set_4bit_mode();
    lcd_clear_display();
    lcd_set_entry_mode(LCD_CURSOR_MOVE_DIRECTION_RIGHT, LCD_SHIFT_DISPLAY_OFF);
    lcd_set_display_mode(LCD_DISPLAY_STATE_ON, LCD_CURSOR_STATE_OFF, LCD_CURSOR_BLINK_OFF);
}
