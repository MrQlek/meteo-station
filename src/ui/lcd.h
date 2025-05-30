#ifndef __LCD_H__
#define __LCD_H__

#include "lang_utils.h"

typedef enum {
    HD44780_PINS_RS_POS = 0,
    HD44780_PINS_RW_POS = 1,
    HD44780_PINS_E_POS = 2,
    HD44780_PINS_K_POS = 3,
    HD44780_PINS_D4_POS = 4,
    HD44780_PINS_D5_POS = 5,
    HD44780_PINS_D6_POS = 6,
    HD44780_PINS_D7_POS = 7,
    hd44780_pins_t_LIMIT
} hd44780_pins_t;

typedef enum {
    HD44780_REGISTER_COMMAND = 0,
    HD44780_REGISTER_DATA = 1,
    hd44780_register_t_LIMIT
} hd44780_register_t;

typedef enum {
    LCD_CURSOR_MOVE_DIRECTION_LEFT = 0,
    LCD_CURSOR_MOVE_DIRECTION_RIGHT = 1,
    lcd_cursor_move_direction_t_LIMIT
} lcd_cursor_move_direction_t;

typedef enum {
    LCD_SHIFT_DISPLAY_OFF = 0,
    LCD_SHIFT_DISPLAY_ON = 1,
    lcd_shift_display_t_LIMIT
} lcd_shift_display_t;

typedef enum {
    LCD_DISPLAY_STATE_OFF = 0,
    LCD_DISPLAY_STATE_ON = 1,
    lcd_display_state_t_LIMIT
} lcd_display_state_t;


typedef enum {
    LCD_CURSOR_STATE_OFF = 0,
    LCD_CURSOR_STATE_ON = 1,
    lcd_cursor_state_t_LIMIT
} lcd_cursor_state_t;

typedef enum {
    LCD_CURSOR_BLINK_OFF = 0,
    LCD_CURSOR_BLINK_ON = 1,
    lcd_cursor_blink_t_LIMIT
} lcd_cursor_blink_t;

#define LCD_LINE_LEN 16
#define LCD_COLUMN_MAX (LCD_LINE_LEN - 1)
#define LCD_ROW_MAX 1

static inline bool lcd_is_row_correct(const uint8_t row) {
    return (row <= LCD_ROW_MAX);
}

static inline bool lcd_is_column_correct(const uint8_t column) {
    return (column <= LCD_COLUMN_MAX);
}

extern void lcd_init();
extern void lcd_clear_display();
extern void lcd_write_string(uint8_t row, uint8_t column, const char * const msg);

#endif // !__LCD_H__