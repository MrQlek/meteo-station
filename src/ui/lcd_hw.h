#ifndef __LCD_HW_H__
#define __LCD_HW_H__

#include "results.h"
#include "lang_utils.h"

#define LCD_I2C_ADDRESS (63)
#define I2C_ALL_INTERUPT_FLAGS (0b11111100111000)

typedef enum {
    I2C_DIRECTION_WRITE = 0,
    i2c_direction_t_LIMIT
} i2c_direction_t;

extern void lcd_init_hw();
extern result_t lcd_write_data(const uint8_t * const data, const uint8_t data_len);

#endif // !__LCD_HW_H__