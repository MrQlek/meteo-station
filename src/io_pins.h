#ifndef __IO_PINS_H__
#define __IO_PINS_H__

#include "stm32l4xx.h"

#define LED_PORT GPIOA
#define LED_PIN 5

#define DEBUG_UART_TX_PORT GPIOA
#define DEBUG_UART_TX_PIN 2

#define DEBUG_UART_RX_PORT GPIOA
#define DEBUG_UART_RX_PIN 3

#define WEATHER_SENSOR_SPI_SCK_PORT GPIOA
#define WEATHER_SENSOR_SPI_SCK_PIN 5

#define WEATHER_SENSOR_SPI_MISO_PORT GPIOA
#define WEATHER_SENSOR_SPI_MISO_PIN 6

#define WEATHER_SENSOR_SPI_MOSI_PORT GPIOA
#define WEATHER_SENSOR_SPI_MOSI_PIN 7

#define WEATHER_SENSOR_SPI_CS_PORT GPIOB
#define WEATHER_SENSOR_SPI_CS_PIN 6

#define LIGHT_SENSOR_PORT GPIOC
#define LIGHT_SENSOR_PIN  0

#define LCD_I2C_SCL_PORT GPIOB
#define LCD_I2C_SCL_PIN 8

#define LCD_I2C_SDA_PORT GPIOB
#define LCD_I2C_SDA_PIN 9

#endif // !__IO_PINS_H__