/**
 * I2C LCD Library - Versión Final
 * Compatible con MCC PIC18F57Q43
 */

#ifndef I2C_LCD_H
#define I2C_LCD_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include "mcc_generated_files/i2c_host/i2c1.h"

// Definiciones básicas
#ifndef _XTAL_FREQ
#define _XTAL_FREQ 64000000UL
#endif

// Direcciones I2C comunes para PCF8574
#define PCF8574_ADDR_0    0x20
#define PCF8574_ADDR_1    0x21
#define PCF8574_ADDR_2    0x22
#define PCF8574_ADDR_3    0x23
#define PCF8574_ADDR_4    0x24
#define PCF8574_ADDR_5    0x25
#define PCF8574_ADDR_6    0x26
#define PCF8574_ADDR_7    0x27

// Pines del PCF8574 conectados al LCD
#define LCD_RS_PIN        0
#define LCD_RW_PIN        1
#define LCD_EN_PIN        2
#define LCD_BACKLIGHT_PIN 3
#define LCD_D4_PIN        4
#define LCD_D5_PIN        5
#define LCD_D6_PIN        6
#define LCD_D7_PIN        7

// Máscaras de bits
#define LCD_RS_MASK       (1 << LCD_RS_PIN)
#define LCD_RW_MASK       (1 << LCD_RW_PIN)
#define LCD_EN_MASK       (1 << LCD_EN_PIN)
#define LCD_BACKLIGHT_MASK (1 << LCD_BACKLIGHT_PIN)

// Comandos básicos del LCD HD44780
#define LCD_CMD_CLEAR           0x01
#define LCD_CMD_HOME            0x02
#define LCD_CMD_ENTRY_MODE      0x04
#define LCD_CMD_DISPLAY_CTRL    0x08
#define LCD_CMD_CURSOR_SHIFT    0x10
#define LCD_CMD_FUNCTION_SET    0x20
#define LCD_CMD_SET_CGRAM       0x40
#define LCD_CMD_SET_DDRAM       0x80

// Function Set options
#define LCD_8BIT_MODE           0x10
#define LCD_4BIT_MODE           0x00
#define LCD_2_LINE              0x08
#define LCD_1_LINE              0x00
#define LCD_5x10_DOTS           0x04
#define LCD_5x8_DOTS            0x00

// Display Control options
#define LCD_DISPLAY_ON          0x04
#define LCD_DISPLAY_OFF         0x00
#define LCD_CURSOR_ON           0x02
#define LCD_CURSOR_OFF          0x00
#define LCD_BLINK_ON            0x01
#define LCD_BLINK_OFF           0x00

// Entry Mode options
#define LCD_ENTRY_RIGHT         0x00
#define LCD_ENTRY_LEFT          0x02
#define LCD_ENTRY_SHIFT_INC     0x01
#define LCD_ENTRY_SHIFT_DEC     0x00

// Cursor/Display Shift options
#define LCD_CURSOR_MOVE         0x00
#define LCD_DISPLAY_MOVE        0x08
#define LCD_MOVE_LEFT           0x00
#define LCD_MOVE_RIGHT          0x04

// Estados generales
#define LCD_STATE_ON            1
#define LCD_STATE_OFF           0

// Dimensiones del LCD
#define LCD_ROWS                2
#define LCD_COLS                16

// Variables globales
extern uint8_t lcd_i2c_address;
extern uint8_t lcd_backlight_state;

// Prototipos de funciones principales
bool LCD_I2C_Init(uint8_t address);
bool LCD_I2C_Clear(void);
bool LCD_I2C_Home(void);
bool LCD_I2C_SetCursor(uint8_t row, uint8_t col);
bool LCD_I2C_Print(const char* str);
bool LCD_I2C_PrintAt(uint8_t row, uint8_t col, const char* str);
bool LCD_I2C_PrintChar(char c);
bool LCD_I2C_BacklightOn(void);
bool LCD_I2C_BacklightOff(void);

// Funciones de control del display
bool LCD_I2C_DisplayOn(void);
bool LCD_I2C_DisplayOff(void);
bool LCD_I2C_CursorOn(void);
bool LCD_I2C_CursorOff(void);
bool LCD_I2C_BlinkOn(void);
bool LCD_I2C_BlinkOff(void);

// Funciones internas
bool LCD_I2C_WritePCF8574(uint8_t data);
bool LCD_I2C_PulseEnable(uint8_t data);
bool LCD_I2C_SendNibble(uint8_t nibble, uint8_t rs_state);
bool LCD_I2C_SendByte(uint8_t data, uint8_t rs_state);
bool LCD_I2C_Command(uint8_t cmd);
bool LCD_I2C_Data(uint8_t data);

#endif // I2C_LCD_H