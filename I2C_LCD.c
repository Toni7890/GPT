/**
 * I2C LCD Library Implementation - Versi�n Final
 * Compatible con MCC PIC18F57Q43
 */

#include "I2C_LCD.h"

// Variables globales
uint8_t lcd_i2c_address = PCF8574_ADDR_7;
uint8_t lcd_backlight_state = LCD_STATE_ON;

/**
 * Escribe un byte al PCF8574 usando el driver MCC I2C1
 */
bool LCD_I2C_WritePCF8574(uint8_t data)
{
    // Agregar backlight si est� encendido
    if (lcd_backlight_state == LCD_STATE_ON) {
        data |= LCD_BACKLIGHT_MASK;
    } else {
        data &= ~LCD_BACKLIGHT_MASK;
    }
    
    // Usar driver MCC I2C1
    bool result = I2C1_Write(lcd_i2c_address, &data, 1);
    
    // Esperar que termine la transmisi�n
    while (I2C1_IsBusy()) {
        // Polling hasta completar
    }
    
    // Verificar si hubo errores
    i2c_host_error_t error = I2C1_ErrorGet();
    return (result && (error == I2C_ERROR_NONE));
}

/**
 * Genera un pulso en el pin Enable del LCD
 */
bool LCD_I2C_PulseEnable(uint8_t data)
{
    bool result = true;
    
    // Enable bajo
    result &= LCD_I2C_WritePCF8574(data & ~LCD_EN_MASK);
    __delay_us(1);
    
    // Enable alto
    result &= LCD_I2C_WritePCF8574(data | LCD_EN_MASK);
    __delay_us(1);
    
    // Enable bajo nuevamente
    result &= LCD_I2C_WritePCF8574(data & ~LCD_EN_MASK);
    __delay_us(50);
    
    return result;
}

/**
 * Env�a un nibble (4 bits) al LCD
 */
bool LCD_I2C_SendNibble(uint8_t nibble, uint8_t rs_state)
{
    uint8_t data = 0;
    
    // Configurar los bits de datos D4-D7
    if (nibble & 0x01) data |= (1 << LCD_D4_PIN);
    if (nibble & 0x02) data |= (1 << LCD_D5_PIN);
    if (nibble & 0x04) data |= (1 << LCD_D6_PIN);
    if (nibble & 0x08) data |= (1 << LCD_D7_PIN);
    
    // Configurar RS (Register Select)
    if (rs_state) {
        data |= LCD_RS_MASK;
    }
    
    // RW siempre en 0 (modo escritura)
    data &= ~LCD_RW_MASK;
    
    // Enviar con pulso de Enable
    return LCD_I2C_PulseEnable(data);
}

/**
 * Env�a un byte completo (dos nibbles) al LCD
 */
bool LCD_I2C_SendByte(uint8_t data, uint8_t rs_state)
{
    bool result = true;
    
    // Enviar nibble alto primero
    result &= LCD_I2C_SendNibble((data >> 4) & 0x0F, rs_state);
    
    // Enviar nibble bajo
    result &= LCD_I2C_SendNibble(data & 0x0F, rs_state);
    
    return result;
}

/**
 * Env�a un comando al LCD
 */
bool LCD_I2C_Command(uint8_t cmd)
{
    bool result = LCD_I2C_SendByte(cmd, 0);  // RS = 0 para comando
    
    // Algunos comandos requieren m�s tiempo
    if (cmd == LCD_CMD_CLEAR || cmd == LCD_CMD_HOME) {
        __delay_ms(2);
    } else {
        __delay_us(50);
    }
    
    return result;
}

/**
 * Env�a un dato (car�cter) al LCD
 */
bool LCD_I2C_Data(uint8_t data)
{
    bool result = LCD_I2C_SendByte(data, 1);  // RS = 1 para dato
    __delay_us(50);
    return result;
}

/**
 * Inicializa el LCD I2C
 */
bool LCD_I2C_Init(uint8_t address)
{
    bool result = true;
    
    // Configurar direcci�n I2C
    lcd_i2c_address = address;
    lcd_backlight_state = LCD_STATE_ON;
    
    // Esperar estabilizaci�n del LCD (> 15ms despu�s del power-on)
    __delay_ms(50);
    
    // Secuencia de inicializaci�n especial para modo 4-bit
    // Estos comandos se env�an como nibbles individuales
    
    // Primer intento: modo 8-bit
    result &= LCD_I2C_SendNibble(0x03, 0);
    __delay_ms(5);
    
    // Segundo intento: modo 8-bit
    result &= LCD_I2C_SendNibble(0x03, 0);
    __delay_us(150);
    
    // Tercer intento: modo 8-bit
    result &= LCD_I2C_SendNibble(0x03, 0);
    __delay_us(150);
    
    // Cambiar a modo 4-bit
    result &= LCD_I2C_SendNibble(0x02, 0);
    __delay_us(150);
    
    // Ahora enviar comandos completos de 8 bits en modo 4-bit
    
    // Function Set: 4-bit, 2 l�neas, matriz 5x8
    result &= LCD_I2C_Command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2_LINE | LCD_5x8_DOTS);
    
    // Display Control: Display off inicialmente
    result &= LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_OFF);
    
    // Clear Display
    result &= LCD_I2C_Clear();
    
    // Entry Mode Set: Incrementar cursor, sin shift del display
    result &= LCD_I2C_Command(LCD_CMD_ENTRY_MODE | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DEC);
    
    // Display Control: Display on, cursor off, blink off
    result &= LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    
    return result;
}

/**
 * Borra completamente la pantalla LCD
 */
bool LCD_I2C_Clear(void)
{
    return LCD_I2C_Command(LCD_CMD_CLEAR);
}

/**
 * Mueve el cursor a la posici�n inicial (0,0)
 */
bool LCD_I2C_Home(void)
{
    return LCD_I2C_Command(LCD_CMD_HOME);
}

/**
 * Posiciona el cursor en una fila y columna espec�fica
 */
bool LCD_I2C_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t address;
    
    // Validar par�metros
    if (row >= LCD_ROWS || col >= LCD_COLS) {
        return false;
    }
    
    // Calcular direcci�n DDRAM seg�n la fila
    if (row == 0) {
        address = 0x00 + col;  // Primera fila: 0x00-0x0F
    } else {
        address = 0x40 + col;  // Segunda fila: 0x40-0x4F
    }
    
    return LCD_I2C_Command(LCD_CMD_SET_DDRAM | address);
}

/**
 * Imprime una cadena de caracteres
 */
bool LCD_I2C_Print(const char* str)
{
    bool result = true;
    
    if (str == NULL) {
        return false;
    }
    
    while (*str && result) {
        result &= LCD_I2C_Data(*str);
        str++;
    }
    
    return result;
}

/**
 * Imprime un car�cter individual
 */
bool LCD_I2C_PrintChar(char c)
{
    return LCD_I2C_Data((uint8_t)c);
}

/**
 * Imprime una cadena en una posici�n espec�fica
 */
bool LCD_I2C_PrintAt(uint8_t row, uint8_t col, const char* str)
{
    bool result = LCD_I2C_SetCursor(row, col);
    if (result) {
        result &= LCD_I2C_Print(str);
    }
    return result;
}

/**
 * Enciende el backlight
 */
bool LCD_I2C_BacklightOn(void)
{
    lcd_backlight_state = LCD_STATE_ON;
    // Enviar un comando dummy para actualizar el backlight
    uint8_t data = LCD_BACKLIGHT_MASK;
    return LCD_I2C_WritePCF8574(data);
}

/**
 * Apaga el backlight
 */
bool LCD_I2C_BacklightOff(void)
{
    lcd_backlight_state = LCD_STATE_OFF;
    // Enviar datos sin backlight
    uint8_t data = 0x00;
    return LCD_I2C_WritePCF8574(data);
}

/**
 * Enciende el display
 */
bool LCD_I2C_DisplayOn(void)
{
    return LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
}

/**
 * Apaga el display
 */
bool LCD_I2C_DisplayOff(void)
{
    return LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_OFF);
}

/**
 * Enciende el cursor
 */
bool LCD_I2C_CursorOn(void)
{
    return LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_ON | LCD_BLINK_OFF);
}

/**
 * Apaga el cursor
 */
bool LCD_I2C_CursorOff(void)
{
    return LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
}

/**
 * Enciende el parpadeo del cursor
 */
bool LCD_I2C_BlinkOn(void)
{
    return LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_ON | LCD_BLINK_ON);
}

/**
 * Apaga el parpadeo del cursor
 */
bool LCD_I2C_BlinkOff(void)
{
    return LCD_I2C_Command(LCD_CMD_DISPLAY_CTRL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
}