/*******************************************************************************
 * SOFTWARE I2C LIBRARY OPTIMIZADA para PIC18F57Q43
 * 
 * CORRECCIONES APLICADAS:
 * ? SW_I2C_DELAY_US definido correctamente
 * ? Variable current_i2c_speed inicializada
 * ? Macros de timing corregidos
 *******************************************************************************/

#ifndef SOFTWARE_I2C_H
#define SOFTWARE_I2C_H

#include <xc.h>
#include <stdint.h>
#include "config.h"

/*******************************************************************************
 * DEFINICIONES DE PINES (MANTENIDAS)
 *******************************************************************************/
#define SW_I2C_SCL_TRIS     TRISBbits.TRISB1
#define SW_I2C_SDA_TRIS     TRISBbits.TRISB2
#define SW_I2C_SCL_LAT      LATBbits.LATB1
#define SW_I2C_SDA_LAT      LATBbits.LATB2
#define SW_I2C_SCL_PORT     PORTBbits.RB1
#define SW_I2C_SDA_PORT     PORTBbits.RB2

/*******************************************************************************
 * CONFIGURACI?N DE TIMING CORREGIDA
 *******************************************************************************/

// === TIMING B¡SICO COMPATIBLE ===
#define SW_I2C_DELAY_US     8      // Timing b·sico para compatibilidad

// === CONFIGURACI?N DIN¡MICA DE VELOCIDAD ===
typedef enum {
    SW_I2C_SPEED_FAST,      // 100kHz - M·xima velocidad
    SW_I2C_SPEED_NORMAL,    // 80kHz - Balance velocidad/estabilidad
    SW_I2C_SPEED_ROBUST     // 60kHz - M·xima estabilidad
} sw_i2c_speed_t;

// === VARIABLE GLOBAL INICIALIZADA ===
extern sw_i2c_speed_t current_i2c_speed;

// === TIMING OPTIMIZADO PARA MODOS AVANZADOS ===
#define SW_I2C_DELAY_US_FAST    5      // Modo r·pido (100kHz)
#define SW_I2C_DELAY_US_NORMAL  8      // Modo normal (80kHz)  
#define SW_I2C_DELAY_US_ROBUST  12     // Modo robusto (60kHz)

// === MACRO DE DELAY DIN¡MICO CORREGIDO ===
#define SW_I2C_DELAY() do { \
    switch(current_i2c_speed) { \
        case SW_I2C_SPEED_FAST: __delay_us(SW_I2C_DELAY_US_FAST); break; \
        case SW_I2C_SPEED_NORMAL: __delay_us(SW_I2C_DELAY_US_NORMAL); break; \
        case SW_I2C_SPEED_ROBUST: __delay_us(SW_I2C_DELAY_US_ROBUST); break; \
        default: __delay_us(SW_I2C_DELAY_US); break; \
    } \
} while(0)

/*******************************************************************************
 * MACROS DE CONTROL DE PINES (MANTENIDOS)
 *******************************************************************************/

#define SW_I2C_SCL_OUTPUT()     (SW_I2C_SCL_TRIS = 0)
#define SW_I2C_SCL_INPUT()      (SW_I2C_SCL_TRIS = 1)
#define SW_I2C_SCL_HIGH()       (SW_I2C_SCL_LAT = 1)
#define SW_I2C_SCL_LOW()        (SW_I2C_SCL_LAT = 0)

#define SW_I2C_SDA_OUTPUT()     (SW_I2C_SDA_TRIS = 0)
#define SW_I2C_SDA_INPUT()      (SW_I2C_SDA_TRIS = 1)
#define SW_I2C_SDA_HIGH()       (SW_I2C_SDA_LAT = 1)
#define SW_I2C_SDA_LOW()        (SW_I2C_SDA_LAT = 0)
#define SW_I2C_SDA_READ()       (SW_I2C_SDA_PORT)

/*******************************************************************************
 * C?DIGOS DE ERROR
 *******************************************************************************/
typedef enum {
    SW_I2C_SUCCESS = 0,
    SW_I2C_ERROR_NACK,
    SW_I2C_ERROR_TIMEOUT,
    SW_I2C_ERROR_BUS_BUSY,
    SW_I2C_ERROR_BUS_STUCK,
    SW_I2C_ERROR_CLOCK_STRETCH,
    SW_I2C_ERROR_DATA_CORRUPTION,
    SW_I2C_ERROR_DEVICE_NOT_READY,
    SW_I2C_ERROR_BUS_COLLISION,
    SW_I2C_ERROR_RECOVERY_FAILED
} sw_i2c_error_t;

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES B¡SICAS
 *******************************************************************************/
void SW_I2C_Init(void);
sw_i2c_error_t SW_I2C_Start(void);
void SW_I2C_Stop(void);
sw_i2c_error_t SW_I2C_Restart(void);
sw_i2c_error_t SW_I2C_WriteByte(uint8_t data);
uint8_t SW_I2C_ReadByte(uint8_t ack);
void SW_I2C_SendAck(void);
void SW_I2C_SendNack(void);
uint8_t SW_I2C_IsBusFree(void);

// === FUNCIONES DE ALTO NIVEL ===
sw_i2c_error_t SW_I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data);
uint8_t SW_I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr);
sw_i2c_error_t SW_I2C_ReadBlock(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint8_t length);
uint8_t SW_I2C_ScanBus(uint8_t *found_devices, uint8_t max_devices);
uint8_t SW_I2C_GetPinStatus(void);

// === NUEVAS FUNCIONES DE CONFIGURACI?N ===
void SW_I2C_SetSpeed(sw_i2c_speed_t speed);
sw_i2c_speed_t SW_I2C_GetSpeed(void);

/*******************************************************************************
 * MACROS ESPECÕFICOS PARA MPU6050
 *******************************************************************************/
#define SW_I2C_MPU6050_ADDR         0x68
#define SW_I2C_MPU6050_WHO_AM_I     0x75
#define SW_I2C_MPU6050_PWR_MGMT_1   0x6B

/*******************************************************************************
 * INFORMACI?N DE VERSI?N
 *******************************************************************************/
#define SW_I2C_VERSION_MAJOR    2
#define SW_I2C_VERSION_MINOR    0  
#define SW_I2C_VERSION_PATCH    1
#define SW_I2C_VERSION_STRING   "2.0.1-fixed"

#endif /* SOFTWARE_I2C_H */