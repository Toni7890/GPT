/* 
 * File:   config.h
 * Author: Ingeniero Embebido
 * Description: Configuraci€n para PIC18F57Q43 con oscilador interno HFINTOSC a 64MHz
 * 
 * CARACTER’STICAS DEL SISTEMA:
 * - Oscilador: HFINTOSC 64MHz (interno, calibrado de f∑brica)
 * - Frecuencia de instrucci€n: 16MHz (FOSC/4)
 * - Comunicaci€n: I2C1 en RC5(SCL)/RC6(SDA)
 * - LED de estado: RF3 (LED interno)
 * 
 * Created: [Fecha]
 */

#ifndef CONFIG_H
#define	CONFIG_H

/*******************************************************************************
 * DEFINICIONES DEL SISTEMA
 *******************************************************************************/

// Frecuencia del oscilador principal (64MHz HFINTOSC)
#define _XTAL_FREQ 64000000UL

// Frecuencia de instrucci€n (FOSC/4)
#define FCY (_XTAL_FREQ/4)

/*******************************************************************************
 * MACROS PARA CONTROL DE HARDWARE
 *******************************************************************************/

// Control del LED interno (RF3 - l€gica invertida)
#define LED_ON()        LATFbits.LATF3 = 0  // Encender LED
#define LED_OFF()       LATFbits.LATF3 = 1  // Apagar LED
#define LED_Toggle()    LATFbits.LATF3 ^= 1 // Cambiar estado LED

/*******************************************************************************
 * DEFINICIONES I2C
 *******************************************************************************/

// Pines I2C
#define I2C_SCL_PIN     PORTCbits.RC5       // Pin SCL
#define I2C_SDA_PIN     PORTCbits.RC6       // Pin SDA

// Estados del bus I2C
#define I2C_BUS_FREE    1
#define I2C_BUS_BUSY    0

/*******************************************************************************
 * CONFIGURACIONES DE TEMPORIZACI?N
 *******************************************************************************/

// Delays comunes (basados en 64MHz)
#define DELAY_1MS       __delay_ms(1)
#define DELAY_10MS      __delay_ms(10)
#define DELAY_100MS     __delay_ms(100)
#define DELAY_1S        __delay_ms(1000)

// Delays en microsegundos
#define DELAY_1US       __delay_us(1)
#define DELAY_10US      __delay_us(10)
#define DELAY_100US     __delay_us(100)

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES
 *******************************************************************************/

/**
 * @brief Inicializa el oscilador interno HFINTOSC a 64MHz
 * @param Ninguno
 * @return Nada
 * @note Esta funci€n debe llamarse antes que cualquier otra configuraci€n
 */
void ClockInit(void);

/**
 * @brief Configura los pines del microcontrolador
 * @param Ninguno  
 * @return Nada
 * @note Configura I2C, LED y pines digitales/anal€gicos necesarios
 */
void PinInit(void);

/*******************************************************************************
 * CONFIGURACIONES OPCIONALES PARA OPTIMIZACI?N
 *******************************************************************************/

// Descomenta para habilitar verificaci€n de errores adicional
// #define ENABLE_ERROR_CHECKING

// Descomenta para habilitar modo de bajo consumo
// #define ENABLE_LOW_POWER_MODE

// Descomenta para habilitar debug por UART
// #define ENABLE_DEBUG_UART

/*******************************************************************************
 * INFORMACI?N DEL COMPILADOR
 *******************************************************************************/

#ifndef __XC8
    #warning "Este c€digo est∑ optimizado para XC8 compiler"
#endif

#if _XTAL_FREQ != 64000000UL
    #warning "Verifica que _XTAL_FREQ coincida con la configuraci€n del oscilador"
#endif

#endif	/* CONFIG_H */