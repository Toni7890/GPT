/*******************************************************************************
 * MPU6050_DMP.h - Librería MPU6050 para Detección de Movimientos Direccionales
 * 
 * PROYECTO: Sistema de Control con Movimientos para Personas con Discapacidad Motora
 * MICROCONTROLADOR: PIC18F57Q43
 * COMUNICACIÓN: Software I2C (RB1=SCL, RB2=SDA)
 * 
 * FUNCIONALIDAD:
 * - Detección de movimientos direccionales: ADELANTE, ATRÁS, IZQUIERDA, DERECHA
 * - Fusión de datos del acelerómetro y giroscopio
 * - Filtrado digital para reducir ruido
 * - Calibración automática
 * - Detección de gestos específicos para control adaptativo
 * 
 * AUTOR: Ingeniero Embebido
 * FECHA: 2025
 *******************************************************************************/

#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "config.h"
#include "Software_I2C.h"

/*******************************************************************************
 * DEFINICIONES Y CONSTANTES
 *******************************************************************************/

// Dirección I2C del MPU6050
#define MPU6050_I2C_ADDR            0x68

// Registros principales del MPU6050
#define MPU6050_WHO_AM_I            0x75    // Identificación del chip (debe ser 0x68)
#define MPU6050_PWR_MGMT_1          0x6B    // Gestión de energía 1
#define MPU6050_PWR_MGMT_2          0x6C    // Gestión de energía 2
#define MPU6050_GYRO_CONFIG         0x1B    // Configuración del giroscopio
#define MPU6050_ACCEL_CONFIG        0x1C    // Configuración del acelerómetro
#define MPU6050_SMPLRT_DIV          0x19    // Divisor de frecuencia de muestreo
#define MPU6050_CONFIG              0x1A    // Configuración general

// Registros de datos del acelerómetro (16 bits cada uno)
#define MPU6050_ACCEL_XOUT_H        0x3B    // Acelerómetro X (High Byte)
#define MPU6050_ACCEL_XOUT_L        0x3C    // Acelerómetro X (Low Byte)
#define MPU6050_ACCEL_YOUT_H        0x3D    // Acelerómetro Y (High Byte)
#define MPU6050_ACCEL_YOUT_L        0x3E    // Acelerómetro Y (Low Byte)
#define MPU6050_ACCEL_ZOUT_H        0x3F    // Acelerómetro Z (High Byte)
#define MPU6050_ACCEL_ZOUT_L        0x40    // Acelerómetro Z (Low Byte)

// Registros de temperatura
#define MPU6050_TEMP_OUT_H          0x41    // Temperatura (High Byte)
#define MPU6050_TEMP_OUT_L          0x42    // Temperatura (Low Byte)

// Registros de datos del giroscopio (16 bits cada uno)
#define MPU6050_GYRO_XOUT_H         0x43    // Giroscopio X (High Byte)
#define MPU6050_GYRO_XOUT_L         0x44    // Giroscopio X (Low Byte)
#define MPU6050_GYRO_YOUT_H         0x45    // Giroscopio Y (High Byte)
#define MPU6050_GYRO_YOUT_L         0x46    // Giroscopio Y (Low Byte)
#define MPU6050_GYRO_ZOUT_H         0x47    // Giroscopio Z (High Byte)
#define MPU6050_GYRO_ZOUT_L         0x48    // Giroscopio Z (Low Byte)

// Configuraciones de escala del acelerómetro
#define MPU6050_ACCEL_SCALE_2G      0x00    // ±2g (16384 LSB/g)
#define MPU6050_ACCEL_SCALE_4G      0x08    // ±4g (8192 LSB/g)
#define MPU6050_ACCEL_SCALE_8G      0x10    // ±8g (4096 LSB/g)
#define MPU6050_ACCEL_SCALE_16G     0x18    // ±16g (2048 LSB/g)

// Configuraciones de escala del giroscopio
#define MPU6050_GYRO_SCALE_250      0x00    // ±250°/s (131 LSB/°/s)
#define MPU6050_GYRO_SCALE_500      0x08    // ±500°/s (65.5 LSB/°/s)
#define MPU6050_GYRO_SCALE_1000     0x10    // ±1000°/s (32.8 LSB/°/s)
#define MPU6050_GYRO_SCALE_2000     0x18    // ±2000°/s (16.4 LSB/°/s)

// Factores de conversión para escalas seleccionadas (±4g, ±500°/s)
#define ACCEL_SCALE_FACTOR          8192.0f  // Para ±4g
#define GYRO_SCALE_FACTOR           65.5f    // Para ±500°/s

/*******************************************************************************
 * DEFINICIONES PARA DETECCIÓN DE MOVIMIENTOS DIRECCIONALES
 *******************************************************************************/

// Umbrales para detección de movimientos (ajustables según necesidades)
#define MOVEMENT_THRESHOLD_ACCEL    0.3f    // Umbral de aceleración (g)
#define MOVEMENT_THRESHOLD_GYRO     30.0f   // Umbral de velocidad angular (°/s)
#define MOVEMENT_MIN_DURATION       50      // Duración mínima del movimiento (ms)
#define MOVEMENT_MAX_DURATION       2000    // Duración máxima del movimiento (ms)

// Umbrales específicos para cada dirección
#define FORWARD_THRESHOLD           0.4f    // Adelante: aceleración Y positiva
#define BACKWARD_THRESHOLD          -0.4f   // Atrás: aceleración Y negativa
#define LEFT_THRESHOLD              -30.0f  // Izquierda: giro Z negativo
#define RIGHT_THRESHOLD             30.0f   // Derecha: giro Z positivo

// Filtro de media móvil
#define FILTER_SAMPLES              8       // Número de muestras para el filtro

/*******************************************************************************
 * ENUMERACIONES Y ESTRUCTURAS
 *******************************************************************************/

// Direcciones de movimiento detectadas
typedef enum {
    MOVEMENT_NONE = 0,          // Sin movimiento
    MOVEMENT_FORWARD,           // Adelante
    MOVEMENT_BACKWARD,          // Atrás  
    MOVEMENT_LEFT,              // Izquierda
    MOVEMENT_RIGHT,             // Derecha
    MOVEMENT_TILT_FORWARD,      // Inclinación adelante
    MOVEMENT_TILT_BACKWARD,     // Inclinación atrás
    MOVEMENT_TILT_LEFT,         // Inclinación izquierda
    MOVEMENT_TILT_RIGHT,        // Inclinación derecha
    MOVEMENT_SHAKE,             // Sacudida
    MOVEMENT_TAP                // Golpe/toque
} MPU6050_Movement_t;

// Estados del sistema de detección
typedef enum {
    MPU6050_STATE_IDLE = 0,     // En reposo
    MPU6050_STATE_DETECTING,    // Detectando movimiento
    MPU6050_STATE_CONFIRMED,    // Movimiento confirmado
    MPU6050_STATE_COOLDOWN      // Período de enfriamiento
} MPU6050_State_t;

// Estructura para datos de acelerómetro
typedef struct {
    float x;                    // Aceleración en X (g)
    float y;                    // Aceleración en Y (g)
    float z;                    // Aceleración en Z (g)
} MPU6050_Accel_t;

// Estructura para datos de giroscopio
typedef struct {
    float x;                    // Velocidad angular en X (°/s)
    float y;                    // Velocidad angular en Y (°/s)
    float z;                    // Velocidad angular en Z (°/s)
} MPU6050_Gyro_t;

// Estructura para ángulos calculados (compatibilidad con DMP)
typedef struct {
    float yaw;                  // Ángulo Yaw (rotación Z)
    float pitch;                // Ángulo Pitch (rotación Y)
    float roll;                 // Ángulo Roll (rotación X)
} VectorFloat_t;

// Estructura para quaternion (compatibilidad con DMP)
typedef struct {
    float w;                    // Componente W
    float x;                    // Componente X
    float y;                    // Componente Y
    float z;                    // Componente Z
} Quaternion;

// Estructura principal de datos del MPU6050
typedef struct {
    MPU6050_Accel_t accel;      // Datos del acelerómetro
    MPU6050_Gyro_t gyro;        // Datos del giroscopio
    VectorFloat_t angles;       // Ángulos calculados
    float temperature;          // Temperatura (°C)
    uint16_t timestamp;         // Marca de tiempo
    bool data_ready;            // Flag de datos listos
} MPU6050_Data_t;

// Estructura para filtro de media móvil
typedef struct {
    float buffer[FILTER_SAMPLES];
    uint8_t index;
    bool filled;
} MovingAverage_t;

// Estructura para detección de movimientos
typedef struct {
    MPU6050_Movement_t current_movement;
    MPU6050_State_t state;
    uint16_t movement_start_time;
    uint16_t movement_duration;
    uint8_t confidence_level;
    MovingAverage_t accel_x_filter;
    MovingAverage_t accel_y_filter;
    MovingAverage_t accel_z_filter;
    MovingAverage_t gyro_x_filter;
    MovingAverage_t gyro_y_filter;
    MovingAverage_t gyro_z_filter;
} MPU6050_Movement_Detector_t;

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES PRINCIPALES
 *******************************************************************************/

/**
 * @brief Inicializa el MPU6050
 * @return true si la inicialización fue exitosa, false en caso contrario
 */
bool MPU6050_Init(void);

/**
 * @brief Inicializa el DMP (Digital Motion Processor) simulado
 * @return 0 si fue exitoso, código de error en caso contrario
 */
uint8_t MPU6050_DMP_Init(void);

/**
 * @brief Obtiene datos del DMP (ángulos y quaternion)
 * @param ypr Puntero a estructura VectorFloat_t para ángulos
 * @param quat Puntero a estructura Quaternion
 * @return true si hay nuevos datos disponibles
 */
bool MPU6050_DMP_GetData(VectorFloat_t *ypr, Quaternion *quat);

/**
 * @brief Lee datos del acelerómetro
 * @param accel Puntero a estructura MPU6050_Accel_t
 * @return true si la lectura fue exitosa
 */
bool MPU6050_ReadAccel(MPU6050_Accel_t *accel);

/**
 * @brief Lee datos del giroscopio
 * @param gyro Puntero a estructura MPU6050_Gyro_t
 * @return true si la lectura fue exitosa
 */
bool MPU6050_ReadGyro(MPU6050_Gyro_t *gyro);

/**
 * @brief Lee la temperatura del sensor
 * @return Temperatura en grados Celsius
 */
float MPU6050_ReadTemperature(void);

/**
 * @brief Lee todos los datos del sensor
 * @param data Puntero a estructura MPU6050_Data_t
 * @return true si la lectura fue exitosa
 */
bool MPU6050_ReadAll(MPU6050_Data_t *data);

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES DE DETECCIÓN DE MOVIMIENTOS
 *******************************************************************************/

/**
 * @brief Inicializa el detector de movimientos
 */
void MPU6050_Movement_Init(void);

/**
 * @brief Detecta movimientos direccionales
 * @param data Datos del sensor
 * @return Tipo de movimiento detectado
 */
MPU6050_Movement_t MPU6050_DetectMovement(MPU6050_Data_t *data);

/**
 * @brief Convierte el tipo de movimiento a string
 * @param movement Tipo de movimiento
 * @return Puntero a string con el nombre del movimiento
 */
const char* MPU6050_MovementToString(MPU6050_Movement_t movement);

/**
 * @brief Obtiene el nivel de confianza del último movimiento detectado
 * @return Nivel de confianza (0-100%)
 */
uint8_t MPU6050_GetConfidenceLevel(void);

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES DE CALIBRACIÓN
 *******************************************************************************/

/**
 * @brief Calibra el sensor (debe mantenerse inmóvil durante la calibración)
 * @return true si la calibración fue exitosa
 */
bool MPU6050_Calibrate(void);

/**
 * @brief Obtiene los offsets de calibración
 * @param accel_offset Offsets del acelerómetro
 * @param gyro_offset Offsets del giroscopio
 */
void MPU6050_GetCalibrationOffsets(MPU6050_Accel_t *accel_offset, MPU6050_Gyro_t *gyro_offset);

/**
 * @brief Establece los offsets de calibración
 * @param accel_offset Offsets del acelerómetro
 * @param gyro_offset Offsets del giroscopio
 */
void MPU6050_SetCalibrationOffsets(MPU6050_Accel_t *accel_offset, MPU6050_Gyro_t *gyro_offset);

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES DE UTILIDAD
 *******************************************************************************/

/**
 * @brief Verifica si el sensor está conectado y responde correctamente
 * @return true si el sensor está disponible
 */
bool MPU6050_TestConnection(void);

/**
 * @brief Reinicia el sensor
 * @return true si el reinicio fue exitoso
 */
bool MPU6050_Reset(void);

/**
 * @brief Pone el sensor en modo de bajo consumo
 */
void MPU6050_Sleep(void);

/**
 * @brief Despierta el sensor del modo de bajo consumo
 */
void MPU6050_WakeUp(void);

/*******************************************************************************
 * MACROS DE UTILIDAD
 *******************************************************************************/

// Conversión de radianes a grados
#define RAD_TO_DEG(x)               ((x) * 180.0f / M_PI)

// Conversión de grados a radianes
#define DEG_TO_RAD(x)               ((x) * M_PI / 180.0f)

// Valor absoluto para flotantes
#define ABS_FLOAT(x)                ((x) >= 0 ? (x) : -(x))

// Clamp para mantener valores en un rango
#define CLAMP(value, min, max)      ((value) < (min) ? (min) : ((value) > (max) ? (max) : (value)))

/*******************************************************************************
 * CONFIGURACIONES AVANZADAS (OPCIONALES)
 *******************************************************************************/

// Descomenta para habilitar debug por UART
// #define MPU6050_DEBUG_ENABLED

// Descomenta para habilitar calibración automática al inicio
// #define MPU6050_AUTO_CALIBRATE

// Descomenta para habilitar filtro de Kalman
// #define MPU6050_KALMAN_FILTER

// Descomenta para habilitar detección de gestos avanzados
#define MPU6050_ADVANCED_GESTURES

/*******************************************************************************
 * INFORMACIÓN DEL MÓDULO
 *******************************************************************************/

#define MPU6050_DMP_VERSION_MAJOR   1
#define MPU6050_DMP_VERSION_MINOR   0
#define MPU6050_DMP_VERSION_PATCH   0
#define MPU6050_DMP_VERSION_STRING  "1.0.0-accessibility"

#endif /* MPU6050_DMP_H */

/*******************************************************************************
 * NOTAS DE IMPLEMENTACIÓN:
 * 
 * 1. Esta librería está específicamente diseñada para detectar movimientos
 *    direccionales en aplicaciones de accesibilidad.
 * 
 * 2. Utiliza el Software_I2C implementado en el proyecto para comunicarse
 *    con el MPU6050.
 * 
 * 3. El "DMP" simulado combina datos del acelerómetro y giroscopio para
 *    detectar movimientos específicos.
 * 
 * 4. Los umbrales pueden ajustarse según las necesidades específicas del
 *    usuario final.
 * 
 * 5. Incluye filtros digitales para reducir ruido y mejorar la precisión.
 * 
 * 6. Compatible con el código main.c existente que usa VectorFloat_t y
 *    Quaternion.
 *******************************************************************************/