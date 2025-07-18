/*******************************************************************************
 * MPU6050_DMP.h - Librer�a MPU6050 para Detecci�n de Movimientos Direccionales
 * 
 * PROYECTO: Sistema de Control con Movimientos para Personas con Discapacidad Motora
 * MICROCONTROLADOR: PIC18F57Q43
 * COMUNICACI�N: Software I2C (RB1=SCL, RB2=SDA)
 * 
 * FUNCIONALIDAD:
 * - Detecci�n de movimientos direccionales: ADELANTE, ATR�S, IZQUIERDA, DERECHA
 * - Fusi�n de datos del aceler�metro y giroscopio
 * - Filtrado digital para reducir ruido
 * - Calibraci�n autom�tica
 * - Detecci�n de gestos espec�ficos para control adaptativo
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

// Direcci�n I2C del MPU6050
#define MPU6050_I2C_ADDR            0x68

// Registros principales del MPU6050
#define MPU6050_WHO_AM_I            0x75    // Identificaci�n del chip (debe ser 0x68)
#define MPU6050_PWR_MGMT_1          0x6B    // Gesti�n de energ�a 1
#define MPU6050_PWR_MGMT_2          0x6C    // Gesti�n de energ�a 2
#define MPU6050_GYRO_CONFIG         0x1B    // Configuraci�n del giroscopio
#define MPU6050_ACCEL_CONFIG        0x1C    // Configuraci�n del aceler�metro
#define MPU6050_SMPLRT_DIV          0x19    // Divisor de frecuencia de muestreo
#define MPU6050_CONFIG              0x1A    // Configuraci�n general

// Registros de datos del aceler�metro (16 bits cada uno)
#define MPU6050_ACCEL_XOUT_H        0x3B    // Aceler�metro X (High Byte)
#define MPU6050_ACCEL_XOUT_L        0x3C    // Aceler�metro X (Low Byte)
#define MPU6050_ACCEL_YOUT_H        0x3D    // Aceler�metro Y (High Byte)
#define MPU6050_ACCEL_YOUT_L        0x3E    // Aceler�metro Y (Low Byte)
#define MPU6050_ACCEL_ZOUT_H        0x3F    // Aceler�metro Z (High Byte)
#define MPU6050_ACCEL_ZOUT_L        0x40    // Aceler�metro Z (Low Byte)

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

// Configuraciones de escala del aceler�metro
#define MPU6050_ACCEL_SCALE_2G      0x00    // �2g (16384 LSB/g)
#define MPU6050_ACCEL_SCALE_4G      0x08    // �4g (8192 LSB/g)
#define MPU6050_ACCEL_SCALE_8G      0x10    // �8g (4096 LSB/g)
#define MPU6050_ACCEL_SCALE_16G     0x18    // �16g (2048 LSB/g)

// Configuraciones de escala del giroscopio
#define MPU6050_GYRO_SCALE_250      0x00    // �250�/s (131 LSB/�/s)
#define MPU6050_GYRO_SCALE_500      0x08    // �500�/s (65.5 LSB/�/s)
#define MPU6050_GYRO_SCALE_1000     0x10    // �1000�/s (32.8 LSB/�/s)
#define MPU6050_GYRO_SCALE_2000     0x18    // �2000�/s (16.4 LSB/�/s)

// Factores de conversi�n para escalas seleccionadas (�4g, �500�/s)
#define ACCEL_SCALE_FACTOR          8192.0f  // Para �4g
#define GYRO_SCALE_FACTOR           65.5f    // Para �500�/s

/*******************************************************************************
 * DEFINICIONES PARA DETECCI�N DE MOVIMIENTOS DIRECCIONALES
 *******************************************************************************/

// Umbrales para detecci�n de movimientos (ajustables seg�n necesidades)
#define MOVEMENT_THRESHOLD_ACCEL    0.3f    // Umbral de aceleraci�n (g)
#define MOVEMENT_THRESHOLD_GYRO     30.0f   // Umbral de velocidad angular (�/s)
#define MOVEMENT_MIN_DURATION       50      // Duraci�n m�nima del movimiento (ms)
#define MOVEMENT_MAX_DURATION       2000    // Duraci�n m�xima del movimiento (ms)

// Umbrales espec�ficos para cada direcci�n
#define FORWARD_THRESHOLD           0.4f    // Adelante: aceleraci�n Y positiva
#define BACKWARD_THRESHOLD          -0.4f   // Atr�s: aceleraci�n Y negativa
#define LEFT_THRESHOLD              -30.0f  // Izquierda: giro Z negativo
#define RIGHT_THRESHOLD             30.0f   // Derecha: giro Z positivo

// Filtro de media m�vil
#define FILTER_SAMPLES              8       // N�mero de muestras para el filtro

/*******************************************************************************
 * ENUMERACIONES Y ESTRUCTURAS
 *******************************************************************************/

// Direcciones de movimiento detectadas
typedef enum {
    MOVEMENT_NONE = 0,          // Sin movimiento
    MOVEMENT_FORWARD,           // Adelante
    MOVEMENT_BACKWARD,          // Atr�s  
    MOVEMENT_LEFT,              // Izquierda
    MOVEMENT_RIGHT,             // Derecha
    MOVEMENT_TILT_FORWARD,      // Inclinaci�n adelante
    MOVEMENT_TILT_BACKWARD,     // Inclinaci�n atr�s
    MOVEMENT_TILT_LEFT,         // Inclinaci�n izquierda
    MOVEMENT_TILT_RIGHT,        // Inclinaci�n derecha
    MOVEMENT_SHAKE,             // Sacudida
    MOVEMENT_TAP                // Golpe/toque
} MPU6050_Movement_t;

// Estados del sistema de detecci�n
typedef enum {
    MPU6050_STATE_IDLE = 0,     // En reposo
    MPU6050_STATE_DETECTING,    // Detectando movimiento
    MPU6050_STATE_CONFIRMED,    // Movimiento confirmado
    MPU6050_STATE_COOLDOWN      // Per�odo de enfriamiento
} MPU6050_State_t;

// Estructura para datos de aceler�metro
typedef struct {
    float x;                    // Aceleraci�n en X (g)
    float y;                    // Aceleraci�n en Y (g)
    float z;                    // Aceleraci�n en Z (g)
} MPU6050_Accel_t;

// Estructura para datos de giroscopio
typedef struct {
    float x;                    // Velocidad angular en X (�/s)
    float y;                    // Velocidad angular en Y (�/s)
    float z;                    // Velocidad angular en Z (�/s)
} MPU6050_Gyro_t;

// Estructura para �ngulos calculados (compatibilidad con DMP)
typedef struct {
    float yaw;                  // �ngulo Yaw (rotaci�n Z)
    float pitch;                // �ngulo Pitch (rotaci�n Y)
    float roll;                 // �ngulo Roll (rotaci�n X)
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
    MPU6050_Accel_t accel;      // Datos del aceler�metro
    MPU6050_Gyro_t gyro;        // Datos del giroscopio
    VectorFloat_t angles;       // �ngulos calculados
    float temperature;          // Temperatura (�C)
    uint16_t timestamp;         // Marca de tiempo
    bool data_ready;            // Flag de datos listos
} MPU6050_Data_t;

// Estructura para filtro de media m�vil
typedef struct {
    float buffer[FILTER_SAMPLES];
    uint8_t index;
    bool filled;
} MovingAverage_t;

// Estructura para detecci�n de movimientos
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
 * @return true si la inicializaci�n fue exitosa, false en caso contrario
 */
bool MPU6050_Init(void);

/**
 * @brief Inicializa el DMP (Digital Motion Processor) simulado
 * @return 0 si fue exitoso, c�digo de error en caso contrario
 */
uint8_t MPU6050_DMP_Init(void);

/**
 * @brief Obtiene datos del DMP (�ngulos y quaternion)
 * @param ypr Puntero a estructura VectorFloat_t para �ngulos
 * @param quat Puntero a estructura Quaternion
 * @return true si hay nuevos datos disponibles
 */
bool MPU6050_DMP_GetData(VectorFloat_t *ypr, Quaternion *quat);

/**
 * @brief Lee datos del aceler�metro
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
 * PROTOTIPOS DE FUNCIONES DE DETECCI�N DE MOVIMIENTOS
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
 * @brief Obtiene el nivel de confianza del �ltimo movimiento detectado
 * @return Nivel de confianza (0-100%)
 */
uint8_t MPU6050_GetConfidenceLevel(void);

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES DE CALIBRACI�N
 *******************************************************************************/

/**
 * @brief Calibra el sensor (debe mantenerse inm�vil durante la calibraci�n)
 * @return true si la calibraci�n fue exitosa
 */
bool MPU6050_Calibrate(void);

/**
 * @brief Obtiene los offsets de calibraci�n
 * @param accel_offset Offsets del aceler�metro
 * @param gyro_offset Offsets del giroscopio
 */
void MPU6050_GetCalibrationOffsets(MPU6050_Accel_t *accel_offset, MPU6050_Gyro_t *gyro_offset);

/**
 * @brief Establece los offsets de calibraci�n
 * @param accel_offset Offsets del aceler�metro
 * @param gyro_offset Offsets del giroscopio
 */
void MPU6050_SetCalibrationOffsets(MPU6050_Accel_t *accel_offset, MPU6050_Gyro_t *gyro_offset);

/*******************************************************************************
 * PROTOTIPOS DE FUNCIONES DE UTILIDAD
 *******************************************************************************/

/**
 * @brief Verifica si el sensor est� conectado y responde correctamente
 * @return true si el sensor est� disponible
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

// Conversi�n de radianes a grados
#define RAD_TO_DEG(x)               ((x) * 180.0f / M_PI)

// Conversi�n de grados a radianes
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

// Descomenta para habilitar calibraci�n autom�tica al inicio
// #define MPU6050_AUTO_CALIBRATE

// Descomenta para habilitar filtro de Kalman
// #define MPU6050_KALMAN_FILTER

// Descomenta para habilitar detecci�n de gestos avanzados
#define MPU6050_ADVANCED_GESTURES

/*******************************************************************************
 * INFORMACI�N DEL M�DULO
 *******************************************************************************/

#define MPU6050_DMP_VERSION_MAJOR   1
#define MPU6050_DMP_VERSION_MINOR   0
#define MPU6050_DMP_VERSION_PATCH   0
#define MPU6050_DMP_VERSION_STRING  "1.0.0-accessibility"

#endif /* MPU6050_DMP_H */

/*******************************************************************************
 * NOTAS DE IMPLEMENTACI�N:
 * 
 * 1. Esta librer�a est� espec�ficamente dise�ada para detectar movimientos
 *    direccionales en aplicaciones de accesibilidad.
 * 
 * 2. Utiliza el Software_I2C implementado en el proyecto para comunicarse
 *    con el MPU6050.
 * 
 * 3. El "DMP" simulado combina datos del aceler�metro y giroscopio para
 *    detectar movimientos espec�ficos.
 * 
 * 4. Los umbrales pueden ajustarse seg�n las necesidades espec�ficas del
 *    usuario final.
 * 
 * 5. Incluye filtros digitales para reducir ruido y mejorar la precisi�n.
 * 
 * 6. Compatible con el c�digo main.c existente que usa VectorFloat_t y
 *    Quaternion.
 *******************************************************************************/