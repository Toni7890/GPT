/*******************************************************************************
 * MPU6050_DMP.c - Implementación de la Librería MPU6050 para Detección de Movimientos
 * 
 * PROYECTO: Sistema de Control con Movimientos para Personas con Discapacidad Motora
 * MICROCONTROLADOR: PIC18F57Q43
 * COMUNICACIÓN: Software I2C (RB1=SCL, RB2=SDA)
 * 
 * AUTOR: Ingeniero Embebido
 * FECHA: 2025
 *******************************************************************************/

#include "MPU6050_DMP.h"

/*******************************************************************************
 * VARIABLES GLOBALES
 *******************************************************************************/

// Offsets de calibración
static MPU6050_Accel_t accel_offset = {0.0f, 0.0f, 0.0f};
static MPU6050_Gyro_t gyro_offset = {0.0f, 0.0f, 0.0f};

// Detector de movimientos
static MPU6050_Movement_Detector_t movement_detector;

// Flag de inicialización
static bool mpu6050_initialized = false;

// Último timestamp para cálculo de tiempo
static uint16_t last_timestamp = 0;

/*******************************************************************************
 * FUNCIONES PRIVADAS - PROTOTIPO
 *******************************************************************************/
static bool MPU6050_WriteRegister(uint8_t reg, uint8_t data);
static uint8_t MPU6050_ReadRegister(uint8_t reg);
static bool MPU6050_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length);
static int16_t MPU6050_CombineBytes(uint8_t high, uint8_t low);
static float MovingAverage_Update(MovingAverage_t *filter, float new_value);
static void MovingAverage_Reset(MovingAverage_t *filter);
static uint16_t GetTimestamp_ms(void);
static MPU6050_Movement_t AnalyzeMovementPattern(MPU6050_Data_t *data);
static uint8_t CalculateConfidence(MPU6050_Movement_t movement, MPU6050_Data_t *data);

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES PRINCIPALES
 *******************************************************************************/

/**
 * @brief Inicializa el MPU6050
 */
bool MPU6050_Init(void) {
    
    // Verificar conexión con el sensor
    if (!MPU6050_TestConnection()) {
        return false;
    }
    
    // Despertar el sensor (salir del modo sleep)
    if (!MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x00)) {
        return false;
    }
    __delay_ms(100);
    
    // Configurar el divisor de frecuencia de muestreo (1kHz / (1+7) = 125Hz)
    if (!MPU6050_WriteRegister(MPU6050_SMPLRT_DIV, 0x07)) {
        return false;
    }
    
    // Configurar filtro digital (DLPF = 3, ~44Hz para accel y gyro)
    if (!MPU6050_WriteRegister(MPU6050_CONFIG, 0x03)) {
        return false;
    }
    
    // Configurar escala del acelerómetro (±4g para mayor sensibilidad)
    if (!MPU6050_WriteRegister(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_SCALE_4G)) {
        return false;
    }
    
    // Configurar escala del giroscopio (±500°/s para gestos humanos)
    if (!MPU6050_WriteRegister(MPU6050_GYRO_CONFIG, MPU6050_GYRO_SCALE_500)) {
        return false;
    }
    
    // Inicializar detector de movimientos
    MPU6050_Movement_Init();
    
    // Pequeño delay para estabilización
    __delay_ms(50);
    
    mpu6050_initialized = true;
    return true;
}

/**
 * @brief Inicializa el DMP simulado
 */
uint8_t MPU6050_DMP_Init(void) {
    
    if (!mpu6050_initialized) {
        if (!MPU6050_Init()) {
            return 1; // Error de inicialización
        }
    }
    
#ifdef MPU6050_AUTO_CALIBRATE
    // Calibración automática al inicio
    if (!MPU6050_Calibrate()) {
        return 2; // Error de calibración
    }
#endif
    
    return 0; // Éxito
}

/**
 * @brief Obtiene datos del DMP (simulado)
 */
bool MPU6050_DMP_GetData(VectorFloat_t *ypr, Quaternion *quat) {
    
    static uint16_t last_read_time = 0;
    uint16_t current_time = GetTimestamp_ms();
    
    // Leer solo si han pasado al menos 8ms (125Hz)
    if ((current_time - last_read_time) < 8) {
        return false;
    }
    
    MPU6050_Data_t sensor_data;
    
    // Leer todos los datos del sensor
    if (!MPU6050_ReadAll(&sensor_data)) {
        return false;
    }
    
    // Calcular ángulos usando datos del acelerómetro y giroscopio
    if (ypr != NULL) {
        // Calcular Roll y Pitch desde el acelerómetro
        ypr->roll = RAD_TO_DEG(atan2(sensor_data.accel.y, 
                                     sqrt(sensor_data.accel.x * sensor_data.accel.x + 
                                          sensor_data.accel.z * sensor_data.accel.z)));
        
        ypr->pitch = RAD_TO_DEG(atan2(-sensor_data.accel.x, 
                                      sqrt(sensor_data.accel.y * sensor_data.accel.y + 
                                           sensor_data.accel.z * sensor_data.accel.z)));
        
        // Yaw integrado desde el giroscopio (simplificado)
        static float yaw_integral = 0.0f;
        float dt = (current_time - last_read_time) / 1000.0f; // Convertir a segundos
        yaw_integral += sensor_data.gyro.z * dt;
        ypr->yaw = yaw_integral;
        
        // Limitar yaw entre -180 y 180 grados
        while (ypr->yaw > 180.0f) ypr->yaw -= 360.0f;
        while (ypr->yaw < -180.0f) ypr->yaw += 360.0f;
    }
    
    // Generar quaternion simplificado (para compatibilidad)
    if (quat != NULL) {
        float roll_rad = DEG_TO_RAD(ypr->roll * 0.5f);
        float pitch_rad = DEG_TO_RAD(ypr->pitch * 0.5f);
        float yaw_rad = DEG_TO_RAD(ypr->yaw * 0.5f);
        
        quat->w = cos(roll_rad) * cos(pitch_rad) * cos(yaw_rad) + 
                  sin(roll_rad) * sin(pitch_rad) * sin(yaw_rad);
        quat->x = sin(roll_rad) * cos(pitch_rad) * cos(yaw_rad) - 
                  cos(roll_rad) * sin(pitch_rad) * sin(yaw_rad);
        quat->y = cos(roll_rad) * sin(pitch_rad) * cos(yaw_rad) + 
                  sin(roll_rad) * cos(pitch_rad) * sin(yaw_rad);
        quat->z = cos(roll_rad) * cos(pitch_rad) * sin(yaw_rad) - 
                  sin(roll_rad) * sin(pitch_rad) * cos(yaw_rad);
    }
    
    last_read_time = current_time;
    return true;
}

/**
 * @brief Lee datos del acelerómetro
 */
bool MPU6050_ReadAccel(MPU6050_Accel_t *accel) {
    
    uint8_t data[6];
    
    // Leer 6 bytes consecutivos desde el registro del acelerómetro
    if (!MPU6050_ReadRegisters(MPU6050_ACCEL_XOUT_H, data, 6)) {
        return false;
    }
    
    // Combinar bytes y convertir a valores en g
    int16_t raw_x = MPU6050_CombineBytes(data[0], data[1]);
    int16_t raw_y = MPU6050_CombineBytes(data[2], data[3]);
    int16_t raw_z = MPU6050_CombineBytes(data[4], data[5]);
    
    // Convertir a g y aplicar calibración
    accel->x = (raw_x / ACCEL_SCALE_FACTOR) - accel_offset.x;
    accel->y = (raw_y / ACCEL_SCALE_FACTOR) - accel_offset.y;
    accel->z = (raw_z / ACCEL_SCALE_FACTOR) - accel_offset.z;
    
    return true;
}

/**
 * @brief Lee datos del giroscopio
 */
bool MPU6050_ReadGyro(MPU6050_Gyro_t *gyro) {
    
    uint8_t data[6];
    
    // Leer 6 bytes consecutivos desde el registro del giroscopio
    if (!MPU6050_ReadRegisters(MPU6050_GYRO_XOUT_H, data, 6)) {
        return false;
    }
    
    // Combinar bytes y convertir a valores en °/s
    int16_t raw_x = MPU6050_CombineBytes(data[0], data[1]);
    int16_t raw_y = MPU6050_CombineBytes(data[2], data[3]);
    int16_t raw_z = MPU6050_CombineBytes(data[4], data[5]);
    
    // Convertir a °/s y aplicar calibración
    gyro->x = (raw_x / GYRO_SCALE_FACTOR) - gyro_offset.x;
    gyro->y = (raw_y / GYRO_SCALE_FACTOR) - gyro_offset.y;
    gyro->z = (raw_z / GYRO_SCALE_FACTOR) - gyro_offset.z;
    
    return true;
}

/**
 * @brief Lee la temperatura del sensor
 */
float MPU6050_ReadTemperature(void) {
    
    uint8_t data[2];
    
    // Leer 2 bytes de temperatura
    if (!MPU6050_ReadRegisters(MPU6050_TEMP_OUT_H, data, 2)) {
        return 0.0f;
    }
    
    // Combinar bytes y convertir a temperatura
    int16_t raw_temp = MPU6050_CombineBytes(data[0], data[1]);
    
    // Fórmula de conversión según datasheet: T = (raw/340) + 36.53
    return (raw_temp / 340.0f) + 36.53f;
}

/**
 * @brief Lee todos los datos del sensor
 */
bool MPU6050_ReadAll(MPU6050_Data_t *data) {
    
    uint8_t raw_data[14];
    
    // Leer todos los 14 bytes de datos (accel + temp + gyro)
    if (!MPU6050_ReadRegisters(MPU6050_ACCEL_XOUT_H, raw_data, 14)) {
        return false;
    }
    
    // Procesar acelerómetro
    int16_t raw_ax = MPU6050_CombineBytes(raw_data[0], raw_data[1]);
    int16_t raw_ay = MPU6050_CombineBytes(raw_data[2], raw_data[3]);
    int16_t raw_az = MPU6050_CombineBytes(raw_data[4], raw_data[5]);
    
    data->accel.x = (raw_ax / ACCEL_SCALE_FACTOR) - accel_offset.x;
    data->accel.y = (raw_ay / ACCEL_SCALE_FACTOR) - accel_offset.y;
    data->accel.z = (raw_az / ACCEL_SCALE_FACTOR) - accel_offset.z;
    
    // Procesar temperatura
    int16_t raw_temp = MPU6050_CombineBytes(raw_data[6], raw_data[7]);
    data->temperature = (raw_temp / 340.0f) + 36.53f;
    
    // Procesar giroscopio
    int16_t raw_gx = MPU6050_CombineBytes(raw_data[8], raw_data[9]);
    int16_t raw_gy = MPU6050_CombineBytes(raw_data[10], raw_data[11]);
    int16_t raw_gz = MPU6050_CombineBytes(raw_data[12], raw_data[13]);
    
    data->gyro.x = (raw_gx / GYRO_SCALE_FACTOR) - gyro_offset.x;
    data->gyro.y = (raw_gy / GYRO_SCALE_FACTOR) - gyro_offset.y;
    data->gyro.z = (raw_gz / GYRO_SCALE_FACTOR) - gyro_offset.z;
    
    // Calcular ángulos
    data->angles.roll = RAD_TO_DEG(atan2(data->accel.y, 
                                         sqrt(data->accel.x * data->accel.x + 
                                              data->accel.z * data->accel.z)));
    
    data->angles.pitch = RAD_TO_DEG(atan2(-data->accel.x, 
                                          sqrt(data->accel.y * data->accel.y + 
                                               data->accel.z * data->accel.z)));
    
    data->angles.yaw = 0.0f; // Requiere integración temporal
    
    // Timestamp y flag
    data->timestamp = GetTimestamp_ms();
    data->data_ready = true;
    
    return true;
}

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES DE DETECCIÓN DE MOVIMIENTOS
 *******************************************************************************/

/**
 * @brief Inicializa el detector de movimientos
 */
void MPU6050_Movement_Init(void) {
    
    movement_detector.current_movement = MOVEMENT_NONE;
    movement_detector.state = MPU6050_STATE_IDLE;
    movement_detector.movement_start_time = 0;
    movement_detector.movement_duration = 0;
    movement_detector.confidence_level = 0;
    
    // Inicializar filtros de media móvil
    MovingAverage_Reset(&movement_detector.accel_x_filter);
    MovingAverage_Reset(&movement_detector.accel_y_filter);
    MovingAverage_Reset(&movement_detector.accel_z_filter);
    MovingAverage_Reset(&movement_detector.gyro_x_filter);
    MovingAverage_Reset(&movement_detector.gyro_y_filter);
    MovingAverage_Reset(&movement_detector.gyro_z_filter);
}

/**
 * @brief Detecta movimientos direccionales
 */
MPU6050_Movement_t MPU6050_DetectMovement(MPU6050_Data_t *data) {
    
    if (data == NULL || !data->data_ready) {
        return MOVEMENT_NONE;
    }
    
    // Aplicar filtros de media móvil para reducir ruido
    float filtered_ax = MovingAverage_Update(&movement_detector.accel_x_filter, data->accel.x);
    float filtered_ay = MovingAverage_Update(&movement_detector.accel_y_filter, data->accel.y);
    float filtered_az = MovingAverage_Update(&movement_detector.accel_z_filter, data->accel.z);
    float filtered_gx = MovingAverage_Update(&movement_detector.gyro_x_filter, data->gyro.x);
    float filtered_gy = MovingAverage_Update(&movement_detector.gyro_y_filter, data->gyro.y);
    float filtered_gz = MovingAverage_Update(&movement_detector.gyro_z_filter, data->gyro.z);
    
    // Crear estructura con datos filtrados
    MPU6050_Data_t filtered_data = *data;
    filtered_data.accel.x = filtered_ax;
    filtered_data.accel.y = filtered_ay;
    filtered_data.accel.z = filtered_az;
    filtered_data.gyro.x = filtered_gx;
    filtered_data.gyro.y = filtered_gy;
    filtered_data.gyro.z = filtered_gz;
    
    // Analizar patrón de movimiento
    MPU6050_Movement_t detected_movement = AnalyzeMovementPattern(&filtered_data);
    
    uint16_t current_time = GetTimestamp_ms();
    
    // Máquina de estados para detección
    switch (movement_detector.state) {
        
        case MPU6050_STATE_IDLE:
            if (detected_movement != MOVEMENT_NONE) {
                movement_detector.state = MPU6050_STATE_DETECTING;
                movement_detector.movement_start_time = current_time;
                movement_detector.current_movement = detected_movement;
            }
            break;
            
        case MPU6050_STATE_DETECTING:
            movement_detector.movement_duration = current_time - movement_detector.movement_start_time;
            
            if (detected_movement == movement_detector.current_movement) {
                // Movimiento consistente
                if (movement_detector.movement_duration >= MOVEMENT_MIN_DURATION) {
                    movement_detector.state = MPU6050_STATE_CONFIRMED;
                    movement_detector.confidence_level = CalculateConfidence(detected_movement, &filtered_data);
                }
            } else if (detected_movement == MOVEMENT_NONE) {
                // Movimiento terminó
                if (movement_detector.movement_duration >= MOVEMENT_MIN_DURATION) {
                    movement_detector.state = MPU6050_STATE_CONFIRMED;
                } else {
                    // Movimiento muy corto, ignorar
                    movement_detector.state = MPU6050_STATE_IDLE;
                    movement_detector.current_movement = MOVEMENT_NONE;
                }
            } else {
                // Cambio de movimiento, reiniciar detección
                movement_detector.movement_start_time = current_time;
                movement_detector.current_movement = detected_movement;
            }
            
            // Timeout para movimientos muy largos
            if (movement_detector.movement_duration > MOVEMENT_MAX_DURATION) {
                movement_detector.state = MPU6050_STATE_IDLE;
                movement_detector.current_movement = MOVEMENT_NONE;
            }
            break;
            
        case MPU6050_STATE_CONFIRMED:
            // Período de enfriamiento para evitar detecciones múltiples
            movement_detector.state = MPU6050_STATE_COOLDOWN;
            return movement_detector.current_movement;
            
        case MPU6050_STATE_COOLDOWN:
            // Esperar un poco antes de detectar el siguiente movimiento
            if ((current_time - movement_detector.movement_start_time) > 500) {
                movement_detector.state = MPU6050_STATE_IDLE;
                movement_detector.current_movement = MOVEMENT_NONE;
            }
            break;
    }
    
    return MOVEMENT_NONE;
}

/**
 * @brief Convierte el tipo de movimiento a string
 */
const char* MPU6050_MovementToString(MPU6050_Movement_t movement) {
    
    switch (movement) {
        case MOVEMENT_NONE:         return "Sin movimiento";
        case MOVEMENT_FORWARD:      return "Adelante";
        case MOVEMENT_BACKWARD:     return "Atras";
        case MOVEMENT_LEFT:         return "Izquierda";
        case MOVEMENT_RIGHT:        return "Derecha";
        case MOVEMENT_TILT_FORWARD: return "Inclinacion adelante";
        case MOVEMENT_TILT_BACKWARD:return "Inclinacion atras";
        case MOVEMENT_TILT_LEFT:    return "Inclinacion izquierda";
        case MOVEMENT_TILT_RIGHT:   return "Inclinacion derecha";
        case MOVEMENT_SHAKE:        return "Sacudida";
        case MOVEMENT_TAP:          return "Golpe";
        default:                    return "Desconocido";
    }
}

/**
 * @brief Obtiene el nivel de confianza del último movimiento detectado
 */
uint8_t MPU6050_GetConfidenceLevel(void) {
    return movement_detector.confidence_level;
}

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES DE CALIBRACIÓN
 *******************************************************************************/

/**
 * @brief Calibra el sensor
 */
bool MPU6050_Calibrate(void) {
    
    const uint16_t num_samples = 200;
    float accel_sum_x = 0, accel_sum_y = 0, accel_sum_z = 0;
    float gyro_sum_x = 0, gyro_sum_y = 0, gyro_sum_z = 0;
    
    // Leer múltiples muestras para calcular promedio
    for (uint16_t i = 0; i < num_samples; i++) {
        MPU6050_Data_t data;
        
        if (!MPU6050_ReadAll(&data)) {
            return false;
        }
        
        accel_sum_x += data.accel.x;
        accel_sum_y += data.accel.y;
        accel_sum_z += data.accel.z;
        
        gyro_sum_x += data.gyro.x;
        gyro_sum_y += data.gyro.y;
        gyro_sum_z += data.gyro.z;
        
        __delay_ms(5);
    }
    
    // Calcular offsets (el acelerómetro Z debe ser ~1g cuando está horizontal)
    accel_offset.x = accel_sum_x / num_samples;
    accel_offset.y = accel_sum_y / num_samples;
    accel_offset.z = (accel_sum_z / num_samples) - 1.0f; // Restar 1g
    
    gyro_offset.x = gyro_sum_x / num_samples;
    gyro_offset.y = gyro_sum_y / num_samples;
    gyro_offset.z = gyro_sum_z / num_samples;
    
    return true;
}

/**
 * @brief Obtiene los offsets de calibración
 */
void MPU6050_GetCalibrationOffsets(MPU6050_Accel_t *accel_offset_out, MPU6050_Gyro_t *gyro_offset_out) {
    
    if (accel_offset_out != NULL) {
        *accel_offset_out = accel_offset;
    }
    
    if (gyro_offset_out != NULL) {
        *gyro_offset_out = gyro_offset;
    }
}

/**
 * @brief Establece los offsets de calibración
 */
void MPU6050_SetCalibrationOffsets(MPU6050_Accel_t *accel_offset_in, MPU6050_Gyro_t *gyro_offset_in) {
    
    if (accel_offset_in != NULL) {
        accel_offset = *accel_offset_in;
    }
    
    if (gyro_offset_in != NULL) {
        gyro_offset = *gyro_offset_in;
    }
}

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES DE UTILIDAD
 *******************************************************************************/

/**
 * @brief Verifica si el sensor está conectado
 */
bool MPU6050_TestConnection(void) {
    
    uint8_t who_am_i = MPU6050_ReadRegister(MPU6050_WHO_AM_I);
    return (who_am_i == 0x68);
}

/**
 * @brief Reinicia el sensor
 */
bool MPU6050_Reset(void) {
    
    // Activar bit de reset
    if (!MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x80)) {
        return false;
    }
    
    // Esperar reset
    __delay_ms(100);
    
    // Re-inicializar
    return MPU6050_Init();
}

/**
 * @brief Pone el sensor en modo de bajo consumo
 */
void MPU6050_Sleep(void) {
    MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x40); // Activar bit SLEEP
}

/**
 * @brief Despierta el sensor del modo de bajo consumo
 */
void MPU6050_WakeUp(void) {
    MPU6050_WriteRegister(MPU6050_PWR_MGMT_1, 0x00); // Desactivar bit SLEEP
    __delay_ms(100);
}

/*******************************************************************************
 * IMPLEMENTACIÓN DE FUNCIONES PRIVADAS
 *******************************************************************************/

/**
 * @brief Escribe un registro del MPU6050
 */
static bool MPU6050_WriteRegister(uint8_t reg, uint8_t data) {
    return (SW_I2C_WriteRegister(MPU6050_I2C_ADDR, reg, data) == SW_I2C_SUCCESS);
}

/**
 * @brief Lee un registro del MPU6050
 */
static uint8_t MPU6050_ReadRegister(uint8_t reg) {
    return SW_I2C_ReadRegister(MPU6050_I2C_ADDR, reg);
}

/**
 * @brief Lee múltiples registros del MPU6050
 */
static bool MPU6050_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t length) {
    return (SW_I2C_ReadBlock(MPU6050_I2C_ADDR, reg, data, length) == SW_I2C_SUCCESS);
}

/**
 * @brief Combina dos bytes en un int16_t (formato big-endian del MPU6050)
 */
static int16_t MPU6050_CombineBytes(uint8_t high, uint8_t low) {
    return (int16_t)((high << 8) | low);
}

/**
 * @brief Actualiza filtro de media móvil
 */
static float MovingAverage_Update(MovingAverage_t *filter, float new_value) {
    
    filter->buffer[filter->index] = new_value;
    filter->index = (filter->index + 1) % FILTER_SAMPLES;
    
    if (!filter->filled && filter->index == 0) {
        filter->filled = true;
    }
    
    float sum = 0.0f;
    uint8_t count = filter->filled ? FILTER_SAMPLES : filter->index;
    
    for (uint8_t i = 0; i < count; i++) {
        sum += filter->buffer[i];
    }
    
    return (count > 0) ? (sum / count) : new_value;
}

/**
 * @brief Reinicia filtro de media móvil
 */
static void MovingAverage_Reset(MovingAverage_t *filter) {
    filter->index = 0;
    filter->filled = false;
    
    for (uint8_t i = 0; i < FILTER_SAMPLES; i++) {
        filter->buffer[i] = 0.0f;
    }
}

/**
 * @brief Obtiene timestamp en milisegundos (simplificado)
 */
static uint16_t GetTimestamp_ms(void) {
    static uint16_t timestamp = 0;
    timestamp++; // Incremento simple para simulación
    return timestamp;
}

/**
 * @brief Analiza el patrón de movimiento
 */
static MPU6050_Movement_t AnalyzeMovementPattern(MPU6050_Data_t *data) {
    
    // Obtener valores absolutos para análisis
    float abs_ax = ABS_FLOAT(data->accel.x);
    float abs_ay = ABS_FLOAT(data->accel.y);
    float abs_az = ABS_FLOAT(data->accel.z);
    float abs_gx = ABS_FLOAT(data->gyro.x);
    float abs_gy = ABS_FLOAT(data->gyro.y);
    float abs_gz = ABS_FLOAT(data->gyro.z);
    
    // Detección de movimientos lineales (basados en acelerómetro)
    if (abs_ay > MOVEMENT_THRESHOLD_ACCEL) {
        if (data->accel.y > FORWARD_THRESHOLD) {
            return MOVEMENT_FORWARD;
        } else if (data->accel.y < BACKWARD_THRESHOLD) {
            return MOVEMENT_BACKWARD;
        }
    }
    
    // Detección de giros (basados en giroscopio)
    if (abs_gz > MOVEMENT_THRESHOLD_GYRO) {
        if (data->gyro.z > RIGHT_THRESHOLD) {
            return MOVEMENT_RIGHT;
        } else if (data->gyro.z < LEFT_THRESHOLD) {
            return MOVEMENT_LEFT;
        }
    }
    
    // Detección de inclinaciones (basados en ángulos)
    if (ABS_FLOAT(data->angles.pitch) > 20.0f) {
        if (data->angles.pitch > 20.0f) {
            return MOVEMENT_TILT_FORWARD;
        } else {
            return MOVEMENT_TILT_BACKWARD;
        }
    }
    
    if (ABS_FLOAT(data->angles.roll) > 20.0f) {
        if (data->angles.roll > 20.0f) {
            return MOVEMENT_TILT_RIGHT;
        } else {
            return MOVEMENT_TILT_LEFT;
        }
    }
    
#ifdef MPU6050_ADVANCED_GESTURES
    // Detección de sacudida (movimiento rápido en múltiples ejes)
    if ((abs_ax + abs_ay + abs_az) > 2.0f && (abs_gx + abs_gy + abs_gz) > 100.0f) {
        return MOVEMENT_SHAKE;
    }
    
    // Detección de golpe (pico de aceleración corto)
    if ((abs_ax + abs_ay + abs_az) > 3.0f) {
        return MOVEMENT_TAP;
    }
#endif
    
    return MOVEMENT_NONE;
}

/**
 * @brief Calcula nivel de confianza del movimiento detectado
 */
static uint8_t CalculateConfidence(MPU6050_Movement_t movement, MPU6050_Data_t *data) {
    
    uint8_t confidence = 50; // Confianza base
    
    switch (movement) {
        case MOVEMENT_FORWARD:
        case MOVEMENT_BACKWARD:
            // Mayor confianza si la aceleración Y es dominante
            if (ABS_FLOAT(data->accel.y) > ABS_FLOAT(data->accel.x) && 
                ABS_FLOAT(data->accel.y) > ABS_FLOAT(data->accel.z)) {
                confidence += 30;
            }
            break;
            
        case MOVEMENT_LEFT:
        case MOVEMENT_RIGHT:
            // Mayor confianza si el giro Z es dominante
            if (ABS_FLOAT(data->gyro.z) > ABS_FLOAT(data->gyro.x) && 
                ABS_FLOAT(data->gyro.z) > ABS_FLOAT(data->gyro.y)) {
                confidence += 30;
            }
            break;
            
        case MOVEMENT_TILT_FORWARD:
        case MOVEMENT_TILT_BACKWARD:
        case MOVEMENT_TILT_LEFT:
        case MOVEMENT_TILT_RIGHT:
            // Confianza basada en la estabilidad del ángulo
            confidence += 20;
            break;
            
        default:
            confidence = 60;
            break;
    }
    
    // Limitar confianza entre 0 y 100
    return CLAMP(confidence, 0, 100);
}
