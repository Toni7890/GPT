/*******************************************************************************
 * SOFTWARE I2C IMPLEMENTATION para PIC18F57Q43 - VERSI?N CORREGIDA
 * 
 * CORRECCIONES APLICADAS:
 * ? Variable current_i2c_speed inicializada
 * ? Reemplazado SW_I2C_DELAY_US por macros correctos
 * ? Corregidas conversiones de tipo implÃcitas
 * ? Agregadas funciones de configuraci€n faltantes
 *******************************************************************************/

#include "Software_I2C.h"

/*******************************************************************************
 * VARIABLES GLOBALES INICIALIZADAS
 *******************************************************************************/
// Inicializar la variable de velocidad (CORREGIDO)
sw_i2c_speed_t current_i2c_speed = SW_I2C_SPEED_NORMAL;

/*******************************************************************************
 * FUNCIONES DE CONFIGURACI?N NUEVAS
 *******************************************************************************/
void SW_I2C_SetSpeed(sw_i2c_speed_t speed) {
    current_i2c_speed = speed;
}

sw_i2c_speed_t SW_I2C_GetSpeed(void) {
    return current_i2c_speed;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_Init (MANTENIDA)
 *******************************************************************************/
void SW_I2C_Init(void) {
    // Configurar pines como entrada inicialmente (open-drain simulation)
    SW_I2C_SCL_INPUT();
    SW_I2C_SDA_INPUT();
    
    // Establecer valores bajos en LAT (para cuando se configure como salida)
    SW_I2C_SCL_HIGH();
    SW_I2C_SDA_HIGH();
    
    // Configurar pines como digitales (deshabilitar analog)
    ANSELBbits.ANSELB1 = 0;  // RB1 digital
    ANSELBbits.ANSELB2 = 0;  // RB2 digital
    
    // Configurar open-drain para simular I2C
    ODCONBbits.ODCB1 = 1;    // RB1 open-drain
    ODCONBbits.ODCB2 = 1;    // RB2 open-drain
    
    // Estado inicial: ambas lÃneas en HIGH (idle)
    SW_I2C_SCL_INPUT();      // SCL = HIGH (pull-up)
    SW_I2C_SDA_INPUT();      // SDA = HIGH (pull-up)
    
    // Delay inicial para estabilizaci€n
    __delay_ms(10);
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_Start (MANTENIDA)
 *******************************************************************************/
sw_i2c_error_t SW_I2C_Start(void) {
    // Verificar que el bus est» libre
    if (!SW_I2C_IsBusFree()) {
        return SW_I2C_ERROR_BUS_BUSY;
    }
    
    // Condici€n START: SDA baja mientras SCL est∑ alto
    SW_I2C_SDA_INPUT();      // SDA = HIGH
    SW_I2C_SCL_INPUT();      // SCL = HIGH
    SW_I2C_DELAY();
    
    SW_I2C_SDA_OUTPUT();     // SDA = LOW
    SW_I2C_SDA_LOW();
    SW_I2C_DELAY();
    
    SW_I2C_SCL_OUTPUT();     // SCL = LOW
    SW_I2C_SCL_LOW();
    SW_I2C_DELAY();
    
    return SW_I2C_SUCCESS;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_Stop (MANTENIDA)
 *******************************************************************************/
void SW_I2C_Stop(void) {
    // Asegurar que SCL est» bajo
    SW_I2C_SCL_OUTPUT();
    SW_I2C_SCL_LOW();
    SW_I2C_DELAY();
    
    // Asegurar que SDA est» bajo
    SW_I2C_SDA_OUTPUT();
    SW_I2C_SDA_LOW();
    SW_I2C_DELAY();
    
    // Condici€n STOP: SCL alto, luego SDA alto
    SW_I2C_SCL_INPUT();      // SCL = HIGH
    SW_I2C_DELAY();
    
    SW_I2C_SDA_INPUT();      // SDA = HIGH
    SW_I2C_DELAY();
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_Restart (MANTENIDA)
 *******************************************************************************/
sw_i2c_error_t SW_I2C_Restart(void) {
    // RESTART = STOP seguido de START
    SW_I2C_Stop();
    __delay_us(SW_I2C_DELAY_US);  // CORREGIDO: usar constante definida
    return SW_I2C_Start();
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_WriteByte (MANTENIDA)
 *******************************************************************************/
sw_i2c_error_t SW_I2C_WriteByte(uint8_t data) {
    uint8_t i;
    
    // Enviar 8 bits, empezando por el MSB
    for (i = 0; i < 8; i++) {
        // Preparar el bit en SDA
        if (data & 0x80) {
            SW_I2C_SDA_INPUT();  // SDA = HIGH
        } else {
            SW_I2C_SDA_OUTPUT(); // SDA = LOW
            SW_I2C_SDA_LOW();
        }
        
        SW_I2C_DELAY();
        
        // Pulso de clock
        SW_I2C_SCL_INPUT();      // SCL = HIGH
        SW_I2C_DELAY();
        SW_I2C_SCL_OUTPUT();     // SCL = LOW
        SW_I2C_SCL_LOW();
        SW_I2C_DELAY();
        
        data <<= 1;              // Siguiente bit
    }
    
    // Leer ACK/NACK
    SW_I2C_SDA_INPUT();          // Liberar SDA para ACK
    SW_I2C_DELAY();
    
    SW_I2C_SCL_INPUT();          // SCL = HIGH
    SW_I2C_DELAY();
    
    uint8_t ack = SW_I2C_SDA_READ();  // Leer ACK (0 = ACK, 1 = NACK)
    
    SW_I2C_SCL_OUTPUT();         // SCL = LOW
    SW_I2C_SCL_LOW();
    SW_I2C_DELAY();
    
    return (ack == 0) ? SW_I2C_SUCCESS : SW_I2C_ERROR_NACK;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_ReadByte (MANTENIDA)
 *******************************************************************************/
uint8_t SW_I2C_ReadByte(uint8_t ack) {
    uint8_t i;
    uint8_t data = 0;
    
    SW_I2C_SDA_INPUT();          // SDA como entrada
    
    // Leer 8 bits
    for (i = 0; i < 8; i++) {
        data <<= 1;              // Shift left
        
        SW_I2C_SCL_INPUT();      // SCL = HIGH
        SW_I2C_DELAY();
        
        if (SW_I2C_SDA_READ()) { // Leer bit
            data |= 1;
        }
        
        SW_I2C_SCL_OUTPUT();     // SCL = LOW
        SW_I2C_SCL_LOW();
        SW_I2C_DELAY();
    }
    
    // Enviar ACK/NACK
    if (ack) {
        SW_I2C_SendAck();
    } else {
        SW_I2C_SendNack();
    }
    
    return data;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_SendAck (MANTENIDA)
 *******************************************************************************/
void SW_I2C_SendAck(void) {
    SW_I2C_SDA_OUTPUT();         // SDA = LOW (ACK)
    SW_I2C_SDA_LOW();
    SW_I2C_DELAY();
    
    SW_I2C_SCL_INPUT();          // SCL = HIGH
    SW_I2C_DELAY();
    SW_I2C_SCL_OUTPUT();         // SCL = LOW
    SW_I2C_SCL_LOW();
    SW_I2C_DELAY();
    
    SW_I2C_SDA_INPUT();          // Liberar SDA
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_SendNack (MANTENIDA)
 *******************************************************************************/
void SW_I2C_SendNack(void) {
    SW_I2C_SDA_INPUT();          // SDA = HIGH (NACK)
    SW_I2C_DELAY();
    
    SW_I2C_SCL_INPUT();          // SCL = HIGH
    SW_I2C_DELAY();
    SW_I2C_SCL_OUTPUT();         // SCL = LOW
    SW_I2C_SCL_LOW();
    SW_I2C_DELAY();
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_IsBusFree (MANTENIDA)
 *******************************************************************************/
uint8_t SW_I2C_IsBusFree(void) {
    SW_I2C_SCL_INPUT();
    SW_I2C_SDA_INPUT();
    __delay_us(SW_I2C_DELAY_US);  // CORREGIDO: usar constante definida
    
    // Bus libre si ambas lÃneas est∑n HIGH
    return (SW_I2C_SCL_PORT && SW_I2C_SDA_PORT);
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_WriteRegister (CORREGIDA - Conversiones de tipo)
 *******************************************************************************/
sw_i2c_error_t SW_I2C_WriteRegister(uint8_t device_addr, uint8_t reg_addr, uint8_t data) {
    sw_i2c_error_t result;
    
    // START
    result = SW_I2C_Start();
    if (result != SW_I2C_SUCCESS) {
        return result;
    }
    
    // Enviar direcci€n del dispositivo + WRITE (bit 0 = 0)
    // CORREGIDO: Cast explÃcito para evitar warning
    result = SW_I2C_WriteByte((uint8_t)((device_addr << 1) | 0));
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // Enviar direcci€n del registro
    result = SW_I2C_WriteByte(reg_addr);
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // Enviar dato
    result = SW_I2C_WriteByte(data);
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // STOP
    SW_I2C_Stop();
    
    return SW_I2C_SUCCESS;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_ReadRegister (CORREGIDA - Conversiones de tipo)
 *******************************************************************************/
uint8_t SW_I2C_ReadRegister(uint8_t device_addr, uint8_t reg_addr) {
    sw_i2c_error_t result;
    uint8_t data = 0x00;
    
    // START
    result = SW_I2C_Start();
    if (result != SW_I2C_SUCCESS) {
        return 0x00;
    }
    
    // Enviar direcci€n del dispositivo + WRITE
    // CORREGIDO: Cast explÃcito para evitar warning
    result = SW_I2C_WriteByte((uint8_t)((device_addr << 1) | 0));
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return 0x00;
    }
    
    // Enviar direcci€n del registro
    result = SW_I2C_WriteByte(reg_addr);
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return 0x00;
    }
    
    // RESTART
    result = SW_I2C_Restart();
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return 0x00;
    }
    
    // Enviar direcci€n del dispositivo + READ
    // CORREGIDO: Cast explÃcito para evitar warning
    result = SW_I2C_WriteByte((uint8_t)((device_addr << 1) | 1));
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return 0x00;
    }
    
    // Leer dato (con NACK para terminar)
    data = SW_I2C_ReadByte(0);  // 0 = NACK
    
    // STOP
    SW_I2C_Stop();
    
    return data;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_ReadBlock (CORREGIDA - Conversiones de tipo)
 *******************************************************************************/
sw_i2c_error_t SW_I2C_ReadBlock(uint8_t device_addr, uint8_t reg_addr, uint8_t *data, uint8_t length) {
    sw_i2c_error_t result;
    uint8_t i;
    
    if (data == NULL || length == 0) {
        return SW_I2C_ERROR_TIMEOUT;
    }
    
    // START
    result = SW_I2C_Start();
    if (result != SW_I2C_SUCCESS) {
        return result;
    }
    
    // Enviar direcci€n del dispositivo + WRITE
    // CORREGIDO: Cast explÃcito para evitar warning
    result = SW_I2C_WriteByte((uint8_t)((device_addr << 1) | 0));
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // Enviar direcci€n del registro
    result = SW_I2C_WriteByte(reg_addr);
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // RESTART
    result = SW_I2C_Restart();
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // Enviar direcci€n del dispositivo + READ
    // CORREGIDO: Cast explÃcito para evitar warning
    result = SW_I2C_WriteByte((uint8_t)((device_addr << 1) | 1));
    if (result != SW_I2C_SUCCESS) {
        SW_I2C_Stop();
        return result;
    }
    
    // Leer datos
    for (i = 0; i < length; i++) {
        if (i == (length - 1)) {
            data[i] = SW_I2C_ReadByte(0);  // ?ltimo byte con NACK
        } else {
            data[i] = SW_I2C_ReadByte(1);  // Bytes intermedios con ACK
        }
    }
    
    // STOP
    SW_I2C_Stop();
    
    return SW_I2C_SUCCESS;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_ScanBus (CORREGIDA - Conversiones de tipo)
 *******************************************************************************/
uint8_t SW_I2C_ScanBus(uint8_t *found_devices, uint8_t max_devices) {
    uint8_t devices_found = 0;
    uint8_t addr;
    sw_i2c_error_t result;
    
    if (found_devices == NULL || max_devices == 0) {
        return 0;
    }
    
    // Escanear direcciones de 0x08 a 0x77 (excluyendo direcciones reservadas)
    for (addr = 0x08; addr <= 0x77 && devices_found < max_devices; addr++) {
        // Intentar comunicaci€n con esta direcci€n
        result = SW_I2C_Start();
        if (result == SW_I2C_SUCCESS) {
            // CORREGIDO: Cast explÃcito para evitar warning
            result = SW_I2C_WriteByte((uint8_t)((addr << 1) | 0));  // WRITE
            if (result == SW_I2C_SUCCESS) {
                // Dispositivo encontrado
                found_devices[devices_found] = addr;
                devices_found++;
            }
        }
        SW_I2C_Stop();
        
        // Peque“o delay entre intentos
        __delay_ms(1);
    }
    
    return devices_found;
}

/*******************************************************************************
 * FUNCI?N: SW_I2C_GetPinStatus (MANTENIDA)
 *******************************************************************************/
uint8_t SW_I2C_GetPinStatus(void) {
    uint8_t status = 0;
    
    if (SW_I2C_SCL_PORT) status |= 0x01;  // SCL state
    if (SW_I2C_SDA_PORT) status |= 0x02;  // SDA state
    
    return status;
}
