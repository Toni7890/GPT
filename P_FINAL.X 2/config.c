// PIC18F57Q43 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG3
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9  // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config DEBUG = ON       // Background Debugger (Background Debugger enabled)

// CONFIG8
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#include <xc.h>
#include <stdbool.h>  // ? AGREGADO: Para usar bool, true, false
#include "config.h"

/*******************************************************************************
 * FUNCION: PortD_Init
 * DESCRIPCION: Configura el Puerto D para el control del L298N
 * PARAMETROS: Ninguno
 * RETORNA: Nada
 * 
 * NOTA CRITICA: El PIC18F57Q43 Puerto D puede no tener registro ANSELD
 * Esta funcion maneja ambos casos (con y sin ANSELD)
 *******************************************************************************/
void PortD_Init(void){
    
    // =========================================================================
    // CONFIGURACION CRITICA: PUERTO D COMO DIGITAL
    // =========================================================================
    
    // IMPORTANTE: En algunos PIC18F57Q43, el Puerto D no tiene registro ANSELD
    // Intentar configurar como digital si el registro existe
    #ifdef ANSELD
        ANSELD = 0x00;              // Puerto D como digital (si existe)
    #else
        // Si ANSELD no existe, el Puerto D es digital por defecto
        // No necesita configuracion analogica/digital
    #endif
    
    // =========================================================================
    // CONFIGURACION DE DIRECCION DE PINES (CRITICO PARA L298N)
    // =========================================================================
    
    // Configurar todos los pines del Puerto D como SALIDAS (TRIS = 0)
    TRISD = 0x00;                   // TODO el Puerto D como salidas
    
    // Configuracion individual para mayor claridad (redundante pero seguro)
    TRISDbits.TRISD0 = 0;           // RD0 Motor A IN1 - Salida
    TRISDbits.TRISD1 = 0;           // RD1 Motor A IN2 - Salida  
    TRISDbits.TRISD2 = 0;           // RD2 Motor A ENA - Salida
    TRISDbits.TRISD3 = 0;           // RD3 Motor B ENB - Salida
    TRISDbits.TRISD4 = 0;           // RD4 Motor B IN3 - Salida
    TRISDbits.TRISD5 = 0;           // RD5 Motor B IN4 - Salida
    TRISDbits.TRISD6 = 0;           // RD6 LED debug - Salida
    TRISDbits.TRISD7 = 0;           // RD7 como salida tambien
    
    // =========================================================================
    // CONFIGURACION DE PULL-UPS (DESHABILITADOS PARA SALIDAS)
    // =========================================================================
    
    // Deshabilitar pull-ups en todo el Puerto D
    WPUD = 0x00;                    // Sin pull-ups en Puerto D
    
    // Configuracion individual (redundante pero claro)
    WPUDbits.WPUD0 = 0;             // Sin pull-up en RD0
    WPUDbits.WPUD1 = 0;             // Sin pull-up en RD1
    WPUDbits.WPUD2 = 0;             // Sin pull-up en RD2
    WPUDbits.WPUD3 = 0;             // Sin pull-up en RD3
    WPUDbits.WPUD4 = 0;             // Sin pull-up en RD4
    WPUDbits.WPUD5 = 0;             // Sin pull-up en RD5
    WPUDbits.WPUD6 = 0;             // Sin pull-up en RD6
    WPUDbits.WPUD7 = 0;             // Sin pull-up en RD7
    
    // =========================================================================
    // ESTADO INICIAL SEGURO (TODOS LOS MOTORES PARADOS)
    // =========================================================================
    
    // Limpiar completamente el Puerto D
    LATD = 0x00;                    // Todos los pines en LOW
    
    // Configuracion individual para verificacion
    LATDbits.LATD0 = 0;             // Motor A IN1 = LOW
    LATDbits.LATD1 = 0;             // Motor A IN2 = LOW
    LATDbits.LATD2 = 0;             // Motor A ENA = LOW (deshabilitado)
    LATDbits.LATD3 = 0;             // Motor B ENB = LOW (deshabilitado)  
    LATDbits.LATD4 = 0;             // Motor B IN3 = LOW
    LATDbits.LATD5 = 0;             // Motor B IN4 = LOW
    LATDbits.LATD6 = 0;             // LED debug = OFF
    LATDbits.LATD7 = 0;             // RD7 = LOW
    
    // =========================================================================
    // DELAY DE ESTABILIZACION
    // =========================================================================
    
    __delay_ms(10);                 // Tiempo para estabilizacion de pines
}

/*******************************************************************************
 * FUNCION: ClockInit
 * DESCRIPCION: Configura el oscilador interno HFINTOSC a 64MHz
 *******************************************************************************/
void ClockInit(){
    OSCFRQ = 0x08;              // 64MHz HFINTOSC
    OSCCON1 = 0x60;             // HFINTOSC seleccionado, sin division
    OSCEN = 0x40;               // Habilitar HFINTOSC
    while((OSCCON3 & 0x10) == 0); // Esperar hasta que ORDY = 1 (oscilador listo)
}

/*******************************************************************************
 * FUNCION: PinInit
 * DESCRIPCION: Configura todos los pines del microcontrolador
 * 
 * ORDEN DE CONFIGURACION:
 * 1. Puerto D (L298N) - CRITICO PRIMERO
 * 2. Puerto F (LED y HC-SR04)
 * 3. Puerto C (I2C y UART)
 * 4. Puerto B (Software I2C)
 *******************************************************************************/
void PinInit(){
    
    // =========================================================================
    // PASO 1: CONFIGURAR PUERTO D PARA L298N (CRITICO)
    // =========================================================================
    
    PortD_Init();                   // ESTA ES LA LLAMADA CRITICA
    
    // =========================================================================
    // PASO 2: CONFIGURAR PUERTO F (LED interno y HC-SR04)
    // =========================================================================
    
    // LED interno RF3
    ANSELFbits.ANSELF3 = 0;         // RF3 digital
    TRISFbits.TRISF3 = 0;           // RF3 como salida
    LATFbits.LATF3 = 1;             // LED encendido inicialmente
    
    // HC-SR04 en RF4 (ECHO) y RF5 (TRIG)
    ANSELFbits.ANSELF4 = 0;         // RF4 ECHO digital
    ANSELFbits.ANSELF5 = 0;         // RF5 TRIG digital
    TRISFbits.TRISF4 = 1;           // RF4 ECHO como entrada
    TRISFbits.TRISF5 = 0;           // RF5 TRIG como salida
    LATFbits.LATF5 = 0;             // TRIG inicial LOW
    
    // =========================================================================
    // PASO 3: CONFIGURAR PUERTO C (I2C HARDWARE Y UART)
    // =========================================================================
    
    // --- I2C Hardware en RC5 (SCL) y RC6 (SDA) ---
    ANSELCbits.ANSELC5 = 0;         // RC5 (SCL) como digital
    ANSELCbits.ANSELC6 = 0;         // RC6 (SDA) como digital
    TRISCbits.TRISC5 = 0;           // RC5 (SCL) como salida inicial
    TRISCbits.TRISC6 = 0;           // RC6 (SDA) como salida inicial
    WPUCbits.WPUC5 = 0;             // Sin pull-up interno en RC5
    WPUCbits.WPUC6 = 0;             // Sin pull-up interno en RC6
    ODCONCbits.ODCC5 = 1;           // RC5 Open-Drain habilitado
    ODCONCbits.ODCC6 = 1;           // RC6 Open-Drain habilitado
    
    // I2C PPS (Peripheral Pin Select)
    I2C1SDAPPS = 0x16;              // SDA input desde RC6
    I2C1SCLPPS = 0x15;              // SCL input desde RC5
    RC5PPS = 0x37;                  // RC5 output = I2C1 SCL
    RC6PPS = 0x38;                  // RC6 output = I2C1 SDA
    
    // --- UART en RC2 (TX) y RC3 (RX) ---
    ANSELCbits.ANSELC2 = 0;         // RC2 (TX) como digital
    ANSELCbits.ANSELC3 = 0;         // RC3 (RX) como digital
    TRISCbits.TRISC2 = 0;           // RC2 como salida (TX)
    TRISCbits.TRISC3 = 1;           // RC3 como entrada (RX)
    WPUCbits.WPUC3 = 1;             // Pull-up en RC3 (RX)
    
    // UART PPS (Peripheral Pin Select)
    RC2PPS = 0x20;                  // RC2 -> UART1 TX
    U1RXPPS = 0x13;                 // RC3 -> UART1 RX
    
    // =========================================================================
    // PASO 4: CONFIGURAR PUERTO B (SOFTWARE I2C PARA MPU6050)
    // =========================================================================
    
    // --- Software I2C en RB1 (SCL) y RB2 (SDA) ---
    ANSELBbits.ANSELB1 = 0;         // RB1 SCL como digital
    ANSELBbits.ANSELB2 = 0;         // RB2 SDA como digital
    TRISBbits.TRISB1 = 1;           // RB1 como entrada inicial (open-drain)
    TRISBbits.TRISB2 = 1;           // RB2 como entrada inicial (open-drain)
    WPUBbits.WPUB1 = 1;             // Pull-up en RB1 (SCL)
    WPUBbits.WPUB2 = 1;             // Pull-up en RB2 (SDA)
    ODCONBbits.ODCB1 = 1;           // RB1 Open-Drain habilitado
    ODCONBbits.ODCB2 = 1;           // RB2 Open-Drain habilitado
    
    // Estado inicial I2C (idle = HIGH)
    LATBbits.LATB1 = 1;             // SCL inicial HIGH
    LATBbits.LATB2 = 1;             // SDA inicial HIGH
    
    // =========================================================================
    // DELAY FINAL DE ESTABILIZACION
    // =========================================================================
    __delay_ms(50);                 // Tiempo para que todos los pines se estabilicen
}

/*******************************************************************************
 * FUNCION DE VERIFICACION DE PUERTO D
 *******************************************************************************/

/**
 * @brief Funcion para verificar que Puerto D este correctamente configurado
 * @return true si la configuracion es correcta, false en caso contrario
 */
bool VerificarConfiguracionPuertoD(void) {
    
    // Verificar que TRISD este configurado como salidas (0x00)
    if(TRISD != 0x00) {
        return false;
    }
    
    // Verificar que WPUD este deshabilitado (0x00)
    if(WPUD != 0x00) {
        return false;
    }
    
    // Si llegamos aqui, la configuracion es correcta
    return true;
}

/*******************************************************************************
 * INFORMACION DE COMPILACION
 *******************************************************************************/

#pragma message "====== CONFIG.C CORREGIDO PARA PUERTO D ======"
#pragma message "Puerto D configurado especificamente para L298N"
#pragma message "TRISD forzado a 0x00 (todas salidas)"
#pragma message "ANSELD manejado condicionalmente"
#pragma message "Estado inicial seguro: LATD = 0x00"
#pragma message "=============================================="

/*******************************************************************************
 * VERIFICACION DE COMPILACION
 *******************************************************************************/

#if _XTAL_FREQ != 64000000UL
    #error "ERROR: _XTAL_FREQ debe ser 64000000UL en config.h"
#endif

// Fin del archivo config.c