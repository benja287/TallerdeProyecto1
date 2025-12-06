/**
 * Servo S3003 en T_COL2 con PWM por HARDWARE
 * SIN DELAYS BLOQUEANTES - Usa millis() para control fluido
 */

// IMPORTANTE: Definir ANTES de incluir sapi.h
#define BOARD edu_ciaa_nxp
#define PWM_TOTALS 10

#include "sapi.h"

#define SERVO_PIN T_COL2
#define SERVO_FREQ 50  // 50 Hz para servo

// Variables para control no bloqueante
typedef enum {
    ESTADO_0_GRADOS,
    ESTADO_45_GRADOS,
    ESTADO_90_GRADOS,
    ESTADO_135_GRADOS,
    ESTADO_180_GRADOS,
    ESTADO_BARRIDO_IDA,
    ESTADO_BARRIDO_VUELTA,
    ESTADO_PAUSA
} EstadoServo_t;

EstadoServo_t estadoActual = ESTADO_0_GRADOS;
tick_t tiempoAnterior = 0;
uint8_t anguloBarrido = 0;

/**
 * Configurar ángulo del servo (0-180 grados)
 */
void servoSetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    
    // Calcular duty cycle: 5% (0°) a 10% (180°)
    float dutyPercent = 5.0 + ((float)angle * 5.0 / 180.0);
    uint8_t dutyCycle = (uint8_t)((dutyPercent * 255.0) / 100.0);
    
    pwmWrite(SERVO_PIN, dutyCycle);
}

/**
 * Máquina de estados no bloqueante para control del servo
 */
void actualizarServo(void)
{
    tick_t tiempoActual = tickRead();
    
    switch(estadoActual) {
        
        case ESTADO_0_GRADOS:
            if (tiempoActual - tiempoAnterior >= 2000) {
                printf("-> 0 grados\r\n");
                servoSetAngle(0);
                tiempoAnterior = tiempoActual;
                estadoActual = ESTADO_45_GRADOS;
            }
            break;
            
        case ESTADO_45_GRADOS:
            if (tiempoActual - tiempoAnterior >= 2000) {
                printf("-> 45 grados\r\n");
                servoSetAngle(45);
                tiempoAnterior = tiempoActual;
                estadoActual = ESTADO_90_GRADOS;
            }
            break;
            
        case ESTADO_90_GRADOS:
            if (tiempoActual - tiempoAnterior >= 2000) {
                printf("-> 90 grados\r\n");
                servoSetAngle(90);
                tiempoAnterior = tiempoActual;
                estadoActual = ESTADO_135_GRADOS;
            }
            break;
            
        case ESTADO_135_GRADOS:
            if (tiempoActual - tiempoAnterior >= 2000) {
                printf("-> 135 grados\r\n");
                servoSetAngle(135);
                tiempoAnterior = tiempoActual;
                estadoActual = ESTADO_180_GRADOS;
            }
            break;
            
        case ESTADO_180_GRADOS:
            if (tiempoActual - tiempoAnterior >= 2000) {
                printf("-> 180 grados\r\n");
                servoSetAngle(180);
                tiempoAnterior = tiempoActual;
                anguloBarrido = 0;
                estadoActual = ESTADO_BARRIDO_IDA;
                printf("Barrido 0->180...\r\n");
            }
            break;
            
        case ESTADO_BARRIDO_IDA:
            if (tiempoActual - tiempoAnterior >= 50) {
                servoSetAngle(anguloBarrido);
                anguloBarrido += 5;
                tiempoAnterior = tiempoActual;
                
                if (anguloBarrido > 180) {
                    anguloBarrido = 180;
                    estadoActual = ESTADO_BARRIDO_VUELTA;
                    printf("Barrido 180->0...\r\n");
                }
            }
            break;
            
        case ESTADO_BARRIDO_VUELTA:
            if (tiempoActual - tiempoAnterior >= 50) {
                servoSetAngle(anguloBarrido);
                
                if (anguloBarrido <= 5) {
                    anguloBarrido = 0;
                    estadoActual = ESTADO_PAUSA;
                    tiempoAnterior = tiempoActual;
                    printf("Ciclo completado\r\n\r\n");
                } else {
                    anguloBarrido -= 5;
                }
                
                tiempoAnterior = tiempoActual;
            }
            break;
            
        case ESTADO_PAUSA:
            if (tiempoActual - tiempoAnterior >= 1000) {
                tiempoAnterior = tiempoActual;
                estadoActual = ESTADO_0_GRADOS;
            }
            break;
    }
}

int main(void)
{
    boardConfig();
    uartConfig(UART_USB, 115200);
    tickConfig(1);  // Tick cada 1ms
    
    gpioConfig(LEDR, GPIO_OUTPUT);
    gpioConfig(LEDG, GPIO_OUTPUT);
    
    delay(2000);
    
    printf("\r\n========================================\r\n");
    printf("  Servo S3003 - CONTROL NO BLOQUEANTE\r\n");
    printf("========================================\r\n");
    printf("BOARD: edu_ciaa_nxp\r\n");
    printf("PWM_TOTALS: %d\r\n", PWM_TOTALS);
    printf("Pin: T_COL2\r\n");
    printf("Frecuencia: 50 Hz\r\n");
    printf("========================================\r\n\r\n");
    
    // Intentar inicializar PWM por hardware
    printf("Inicializando PWM en T_COL2...\r\n");
    
    if (pwmConfig(SERVO_PIN, SERVO_FREQ)) {
        printf("? PWM inicializado correctamente\r\n");
        gpioWrite(LEDG, ON);
        
        printf("\r\nIniciando control del servo...\r\n\r\n");
        
        // Posición inicial
        servoSetAngle(90);
        tiempoAnterior = tickRead();
        
        while(TRUE) {
            actualizarServo();
            // Aquí puedes hacer otras tareas sin trabar el servo
        }
        
    } else {
        // Si T_COL2 no soporta PWM hardware, usar software
        printf("? T_COL2 no soporta PWM por hardware\r\n");
        printf("Cambiando a PWM por SOFTWARE...\r\n\r\n");
        gpioWrite(LEDR, ON);
        
        // Configurar como GPIO
        gpioConfig(SERVO_PIN, GPIO_OUTPUT);
        
        printf("Servo funcionando con PWM software en T_COL2\r\n");
        printf("NOTA: PWM software puede tener jitter\r\n\r\n");
        
        // Variables para PWM software no bloqueante
        uint16_t pulseWidth = 1500;
        tick_t tiempoPulso = 0;
        bool_t pulsoActivo = FALSE;
        
        tiempoAnterior = tickRead();
        
        while(TRUE) {
            tick_t ahora = tickRead();
            
            // Generar pulsos PWM
            if (!pulsoActivo && (ahora - tiempoPulso >= 20)) {
                gpioWrite(SERVO_PIN, ON);
                pulsoActivo = TRUE;
                tiempoPulso = ahora;
            }
            
            if (pulsoActivo && (ahora - tiempoPulso >= (pulseWidth/1000))) {
                gpioWrite(SERVO_PIN, OFF);
                pulsoActivo = FALSE;
            }
            
            // Cambiar posiciones cada 2 segundos
            if (ahora - tiempoAnterior >= 2000) {
                static uint8_t posicion = 0;
                
                switch(posicion) {
                    case 0: pulseWidth = 1000; printf("-> 0°\r\n"); break;
                    case 1: pulseWidth = 1500; printf("-> 90°\r\n"); break;
                    case 2: pulseWidth = 2000; printf("-> 180°\r\n"); break;
                }
                
                posicion = (posicion + 1) % 3;
                tiempoAnterior = ahora;
            }
        }
    }
    
    return 0;
}