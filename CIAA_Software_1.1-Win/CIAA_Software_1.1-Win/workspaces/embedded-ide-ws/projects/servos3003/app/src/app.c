/**
 * Servo S3003 en T_COL2 con TIMER HARDWARE
 * Control por hardware usando TIMER0 - NO BLOQUEANTE
 */

#define BOARD edu_ciaa_nxp
#include "sapi.h"
#include "sapi_timer.h"
#define SERVO_PIN T_COL2

// Parámetros del servo (estándar para servos tipo S3003)
#define SERVO_PERIODO_US       20000  // 20ms = 50 Hz
#define SERVO_PULSO_MIN_US     500    // 0.5ms = 0 grados
#define SERVO_PULSO_MAX_US     2000   // 2.0ms = 180 grados

// Variable global para el ángulo actual del servo
static uint8_t servoAngle = 90;  // Ángulo inicial: 90 grados (centro)

/**
 * Convierte ángulo (0-180) a ancho de pulso en microsegundos
 */
static uint32_t angleToPulseWidth(uint8_t angle)
{
    if (angle > 180) angle = 180;
    return SERVO_PULSO_MIN_US + ((angle * (SERVO_PULSO_MAX_US - SERVO_PULSO_MIN_US)) / 180);
}

/**
 * Callback del timer: inicio del período (pulso en ALTO)
 * Se ejecuta cada 20ms (50 Hz)
 */
static void timer0CompareMatch0Callback(void* ptr)
{
    // Iniciar pulso: poner pin en ALTO
    gpioWrite(SERVO_PIN, ON);
    
    // Configurar compare match 1 para terminar el pulso
    uint32_t pulseWidth = angleToPulseWidth(servoAngle);
    Timer_SetCompareMatch(TIMER0, TIMERCOMPAREMATCH1, 
                         Timer_microsecondsToTicks(pulseWidth));
}

/**
 * Callback del timer: fin del pulso (pulso en BAJO)
 * Se ejecuta después del ancho de pulso calculado
 */
static void timer0CompareMatch1Callback(void* ptr)
{
    // Terminar pulso: poner pin en BAJO
    gpioWrite(SERVO_PIN, OFF);
}

/**
 * Configurar el ángulo del servo (0-180 grados)
 * Esta función puede llamarse desde cualquier parte del código
 */
void servoSetAngle(uint8_t angle)
{
    if (angle > 180) angle = 180;
    servoAngle = angle;
    // El timer hardware se encargará automáticamente de actualizar el pulso
}

int main(void)
{
    boardConfig();
    uartConfig(UART_USB, 115200);
    tickConfig(1);  // Tick cada 1ms para control no bloqueante
    
    gpioConfig(LEDR, GPIO_OUTPUT);
    gpioConfig(LEDG, GPIO_OUTPUT);
    gpioConfig(SERVO_PIN, GPIO_OUTPUT);
    
    delay(1000);
    
    printf("\r\n========================================\r\n");
    printf("  Servo S3003 - TIMER HARDWARE\r\n");
    printf("========================================\r\n");
    printf("Pin: T_COL2\r\n");
    printf("Timer: TIMER0 (Hardware)\r\n");
    printf("Frecuencia: 50 Hz (20ms periodo)\r\n");
    printf("Pulso: 500-2000 us (0-180 grados)\r\n");
    printf("========================================\r\n\r\n");
    
    // Inicializar TIMER0 para generar pulsos de servo
    // Período: 20ms (50 Hz)
    Timer_Init(TIMER0, 
               Timer_microsecondsToTicks(SERVO_PERIODO_US),
               timer0CompareMatch0Callback);
    
    // Habilitar compare match 1 para controlar el ancho del pulso
    Timer_EnableCompareMatch(TIMER0, 
                            TIMERCOMPAREMATCH1,
                            Timer_microsecondsToTicks(angleToPulseWidth(servoAngle)),
                            timer0CompareMatch1Callback);
    
    printf("Timer hardware inicializado correctamente\r\n");
    gpioWrite(LEDG, ON);
    
    printf("\r\nIniciando test del servo...\r\n");
    printf("Movimiento: 0° -> 180° -> 0°\r\n\r\n");
    
    // Control no bloqueante para cambiar posiciones
    tick_t tiempoAnterior = tickRead();
    uint8_t estado = 0;  // 0: 0 grados, 1: 180 grados
    
    while(TRUE) {
        tick_t tiempoActual = tickRead();
        
        // Cambiar posición cada 2 segundos
        if (tiempoActual - tiempoAnterior >= 2000) {
            if (estado == 0) {
                servoSetAngle(0);
                printf("-> 0 grados\r\n");
                estado = 1;
            } else {
                servoSetAngle(180);
                printf("-> 180 grados\r\n");
                estado = 0;
            }
            tiempoAnterior = tiempoActual;
        }
        
        // Aquí puedes hacer otras tareas sin bloquear el servo
        // El timer hardware se encarga de generar los pulsos automáticamente
    }
    
    return 0;
}
