/**
 * Sensor Ultrasónico HC-SR04 - Control NO BLOQUEANTE con HARDWARE
 * 
 * Implementación usando:
 * - TIMER1: Genera pulso TRIG automáticamente (hardware)
 * - Interrupciones GPIO (PININT): Captura flancos ECHO (hardware)
 * - Timer Capture (TIMER0): Mide ancho de pulso con precisión hardware
 * 
 * Pines:
 * - TRIG3: LCD2 (P4.5) = GPIO2[5]
 * - ECHO3: T_FIL2 (P4.2) = GPIO2[2]
 */

#define BOARD edu_ciaa_nxp
#include "sapi.h"
#include "sapi_timer.h"
#include "chip.h"

// Pines del sensor
#define TRIG3_PIN      LCD2    // P4.5 = GPIO2[5]
#define ECHO3_PIN      T_FIL2  // P4.2 = GPIO2[2]

// Configuración GPIO para interrupciones (LPCOpen)
#define ECHO3_PIN_PORT    4
#define ECHO3_PIN_NUM    2
#define ECHO3_GPIO_PORT   2
#define ECHO3_GPIO_PIN    2
#define ECHO3_IRQ_CHANNEL 0    // Canal de interrupción GPIO (PININT0)

// Parámetros del sensor
#define SENSOR_SAMPLING_RATE_US    60000  // 60ms entre mediciones
#define SENSOR_TRIGGER_PULSE_WIDTH_US  10  // 10µs pulso TRIG

// Factores de conversión
#define TICKS_TO_US_FACTOR    (204000000 / 1000000)  // 204 MHz / 1MHz
#define US_TO_CM_FACTOR       58  // Fórmula: distancia = tiempo_us / 58

// Estados del sensor
typedef enum {
    SENSOR_IDLE,
    SENSOR_WAITING_ECHO,
    SENSOR_MEASURING,
    SENSOR_READY
} sensorState_t;

// Estructura de datos del sensor
static struct {
    sensorState_t state;
    uint32_t echoRiseTime;      // Ticks cuando ECHO sube
    uint32_t echoFallTime;      // Ticks cuando ECHO baja
    uint32_t lastEchoWidth;     // Ancho del pulso en ticks
    float lastDistance;          // Última distancia medida en cm
    bool_t newMeasurement;      // Flag de nueva medición disponible
} sensorData = {
    .state = SENSOR_IDLE,
    .echoRiseTime = 0,
    .echoFallTime = 0,
    .lastEchoWidth = 0,
    .lastDistance = 0.0,
    .newMeasurement = FALSE
};

/**
 * Callback dummy para TIMER0 (solo para inicializarlo como contador libre)
 */
static void timer0DummyCallback(void* ptr)
{
    /* No hacer nada - TIMER0 solo se usa para leer el contador */
}

/**
 * Callback del timer: inicio del período (disparar TRIG)
 * Se ejecuta cada 60ms automáticamente
 */
static void timer1CompareMatch0Callback(void* ptr)
{
    // Iniciar pulso TRIG: poner pin en ALTO
    gpioWrite(TRIG3_PIN, ON);
    
    // Cambiar estado
    sensorData.state = SENSOR_WAITING_ECHO;
    sensorData.newMeasurement = FALSE;
    
    // Configurar compare match 1 para terminar el pulso TRIG (10µs)
    Timer_SetCompareMatch(TIMER1, TIMERCOMPAREMATCH1, 
                        Timer_microsecondsToTicks(SENSOR_TRIGGER_PULSE_WIDTH_US));
}

/**
 * Callback del timer: fin del pulso TRIG
 * Se ejecuta después de 10µs
 */
static void timer1CompareMatch1Callback(void* ptr)
{
    // Terminar pulso TRIG: poner pin en BAJO
    gpioWrite(TRIG3_PIN, OFF);
}

/**
 * Handler de interrupción GPIO para ECHO
 * Captura los flancos de subida y bajada del pulso ECHO
 * NOTA: Este handler sobrescribe el de la librería sapi_ultrasonic_hcsr04
 * El linker usará este handler porque está definido en nuestro código
 */
void GPIO0_IRQHandler(void)
{
    // Verificar si la interrupción es del canal 0 (nuestro ECHO)
    // Flanco de SUBIDA: inicio del pulso ECHO
    if (Chip_PININT_GetRiseStates(LPC_GPIO_PIN_INT) & PININTCH(ECHO3_IRQ_CHANNEL)) {
        sensorData.echoRiseTime = Chip_TIMER_ReadCount(LPC_TIMER0);
        sensorData.state = SENSOR_MEASURING;
        
        // Limpiar flag de flanco de subida
        Chip_PININT_ClearRiseStates(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
    }
    
    // Flanco de BAJADA: fin del pulso ECHO
    if (Chip_PININT_GetFallStates(LPC_GPIO_PIN_INT) & PININTCH(ECHO3_IRQ_CHANNEL)) {
        sensorData.echoFallTime = Chip_TIMER_ReadCount(LPC_TIMER0);
        
        // Calcular ancho del pulso en ticks
        if (sensorData.echoFallTime > sensorData.echoRiseTime) {
            sensorData.lastEchoWidth = sensorData.echoFallTime - sensorData.echoRiseTime;
            
            // Convertir ticks a microsegundos y luego a centímetros
            float echoTime_us = (float)(sensorData.lastEchoWidth / TICKS_TO_US_FACTOR);
            sensorData.lastDistance = echoTime_us / US_TO_CM_FACTOR;
            
            sensorData.state = SENSOR_READY;
            sensorData.newMeasurement = TRUE;
        }
        
        // Limpiar flag de flanco de bajada
        Chip_PININT_ClearFallStates(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
    }
    
    // Limpiar estado de interrupción
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
}

/**
 * Configurar interrupciones GPIO para el pin ECHO
 */
static void configEchoInterrupt(void)
{
    // Inicializar sistema de interrupciones GPIO (PININT)
    Chip_PININT_Init(LPC_GPIO_PIN_INT);
    
    // Seleccionar pin para interrupción (Port 2, Pin 2 = GPIO2[2] = T_FIL2)
    Chip_SCU_GPIOIntPinSel(ECHO3_IRQ_CHANNEL, ECHO3_GPIO_PORT, ECHO3_GPIO_PIN);
    
    // Limpiar estado de interrupción
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
    
    // Configurar modo de detección por flancos
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
    
    // Habilitar detección de flancos de subida y bajada
    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(ECHO3_IRQ_CHANNEL));
    
    // Limpiar interrupciones pendientes
    NVIC_ClearPendingIRQ(PIN_INT0_IRQn + ECHO3_IRQ_CHANNEL);
    
    // Habilitar interrupción en NVIC
    NVIC_EnableIRQ(PIN_INT0_IRQn + ECHO3_IRQ_CHANNEL);
}

/**
 * Inicializar sensor ultrasónico
 */
static void sensorInit(void)
{
    // Configurar pin TRIG como salida
    gpioConfig(TRIG3_PIN, GPIO_OUTPUT);
    gpioWrite(TRIG3_PIN, OFF);
    
    // Configurar pin ECHO como entrada
    gpioConfig(ECHO3_PIN, GPIO_INPUT);
    
    // Configurar interrupciones GPIO para ECHO
    configEchoInterrupt();
    
    // Inicializar TIMER0 como contador libre para Timer Capture
    // Se usa solo para leer el contador, no para generar interrupciones
    // Período muy largo (1 segundo) para que funcione como contador libre
    Timer_Init(TIMER0, 
               Timer_microsecondsToTicks(1000000),  // 1 segundo (solo para inicializar)
               timer0DummyCallback);
    
    // Inicializar TIMER1 para generar pulso TRIG automáticamente
    // Período: 60ms (frecuencia de muestreo)
    Timer_Init(TIMER1, 
               Timer_microsecondsToTicks(SENSOR_SAMPLING_RATE_US),
               timer1CompareMatch0Callback);
    
    // Habilitar compare match 1 para controlar ancho del pulso TRIG (10µs)
    Timer_EnableCompareMatch(TIMER1, 
                            TIMERCOMPAREMATCH1,
                            Timer_microsecondsToTicks(SENSOR_TRIGGER_PULSE_WIDTH_US),
                            timer1CompareMatch1Callback);
    
    printf("Sensor inicializado:\r\n");
    printf("  TRIG: LCD2 (P4.5)\r\n");
    printf("  ECHO: T_FIL2 (P4.2)\r\n");
    printf("  Frecuencia muestreo: 60ms\r\n");
    printf("  Pulso TRIG: 10µs\r\n");
    printf("  Timer: TIMER1 (TRIG), TIMER0 (Capture)\r\n\r\n");
}

/**
 * Obtener última distancia medida (no bloqueante)
 * @return Distancia en cm, o 0.0 si no hay medición válida
 */
float sensorGetDistance(void)
{
    if (sensorData.newMeasurement && sensorData.lastDistance > 0.0) {
        return sensorData.lastDistance;
    }
    return 0.0;
}

/**
 * Verificar si hay nueva medición disponible
 */
bool_t sensorHasNewMeasurement(void)
{
    return sensorData.newMeasurement;
}

int main(void)
{
    boardConfig();
    uartConfig(UART_USB, 115200);
    tickConfig(1);  // Tick cada 1ms para control no bloqueante
    
    // Configurar LEDs
    gpioConfig(LEDR, GPIO_OUTPUT);
    gpioConfig(LEDG, GPIO_OUTPUT);
    gpioConfig(LEDB, GPIO_OUTPUT);
    gpioConfig(LED1, GPIO_OUTPUT);
    gpioConfig(LED2, GPIO_OUTPUT);
    gpioConfig(LED3, GPIO_OUTPUT);
    
    delay(1000);
    
    printf("\r\n========================================\r\n");
    printf("  SENSOR ULTRASONICO HC-SR04\r\n");
    printf("  Control NO BLOQUEANTE con HARDWARE\r\n");
    printf("========================================\r\n\r\n");
    
    // Inicializar sensor (configura hardware automáticamente)
    sensorInit();
    
    printf("Iniciando mediciones automáticas...\r\n\r\n");
    
    // Control no bloqueante para leer y mostrar mediciones
    tick_t tiempoAnterior = tickRead();
    uint32_t medicion_num = 0;
    
    while(TRUE) {
        tick_t tiempoActual = tickRead();
        
        // Verificar nueva medición cada 100ms (más rápido que la frecuencia de muestreo)
        if (tiempoActual - tiempoAnterior >= 100) {
            tiempoAnterior = tiempoActual;
            
            // Leer distancia (no bloqueante)
            float distancia = sensorGetDistance();
            
            if (sensorHasNewMeasurement() && distancia > 0.0) {
                medicion_num++;
                
                // Apagar todos los LEDs primero
                gpioWrite(LEDR, OFF);
                gpioWrite(LEDG, OFF);
                gpioWrite(LEDB, OFF);
                gpioWrite(LED1, OFF);
                gpioWrite(LED2, OFF);
                gpioWrite(LED3, OFF);
                
                printf("[%lu] Distancia: %.1f cm\r\n", medicion_num, distancia);
                
                // Indicación visual con LEDs según distancia
                if (distancia < 50.0) {
                    gpioWrite(LEDB, ON);  // Azul: detectado
                    
                    if (distancia < 30.0) {
                        gpioWrite(LED1, ON);
                    }
                    
                    if (distancia < 20.0) {
                        gpioWrite(LEDB, OFF);
                        gpioWrite(LEDG, ON);  // Verde: cerca
                        gpioWrite(LED2, ON);
                    }
                    
                    if (distancia < 10.0) {
                        gpioWrite(LEDG, OFF);
                        gpioWrite(LEDR, ON);  // Rojo: muy cerca
                        gpioWrite(LED3, ON);
                    }
                    
                    if (distancia < 5.0) {
                        printf("*** ALERTA: Objeto muy cercano! ***\r\n");
                    }
                }
                
                // Marcar medición como leída
                sensorData.newMeasurement = FALSE;
            }
        }
        
        // Aquí puedes hacer otras tareas sin bloquear
        // El sensor funciona completamente en hardware:
        // - TIMER1 genera TRIG automáticamente
        // - Interrupciones GPIO capturan ECHO
        // - TIMER0 mide el ancho del pulso
    }
    
    return 0;
}
