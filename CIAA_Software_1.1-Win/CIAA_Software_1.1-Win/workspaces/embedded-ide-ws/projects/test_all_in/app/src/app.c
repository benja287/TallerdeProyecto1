
/*
 * Test All In - Integracion de Sensores, Motores y Servo con FreeRTOS
 *
 */

#include "sapi.h"
#include "sapi_uart.h" 
#include "FreeRTOS.h"
#include "task.h"
#include "chip.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ==============================================================================
// DEFINICIONES DE HARDWARE
// ==============================================================================

// --- MOTORES (TB6612FNG) ---
#define PWMA_PIN       PWM2   // T_COL0
#define PWMB_PIN       PWM5   // T_COL1
#define MOTOR_AIN1     GPIO3
#define MOTOR_AIN2     GPIO1
#define MOTOR_BIN1     GPIO7
#define MOTOR_BIN2     GPIO8
#define MOTOR_STBY     GPIO5

#define VEL_MEDIA      150

// --- SERVO (S3003) ---
#define SERVO_PIN      T_COL2
#define SERVO_FREQ     50

// --- SENSORES ULTRASONICOS ---
// Sensor 1
#define TRIG1_PORT     4
#define TRIG1_PIN      8
#define TRIG1_GPIO_P   5
#define TRIG1_GPIO_B   12
#define ECHO1_PORT     4
#define ECHO1_PIN      0
#define ECHO1_GPIO_P   2
#define ECHO1_GPIO_B   0

// Sensor 2
#define TRIG2_PORT     4
#define TRIG2_PIN      6
#define TRIG2_GPIO_P   2
#define TRIG2_GPIO_B   6
#define ECHO2_PORT     4
#define ECHO2_PIN      3
#define ECHO2_GPIO_P   2
#define ECHO2_GPIO_B   3

// Sensor 3
#define TRIG3_PORT     4
#define TRIG3_PIN      5
#define TRIG3_GPIO_P   2
#define TRIG3_GPIO_B   5
#define ECHO3_PORT     4
#define ECHO3_PIN      2
#define ECHO3_GPIO_P   2
#define ECHO3_GPIO_B   2

// ==============================================================================
// VARIABLES GLOBALES (Compartidas)
// ==============================================================================
// Guardamos las distancias para debug
volatile uint32_t dist1_cm = 0;
volatile uint32_t dist2_cm = 0;
volatile uint32_t dist3_cm = 0;

// --- VARIABLES GLOBALES DE CONTROL (UART) ---
volatile int16_t global_joy_x = 0; // -512 a 512
volatile int16_t global_joy_y = 0; // -512 a 512
volatile uint16_t global_throttle = 0; // 0 - 1023
volatile uint16_t global_brake = 0;    // 0 - 1023

// --- MODOS DE OPERACION ---
typedef enum {
    MODE_MANUAL,
    MODE_SAFETY_STOP,
    MODE_AVOIDANCE
} OperationMode;

volatile OperationMode currentMode = MODE_MANUAL;
const char* MODE_NAMES[] = {"MANUAL", "SAFETY STOP", "AVOIDANCE"};

// --- CONSTANTES DE COMANDO Y SEGURIDAD ---
#define BTN_TRIANGLE_MASK 16 // Bit 4 (0x10) - Tipico en Bluepad para Triangulo
#define STOP_DIST_CM      10 // Distancia critica para frenado (Fixed)
#define AVOID_DIST_CM     50 // Distancia para inicio de esquiva
#define AVOID_SPEED_PWM   120 // PWM moderado para maniobra de esquiva



// --- RING BUFFER UART ---
#define RX_BUFFER_SIZE 512
volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;

static TaskHandle_t xUARTTaskHandle = NULL;

// ==============================================================================
// FUNCIONES AUXILIARES
// ==============================================================================

#include "sapi_cyclesCounter.h"

// ... existing includes ...

// ==============================================================================
// FUNCIONES AUXILIARES
// ==============================================================================

// ==============================================================================
// FUNCIONES AUXILIARES
// ==============================================================================

// Manejadores de Tareas para notificaciones
static TaskHandle_t xSensorTaskHandle = NULL;

// Variables volatiles para ISR
volatile uint32_t start_cycles_s1 = 0;
volatile uint32_t start_cycles_s2 = 0;
volatile uint32_t start_cycles_s3 = 0;

// Configuración de bajo nivel para sensores
void config_sensor_pins(void) {
    // Inicializar contador de ciclos para medicion precisa
    cyclesCounterConfig(EDU_CIAA_NXP_CLOCK_SPEED);

    // --- S1 Config (Echo: P4_0 -> GPIO2[0]) ---
    Chip_SCU_PinMuxSet(TRIG1_PORT, TRIG1_PIN, (SCU_MODE_INACT | SCU_MODE_FUNC4));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG1_GPIO_P, TRIG1_GPIO_B);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG1_GPIO_P, TRIG1_GPIO_B, false);

    // Echo 1: P4_0 -> Pin Int 0
    Chip_SCU_PinMuxSet(ECHO1_PORT, ECHO1_PIN, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO1_GPIO_P, ECHO1_GPIO_B);
    
    Chip_SCU_GPIOIntPinSel(0, ECHO1_GPIO_P, ECHO1_GPIO_B); // Channel 0
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(0)); // Edge sensitive
    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(0));  // Rising
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(0));   // Falling
    NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
    NVIC_EnableIRQ(PIN_INT0_IRQn);


    // --- S2 Config (Echo: P4_3 -> GPIO2[3]) ---
    Chip_SCU_PinMuxSet(TRIG2_PORT, TRIG2_PIN, (SCU_MODE_INACT | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG2_GPIO_P, TRIG2_GPIO_B);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG2_GPIO_P, TRIG2_GPIO_B, false);

    // Echo 2: P4_3 -> Pin Int 1
    Chip_SCU_PinMuxSet(ECHO2_PORT, ECHO2_PIN, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO2_GPIO_P, ECHO2_GPIO_B);
    
    Chip_SCU_GPIOIntPinSel(1, ECHO2_GPIO_P, ECHO2_GPIO_B); // Channel 1
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(1));
    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(1));
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(1));
    NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
    NVIC_EnableIRQ(PIN_INT1_IRQn);

    // --- S3 Config (Echo: P4_2 -> GPIO2[2]) ---
    Chip_SCU_PinMuxSet(TRIG3_PORT, TRIG3_PIN, (SCU_MODE_INACT | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG3_GPIO_P, TRIG3_GPIO_B);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG3_GPIO_P, TRIG3_GPIO_B, false);

    // Echo 3: P4_2 -> Pin Int 2
    Chip_SCU_PinMuxSet(ECHO3_PORT, ECHO3_PIN, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO3_GPIO_P, ECHO3_GPIO_B);

    Chip_SCU_GPIOIntPinSel(2, ECHO3_GPIO_P, ECHO3_GPIO_B); // Channel 2
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
    Chip_PININT_SetPinModeEdge(LPC_GPIO_PIN_INT, PININTCH(2));
    Chip_PININT_EnableIntHigh(LPC_GPIO_PIN_INT, PININTCH(2));
    Chip_PININT_EnableIntLow(LPC_GPIO_PIN_INT, PININTCH(2));
    NVIC_ClearPendingIRQ(PIN_INT2_IRQn);
    NVIC_EnableIRQ(PIN_INT2_IRQn);
}

// ... trigger_sensor ... (sin cambios)
void trigger_sensor(uint8_t port, uint8_t pin) {
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, port, pin, false);
    delayInaccurateUs(2);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, port, pin, true);
    delayInaccurateUs(10);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, port, pin, false);
}

// ISRs para captura de ECOS
void GPIO0_IRQHandler(void) {
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(0));
    // Check Value para saber si es Rise o Fall
    if (Chip_GPIO_GetPinState(LPC_GPIO_PORT, ECHO1_GPIO_P, ECHO1_GPIO_B)) {
        // Rise
        start_cycles_s1 = cyclesCounterRead();
    } else {
        // Fall
        uint32_t end = cyclesCounterRead();
        uint32_t diff = end - start_cycles_s1;
        float us = cyclesCounterToUs(diff);
        dist1_cm = (uint32_t)(us / 58.0);
        
        // Notificar tarea
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void GPIO1_IRQHandler(void) {
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(1));
    if (Chip_GPIO_GetPinState(LPC_GPIO_PORT, ECHO2_GPIO_P, ECHO2_GPIO_B)) {
        start_cycles_s2 = cyclesCounterRead();
    } else {
        uint32_t end = cyclesCounterRead();
        uint32_t diff = end - start_cycles_s2;
        float us = cyclesCounterToUs(diff);
        dist2_cm = (uint32_t)(us / 58.0);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void GPIO2_IRQHandler(void) {
    Chip_PININT_ClearIntStatus(LPC_GPIO_PIN_INT, PININTCH(2));
    if (Chip_GPIO_GetPinState(LPC_GPIO_PORT, ECHO3_GPIO_P, ECHO3_GPIO_B)) {
        start_cycles_s3 = cyclesCounterRead();
    } else {
        uint32_t end = cyclesCounterRead();
        uint32_t diff = end - start_cycles_s3;
        float us = cyclesCounterToUs(diff);
        dist3_cm = (uint32_t)(us / 58.0);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// Header de Timer para PWM por Software/Interrupcion
#include "sapi_timer.h"

// ... existing includes ...

// ==============================================================================
// FUNCIONES AUXILIARES
// ==============================================================================

// ... existing config_sensor_pins ...
// ... existing trigger_sensor ...
// REMOVIDO: get_distance (reemplazado por ISR)

// --- CONTROL SERVO (TIMER0) ---
// ... (Codigo servomotor sin cambios) ...

// ==============================================================================
// TAREAS FREERTOS
// ==============================================================================

// --- CONTROL SERVO (TIMER0) ---
#define SERVO_PERIODO_US       20000  // 20ms = 50 Hz
#define SERVO_PULSO_MIN_US     500    // 0.5ms = 0 grados
#define SERVO_PULSO_MAX_US     2000   // 2.0ms = 180 grados

static uint8_t servoAngle = 90;

static uint32_t angleToPulseWidth(uint8_t angle) {
    if (angle > 180) angle = 180;
    // Mapeo lineal simple
    return SERVO_PULSO_MIN_US + ((angle * (SERVO_PULSO_MAX_US - SERVO_PULSO_MIN_US)) / 180);
}

// ISR Inicio de Ciclo (Pone en 1)
static void timer0CompareMatch0Callback(void* ptr) {
    gpioWrite(SERVO_PIN, ON);
    // Actualizar el tiempo de apagado para el ciclo actual (duty cycle dinamico)
    Timer_SetCompareMatch(TIMER0, TIMERCOMPAREMATCH1, 
                         Timer_microsecondsToTicks(angleToPulseWidth(servoAngle)));
}

// ISR Fin de Pulso (Pone en 0)
static void timer0CompareMatch1Callback(void* ptr) {
    gpioWrite(SERVO_PIN, OFF);
}

// Funcion publica para tareas
void set_servo_angle(uint8_t angle) {
    if (angle > 180) angle = 180;
    servoAngle = angle;
    // El cambio tomara efecto en el siguiente ciclo de interrupcion (max 20ms)
}

// Tarea de Estado (Luces)
void TaskStatus(void* taskParm) {
    // Configurar LEDs de estado
    gpioConfig(LED1, GPIO_OUTPUT);
    gpioConfig(LED2, GPIO_OUTPUT);
    gpioConfig(LED3, GPIO_OUTPUT);

    while(1) {
        // Actualizar LEDs segun Modo
        switch(currentMode) {
            case MODE_MANUAL:
                gpioWrite(LED1, ON);
                gpioWrite(LED2, OFF);
                gpioWrite(LED3, OFF);
                break;
            case MODE_SAFETY_STOP:
                gpioWrite(LED1, OFF);
                gpioWrite(LED2, ON); 
                gpioWrite(LED3, OFF);
                break;
            case MODE_AVOIDANCE:
                gpioWrite(LED1, OFF);
                gpioWrite(LED2, OFF);
                gpioWrite(LED3, ON);
                break;
        }
        
        // Blink del LEDB como health check
        gpioToggle(LEDB);
        
        vTaskDelay(200 / portTICK_RATE_MS); // 5Hz update
    }
}


void TaskSensors(void* taskParm) {
    config_sensor_pins();
    
    // Guardar handle para notificaciones
    xSensorTaskHandle = xTaskGetCurrentTaskHandle();

    while(1) {
        // Sensor 1
        portENTER_CRITICAL();
        trigger_sensor(TRIG1_GPIO_P, TRIG1_GPIO_B);
        portEXIT_CRITICAL();
        // Esperar max 30ms (eco maximo seguro + margen)
        ulTaskNotifyTake(pdTRUE, 30 / portTICK_RATE_MS);
        
        vTaskDelay(15 / portTICK_RATE_MS); // Pequeño delay entre sensores

        // Sensor 2
        portENTER_CRITICAL();
        trigger_sensor(TRIG2_GPIO_P, TRIG2_GPIO_B);
        portEXIT_CRITICAL();
        ulTaskNotifyTake(pdTRUE, 35 / portTICK_RATE_MS);
        
        vTaskDelay(15 / portTICK_RATE_MS);

        // Sensor 3
        portENTER_CRITICAL();
        trigger_sensor(TRIG3_GPIO_P, TRIG3_GPIO_B);
        portEXIT_CRITICAL();
        ulTaskNotifyTake(pdTRUE, 35 / portTICK_RATE_MS);

        // Loguear
        printf("S1: %dcm, S2: %dcm, S3: %dcm\r\n", dist1_cm, dist2_cm, dist3_cm);
        
        // Retardo para la siguiente vuelta - Aumentado a 50Hz aprox para mayor velocidad
        vTaskDelay(50 / portTICK_RATE_MS);
    }
}

// ==============================================================================
// TAREAS UART / DECODER
// ==============================================================================

// Callback de Interrupcion UART (Se ejecuta en contexto de ISR)
void onRx(void *noData) {
    uint8_t byte;
    // Leer byte mientras haya disponibles 
    while (uartReadByte(UART_232, &byte)) {
        // Guardar en buffer circular
        uint16_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
        if (nextHead != rxTail) { 
            rxBuffer[rxHead] = byte;
            rxHead = nextHead;
        }
        
        // Si detectamos fin de linea, notificar a la tarea de debodificacion
        if (byte == '\n') {
            // CRITICAL: Verificar que la tarea ha sido creada
            if (xUARTTaskHandle != NULL) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

// Tarea para decodificar el protocolo CSV
void TaskUARTDecode(void* taskParm) {
    xUARTTaskHandle = xTaskGetCurrentTaskHandle();
    
    char lineBuffer[128];
    uint8_t lineIdx = 0;
    
    while(1) {
        // Esperar notificacion de la ISR con TIMEOUT
        // Si no llegan datos en 200ms, reseteamos las variables (Soft Timeout)
        if (ulTaskNotifyTake(pdTRUE, 200 / portTICK_RATE_MS) == 0) {
            // Timeout: Resetear valores a reposo
            global_joy_x = 0;
            global_joy_y = 0;
            global_throttle = 0;
            global_brake = 0;
            continue; // Volver a esperar
        }
        
        // Procesar todo lo que haya en el Ring Buffer
        while (rxTail != rxHead) {
            uint8_t byte = rxBuffer[rxTail];
            rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
            
            if (byte == '$') {
                // Inicio de trama, resetear buffer lineal
                lineIdx = 0;
            } else if (byte == '\n') {
                // Fin de trama, procesar
                lineBuffer[lineIdx] = '\0';
                
                // Variables temporales para sscanf
                int idx, dpad, btns, lx, ly, rx, ry, brk, thr, misc, gx, gy, gz, ax, ay, az;
                
                int count = sscanf(lineBuffer, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                                   &idx, &dpad, &btns, &lx, &ly, &rx, &ry, &brk, &thr, &misc, 
                                   &gx, &gy, &gz, &ax, &ay, &az);
                                   
                if (count >= 10) { // Necesitamos hasta brk/thr
                    // Actualizar variables globales de control
                    global_joy_x = lx;
                    global_joy_y = ly;
                    global_throttle = thr;
                    global_brake = brk;

                    // --- DETECCION DE CAMBIO DE MODO (Triangulo) ---
                    static uint16_t last_btns = 0;
                    // Detectar flanco ascendente (Rising Edge)
                    if ((btns & BTN_TRIANGLE_MASK) && !(last_btns & BTN_TRIANGLE_MASK)) {
                        if (currentMode == MODE_MANUAL) currentMode = MODE_SAFETY_STOP;
                        else if (currentMode == MODE_SAFETY_STOP) currentMode = MODE_AVOIDANCE;
                        else currentMode = MODE_MANUAL;
                        
                        printf(">> CAMBIO MODO: %s (Btns: %d)\r\n", MODE_NAMES[currentMode], btns);
                    }
                    last_btns = btns;

                    // Debug intermitente (cada 20 updates, aprox 1 seg)
                    static int debug_cnt = 0;

                    if (debug_cnt++ > 20) {
                         debug_cnt = 0;
                         printf("Parsed: thr=%d, brk=%d\r\n", thr, brk);
                    }
                } else {
                     printf("Parse Err: count=%d. Line: %s\r\n", count, lineBuffer);
                }
                
                lineIdx = 0; // Reset para seguridad
            } else {
                // Caracter normal, agregar al buffer
                if (lineIdx < sizeof(lineBuffer) - 1) {
                    lineBuffer[lineIdx++] = byte;
                }
            }
        }
    }
}

void TaskMotors(void* taskParm) {
    // Config Motor
    gpioConfig(MOTOR_AIN1, GPIO_OUTPUT);
    gpioConfig(MOTOR_AIN2, GPIO_OUTPUT);
    gpioConfig(MOTOR_BIN1, GPIO_OUTPUT);
    gpioConfig(MOTOR_BIN2, GPIO_OUTPUT);
    gpioConfig(MOTOR_STBY, GPIO_OUTPUT);
    
    // PWM Config
    // Inicializar PWM hardware
    // Nota: La config de frecuencia se hace en main
    
#define RAMP_STEP 25 // Paso de rampa (25 * 50Hz = 1250 units/s => 255/1250 = ~0.2s full scale)

    gpioWrite(MOTOR_STBY, ON); // Activar driver
    
    // Estado interno para rampa (Signed: +Adelante, -Atras)
    static int16_t current_pwm_signed = 0;

    while(1) {
        // --- CALCULO OBJETIVO ---
        uint16_t thr = global_throttle;
        uint16_t brk = global_brake;
        int16_t target_pwm_signed = 0;
        
        // Zona muerta de 50 (Rango 0-1023)
        if (thr > 50 && brk < 50) {
            // ADELANTE (Objetivo Positivo)
            // Map 50..1023 -> 80..255 
            uint32_t val = 80 + ((thr - 50) * (255 - 80) / (1023 - 50));
            target_pwm_signed = (int16_t)val;
            
        } else if (brk > 50 && thr < 50) {
            // REVERSA (Objetivo Negativo)
            uint32_t val = 80 + ((brk - 50) * (255 - 80) / (1023 - 50));
            target_pwm_signed = -(int16_t)val;
        } else {
            // STOP
            target_pwm_signed = 0;
        }

        // --- OVERRIDES DE SEGURIDAD (MODOS) ---
        // Tratamiento de distancias (filtrar 0s/errores)
        uint32_t d1 = (dist1_cm == 0) ? 999 : dist1_cm; // S1 (Izq?)
        uint32_t d2 = (dist2_cm == 0) ? 999 : dist2_cm; // S2 (Centro?)
        uint32_t d3 = (dist3_cm == 0) ? 999 : dist3_cm; // S3 (Der?)
        
        uint32_t min_dist = d1;
        if (d2 < min_dist) min_dist = d2;
        if (d3 < min_dist) min_dist = d3;

        if (currentMode == MODE_SAFETY_STOP) {
            // -- UMBRAL FIJO (5cm) --
            // Si hay obstaculo cerca Y queremos ir ADELANTE (>0)
            if (min_dist < STOP_DIST_CM && target_pwm_signed > 0) {
                target_pwm_signed = 0; // Frenar
                current_pwm_signed = 0; // Stop inmediato
            }
        
        } else if (currentMode == MODE_AVOIDANCE) {
            // Si hay obstaculo, limitar velocidad para maniobrar
            if (min_dist < AVOID_DIST_CM && target_pwm_signed != 0) {
                // Si ibamos rapido, bajamos la velocidad
                int16_t sign = (target_pwm_signed > 0) ? 1 : -1;
                // Si el target es mayor que la velocidad de maniobra, lo topeamos
                if (abs(target_pwm_signed) > AVOID_SPEED_PWM) {
                    target_pwm_signed = sign * AVOID_SPEED_PWM;
                }
            }
        }
        
        // --- APLICAR RAMPA (SOFT START) ---

        if (current_pwm_signed < target_pwm_signed) {
            current_pwm_signed += RAMP_STEP;
            if (current_pwm_signed > target_pwm_signed) current_pwm_signed = target_pwm_signed;
        } else if (current_pwm_signed > target_pwm_signed) {
            current_pwm_signed -= RAMP_STEP;
            if (current_pwm_signed < target_pwm_signed) current_pwm_signed = target_pwm_signed;
        }
        
        // --- ACTUAR HARDWARE ---
        if (current_pwm_signed > 0) {
            // Adelante
            gpioWrite(MOTOR_AIN1, ON); gpioWrite(MOTOR_AIN2, OFF);
            gpioWrite(MOTOR_BIN1, ON); gpioWrite(MOTOR_BIN2, OFF);
            pwmWrite(PWMA_PIN, (uint8_t)current_pwm_signed);
            pwmWrite(PWMB_PIN, (uint8_t)current_pwm_signed);
            gpioWrite(LEDG, ON); gpioWrite(LEDR, OFF);
            
        } else if (current_pwm_signed < 0) {
            // Reversa
            gpioWrite(MOTOR_AIN1, OFF); gpioWrite(MOTOR_AIN2, ON);
            gpioWrite(MOTOR_BIN1, OFF); gpioWrite(MOTOR_BIN2, ON);
            // Invertir para PWM absoluto
            pwmWrite(PWMA_PIN, (uint8_t)(-current_pwm_signed));
            pwmWrite(PWMB_PIN, (uint8_t)(-current_pwm_signed));
            gpioWrite(LEDR, ON); gpioWrite(LEDG, OFF);
            
        } else {
            // Stop
            pwmWrite(PWMA_PIN, 0); pwmWrite(PWMB_PIN, 0);
            gpioWrite(LEDG, OFF); gpioWrite(LEDR, OFF);
        }
        
        static int m_debug = 0;
        if (m_debug++ > 50) { // 1 seg
            m_debug = 0;
            printf("Motors: Tgt=%d, Curr=%d\r\n", target_pwm_signed, current_pwm_signed);
        }
        
        vTaskDelay(20 / portTICK_RATE_MS); // 50Hz control loop
    }
}
// --- SERVO LIMITS ---
#define SERVO_CENTER   150
#define SERVO_MIN      122 // 150 - 30
#define SERVO_MAX      176 // 150 + 30 (Limite fisico software)

void TaskServo(void* taskParm) {
    // Restauramos barrido
    printf("Servo Barrido Centrado en 150 (120-180)...\r\n");
    set_servo_angle(SERVO_CENTER);
    vTaskDelay(2000 / portTICK_RATE_MS);

    uint8_t angle = SERVO_CENTER;
    uint8_t movingLeft = 0; 
    uint8_t step = 1;

    uint8_t min_angle = SERVO_MIN;
    uint8_t max_angle = SERVO_MAX;

    while(1) {
        // Leer Joystick X (-512 a 511)
        int16_t joyX = global_joy_x;
        
        // Mapear -512..512 a 120..180 (o rango seguro)
        // Centro = 150
        // Rango +/- 30 grados
        
        // Calculo: 150 + (joyX * 30 / 512)
        int16_t targetAngle = 150 + (joyX * 30 / 512);
        
        // Limitar por seguridad
        if (targetAngle < SERVO_MIN) targetAngle = SERVO_MIN;
        if (targetAngle > SERVO_MAX) targetAngle = SERVO_MAX;
        
        // --- OVERRIDES DE SEGURIDAD (MODO AVOIDANCE) ---
        if (currentMode == MODE_AVOIDANCE) {
            uint32_t d1 = (dist1_cm == 0) ? 999 : dist1_cm; 
            uint32_t d2 = (dist2_cm == 0) ? 999 : dist2_cm;
            uint32_t d3 = (dist3_cm == 0) ? 999 : dist3_cm;
            uint32_t min_dist = d1;
            if (d2 < min_dist) min_dist = d2;
            if (d3 < min_dist) min_dist = d3;

            if (min_dist < AVOID_DIST_CM) {
                // Volantazo hacia el lado con mas despeje
                // Asumimos: S1=Izquierda, S2=Centro, S3=Derecha
                // Logica simple: Comparar Izq (d1) vs Der (d3)
                // Si d1 (Izq) > d3 (Der) -> Girar Izquierda (Angulo MENOR a 150? Depende servo)
                
                // NOTA: En servos RC standard:
                // 150 = Centro
                // 120 = Izquierda (Min) -> Si el brazo apunta izq con pulso corto
                // 180 = Derecha (Max) -> Si el brazo apunta der con pulso largo
                // *Verificar orientacion fisica*. Asumiremos esta convencion.
                
                if (d1 > d3) {
                    // Mas lugar a la izquierda -> Girar Izquierda (MIN)
                    targetAngle = SERVO_MIN; 
                } else {
                    // Mas lugar a la derecha (o igual) -> Girar Derecha (MAX)
                    targetAngle = SERVO_MAX;
                }
            }
        }

        
        set_servo_angle((uint8_t)targetAngle);
        
        vTaskDelay(20 / portTICK_RATE_MS); 
    }
}


// ==============================================================================
// MAIN
// ==============================================================================
int main(void) {
    boardConfig();
    
    // Configurar Tick a 1ms (si no lo hace FreeRTOS ya... cuidado con sapi_tick)
    // sapi_tick se inicializa en boardConfig normalmete.
    // Pero FreeRTOS usa su propio tick. sapi_timer usa TIMER periferico hardware,
    // asi que no depende del Systick del OS.

    // --- CONFIGURACION MOTORES (PWM HARDWARE ORIGINAL) ---
    // Usamos SCT PWM para los motores (PWMA_PIN, PWMB_PIN)
    // El servo YA NO USA ESTO.
    // Inicializar el periferico PWM a 1000Hz (default) para los motores.
    pwmInit(0, PWM_ENABLE); 
    pwmInit(PWMA_PIN, PWM_ENABLE_OUTPUT);
    pwmInit(PWMB_PIN, PWM_ENABLE_OUTPUT);
    
    // --- CONFIGURACION SERVO (TIMER0 INTERRUPT) ---
    gpioConfig(SERVO_PIN, GPIO_OUTPUT);
    
    // Inicializar TIMER0
    // Comparacion 0: Inicio del periodo (20ms) -> Callback0 (Pone en 1)
    Timer_Init(TIMER0, 
               Timer_microsecondsToTicks(SERVO_PERIODO_US),
               timer0CompareMatch0Callback);
    
    // Comparacion 1: Fin del pulso (Variable) -> Callback1 (Pone en 0)
    Timer_EnableCompareMatch(TIMER0, 
                            TIMERCOMPAREMATCH1,
                            Timer_microsecondsToTicks(angleToPulseWidth(servoAngle)),
                            timer0CompareMatch1Callback);

    // --- CONFIGURACION UART 232 (ESP32) ---
    // Inicializar UART_232 a 115200 baudios
    uartConfig(UART_232, 115200);
    // Configurar Callback de Recepcion
    uartCallbackSet(UART_232, UART_RECEIVE, onRx, NULL);
    // Habilitar Interrupciones para esta UART
    uartInterrupt(UART_232, true);

    printf("Sistema iniciado: Motores PWM Hardware + Servo Timer0 IRQ + UART232 ESP32\r\n");
    
    // Crear tareas
    // Crear tareas
    xTaskCreate(TaskStatus,  "Status",  128, NULL, tskIDLE_PRIORITY + 1, NULL);

    xTaskCreate(TaskSensors, "Sensors", 512, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(TaskUARTDecode, "Decoder", 512, NULL, tskIDLE_PRIORITY + 3, NULL); // Prioridad alta para vaciar buffer
    xTaskCreate(TaskMotors,  "Motors",  256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskServo,   "Servo",   256, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Iniciar Scheduler
    vTaskStartScheduler();

    while(1); 
    return 0;
}
