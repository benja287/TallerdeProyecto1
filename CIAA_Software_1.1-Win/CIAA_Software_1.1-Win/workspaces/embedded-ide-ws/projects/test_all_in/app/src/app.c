
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
#include "test_hardware.h" // Modulo de Test

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
#define SERVO_CENTER   150
#define SERVO_MIN      132 // 150 - 30
#define SERVO_MAX      177 // 150 + 30 (Limite fisico software)

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
#define STOP_DIST_CM      20 // Distancia critica para frenado (Safe)
#define AVOID_DIST_CM     50 // Distancia para inicio de esquiva

#define AVOID_SPEED_PWM   120 // PWM moderado para maniobra de esquiva



// --- RING BUFFER UART ---
#define RX_BUFFER_SIZE 512
volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;

static TaskHandle_t xUARTTaskHandle = NULL;

// --- VARIABLES PARA MANIOBRA EVASION ---
typedef enum {
    AVOID_IDLE,      // Control Manual
    AVOID_REVERSING, // Auto: Atras
    AVOID_TURNING,   // Auto: Espera giro servo
    AVOID_FORWARDING // Auto: Adelante (Esquivando)
} AvoidState;

volatile AvoidState avoidState = AVOID_IDLE;
volatile int8_t avoidTurnTarget = 0; // -1: Izq, 1: Der (LEGACY - Replaced by avoidTurnAngle)
volatile uint8_t avoidTurnAngle = 150; // Angulo calculado dinamicamente

// Snapshots de memoria
volatile uint32_t old_d1 = 0, old_d2 = 0, old_d3 = 0;

// --- PROTOCOLO BINARIO ---
#define BIN_HEADER 0xA5
#define BIN_FOOTER 0x5A
#define FB_CMD_RUMBLE  0x01
#define FB_CMD_LED     0x02

struct ControlPacket {
    uint8_t header;    
    int8_t  joyX;      
    int8_t  joyY;      
    uint8_t throttle;  
    uint8_t brake;     
    uint8_t buttons;   
    uint8_t checksum;  
    uint8_t footer;    
} __attribute__((packed));

// --- VARIABLES COMPARTIDAS DE SEGURIDAD ---
volatile TickType_t lastValidTime = 0;

struct FeedbackPacket {
    uint8_t header;    // 0x5B
    uint8_t command;   
    uint8_t val1;
    uint8_t val2;
    uint8_t val3;
    uint8_t footer;    // 0xB5
} __attribute__((packed));

void sendFeedback(uint8_t cmd, uint8_t v1, uint8_t v2, uint8_t v3) {
    struct FeedbackPacket fb;
    fb.header = 0x5B;
    fb.command = cmd;
    fb.val1 = v1; fb.val2 = v2; fb.val3 = v3;
    fb.footer = 0xB5;
    
    // CRITICAL: Proteger el recurso UART compartido para evitar race conditions
    // si dos tareas intentan enviar datos al mismo tiempo.
    portENTER_CRITICAL();
    uartWriteByteArray(UART_232, (uint8_t*)&fb, sizeof(fb));
    portEXIT_CRITICAL();
}

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
TaskHandle_t xSensorTaskHandle = NULL;

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
        if (xSensorTaskHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(xSensorTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
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

// (Protocolo Binario definido arriba globalmente)

// Tarea de Estado (Luces)
void TaskStatus(void* taskParm) {
    // Configurar LEDs RGB
    gpioConfig(LEDR, GPIO_OUTPUT);
    gpioConfig(LEDG, GPIO_OUTPUT);
    gpioConfig(LEDB, GPIO_OUTPUT);
    gpioConfig(LED1, GPIO_OUTPUT); // Heartbeat

    uint8_t lastSentMode = 0xFF;

    while(1) {
        // Actualizar LED RGB CIAA
        switch(currentMode) {
            case MODE_MANUAL:
                gpioWrite(LEDR, ON); gpioWrite(LEDG, OFF); gpioWrite(LEDB, OFF);
                if (lastSentMode != 0) { sendFeedback(FB_CMD_LED, 255, 0, 0); lastSentMode = 0; }
                break;
            case MODE_SAFETY_STOP:
                gpioWrite(LEDR, OFF); gpioWrite(LEDG, ON); gpioWrite(LEDB, OFF);
                if (lastSentMode != 1) { sendFeedback(FB_CMD_LED, 0, 255, 0); lastSentMode = 1; }
                break;
            case MODE_AVOIDANCE:
                gpioWrite(LEDR, OFF); gpioWrite(LEDG, OFF); gpioWrite(LEDB, ON);
                if (lastSentMode != 2) { sendFeedback(FB_CMD_LED, 0, 0, 255); lastSentMode = 2; }
                break;
        }
        
        // Blink lento de LED1 como "Heartbeat" (Sistema vivo)
        static int hb_cnt = 0;
        if(hb_cnt++ > 5) { // Cada 5x200ms = 1s
            hb_cnt = 0;
            gpioToggle(LED1);
        }
        
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
        // Esperar max 12ms (Rango util ~2 metros. 60us*200cm = 12ms)
        // Esto acelera drasticamente el loop cuando no hay paredes cerca.
        ulTaskNotifyTake(pdTRUE, 12 / portTICK_RATE_MS);
        
        vTaskDelay(2 / portTICK_RATE_MS); // Minimo delay inter-sensor

        // Sensor 2
        portENTER_CRITICAL();
        trigger_sensor(TRIG2_GPIO_P, TRIG2_GPIO_B);
        portEXIT_CRITICAL();
        ulTaskNotifyTake(pdTRUE, 12 / portTICK_RATE_MS);
        
        vTaskDelay(2 / portTICK_RATE_MS); 

        // Sensor 3
        portENTER_CRITICAL();
        trigger_sensor(TRIG3_GPIO_P, TRIG3_GPIO_B);
        portEXIT_CRITICAL();
        ulTaskNotifyTake(pdTRUE, 12 / portTICK_RATE_MS);

        // Loguear (Throttled)
        static int log_cnt = 0;
        if (log_cnt++ > 15) { // Cada ~200-300ms
             log_cnt = 0;
             printf("S1: %dcm, S2: %dcm, S3: %dcm\r\n", dist1_cm, dist2_cm, dist3_cm);
        }
        
        // Retardo para la siguiente vuelta - VOLOCIDAD EXTREMA
        vTaskDelay(5 / portTICK_RATE_MS);
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
        if (byte == BIN_FOOTER) { // Notificar al recibir el footer del paquete binario
            // CRITICAL: Verificar que la tarea ha sido creada
            if (xUARTTaskHandle != NULL) {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
        }
    }
}

// Tarea para decodificar el PROTOCOLO BINARIO
void TaskUARTDecode(void* taskParm) {
    xUARTTaskHandle = xTaskGetCurrentTaskHandle();
    
    // Buffer local para armar paquete
    static uint8_t state = 0; // 0:WAIT_HEADER, 1:READ_DATA
    static uint8_t rawBuf[8]; // sizeof(ControlPacket)
    static uint8_t idx = 0;
    // static TickType_t lastValidTime = 0; // Ahora es global para TaskMotors

    while(1) {
        // Esperar notificacion de datos
        if (ulTaskNotifyTake(pdTRUE, 200 / portTICK_RATE_MS) == 0) {
            // Timeout - Reset
            global_joy_x = 0; global_joy_y = 0; global_throttle = 0; global_brake = 0;
            continue;
        }
        
        while (rxTail != rxHead) {
            uint8_t byte = rxBuffer[rxTail];
            rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
            
            // --- SAFETY FIX: TIME-BASED RESET ---
            // Si pasaron mas de 200ms sin un paquete VALIDO (Checksum OK),
            // asumimos perdida de control (ruido o desconexion) y frenamos.
            if ( (xTaskGetTickCount() - lastValidTime) > (200 / portTICK_RATE_MS) ) {
                global_joy_x = 0; 
                global_joy_y = 0; 
                global_throttle = 0; 
                global_brake = 0;
            }

            switch (state) {
                case 0: // WAIT HEADER
                    if (byte == BIN_HEADER) {
                        idx = 0;
                        rawBuf[idx++] = byte;
                        state = 1;
                    }
                    break;
                    
                case 1: // READ DATA
                    rawBuf[idx++] = byte;
                    if (idx >= 8) { // Packet Completed (8 bytes)
                        // Validar Footer
                        if (rawBuf[7] == BIN_FOOTER) {
                            // Validar Checksum (XOR Fields: idx 1 to 6)
                            // Fields: joyX(1), joyY(2), thr(3), brk(4), btns(5) -> Checksum es idx 6?
                            // struct: Header, jX, jY, Thr, Brk, Btns, Cksum, Footer
                            // Indices: 0,      1,  2,   3,   4,    5,    6,      7
                            // Checksum se calculo con: jX^jY^Thr^Brk^Btns
                            uint8_t calc = rawBuf[1] ^ rawBuf[2] ^ rawBuf[3] ^ rawBuf[4] ^ rawBuf[5];
                            
                            if (calc == rawBuf[6]) {
                                // PAQUETE VALIDO -> MAPEAR A GLOBALES
                                struct ControlPacket *pkt = (struct ControlPacket*)rawBuf;
                                
                                // Map int8 (-127..127) to System (-512..512)
                                // -127 * 4 = -508. Aprox OK.
                                global_joy_x = (int16_t)pkt->joyX * 4; 
                                global_joy_y = (int16_t)pkt->joyY * 4;
                                
                                // Map uint8 (0..255) to System (0..1023)
                                global_throttle = (uint16_t)pkt->throttle * 4; 
                                global_brake    = (uint16_t)pkt->brake * 4;
                                
                                // Botones
                                uint8_t btns = pkt->buttons;

                                // --- PANIC BUTTON (Circulo - Bit 1) ---
                                // Si se presiona Circulo, RESET TOTAL a Manual y Freno
                                if (btns & 0x02) { 
                                    currentMode = MODE_MANUAL;
                                    global_throttle = 0;
                                    global_brake = 0;
                                    global_joy_x = 0;
                                    global_joy_y = 0;
                                    // Feedback visual/haptico de reset
                                    sendFeedback(FB_CMD_RUMBLE, 50, 255, 255); // Vibracion fuerte
                                } 
                                else {
                                    // Detectar cambio de Modo (Triangulo - Bit 3)
                                    static uint8_t lastTriangle = 0;
                                    uint8_t currTriangle = (btns & 0x08) ? 1 : 0; 
                                    
                                    if (currTriangle && !lastTriangle) {
                                        // Cambiar Modo Ciclico
                                        if(currentMode == MODE_MANUAL) currentMode = MODE_SAFETY_STOP;
                                        else if(currentMode == MODE_SAFETY_STOP) currentMode = MODE_AVOIDANCE;
                                        else currentMode = MODE_MANUAL;
                                    }
                                    lastTriangle = currTriangle;
                                }
                                
                            } else {
                                // Checksum Fail
                                // printf("Cksum Fail\r\n");
                            }
                        }
                        state = 0; // Reset para buscar siguiente Header
                    }
                    break;
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
    
// 255/100 = ~2.5 frames. 50Hz -> 20ms. 2.5 * 20ms = 50ms (Muy Rapido response)
// Anterior: 25 * 200Hz = 5000 units/s.
// Nuevo: 100 * 50Hz = 5000 units/s. (Misma fisica)
#define RAMP_STEP 100 

    gpioWrite(MOTOR_STBY, ON); // Activar driver
    
    // Estado interno para rampa (Signed: +Adelante, -Atras)
    static int16_t current_pwm_signed = 0;

    while(1) {
        // --- DEAD MAN SWITCH (Redundancia) ---
        // Si TaskUARTDecode muere o se cuelga, lastValidTime deja de actualizarse.
        // TaskMotors detecta esto y corta la energia por si acaso.
        if (lastValidTime != 0 && (xTaskGetTickCount() - lastValidTime) > (300 / portTICK_RATE_MS)) {
             // Forzar corte de energia
             global_throttle = 0;
             global_brake = 0;
        }

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

        // Maquina de Estados de Evasion (Supera al Control Manual)
        static int avoid_timer = 0;

        // Si cambiamos de modo fuera de avoidance, resetear estado
        if (currentMode != MODE_AVOIDANCE) {
            avoidState = AVOID_IDLE;
        }

// Tuneado para 50Hz (20ms)
// Antes: 1500ms / 5ms = 300 ticks
// Ahora: 1500ms / 20ms = 75 ticks
#define MANEUVER_90_TIME_MS 1500 
#define MANEUVER_TICKS      (MANEUVER_90_TIME_MS / 20) 

        if (currentMode == MODE_SAFETY_STOP) {
            // -- Lógica Safety Stop --
             if (min_dist < STOP_DIST_CM && target_pwm_signed > 0) {
                target_pwm_signed = 0; // Frenar
                current_pwm_signed = 0; // Stop inmediato
            }

        } else if (currentMode == MODE_AVOIDANCE) {
            
            switch(avoidState) {
                case AVOID_IDLE:
                    // 1. Control Manual (Pass-through)
                    // ... target_pwm_signed ya tiene el valor del joystick
                    
                    // 2. Trigger Check: Pared cerca (<20cm) Y Acelerando
                    if (min_dist < 20 && target_pwm_signed > 0) {
                        // HAPTIC FEEDBACK: Vibracion Media
                        sendFeedback(FB_CMD_RUMBLE, 30, 200, 0); // 300ms, Fuerza Alta
                        
                        // INICIAR MANIOBRA
                        // printf("[AVOID] Triggered!\r\n");
                        avoidState = AVOID_REVERSING;
                        avoid_timer = 0;
                        current_pwm_signed = 0; // Stop inicial
                        target_pwm_signed = 0;
                        
                        // SNAPSHOT: Recordar situacion actual
                        old_d1 = d1; old_d2 = d2; old_d3 = d3;
                    }
                    break;
                    
                case AVOID_REVERSING:
                    // Retroceder Fuerte (Ganar espacio)
                    target_pwm_signed = -150; 
                    
                    if (avoid_timer++ > MANEUVER_TICKS) {
                        // FIN RETROCESO -> CALCULAR GIRO INTELIGENTE
                        avoidState = AVOID_TURNING;
                        avoid_timer = 0;
                        target_pwm_signed = 0; // Stop
                        
                        // --- FUSION DE DATOS ---
                        // Ponderar: El espacio actual vale el doble que el anterior
                        int32_t score_left  = d1 + (old_d1 / 2);
                        int32_t score_right = d3 + (old_d3 / 2);
                        
                        // Diferencia: Positivo = Mas espacio Dcha, Negativo = Mas espacio Izq
                        int32_t error = score_right - score_left;
                        
                        int32_t raw_angle = 150 + error;
                        
                        // Clamp
                        if (raw_angle > SERVO_MAX) raw_angle = SERVO_MAX;
                        if (raw_angle < SERVO_MIN) raw_angle = SERVO_MIN;
                        
                        avoidTurnAngle = (uint8_t)raw_angle;
                        // printf("SmartTurn: %d\r\n", avoidTurnAngle);
                    }
                    break;
                    
                case AVOID_TURNING:
                    // Stop mientras las ruedas giran al angulo calculado
                    target_pwm_signed = 0; 
                    // Antes 500ms = 100 ticks (5ms). Ahora 500ms = 25 ticks (20ms)
                    if (avoid_timer++ > 25) { 
                        avoidState = AVOID_FORWARDING;
                        avoid_timer = 0;
                    }
                    break;
                    
                case AVOID_FORWARDING:
                    // Avanzar Esquivando
                    target_pwm_signed = 150;
                    
                    // --- SEGURIDAD RECURSIVA ---
                    // "Tiempo de Gracia": Ignorar sensores los primeros 500ms 
                    // Antes 100 ticks. Ahora 25 ticks.
                    if (avoid_timer > 25) { 
                        // Si mientras esquivamos nos encontramos OTRA pared (<20cm)
                        if (min_dist < 20) {
                             // ABORTAR y REINICIAR CICLO
                             sendFeedback(FB_CMD_RUMBLE, 50, 255, 0); 
                             
                             // printf("[AVOID] ABORT\r\n");
                             avoidState = AVOID_REVERSING; // Volver a atras
                             avoid_timer = 0;
                             target_pwm_signed = 0;
                             old_d1 = d1; old_d2 = d2; old_d3 = d3;
                             break; 
                        }
                    }

                    if (avoid_timer++ > MANEUVER_TICKS) {
                        avoidState = AVOID_IDLE; // Devolver Control
                        avoid_timer = 0;
                    }
                    break;
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
            // LED Feedback removido para usar RGB en MODOS
            
        } else if (current_pwm_signed < 0) {
            // Reversa
            gpioWrite(MOTOR_AIN1, OFF); gpioWrite(MOTOR_AIN2, ON);
            gpioWrite(MOTOR_BIN1, OFF); gpioWrite(MOTOR_BIN2, ON);
            // Invertir para PWM absoluto
            pwmWrite(PWMA_PIN, (uint8_t)(-current_pwm_signed));
            pwmWrite(PWMB_PIN, (uint8_t)(-current_pwm_signed));
            // LED Feedback removido
            
        } else {
            // Stop
            pwmWrite(PWMA_PIN, 0); pwmWrite(PWMB_PIN, 0);
            // LED Feedback removido
        }
        
        // Ticks ajustados para delay de 20ms
        vTaskDelay(20 / portTICK_RATE_MS); // 50Hz control loop (Standard)
    }
}
// --- SERVO LIMITS ---
// (Definidos globalmente arriba)

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
            
            // Comportamiento segun Estado de Maniobra
            switch(avoidState) {
                case AVOID_IDLE:
                    // Control Manual Normal (Joystick)
                    // ... targetAngle ya calculado arriba
                    break;
                    
                case AVOID_REVERSING:
                    // Al retroceder, Enderezar ruedas para ir recto atras
                    targetAngle = SERVO_CENTER; 
                    break;
                    
                case AVOID_TURNING:
                case AVOID_FORWARDING:
                    // Girar al angulo CALCULADO DINAMICAMENTE
                    targetAngle = avoidTurnAngle;
                    break;
            }
        }

        
        set_servo_angle((uint8_t)targetAngle);
        
        vTaskDelay(20 / portTICK_RATE_MS); 
    }
}


void App_StartSystem(void) {
    // Crear tareas del sistema normal
    xTaskCreate(TaskStatus,  "Status",  128, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskSensors, "Sensors", 384, NULL, tskIDLE_PRIORITY + 2, NULL); // Optimizado
    // STACK AJUSTADO para equilibrar memoria y seguridad
    xTaskCreate(TaskUARTDecode, "Decoder", 640, NULL, tskIDLE_PRIORITY + 3, NULL); 
    xTaskCreate(TaskMotors,  "Motors",  256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskServo,   "Servo",   256, NULL, tskIDLE_PRIORITY + 1, NULL);
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
    
    // Decision de Arranque
    #ifdef ENABLE_HARDWARE_TEST
        printf(">>> MODO TEST DE HARDWARE ACTIVADO <<<\r\n");
        // Crear Solo la tarea de Test. Esta tarea lanzara App_StartSystem si todo sale bien.
        xTaskCreate(TaskHardwareTest, "HwTest", 512, NULL, tskIDLE_PRIORITY + 4, NULL);
    #else
        printf(">>> MODO NORMAL <<<\r\n");
        App_StartSystem();
    #endif

    // Iniciar Scheduler
    vTaskStartScheduler();

    while(1); 
    return 0;
}
