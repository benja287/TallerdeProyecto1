
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

void TaskBlink(void* taskParm) {
    while(1) {
        gpioToggle(LEDB);
        vTaskDelay(500 / portTICK_RATE_MS); // 0.5Hz
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
        // Esperar max 35ms (eco maximo seguro + margen)
        ulTaskNotifyTake(pdTRUE, 35 / portTICK_RATE_MS);
        
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
        // printf("S1: %dcm, S2: %dcm, S3: %dcm\r\n", dist1_cm, dist2_cm, dist3_cm);
        
        // Retardo para la siguiente vuelta
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}

// ==============================================================================
// TAREAS UART / DECODER
// ==============================================================================

// Callback de Interrupcion UART (Se ejecuta en contexto de ISR)
void onRx(void *noData) {
    uint8_t byte;
    // Leer byte mientras haya disponibles (aunque es interrupcion byte a byte)
    while (uartReadByte(UART_232, &byte)) {
        // Guardar en buffer circular
        uint16_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
        if (nextHead != rxTail) { // Si no esta lleno
            rxBuffer[rxHead] = byte;
            rxHead = nextHead;
        }
        
        // Si detectamos fin de linea, notificar a la tarea de debodificacion
        // Para procesar el paquete completo lo antes posible
        if (byte == '\n') {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(xUARTTaskHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    
    gpioWrite(MOTOR_STBY, ON); // Activar driver

    while(1) {
        // --- CONTROL POR GATILLOS (R2/L2) ---
        uint16_t thr = global_throttle;
        uint16_t brk = global_brake;
        
        // Zona muerta de 50 (Rango 0-1023)
        if (thr > 50 && brk < 50) {
            // ADELANTE
            // Map 50..1023 -> 80..255 (Arrancar con un poco de fuerza)
            uint32_t pwm = 80 + ((thr - 50) * (255 - 80) / (1023 - 50));
            
            gpioWrite(MOTOR_AIN1, ON); gpioWrite(MOTOR_AIN2, OFF);
            gpioWrite(MOTOR_BIN1, ON); gpioWrite(MOTOR_BIN2, OFF);
            pwmWrite(PWMA_PIN, (uint8_t)pwm);
            pwmWrite(PWMB_PIN, (uint8_t)pwm);
            gpioWrite(LEDG, ON);
            gpioWrite(LEDR, OFF);
            
        } else if (brk > 50 && thr < 50) {
            // REVERSA
            uint32_t pwm = 80 + ((brk - 50) * (255 - 80) / (1023 - 50));
            
            gpioWrite(MOTOR_AIN1, OFF); gpioWrite(MOTOR_AIN2, ON);
            gpioWrite(MOTOR_BIN1, OFF); gpioWrite(MOTOR_BIN2, ON);
            pwmWrite(PWMA_PIN, (uint8_t)pwm);
            pwmWrite(PWMB_PIN, (uint8_t)pwm);
            gpioWrite(LEDR, ON);
            gpioWrite(LEDG, OFF);
            
        } else {
            // STOP (Freno o ambos apretados o reposo)
            pwmWrite(PWMA_PIN, 0);
            pwmWrite(PWMB_PIN, 0);
            gpioWrite(LEDG, OFF);
            gpioWrite(LEDR, OFF);
        }
        
        vTaskDelay(20 / portTICK_RATE_MS); // 50Hz control loop
    }
}
// --- SERVO LIMITS ---
#define SERVO_CENTER   150
#define SERVO_MIN      120 // 150 - 30
#define SERVO_MAX      180 // 150 + 30 (Limite fisico software)

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
    xTaskCreate(TaskBlink,   "Blink",   128, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskSensors, "Sensors", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(TaskUARTDecode, "Decoder", 512, NULL, tskIDLE_PRIORITY + 3, NULL); // Prioridad alta para vaciar buffer
    xTaskCreate(TaskMotors,  "Motors",  256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskServo,   "Servo",   256, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Iniciar Scheduler
    vTaskStartScheduler();

    while(1); 
    return 0;
}
