
/*
 * Test All In - Integracion de Sensores, Motores y Servo con FreeRTOS
 *
 */

#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "chip.h"

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

// ==============================================================================
// FUNCIONES AUXILIARES
// ==============================================================================

// Configuración de bajo nivel para sensores
void config_sensor_pins(void) {
    // S1
    Chip_SCU_PinMuxSet(TRIG1_PORT, TRIG1_PIN, (SCU_MODE_INACT | SCU_MODE_FUNC4));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG1_GPIO_P, TRIG1_GPIO_B);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG1_GPIO_P, TRIG1_GPIO_B, false);

    Chip_SCU_PinMuxSet(ECHO1_PORT, ECHO1_PIN, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO1_GPIO_P, ECHO1_GPIO_B);

    // S2
    Chip_SCU_PinMuxSet(TRIG2_PORT, TRIG2_PIN, (SCU_MODE_INACT | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG2_GPIO_P, TRIG2_GPIO_B);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG2_GPIO_P, TRIG2_GPIO_B, false);

    Chip_SCU_PinMuxSet(ECHO2_PORT, ECHO2_PIN, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO2_GPIO_P, ECHO2_GPIO_B);

    // S3
    Chip_SCU_PinMuxSet(TRIG3_PORT, TRIG3_PIN, (SCU_MODE_INACT | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG3_GPIO_P, TRIG3_GPIO_B);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG3_GPIO_P, TRIG3_GPIO_B, false);

    Chip_SCU_PinMuxSet(ECHO3_PORT, ECHO3_PIN, (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO3_GPIO_P, ECHO3_GPIO_B);
}

// Disparo del pulso
void trigger_sensor(uint8_t port, uint8_t pin) {
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, port, pin, false);
    delayInaccurateUs(2);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, port, pin, true);
    delayInaccurateUs(10);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, port, pin, false);
}

// Medición de eco (bloqueante por corto tiempo - max 50ms)
// Nota: En FreeRTOS idealmente usariamos interrupciones, pero por simplicidad
// y dado el codigo base, usaremos este metodo con timeout.
uint32_t get_distance(uint8_t echo_port, uint8_t echo_pin) {
    uint32_t timeout = 500000;
    uint32_t count = 0;

    // Esperar 1
    while(!Chip_GPIO_GetPinState(LPC_GPIO_PORT, echo_port, echo_pin) && timeout > 0) timeout--;
    if(timeout == 0) return 0;

    // Contar
    timeout = 500000;
    while(Chip_GPIO_GetPinState(LPC_GPIO_PORT, echo_port, echo_pin) && timeout > 0) {
        timeout--;
        count++;
    }
    
    // Calibración aproximada basada en el código original (count / 10 * factor??)
    // El codigo original hacia: pulse_width / 10 luego * 343 / 20000
    // Simplificado: count es proporcional al tiempo.
    // Usaremos la fórmula del ejemplo original tal cual
    uint32_t raw_time = count / 10;
    return (raw_time * 343) / 20000;
}

// Header de Timer para PWM por Software/Interrupcion
#include "sapi_timer.h"

// ... existing includes ...

// ==============================================================================
// FUNCIONES AUXILIARES
// ==============================================================================

// ... existing config_sensor_pins ...
// ... existing trigger_sensor ...
// ... existing get_distance ...

// --- CONTROL SERVO (TIMER0) ---
// Logica copiada de servos3003

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

// ==============================================================================
// TAREAS FREERTOS
// ==============================================================================

void TaskBlink(void* taskParm) {
    while(1) {
        gpioToggle(LEDB);
        vTaskDelay(500 / portTICK_RATE_MS); // 0.5Hz
    }
}

void TaskSensors(void* taskParm) {
    config_sensor_pins();

    while(1) {
        // Sensor 1
        portENTER_CRITICAL(); // Breve sección crítica para timing preciso del trigger
        trigger_sensor(TRIG1_GPIO_P, TRIG1_GPIO_B);
        portEXIT_CRITICAL();
        dist1_cm = get_distance(ECHO1_GPIO_P, ECHO1_GPIO_B);
        vTaskDelay(50 / portTICK_RATE_MS);

        // Sensor 2
        portENTER_CRITICAL();
        trigger_sensor(TRIG2_GPIO_P, TRIG2_GPIO_B);
        portEXIT_CRITICAL();
        dist2_cm = get_distance(ECHO2_GPIO_P, ECHO2_GPIO_B);
        vTaskDelay(50 / portTICK_RATE_MS);

        // Sensor 3
        portENTER_CRITICAL();
        trigger_sensor(TRIG3_GPIO_P, TRIG3_GPIO_B);
        portEXIT_CRITICAL();
        dist3_cm = get_distance(ECHO3_GPIO_P, ECHO3_GPIO_B);
        vTaskDelay(50 / portTICK_RATE_MS);

        // Loguear (Opcional, puede saturar UART)
        // printf("S1: %dcm, S2: %dcm, S3: %dcm\r\n", dist1_cm, dist2_cm, dist3_cm);
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
        // Adelante
        gpioWrite(MOTOR_AIN1, ON); gpioWrite(MOTOR_AIN2, OFF);
        gpioWrite(MOTOR_BIN1, ON); gpioWrite(MOTOR_BIN2, OFF);
        pwmWrite(PWMA_PIN, VEL_MEDIA);
        pwmWrite(PWMB_PIN, VEL_MEDIA);
        gpioWrite(LEDG, ON);
        vTaskDelay(2000 / portTICK_RATE_MS);

        // Parar
        pwmWrite(PWMA_PIN, 0);
        pwmWrite(PWMB_PIN, 0);
        gpioWrite(LEDG, OFF);
        vTaskDelay(1000 / portTICK_RATE_MS);

        // Atras
        gpioWrite(MOTOR_AIN1, OFF); gpioWrite(MOTOR_AIN2, ON);
        gpioWrite(MOTOR_BIN1, OFF); gpioWrite(MOTOR_BIN2, ON);
        pwmWrite(PWMA_PIN, VEL_MEDIA);
        pwmWrite(PWMB_PIN, VEL_MEDIA);
        gpioWrite(LEDR, ON); // Indicar reversa
        vTaskDelay(2000 / portTICK_RATE_MS);

        // Parar
        pwmWrite(PWMA_PIN, 0);
        pwmWrite(PWMB_PIN, 0);
        gpioWrite(LEDR, OFF);
        vTaskDelay(1000 / portTICK_RATE_MS);
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
        set_servo_angle(angle);
        
        if (movingLeft == 0) {
            if (angle + step >= max_angle) {
                angle = max_angle;
                movingLeft = 1; 
            } else {
                angle += step;
            }
        } else {
            if (angle <= min_angle + step) {
                angle = min_angle;
                movingLeft = 0; 
            } else {
                angle -= step;
            }
        }
        
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

    printf("Sistema iniciado: Motores PWM Hardware + Servo Timer0 IRQ\r\n");
    
    // Crear tareas
    xTaskCreate(TaskBlink,   "Blink",   128, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskSensors, "Sensors", 256, NULL, tskIDLE_PRIORITY + 2, NULL);
    xTaskCreate(TaskMotors,  "Motors",  256, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(TaskServo,   "Servo",   256, NULL, tskIDLE_PRIORITY + 1, NULL);

    // Iniciar Scheduler
    vTaskStartScheduler();

    while(1); 
    return 0;
}
