#include "test_hardware.h"
#include "sapi.h" // Includes boardConfig, gpio, etc
#include "sapi_uart.h"
#include "chip.h" // For GPIO direct access if needed
#include <stdio.h>
#include <string.h>

// --- EXTERNALS FROM app.c ---
extern void App_StartSystem(void);

// Variables y Funciones de Sensores (app.c debe haberlas expuesto quitando static)
// Nota: En C, si no estan en un .h, usamos extern.
extern void config_sensor_pins(void);
extern void trigger_sensor(uint8_t port, uint8_t pin);
extern volatile uint32_t dist1_cm;
extern volatile uint32_t dist2_cm;
extern volatile uint32_t dist3_cm;
extern TaskHandle_t xSensorTaskHandle;

// Funciones de Servo (app.c)
extern void set_servo_angle(uint8_t angle);

// Funciones de Motor (Reusamos el PWM ya configurado en main)
// Definiciones copiadas de app.c (Idealmente estarian en app.h)
#define PWMA_PIN       PWM2   
#define PWMB_PIN       PWM5   
#define MOTOR_AIN1     GPIO3
#define MOTOR_AIN2     GPIO1
#define MOTOR_BIN1     GPIO7
#define MOTOR_BIN2     GPIO8
#define MOTOR_STBY     GPIO5

// Definiciones de Sensores copiadas para trigger
// Sensor 1
#define TRIG1_GPIO_P   5
#define TRIG1_GPIO_B   12
// Sensor 2
#define TRIG2_GPIO_P   2
#define TRIG2_GPIO_B   6
// Sensor 3
#define TRIG3_GPIO_P   2
#define TRIG3_GPIO_B   5


// --- UTILS ---
static void leds_busy(void) {
    // BLUE = Working/Busy
    gpioWrite(LEDB, ON);
    gpioWrite(LEDR, OFF);
    gpioWrite(LEDG, OFF);
    gpioWrite(LED1, OFF); gpioWrite(LED2, OFF); gpioWrite(LED3, OFF);
}

static void leds_pass(void) {
    // GREEN = EXITO (2 segundos solido)
    gpioWrite(LEDB, OFF);
    gpioWrite(LEDR, OFF);
    gpioWrite(LEDG, ON); // Green On
    vTaskDelay(2000 / portTICK_RATE_MS);
    gpioWrite(LEDG, OFF);
}

static void leds_fail_recoverable(void) {
    // RED BLINK = ERROR (Advertencia)
    gpioWrite(LEDB, OFF);
    gpioWrite(LEDG, OFF);
    for(int i=0; i<5; i++) {
        gpioWrite(LEDR, ON); 
        vTaskDelay(200 / portTICK_RATE_MS);
        gpioWrite(LEDR, OFF); 
        vTaskDelay(200 / portTICK_RATE_MS);
    }
}

// --- TASKS ---

void TaskHardwareTest(void* taskParm) {
    vTaskDelay(500 / portTICK_RATE_MS); // Esperar estabilizacion electrica
    leds_busy(); // Blue ON
    
    printf("\r\n========================================\r\n");
    printf("[POST] INICIANDO TEST DE HARDWARE\r\n");
    printf("========================================\r\n");

    int errors = 0;

    // --- 1. TEST SERVO ---
    printf("[POST] > Servo Check... ");
    // Barrido Centro -> Min -> Max -> Centro
    set_servo_angle(150); vTaskDelay(200 / portTICK_RATE_MS);
    set_servo_angle(132); vTaskDelay(200 / portTICK_RATE_MS);
    set_servo_angle(177); vTaskDelay(200 / portTICK_RATE_MS);
    set_servo_angle(150);
    printf("DONE (Verificar visualmente)\r\n");

    // --- 2. TEST MOTORES (TWITCH) ---
    printf("[POST] > Motores Check... ");
    
    // Configurar Pines como Salida (IMPORTANTE: TaskMotors aun no corrio)
    gpioConfig(MOTOR_AIN1, GPIO_OUTPUT);
    gpioConfig(MOTOR_AIN2, GPIO_OUTPUT);
    gpioConfig(MOTOR_BIN1, GPIO_OUTPUT);
    gpioConfig(MOTOR_BIN2, GPIO_OUTPUT);
    gpioConfig(MOTOR_STBY, GPIO_OUTPUT);

    // Habilitar H-Bridge
    gpioWrite(MOTOR_STBY, ON);
    
    // Adelante suave (Mas visible)
    gpioWrite(MOTOR_AIN1, ON); gpioWrite(MOTOR_AIN2, OFF);
    gpioWrite(MOTOR_BIN1, ON); gpioWrite(MOTOR_BIN2, OFF);
    pwmWrite(PWMA_PIN, 110); pwmWrite(PWMB_PIN, 110); 
    vTaskDelay(150 / portTICK_RATE_MS); // 150ms
    
    // Stop
    pwmWrite(PWMA_PIN, 0); pwmWrite(PWMB_PIN, 0);
    vTaskDelay(500 / portTICK_RATE_MS); // Pausa mas larga para notar el stop
    
    // Atras suave (Mas visible)
    gpioWrite(MOTOR_AIN1, OFF); gpioWrite(MOTOR_AIN2, ON);
    gpioWrite(MOTOR_BIN1, OFF); gpioWrite(MOTOR_BIN2, ON);
    pwmWrite(PWMA_PIN, 110); pwmWrite(PWMB_PIN, 110);
    vTaskDelay(150 / portTICK_RATE_MS);
    
    // Stop Final
    pwmWrite(PWMA_PIN, 0); pwmWrite(PWMB_PIN, 0);
    gpioWrite(MOTOR_STBY, OFF); // Standby por seguridad
    
    printf("DONE (Verificar visualmente movimiento)\r\n");

    // --- 3. TEST SENSORES ---
    printf("[POST] > Sensores Check...\r\n");
    
    // Configurar pines (si no se hizo ya, aunque app.c no lo hace en main, lo hace en task sensors)
    config_sensor_pins();
    
    // Enganchar handle para recibir notificaciones de ISR
    xSensorTaskHandle = xTaskGetCurrentTaskHandle();
    
    // SENSOR 1
    printf("         > S1 (Izq): ");
    xTaskNotifyStateClear(NULL); // Limpiar pendientes
    dist1_cm = 0;
    portENTER_CRITICAL();
    trigger_sensor(TRIG1_GPIO_P, TRIG1_GPIO_B);
    portEXIT_CRITICAL();
    
    if (ulTaskNotifyTake(pdTRUE, 100 / portTICK_RATE_MS) > 0) {
        printf("PASS (%d cm)\r\n", (int)dist1_cm);
    } else {
        printf("FAIL (Timeout)\r\n");
        errors++; // Error no critico (Soft Fail)
    }
    vTaskDelay(50 / portTICK_RATE_MS);

    // SENSOR 2
    printf("         > S2 (Cen): ");
    xTaskNotifyStateClear(NULL);
    dist2_cm = 0;
    portENTER_CRITICAL();
    trigger_sensor(TRIG2_GPIO_P, TRIG2_GPIO_B);
    portEXIT_CRITICAL();
    
    if (ulTaskNotifyTake(pdTRUE, 100 / portTICK_RATE_MS) > 0) {
        printf("PASS (%d cm)\r\n", (int)dist2_cm);
    } else {
        printf("FAIL (Timeout)\r\n");
        errors++;
    }
    vTaskDelay(50 / portTICK_RATE_MS);

    // SENSOR 3
    printf("         > S3 (Der): ");
    xTaskNotifyStateClear(NULL);
    dist3_cm = 0;
    portENTER_CRITICAL();
    trigger_sensor(TRIG3_GPIO_P, TRIG3_GPIO_B);
    portEXIT_CRITICAL();
    
    if (ulTaskNotifyTake(pdTRUE, 100 / portTICK_RATE_MS) > 0) {
        printf("PASS (%d cm)\r\n", (int)dist3_cm);
    } else {
        printf("FAIL (Timeout)\r\n");
        errors++;
    }
    
    // Liberar handle para que la tarea normal lo tome despues
    xSensorTaskHandle = NULL;

    // --- RESULTADO FINAL ---
    printf("========================================\r\n");
    if (errors == 0) {
        printf("[POST] RESULTADO: EXITO TOTAL\r\n");
        leds_pass();
    } else {
        printf("[POST] RESULTADO: PARCIAL (%d Errores detectados)\r\n", errors);
        printf("[POST] ADVERTENCIA: Iniciando sistema a pesar de fallos.\r\n");
        leds_fail_recoverable();
    }
    printf("========================================\r\n");

    // --- INICIAR SISTEMA ---
    vTaskDelay(500 / portTICK_RATE_MS);
    App_StartSystem();
    
    // Auto-eliminar esta tarea
    vTaskDelete(NULL);
}
