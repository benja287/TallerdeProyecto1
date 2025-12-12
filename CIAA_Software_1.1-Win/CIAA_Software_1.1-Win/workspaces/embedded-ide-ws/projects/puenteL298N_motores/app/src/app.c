/**
 * Puente H TB6612FNG - Control de Motores DC con PWM HARDWARE
 * Control por hardware usando PWM2 y PWM5 - NO BLOQUEANTE
 */

#define BOARD edu_ciaa_nxp
#include "sapi.h"

// Pines del puente H TB6612FNG
#define PWMA   PWM2   // T_COL0 - Motor A (PWM hardware)
#define PWMB   PWM5   // T_COL1 - Motor B (PWM hardware)
#define AIN1   GPIO3  // Control dirección Motor A
#define AIN2   GPIO1  // Control dirección Motor A
#define BIN1   GPIO7  // Control dirección Motor B
#define BIN2   GPIO8  // Control dirección Motor B
#define STBY   GPIO5  // Standby (ON = activo)

// Velocidades de prueba (0-255)
#define VELOCIDAD_BAJA    100
#define VELOCIDAD_MEDIA   150
#define VELOCIDAD_ALTA    200
#define VELOCIDAD_MAXIMA  255

int main(void)
{
    boardConfig();
    uartConfig(UART_USB, 115200);
    tickConfig(1);  // Tick cada 1ms para control no bloqueante
    
    // Configurar GPIOs para control de dirección y standby
    gpioConfig(AIN1, GPIO_OUTPUT);
    gpioConfig(AIN2, GPIO_OUTPUT);
    gpioConfig(BIN1, GPIO_OUTPUT);
    gpioConfig(BIN2, GPIO_OUTPUT);
    gpioConfig(STBY, GPIO_OUTPUT);
    
    // Configurar LEDs para indicación
    gpioConfig(LEDR, GPIO_OUTPUT);
    gpioConfig(LEDG, GPIO_OUTPUT);
    gpioConfig(LEDB, GPIO_OUTPUT);
    
    delay(1000);
    
    printf("\r\n========================================\r\n");
    printf("  TB6612FNG - PWM HARDWARE\r\n");
    printf("========================================\r\n");
    printf("Motor A: PWM2 (T_COL0)\r\n");
    printf("Motor B: PWM5 (T_COL1)\r\n");
    printf("Frecuencia PWM: 1 kHz (hardware)\r\n");
    printf("========================================\r\n\r\n");
    
    // Inicializar sistema PWM (solo una vez, inicializa los timers SCT)
    printf("Inicializando PWM hardware...\r\n");
    if (!pwmInit(PWMA, PWM_ENABLE)) {
        printf("ERROR: No se pudo inicializar PWM\r\n");
        gpioWrite(LEDR, ON);
        while(1);  // Detener si falla
    }
    
    // Habilitar salidas PWM para ambos motores
    if (!pwmInit(PWMA, PWM_ENABLE_OUTPUT)) {
        printf("ERROR: No se pudo habilitar PWM2 (Motor A)\r\n");
        gpioWrite(LEDR, ON);
        while(1);
    }
    
    if (!pwmInit(PWMB, PWM_ENABLE_OUTPUT)) {
        printf("ERROR: No se pudo habilitar PWM5 (Motor B)\r\n");
        gpioWrite(LEDR, ON);
        while(1);
    }
    
    printf("PWM hardware inicializado correctamente\r\n");
    gpioWrite(LEDG, ON);
    
    // Activar puente H (salir de standby)
    gpioWrite(STBY, ON);
    
    printf("\r\nIniciando test de motores...\r\n");
    printf("Secuencia: Parar -> Adelante -> Parar -> Atrás -> Parar\r\n\r\n");
    
    // Control no bloqueante para test de motores
    tick_t tiempoAnterior = tickRead();
    uint8_t estado = 0;  // 0: parado, 1: adelante, 2: parado, 3: atrás
    
    while(TRUE) {
        tick_t tiempoActual = tickRead();
        
        // Cambiar estado cada 3 segundos
        if (tiempoActual - tiempoAnterior >= 3000) {
            
            switch(estado) {
                case 0:  // Parar motores
                    printf("-> Parando motores\r\n");
                    pwmWrite(PWMA, 0);
                    pwmWrite(PWMB, 0);
                    gpioWrite(AIN1, OFF);
                    gpioWrite(AIN2, OFF);
                    gpioWrite(BIN1, OFF);
                    gpioWrite(BIN2, OFF);
                    gpioWrite(LEDR, OFF);
                    gpioWrite(LEDG, OFF);
                    gpioWrite(LEDB, OFF);
                    estado = 1;
                    break;
                    
                case 1:  // Adelante (ambos motores)
                    printf("-> Adelante (velocidad media)\r\n");
                    // Motor A: adelante (AIN1=ON, AIN2=OFF)
                    gpioWrite(AIN1, ON);
                    gpioWrite(AIN2, OFF);
                    pwmWrite(PWMA, VELOCIDAD_MEDIA);
                    
                    // Motor B: adelante (BIN1=ON, BIN2=OFF)
                    gpioWrite(BIN1, ON);
                    gpioWrite(BIN2, OFF);
                    pwmWrite(PWMB, VELOCIDAD_MEDIA);
                    
                    gpioWrite(LEDG, ON);  // Verde: adelante
                    estado = 2;
                    break;
                    
                case 2:  // Parar motores
                    printf("-> Parando motores\r\n");
                    pwmWrite(PWMA, 0);
                    pwmWrite(PWMB, 0);
                    gpioWrite(LEDG, OFF);
                    estado = 3;
                    break;
                    
                case 3:  // Atrás (ambos motores)
                    printf("-> Atrás (velocidad media)\r\n");
                    // Motor A: atrás (AIN1=OFF, AIN2=ON)
                    gpioWrite(AIN1, OFF);
                    gpioWrite(AIN2, ON);
                    pwmWrite(PWMA, VELOCIDAD_MEDIA);
                    
                    // Motor B: atrás (BIN1=OFF, BIN2=ON)
                    gpioWrite(BIN1, OFF);
                    gpioWrite(BIN2, ON);
                    pwmWrite(PWMB, VELOCIDAD_MEDIA);
                    
                    gpioWrite(LEDR, ON);  // Rojo: atrás
                    estado = 0;
                    break;
            }
            
            tiempoAnterior = tiempoActual;
        }
        
        // Aquí puedes hacer otras tareas sin bloquear los motores
        // El PWM hardware se encarga de generar las señales automáticamente
    }
    
    return 0;
}
