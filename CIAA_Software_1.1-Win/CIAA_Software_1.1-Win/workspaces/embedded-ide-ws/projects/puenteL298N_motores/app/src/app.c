#define BOARD edu_ciaa_nxp
#include "sapi.h"

// Pines actuales (sin cambiar hardware)
#define PWMA   T_COL0
#define PWMB   T_COL1
#define AIN1   GPIO3
#define AIN2   GPIO1
#define BIN1   GPIO7
#define BIN2   GPIO8
#define STBY   GPIO5

// Variables para PWM software
volatile uint8_t speedA = 255;  // 0-255 (78%)
volatile uint8_t speedB = 255;

void softwarePWM(void)
{
    static uint8_t counter = 0;
    
    // Motor A
    if(counter < speedA) {
        gpioWrite(PWMA, ON);
    } else {
        gpioWrite(PWMA, OFF);
    }
    
    // Motor B
    if(counter < speedB) {
        gpioWrite(PWMB, ON);
    } else {
        gpioWrite(PWMB, OFF);
    }
    
    counter++;
}

int main(void)
{
    boardConfig();
    
    // Configurar TODOS como GPIO (sin PWM hardware)
    gpioConfig(PWMA, GPIO_OUTPUT);
    gpioConfig(PWMB, GPIO_OUTPUT);
    gpioConfig(AIN1, GPIO_OUTPUT);
    gpioConfig(AIN2, GPIO_OUTPUT);
    gpioConfig(BIN1, GPIO_OUTPUT);
    gpioConfig(BIN2, GPIO_OUTPUT);
    gpioConfig(STBY, GPIO_OUTPUT);
    
    // Activar puente
    gpioWrite(STBY, ON);
    
    // Dirección: ADELANTE
    gpioWrite(AIN1, ON);
    gpioWrite(AIN2, OFF);
    gpioWrite(BIN1, ON);
    gpioWrite(BIN2, OFF);
    
    // Bucle PWM software
    while(TRUE)
    {
        softwarePWM();
        delayInaccurateUs(40);  // ~25kHz PWM
    }
    
    return 0;
}