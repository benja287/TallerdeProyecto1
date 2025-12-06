#include "sapi.h"
#include "chip.h"

// SENSOR 1 - Configuración según el esquemático original
#define TRIG1_PIN_PORT    4
#define TRIG1_PIN_NUM     8
#define TRIG1_GPIO_PORT   5
#define TRIG1_GPIO_PIN    12

#define ECHO1_PIN_PORT    4
#define ECHO1_PIN_NUM     0
#define ECHO1_GPIO_PORT   2
#define ECHO1_GPIO_PIN    0

// SENSOR 2 - Configuración según esquemático
#define TRIG2_PIN_PORT    4
#define TRIG2_PIN_NUM     6
#define TRIG2_GPIO_PORT   2
#define TRIG2_GPIO_PIN    6

#define ECHO2_PIN_PORT    4
#define ECHO2_PIN_NUM     3
#define ECHO2_GPIO_PORT   2
#define ECHO2_GPIO_PIN    3

// SENSOR 3 - Nueva configuración
// TRIG3: LCD2 = P4.5 como GPIO2[5]
#define TRIG3_PIN_PORT    4
#define TRIG3_PIN_NUM     5
#define TRIG3_GPIO_PORT   2
#define TRIG3_GPIO_PIN    5

// ECHO3: T_FIL2 = P4.2 como GPIO2[2]
#define ECHO3_PIN_PORT    4
#define ECHO3_PIN_NUM     2
#define ECHO3_GPIO_PORT   2
#define ECHO3_GPIO_PIN    2

void config_pins_custom(void){
    // ===== SENSOR 1 =====
    // Configurar P4.8 como GPIO5[12] - TRIG1
    Chip_SCU_PinMuxSet(TRIG1_PIN_PORT, TRIG1_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC4));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG1_GPIO_PORT, TRIG1_GPIO_PIN);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG1_GPIO_PORT, TRIG1_GPIO_PIN, false);
    
    // Configurar P4.0 como GPIO2[0] - ECHO1
    Chip_SCU_PinMuxSet(ECHO1_PIN_PORT, ECHO1_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO1_GPIO_PORT, ECHO1_GPIO_PIN);
    
    // ===== SENSOR 2 =====
    // Configurar P4.6 (LCD3) como GPIO2[6] - TRIG2
    Chip_SCU_PinMuxSet(TRIG2_PIN_PORT, TRIG2_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG2_GPIO_PORT, TRIG2_GPIO_PIN);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG2_GPIO_PORT, TRIG2_GPIO_PIN, false);
    
    // Configurar P4.3 (T_FIL3) como GPIO2[3] - ECHO2
    Chip_SCU_PinMuxSet(ECHO2_PIN_PORT, ECHO2_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO2_GPIO_PORT, ECHO2_GPIO_PIN);
    
    // ===== SENSOR 3 =====
    // Configurar P4.5 (LCD2) como GPIO2[5] - TRIG3
    Chip_SCU_PinMuxSet(TRIG3_PIN_PORT, TRIG3_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIROutput(LPC_GPIO_PORT, TRIG3_GPIO_PORT, TRIG3_GPIO_PIN);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, TRIG3_GPIO_PORT, TRIG3_GPIO_PIN, false);
    
    // Configurar P4.2 (T_FIL2) como GPIO2[2] - ECHO3
    Chip_SCU_PinMuxSet(ECHO3_PIN_PORT, ECHO3_PIN_NUM, 
                       (SCU_MODE_INACT | SCU_MODE_INBUFF_EN | SCU_MODE_FUNC0));
    Chip_GPIO_SetPinDIRInput(LPC_GPIO_PORT, ECHO3_GPIO_PORT, ECHO3_GPIO_PIN);
}

void trigger_pulse(uint8_t gpio_port, uint8_t gpio_pin){
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio_port, gpio_pin, false);
    delayInaccurateUs(2);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio_port, gpio_pin, true);
    delayInaccurateUs(10);
    Chip_GPIO_SetPinState(LPC_GPIO_PORT, gpio_port, gpio_pin, false);
}

uint32_t measure_echo(uint8_t gpio_port, uint8_t gpio_pin){
    uint32_t timeout = 0;
    uint32_t max_timeout = 500000;
    uint32_t pulse_width = 0;
    
    // Esperar subida del pulso ECHO
    timeout = 0;
    while(!Chip_GPIO_GetPinState(LPC_GPIO_PORT, gpio_port, gpio_pin) 
          && timeout++ < max_timeout);
    
    if(timeout >= max_timeout) return 0;
    
    // Contar mientras ECHO está en alto
    pulse_width = 0;
    while(Chip_GPIO_GetPinState(LPC_GPIO_PORT, gpio_port, gpio_pin) 
          && pulse_width++ < max_timeout);
    
    if(pulse_width >= max_timeout) return 0;
    
    return pulse_width / 10;
}

int main(void){
    boardConfig();
    
    // Configurar pines personalizados
    config_pins_custom();
    
    // Test inicial - parpadeo RGB
    for(int i = 0; i < 3; i++) {
        gpioWrite(LEDR, ON);
        gpioWrite(LEDG, ON);
        gpioWrite(LEDB, ON);
        delay(200);
        gpioWrite(LEDR, OFF);
        gpioWrite(LEDG, OFF);
        gpioWrite(LEDB, OFF);
        delay(200);
    }
    
    while(TRUE){
        // ===== MEDIR SENSOR 1 =====
        trigger_pulse(TRIG1_GPIO_PORT, TRIG1_GPIO_PIN);
        delayInaccurateUs(50);
        uint32_t echo_time1 = measure_echo(ECHO1_GPIO_PORT, ECHO1_GPIO_PIN);
        
        delay(60); // Pausa entre sensores para evitar interferencia
        
        // ===== MEDIR SENSOR 2 =====
        trigger_pulse(TRIG2_GPIO_PORT, TRIG2_GPIO_PIN);
        delayInaccurateUs(50);
        uint32_t echo_time2 = measure_echo(ECHO2_GPIO_PORT, ECHO2_GPIO_PIN);
        
        delay(60); // Pausa entre sensores
        
        // ===== MEDIR SENSOR 3 =====
        trigger_pulse(TRIG3_GPIO_PORT, TRIG3_GPIO_PIN);
        delayInaccurateUs(50);
        uint32_t echo_time3 = measure_echo(ECHO3_GPIO_PORT, ECHO3_GPIO_PIN);
        
        // Apagar todos los LEDs primero
        gpioWrite(LEDR, OFF);
        gpioWrite(LEDG, OFF);
        gpioWrite(LEDB, OFF);
        gpioWrite(LED1, OFF);
        gpioWrite(LED2, OFF);
        gpioWrite(LED3, OFF);
        
        // Calcular distancias
        uint32_t distance1_cm = 0;
        uint32_t distance2_cm = 0;
        uint32_t distance3_cm = 0;
        
        if(echo_time1 > 0) {
            distance1_cm = (echo_time1 * 343) / 20000;
        }
        
        if(echo_time2 > 0) {
            distance2_cm = (echo_time2 * 343) / 20000;
        }
        
        if(echo_time3 > 0) {
            distance3_cm = (echo_time3 * 343) / 20000;
        }
        
        // Encontrar la MENOR distancia (objeto más cercano) de los 3 sensores
        uint32_t distance_cm = 0;
        uint8_t valid_readings = 0;
        
        // Inicializar con la primera lectura válida
        if(distance1_cm > 0) {
            distance_cm = distance1_cm;
            valid_readings++;
        }
        
        // Comparar con sensor 2
        if(distance2_cm > 0) {
            if(valid_readings == 0) {
                distance_cm = distance2_cm;
            } else if(distance2_cm < distance_cm) {
                distance_cm = distance2_cm;
            }
            valid_readings++;
        }
        
        // Comparar con sensor 3
        if(distance3_cm > 0) {
            if(valid_readings == 0) {
                distance_cm = distance3_cm;
            } else if(distance3_cm < distance_cm) {
                distance_cm = distance3_cm;
            }
            valid_readings++;
        }
        
        // Sistema de barras PROGRESIVO
        if(valid_readings > 0 && distance_cm > 0) {
            
            if(distance_cm < 40) {
                gpioWrite(LEDB, ON);
            }
            
            if(distance_cm < 30) {
                gpioWrite(LED1, ON);
            }
            
            if(distance_cm < 20) {
                gpioWrite(LEDB, OFF);
                gpioWrite(LEDG, ON);
                gpioWrite(LED2, ON);
            }
            
            if(distance_cm < 10) {
                gpioWrite(LEDG, OFF);
                gpioWrite(LEDR, ON);
                gpioWrite(LEDG, ON);
                gpioWrite(LED3, ON);
            }
            
            if(distance_cm < 5) {
                // Alerta roja parpadeante
                gpioWrite(LEDG, OFF);
                gpioWrite(LEDR, ON);
                gpioWrite(LED1, ON);
                gpioWrite(LED2, ON);
                gpioWrite(LED3, ON);
                delay(50);
                gpioWrite(LEDR, OFF);
                gpioWrite(LED1, OFF);
                gpioWrite(LED2, OFF);
                gpioWrite(LED3, OFF);
                delay(50);
            } else {
                delay(100);
            }
            
        } else {
            // Sin detección en ningún sensor: LED rojo fijo
            gpioWrite(LEDR, ON);
            delay(100);
        }
    }
}
