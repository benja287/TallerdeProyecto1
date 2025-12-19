#ifndef MOCK_OS_H
#define MOCK_OS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

// --- MOCK FREERTOS TYPES ---
typedef uint32_t TickType_t;
typedef void* TaskHandle_t;
#define pdTRUE 1
#define pdFALSE 0
#define portTICK_RATE_MS 1

// --- MOCK HARDWARE TYPES ---
#define UART_232 0
#define GPIO_OUTPUT 1
#define ON 1
#define OFF 0

// --- MOCK FUNCTIONS ---
void vTaskDelay(uint32_t ticks);
TickType_t xTaskGetTickCount(void);
uint32_t ulTaskNotifyTake(bool clearCount, uint32_t waitTicks);

// Helper for Mocking UART Data Injection
void mock_uart_inject(uint8_t byte);
bool uartReadByte(int uartID, uint8_t* outByte);

// Helper for Inspection
void gpioWrite(int pin, int value);
void pwmWrite(int pin, uint8_t value);
void gpioConfig(int pin, int dir);

#endif // MOCK_OS_H
