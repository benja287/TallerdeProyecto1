#include "mock_os.h"
#include <stdlib.h>
#include <string.h>

// --- SIMULATION STATE ---
static TickType_t currentTick = 0;
#define UART_BUF_SIZE 1024
static uint8_t sUartBuf[UART_BUF_SIZE];
static int sUartHead = 0;
static int sUartTail = 0;

// --- TIME ---
void vTaskDelay(uint32_t ticks) {
    currentTick += ticks;
}

TickType_t xTaskGetTickCount(void) {
    return currentTick;
}

// --- NOTIFICATIONS ---
uint32_t ulTaskNotifyTake(bool clearCount, uint32_t waitTicks) {
    // SIMULATION LOGIC:
    // If we have data in UART, return immediately (Task Woken)
    // If buffer empty, advance time by waitTicks and return 0 (Timeout)
    
    if (sUartHead != sUartTail) {
        return 1; // Notified
    } else {
        currentTick += waitTicks;
        return 0; // Timeout
    }
}

// --- UART MOCK ---
void mock_uart_inject(uint8_t byte) {
    sUartBuf[sUartHead] = byte;
    sUartHead = (sUartHead + 1) % UART_BUF_SIZE;
}

bool uartReadByte(int uartID, uint8_t* outByte) {
    if (sUartHead == sUartTail) return false;
    
    *outByte = sUartBuf[sUartTail];
    sUartTail = (sUartTail + 1) % UART_BUF_SIZE;
    return true;
}

// --- GPIO/PWM MOCK ---
void gpioWrite(int pin, int value) {
    // No-op for logic test
}

void pwmWrite(int pin, uint8_t value) {
    // No-op
}
void gpioConfig(int pin, int dir) {
    // No-op
}
