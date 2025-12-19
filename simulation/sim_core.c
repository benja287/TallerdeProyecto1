#include "sim_core.h"
#include "mock_os.h"
#include <string.h>

// --- GLOBALS ---
volatile uint16_t global_throttle = 0;
volatile uint16_t global_brake = 0;
volatile int16_t global_joy_x = 0;
volatile int16_t global_joy_y = 0;

volatile OperationMode currentMode = MODE_MANUAL;

// --- CONSTANTS ---
#define BIN_HEADER 0xA5
#define BIN_FOOTER 0x5A
#define RX_BUFFER_SIZE 512

volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile uint16_t rxHead = 0;
volatile uint16_t rxTail = 0;

TaskHandle_t xUARTTaskHandle = (TaskHandle_t)1; // Mock Handle

// --- PROTOCOL STRUCTS ---
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

// --- UART TASK (Logic from app.c) ---
void TaskUARTDecode(void* taskParm) {
    // Shared Buffers (Mocked locals for context)
    static uint8_t state = 0; 
    static uint8_t rawBuf[8]; 
    static uint8_t idx = 0;
    static TickType_t lastValidTime = 0;

    // We simulate ONE LOOP iteration per call for easier testing control, 
    // or we can make it an infinite loop that breaks on empty buffer.
    // For this simulation, we'll process ALL available data then return.
    
    // Simulate Rx Callback filling buffer first (done by test runner)
    // Transfer from Mock OS UART Buffer to Firmware Ring Buffer
    uint8_t byte;
    while (uartReadByte(UART_232, &byte)) {
        uint16_t nextHead = (rxHead + 1) % RX_BUFFER_SIZE;
        if (nextHead != rxTail) { 
            rxBuffer[rxHead] = byte;
            rxHead = nextHead;
        }
    }

    // Now the Main Loop Logic (Reduced to non-blocking for sim)
    // In real RTOS this is while(1). Here we run until buffer empty or processed.
    
    // 1. WATCHDOG CHECK (From app.c)
    if (lastValidTime == 0) lastValidTime = xTaskGetTickCount();
    
    // --- FIX IMPLEMENTATION: TIME-BASED SAFETY ---
    // If we haven't received a VALID packet (Checksum OK) in 200ms, 
    // we MUST stop the car, regardless of whether we are receiving "data" (noise) or not.
    if ( (xTaskGetTickCount() - lastValidTime) > 200 ) {
         // Safety Fallback: Reset Controls
         if (global_throttle != 0) {
             // Only print once to avoid spam in logical analizer/logs
             // printf("[SIM] Safety Triggered: Resetting Controls (Timeout/Noise)\n");
         }
         global_throttle = 0;
         global_brake = 0;
         global_joy_x = 0;
         global_joy_y = 0;
         
         // Note: We do NOT reset the UART driver here (app.c does that at 2000ms),
         // we just want to stop the car safely.
    }

    if ( (xTaskGetTickCount() - lastValidTime) > 2000 ) {
         printf("[SIM] Watchdog Triggered! (No valid packet for 2s)\n");
         // Real code resets UART here.
         lastValidTime = xTaskGetTickCount(); // Reset timer
    }

    // 2. TIMEOUT CHECK
    // In app.c: ulTaskNotifyTake(pdTRUE, 200...).
    // If no data in buffer, we simulate timeout?
    if (rxHead == rxTail) {
        // No new data. Check Mock OS Time
        // Logic: if we haven't received data for 200ms -> Reset.
        // But app.c uses ulTaskNotifyTake.
        // If we call this function, it means we "woke up".
        // If wake up reason was Timeout -> Reset.
        
        // Sim: If buffer empty, we assume timeout event? 
        // Actually, let's keep it simple: The test runner simulates the passage of time.
        return; 
    }
    
    // Process Buffer
    while (rxTail != rxHead) {
        uint8_t byte = rxBuffer[rxTail];
        rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
        
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
                if (idx >= 8) { 
                    if (rawBuf[7] == BIN_FOOTER) {
                        uint8_t calc = rawBuf[1] ^ rawBuf[2] ^ rawBuf[3] ^ rawBuf[4] ^ rawBuf[5];
                        
                        if (calc == rawBuf[6]) {
                            // VALID
                            // printf("[SIM] Packet Valid!\n");
                            lastValidTime = xTaskGetTickCount();
                            
                            struct ControlPacket *pkt = (struct ControlPacket*)rawBuf;
                            global_joy_x = (int16_t)pkt->joyX * 4; 
                            global_joy_y = (int16_t)pkt->joyY * 4;
                            global_throttle = (uint16_t)pkt->throttle * 4; 
                            global_brake    = (uint16_t)pkt->brake * 4;
                            
                            // (Mode switching logic omitted for brevity as it doesn't affect stuck throttle)
                        } else {
                            // printf("[SIM] Checksum Fail!\n");
                            // BUG IS HERE: No reset of globals
                        }
                    }
                    state = 0; 
                }
                break;
        }
    }
}
