#include <stdio.h>
#include "mock_os.h"
#include "sim_core.h"

// Helper to create packet
void inject_packet(uint8_t thr, uint8_t brk, bool corrupt) {
    uint8_t buf[8];
    buf[0] = 0xA5; // Header
    buf[1] = 0;    // JoyX
    buf[2] = 0;    // JoyY
    buf[3] = thr;  // Thr
    buf[4] = brk;  // Brk
    buf[5] = 0;    // Btns
    
    // Checksum
    buf[6] = buf[1]^buf[2]^buf[3]^buf[4]^buf[5];
    
    if (corrupt) {
        buf[6] = ~buf[6]; // Invert checksum -> Fail
    }
    
    buf[7] = 0x5A; // Footer
    
    for(int i=0; i<8; i++) mock_uart_inject(buf[i]);
}

void inject_noise(int bytes) {
    for(int i=0; i<bytes; i++) {
        mock_uart_inject( (uint8_t)(i % 255) );
    }
}

int main() {
    printf("=== SIMULATION START ===\n");
    
    // 1. Initial State
    printf("Time 0ms: Throttle=%d\n", global_throttle);
    
    // 2. Inject Valid Acceleration (100% Throttle = 255)
    printf("--> Injecting VALID Acceleration...\n");
    inject_packet(255, 0, false);
    
    // Run Tasks
    TaskUARTDecode(NULL);
    
    printf("Time 10ms: Throttle=%d (Expected: >0)\n", global_throttle);
    
    if (global_throttle == 0) {
        printf("ERROR: Throttle didnt update!\n");
        return 1;
    }

    // 3. Inject NOISE (The Bug Trigger)
    // The Fix should make the throttle drop to 0 after ~200ms
    
    printf("--> Injecting NOISE/CORRUPT Data (Simulating Interference)...\n");
    
    for(int i=0; i<10; i++) {
        vTaskDelay(50); // 50ms steps
        inject_packet(0, 0, true); // Bad Checksum
        TaskUARTDecode(NULL);
        // printf("Time %dms: Throttle=%d\n", (int)xTaskGetTickCount(), global_throttle);
    }
    
    printf("Time %dms: Throttle=%d (Expected: 0)\n", (int)xTaskGetTickCount(), global_throttle);
    
    // Check if it stopped
    if (global_throttle != 0) {
        printf("\nFAIL: The car did NOT stop!\n");
        return 1;
    } else {
        printf("[SUCCESS] The car stopped safely during noise.\n");
    }

    // 4. TEST RECOVERY
    // The user wants to ensure the car works again immediately when signal returns.
    printf("--> Injecting VALID Data again (Recovery Test)...\n");
    
    vTaskDelay(50);
    inject_packet(200, 0, false); // Valid Acceleration
    TaskUARTDecode(NULL);
    
    printf("Time %dms: Throttle=%d (Expected: 800)\n", (int)xTaskGetTickCount(), global_throttle);
    
    if (global_throttle > 0) {
        printf("\nRESULT: [FIX VERIFIED] (Safety + Recovery OK)\n");
    } else {
        printf("\nFAIL: The car did not recover!\n");
        return 1;
    }
    
    return 0;
}
