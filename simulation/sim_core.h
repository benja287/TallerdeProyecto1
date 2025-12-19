#ifndef SIM_CORE_H
#define SIM_CORE_H

#include <stdint.h>
#include "mock_os.h"

// --- GLOBALS (Exposed for Inspection) ---
extern volatile uint16_t global_throttle;
extern volatile uint16_t global_brake;
extern volatile int16_t global_joy_x;
extern volatile int16_t global_joy_y;

typedef enum {
    MODE_MANUAL,
    MODE_SAFETY_STOP,
    MODE_AVOIDANCE
} OperationMode;

extern volatile OperationMode currentMode;

// --- TASKS ---
void TaskUARTDecode(void* taskParm);
void TaskMotors(void* taskParm);

#endif
