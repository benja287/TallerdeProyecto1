#ifndef TEST_HARDWARE_H
#define TEST_HARDWARE_H
#include "sapi.h"
#include "FreeRTOS.h"
#include "task.h"

// Feature Flag para activar/desactivar el test
#define ENABLE_HARDWARE_TEST

// Prototipo de la tarea de test
void TaskHardwareTest(void* taskParm);

#endif
