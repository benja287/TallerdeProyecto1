#include <Bluepad32.h>

// --- CONFIGURACION DE PINES ---
#define UART_TX 17
#define UART_RX 16

// CONFIGURACION DE DEBUG
#define ENABLE_LOGS        0  // 0 = Apagar logs (Rendimiento maximo)
#define ENABLE_HAPTIC_TEST 1  // 1 = Habilitar test de Cuadrado

// --- PROTOCOLO BINARIO ---
// Header y Footer para sincronizacion
#define BIN_HEADER 0xA5
#define BIN_FOOTER 0x5A

// Comandos de Feedback (CIAA -> ESP32)
#define FB_CMD_RUMBLE  0x01
#define FB_CMD_LED     0x02
#define FB_CMD_HEARTBEAT 0x03

struct ControlPacket {
    uint8_t header;    // 0xA5
    int8_t  joyX;      // -127 a 127
    int8_t  joyY;      // -127 a 127
    uint8_t throttle;  // 0-255
    uint8_t brake;     // 0-255
    uint8_t buttons;   // Bitmask
    uint8_t checksum;  // XOR
    uint8_t footer;    // 0x5A
};

// Variable para control (solo 1 jugador soportado para el coche)
ControllerPtr myController = nullptr;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// --- CALLBACKS DE CONEXION ---

void onConnectedController(ControllerPtr ctl) {
    if (myController == nullptr) {
        Serial.printf("CALLBACK: Controller connected idx=%d\n", ctl->index());
        // Establecer LED Blanco inicial
        ctl->setColorLED(255, 255, 255);
        myController = ctl;
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    if (myController == ctl) {
        Serial.printf("CALLBACK: Controller disconnected idx=%d\n", ctl->index());
        myController = nullptr;
    }
}

// --- LOGICA DE ENVIO (TX) ---

void sendBinaryPacket(ControllerPtr ctl) {
    if (!ctl) return;

    struct ControlPacket pkt;
    pkt.header = BIN_HEADER;

    // 1. Mapear Joysticks (-512..512 -> -127..127)
    // Invertir Y si es necesario dependiendo de orientacion
    int32_t rawX = ctl->axisX();
    int32_t rawY = ctl->axisY();

    pkt.joyX = (int8_t)(rawX * 127 / 512);
    pkt.joyY = (int8_t)(rawY * 127 / 512);

    // 2. Mapear Gatillos (0..1023 -> 0..255)
    pkt.throttle = (uint8_t)(ctl->throttle() / 4);
    pkt.brake    = (uint8_t)(ctl->brake() / 4);

    // 3. Mapear Botones (Bitmask)
    // Bit 0: X (Cross)
    // Bit 1: Circulo
    // Bit 2: Cuadrado
    // Bit 3: Triangulo
    // Bit 4: L1
    // Bit 5: R1
    // Bit 6: Select/Share
    // Bit 7: Start/Options
    pkt.buttons = 0;
    if (ctl->a()) pkt.buttons |= 0x01; // Cross
    if (ctl->b()) pkt.buttons |= 0x02; // Circle
    if (ctl->x()) pkt.buttons |= 0x04; // Square
    if (ctl->y()) pkt.buttons |= 0x08; // Triangle
    if (ctl->l1()) pkt.buttons |= 0x10;
    if (ctl->r1()) pkt.buttons |= 0x20;
    if (ctl->miscButtons() & 0x01) pkt.buttons |= 0x40; // Share?
    if (ctl->miscButtons() & 0x02) pkt.buttons |= 0x80; // Options?

    // 4. Checksum (XOR Simple de payload)
    pkt.checksum = pkt.joyX ^ pkt.joyY ^ pkt.throttle ^ pkt.brake ^ pkt.buttons;
    
    pkt.footer = BIN_FOOTER;

    // 5. Enviar por Serial2
    Serial2.write((uint8_t*)&pkt, sizeof(pkt));
}

// --- LOGICA DE RECEPCION (RX - Feedback) ---

// Estado de recepcion
enum RxState { WAIT_HEADER, READ_CMD, READ_VAL1, READ_VAL2, READ_VAL3, WAIT_FOOTER };
RxState rxState = WAIT_HEADER;
uint8_t fbCmd, fbVal1, fbVal2, fbVal3;

void processFeedback() {
    while (Serial2.available()) {
        uint8_t byte = Serial2.read();

        switch (rxState) {
            case WAIT_HEADER:
                if (byte == 0x5B) rxState = READ_CMD;
                break;
            
            case READ_CMD:
                fbCmd = byte;
                rxState = READ_VAL1;
                break;

            case READ_VAL1:
                fbVal1 = byte;
                rxState = READ_VAL2;
                break;

            case READ_VAL2:
                fbVal2 = byte;
                rxState = READ_VAL3;
                break;

            case READ_VAL3:
                fbVal3 = byte;
                rxState = WAIT_FOOTER;
                break;

            case WAIT_FOOTER:
                if (byte == 0xB5) {
                    // PAQUETE VALIDO -> EJECUTAR ACCION
                    if (myController && myController->isConnected()) {
                        
                        if (fbCmd == FB_CMD_RUMBLE) {
                            // Val1: Duracion (x10ms), Val2: Fuerza (0-255)
                            // playDualRumble(delay, duration, weak, strong)
                            myController->playDualRumble(0, fbVal1 * 10, fbVal2 / 2, fbVal2);
                        
                        } else if (fbCmd == FB_CMD_LED) {
                            // Val1: R, Val2: G, Val3: B
                            myController->setColorLED(fbVal1, fbVal2, fbVal3);
                        }
                    }
                }
                rxState = WAIT_HEADER; // Reset siempre
                break;
        }
    }
}


// --- ARDUINO SETUP & LOOP ---

void setup() {
    Serial.begin(115200);
    // UART para CIAA (Tx=17, Rx=16) a 115200
    Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);

    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    
    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
    BP32.enableVirtualDevice(false);
}

void loop() {
    // 1. Actualizar estado de control
    bool dataUpdated = BP32.update();

    if (dataUpdated && myController && myController->isConnected()) {
        // 2. Enviar paquete binario al coche
        sendBinaryPacket(myController);

#if ENABLE_HAPTIC_TEST
        // --- TEST DE HAPTICOS (Cuadrado) ---
        if (myController->x()) {
             myController->playDualRumble(0, 200, 100, 255);
             Serial.println(">> TEST HAPTICO: Cuadrado! <<");
        }
#endif
        
#if ENABLE_LOGS
        // --- DEBUG LOGGING ---
        static int debugTimer = 0;
        if (millis() - debugTimer > 200) { // Cada 200ms
             debugTimer = millis();
             Serial.printf("JoyL: %4d,%4d | JoyR: %4d,%4d | Thr: %4d | Brk: %4d | Btn: ", 
                           myController->axisX(), myController->axisY(), 
                           myController->axisRX(), myController->axisRY(),
                           myController->throttle(), myController->brake());
                           
             if(myController->a()) Serial.print("X ");
             if(myController->b()) Serial.print("O ");
             if(myController->x()) Serial.print("[] ");
             if(myController->y()) Serial.print("^ ");
             if(myController->l1()) Serial.print("L1 ");
             if(myController->r1()) Serial.print("R1 ");
             Serial.println();
        }
#endif
    }

    // 3. Procesar Feedback desde el coche (RX)
    processFeedback();

    // 4. Delay para mantener 50Hz aprox (20ms)
    // El BP32.update toma algo de tiempo, ajustamos delay.
    delay(15); 
}
