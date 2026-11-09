#include "BluetoothSerial.h"

// ============================================================================
//                    CONFIGURAÇÃO DO BLUETOOTH
// ============================================================================

BluetoothSerial bt;


// ============================================================================
//                       PINOS DO DRIVER L298N
// ============================================================================

// Motor esquerdo
const int IN1 = 32;
const int IN2 = 33;
const int ENA = 27;

// Motor direito
const int IN3 = 25;
const int IN4 = 26;
const int ENB = 14;


// ============================================================================
//                       CONFIGURAÇÃO DO PWM
// ============================================================================

const int PWM_FREQ = 1000;
const int PWM_RESOLUTION = 8;


// ============================================================================
//                       CONSTANTES DE CONTROLE
// ============================================================================

const uint8_t MIN_PWM_TO_MOVE = 30;

const unsigned long FAILSAFE_TIMEOUT = 500;
const unsigned long PACKET_TIMEOUT = 20;
const unsigned long PRINT_INTERVAL = 500;

bool invertLeft = false;
bool invertRight = false;


// ============================================================================
//                       VARIÁVEIS DE ESTADO
// ============================================================================

unsigned long lastPacketTime = 0;
unsigned long lastByteTime = 0;
unsigned long lastPrintTime = 0;

uint8_t buf[3];
uint8_t idx = 0;

uint8_t appliedLeft = 0;
uint8_t appliedRight = 0;


// ============================================================================
//                                  SETUP
// ============================================================================

void setup() {

    Serial.begin(115200);

    // Inicializa Bluetooth Classic
    if (!bt.begin("zenitinho")) {
        Serial.println("Erro ao iniciar Bluetooth!");

        while (true) {
            delay(1000);
        }
    }

    Serial.println("Bluetooth iniciado!");
    Serial.println("Dispositivo: zenitinho");

    // Configura pinos de direção
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    // Configura PWM dos motores
    ledcAttach(ENA, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(ENB, PWM_FREQ, PWM_RESOLUTION);

    // Garante que os motores iniciem parados
    stopMotors();

    lastPacketTime = millis();

    Serial.println("Firmware pronto!");
    Serial.println("Aguardando pacotes (DIR, PWM_L, PWM_R)");
}


// ============================================================================
//                              LOOP PRINCIPAL
// ============================================================================

void loop() {

    // =========================================================================
    //                   LEITURA NÃO BLOQUEANTE VIA BLUETOOTH
    // =========================================================================

    while (bt.available()) {

        uint8_t b = bt.read();

        buf[idx++] = b;

        lastByteTime = millis();

        // Quando recebe os 3 bytes, processa o pacote
        if (idx == 3) {

            lastPacketTime = millis();

            processPacket(buf[0], buf[1], buf[2]);

            idx = 0;
        }
    }

    // Descarta pacote incompleto
    if (idx > 0 && (millis() - lastByteTime) > PACKET_TIMEOUT) {
        idx = 0;
    }


    // =========================================================================
    //                                  FAILSAFE
    // =========================================================================

    if ((millis() - lastPacketTime) > FAILSAFE_TIMEOUT) {
        stopMotors();
    }


    // =========================================================================
    //                         DEBUG VIA SERIAL
    // =========================================================================

    if ((millis() - lastPrintTime) > PRINT_INTERVAL) {

        Serial.print("Uptime: ");
        Serial.print(millis());

        Serial.print(" | LastPkt(ms): ");
        Serial.println(millis() - lastPacketTime);

        lastPrintTime = millis();
    }
}


// ============================================================================
//                      PROCESSAMENTO DO PACOTE RECEBIDO
// ============================================================================

void processPacket(uint8_t direction, uint8_t pwmL_in, uint8_t pwmR_in) {

    uint8_t left = pwmL_in;
    uint8_t right = pwmR_in;

    // Impede PWM muito baixo
    if (left > 0 && left < MIN_PWM_TO_MOVE) {
        left = 0;
    }

    if (right > 0 && right < MIN_PWM_TO_MOVE) {
        right = 0;
    }

    // Imprime pacote recebido
    Serial.print("RX DIR=0x");
    Serial.print(direction, HEX);

    Serial.print(" | L=");
    Serial.print(left);

    Serial.print(" | R=");
    Serial.println(right);

    applyMovement(direction, left, right);
}


// ============================================================================
//                       CONTROLE DOS MOTORES NO L298N
// ============================================================================

void applyMovement(uint8_t direction, uint8_t left, uint8_t right) {

    // Decodifica bits de direção
    bool right_forward = direction & 0x01;
    bool left_forward  = direction & 0x02;

    // Corrige direção caso motor esteja invertido
    if (invertRight) right_forward = !right_forward;
    if (invertLeft) left_forward = !left_forward;

    // Se ambos PWM forem zero, para os motores
    if (left == 0 && right == 0) {
        stopMotors();
        return;
    }

    // ------------------------------------------------------------------------
    // MOTOR ESQUERDO
    // ------------------------------------------------------------------------

    if (left == 0) {

        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);

        ledcWrite(ENA, 0);

    } else {

        if (left_forward) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }

        ledcWrite(ENA, left);
    }


    // ------------------------------------------------------------------------
    // MOTOR DIREITO
    // ------------------------------------------------------------------------

    if (right == 0) {

        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);

        ledcWrite(ENB, 0);

    } else {

        if (right_forward) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }

        ledcWrite(ENB, right);
    }

    // Guarda PWM aplicado
    appliedLeft = left;
    appliedRight = right;
}


// ============================================================================
//                               PARAR MOTORES
// ============================================================================

void stopMotors() {

    ledcWrite(ENA, 0);
    ledcWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);

    appliedLeft = 0;
    appliedRight = 0;
}
