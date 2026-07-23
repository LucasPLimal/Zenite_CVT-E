#include <SoftwareSerial.h>

// ============================================================================
//                          CONFIGURAÇÃO DO BLUETOOTH
// ============================================================================

// Cria uma porta serial por software para o módulo Bluetooth (HC-06/HC-05).
// pino 11 = RX (entrada do Arduino)
// pino 10 = TX (saída do Arduino)
SoftwareSerial bt(11, 10);


// ============================================================================
//                          PINOS DO DRIVER L298N
// ============================================================================

// Motor esquerdo
const int IN1 = 7;      // direção esquerda IN1
const int IN2 = 8;      // direção esquerda IN2
const int ENA = 5;      // PWM do motor esquerdo

// Motor direito
const int IN3 = 4;      // direção direita IN3
const int IN4 = 3;      // direção direita IN4
const int ENB = 6;      // PWM do motor direito


// ============================================================================
//                          CONSTANTES DE CONTROLE
// ============================================================================

// Mínimo valor PWM para que o motor realmente comece a girar (evita “zona morta”)
const uint8_t MIN_PWM_TO_MOVE = 30;

// Tempo máximo sem receber pacotes (ms) → ativa failsafe e para o robô
const unsigned long FAILSAFE_TIMEOUT = 500;

// Tempo máximo permitido entre bytes de um pacote (ms) → previne pacotes travados
const unsigned long PACKET_TIMEOUT = 20;

// Intervalo entre mensagens de debug no monitor serial
const unsigned long PRINT_INTERVAL = 500;

// Permite inverter um motor caso esteja montado ao contrário
bool invertLeft = false;
bool invertRight = false;


// ============================================================================
//                       VARIÁVEIS DE ESTADO DO SISTEMA
// ============================================================================

// Controle de tempo para failsafe e timeout
unsigned long lastPacketTime = 0;
unsigned long lastByteTime = 0;
unsigned long lastPrintTime = 0;

// Buffer para receber pacotes do Bluetooth (3 bytes)
uint8_t buf[3];
uint8_t idx = 0;

// PWM efetivamente aplicado aos motores
uint8_t appliedLeft = 0;
uint8_t appliedRight = 0;


// ============================================================================
//                                   SETUP
// ============================================================================

void setup() {

    Serial.begin(115200);   // Debug
    bt.begin(9600);         // Porta Bluetooth

    // Configura todos os pinos do driver como saída
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

    // Alimentação do módulo Bluetooth quando ligado ao Arduino
    pinMode(13, OUTPUT);  digitalWrite(13, HIGH);
    pinMode(12, OUTPUT);  digitalWrite(12, LOW);

    // Garante que o robô comece parado
    stopMotors();

    Serial.println("✅ Firmware pronto. Aguardando pacotes (DIR, PWM_L, PWM_R)");
}


// ============================================================================
//                             LOOP PRINCIPAL
// ============================================================================

void loop() {

    // =========================================================================
    //                   LEITURA NÃO BLOQUEANTE VIA BLUETOOTH
    // =========================================================================
    while (bt.available()) {

        uint8_t b = bt.read();    // Lê byte
        buf[idx++] = b;           // Armazena no buffer
        lastByteTime = millis();  // Marca horário

        // Quando recebe os 3 bytes → processa pacote
        if (idx == 3) {
            lastPacketTime = millis();
            processPacket(buf[0], buf[1], buf[2]);
            idx = 0;
        }
    }

    // Se os bytes demorarem demais → descarta pacote incompleto
    if (idx > 0 && (millis() - lastByteTime) > PACKET_TIMEOUT) {
        idx = 0;  // Reseta leitura
    }


    // =========================================================================
    //                                   FAILSAFE
    // =========================================================================

    // Se o robô ficou muito tempo sem receber comandos → parar motores
    if ((millis() - lastPacketTime) > FAILSAFE_TIMEOUT) {
        stopMotors();
    }


    // =========================================================================
    //                         DEBUG PERIÓDICO VIA SERIAL
    // =========================================================================
    if ((millis() - lastPrintTime) > PRINT_INTERVAL) {
        Serial.print("⏱ uptime: ");
        Serial.print(millis());
        Serial.print(" | lastPkt(ms): ");
        Serial.println(millis() - lastPacketTime);
        lastPrintTime = millis();
    }
}



// ============================================================================
//                      PROCESSAMENTO DO PACOTE RECEBIDO
// ============================================================================
//
// Pacote vindo do TransmissionNode (ROS):
//   [0] = direção (bitmap)
//   [1] = PWM do motor esquerdo  (0–255)
//   [2] = PWM do motor direito   (0–255)
//
// Direção (bits):
//   bit0 → motor direito positivo
//   bit1 → motor esquerdo positivo
// ============================================================================

void processPacket(uint8_t direction, uint8_t pwmL_in, uint8_t pwmR_in) {

    // PWM já vem convertido para 0–255
    uint8_t left = pwmL_in;
    uint8_t right = pwmR_in;

    // Impede PWM muito baixo (motor não se mexe)
    if (left > 0 && left < MIN_PWM_TO_MOVE)  left = 0;
    if (right > 0 && right < MIN_PWM_TO_MOVE) right = 0;

    // Log recebido
    Serial.print("📦 RX DIR=0x");
    Serial.print(direction, HEX);
    Serial.print(" | L=");
    Serial.print(left);
    Serial.print(" | R=");
    Serial.println(right);

    // Aplica no driver L298N
    applyMovement(direction, left, right);
}



// ============================================================================
//                       CONTROLE DOS MOTORES NO L298N
// ============================================================================

void applyMovement(uint8_t direction, uint8_t left, uint8_t right) {

    // Decodifica bits de direção
    bool right_forward = (direction & 0x01);
    bool left_forward  = (direction & 0x02);

    // Corrige direção caso motor esteja fisicamente invertido
    if (invertRight) right_forward = !right_forward;
    if (invertLeft)  left_forward  = !left_forward;

    // Se ambos PWM forem zero → para tudo
    if (left == 0 && right == 0) {
        stopMotors();
        return;
    }

    // -------------------------------------------------------------------------
    //                          MOTOR ESQUERDO
    // -------------------------------------------------------------------------
    if (left == 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        analogWrite(ENA, 0);
    } else {
        if (left_forward) {
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
        } else {
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
        }
        analogWrite(ENA, left);
    }

    // -------------------------------------------------------------------------
    //                          MOTOR DIREITO
    // -------------------------------------------------------------------------
    if (right == 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        analogWrite(ENB, 0);
    } else {
        if (right_forward) {
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
        } else {
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
        }
        analogWrite(ENB, right);
    }

    // Guarda PWM aplicado
    appliedLeft = left;
    appliedRight = right;
}



// ============================================================================
//                               PARAR MOTORES
// ============================================================================
// Força zero PWM e zero direção — robô totalmente parado.
// ============================================================================

void stopMotors() {

    analogWrite(ENA, 0);
    analogWrite(ENB, 0);

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}
