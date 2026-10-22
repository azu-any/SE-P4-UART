#include <Arduino.h>

#define RXD2 16 // Conectar a UART_TX de la FPGA (PIN_W10 o PIN_AB6)
#define TXD2 17 // Conectar a UART_RX de la FPGA (PIN_V10 o PIN_AB5)
#define LED_PIN 2 // LED integrado del ESP32 para diagnostico visual

const uint8_t HEADER = 0xAA;
const uint8_t FOOTER = 0x55;

const size_t MATLAB_FRAME_LEN = 6;  // [0xAA, n0, n1, n2, n3, 0x55]
const size_t FPGA_FRAME_LEN   = 14; // [0xAA, b0..b11 (Q80.16), 0x55]

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // IMPORTANTE: Ambos a 9600 baudios para coincidir con MATLAB (BAUD = 9600) y la FPGA
  Serial.begin(9600);                          // USB hacia MATLAB (PC)
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // UART fisico hacia la FPGA DE10-Lite

  // Limpiar buffers iniciales tras el arranque
  delay(500);
  while (Serial.available())  Serial.read();
  while (Serial2.available()) Serial2.read();
}

void loop() {
  // 1. Esperar solicitud de MATLAB (6 bytes comenzando con 0xAA)
  if (Serial.available() > 0) {
    // Sincronizar con la cabecera 0xAA enviada por MATLAB
    uint8_t b = Serial.read();
    if (b == HEADER) {
      uint8_t tx_frame[MATLAB_FRAME_LEN];
      tx_frame[0] = HEADER;

      // Leer los 5 bytes restantes (4 de 'n' + 1 de footer 0x55)
      size_t count = Serial.readBytes(&tx_frame[1], MATLAB_FRAME_LEN - 1);

      if (count == (MATLAB_FRAME_LEN - 1) && tx_frame[5] == FOOTER) {
        // Limpiar cualquier basura previa en el receptor de la FPGA
        while (Serial2.available()) Serial2.read();

        // Enviar la trama de 6 bytes integra hacia la FPGA
        Serial2.write(tx_frame, MATLAB_FRAME_LEN);
        digitalWrite(LED_PIN, HIGH); // Encender LED: esperando respuesta de FPGA

        // 2. Esperar los 14 bytes de respuesta de la FPGA (timeout de 1000 ms)
        unsigned long t_start = millis();
        bool header_found = false;

        while (millis() - t_start < 1000) {
          if (Serial2.available() > 0) {
            uint8_t fpga_byte = Serial2.read();
            if (fpga_byte == HEADER) {
              header_found = true;
              break;
            }
          }
        }

        if (header_found) {
          uint8_t rx_frame[FPGA_FRAME_LEN];
          rx_frame[0] = HEADER;

          // Leer los 13 bytes restantes (12 de payload Q80.16 + 1 de footer 0x55)
          size_t fpga_count = Serial2.readBytes(&rx_frame[1], FPGA_FRAME_LEN - 1);

          if (fpga_count == (FPGA_FRAME_LEN - 1)) {
            // Reenviar los 14 bytes binarios crudos directamente a MATLAB
            Serial.write(rx_frame, FPGA_FRAME_LEN);
          }
        }

        digitalWrite(LED_PIN, LOW); // Apagar LED: transaccion terminada
      }
    }
  }
}
