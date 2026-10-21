#include <Arduino.h>

#define RXD2 16 // Conectar a UART_TX de la FPGA
#define TXD2 17 // Conectar a UART_RX de la FPGA

const uint8_t HEADER = 0xAA;
const uint8_t FOOTER = 0x55;

void setup() {
  Serial.begin(115200);   // Monitor Serie hacia la PC
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2); // Enlace UART con la FPGA

  delay(2000);
  Serial.println("\n=== ESP32 listo: Comunicacion con FPGA DE10-Lite ===");
}

void send_and_receive(uint32_t n_val) {
  Serial.println("----------------------------------------------");
  Serial.printf("Enviando n = %u\n", n_val);

  // 1. Construir y enviar trama hacia la FPGA
  uint8_t tx_frame[6];
  tx_frame[0] = HEADER;
  tx_frame[1] = (uint8_t)(n_val & 0xFF);
  tx_frame[2] = (uint8_t)((n_val >> 8) & 0xFF);
  tx_frame[3] = (uint8_t)((n_val >> 16) & 0xFF);
  tx_frame[4] = (uint8_t)((n_val >> 24) & 0xFF);
  tx_frame[5] = FOOTER;

  // Limpiar buffer previo y transmitir
  while (Serial2.available()) Serial2.read();
  Serial2.write(tx_frame, 6);

  // 2. Esperar respuesta de la FPGA
  unsigned long start_time = millis();
  while (Serial2.available() < 14) {
    if (millis() - start_time > 1500) {
      Serial.println("[ERROR] Tiempo de espera agotado (Timeout).");
      return;
    }
  }

  // 3. Leer cabecera 0xAA
  uint8_t rx_header = Serial2.read();
  if (rx_header != HEADER) {
    Serial.printf("[ERROR] Cabecera invalida: 0x%02X\n", rx_header);
    return;
  }

  // 4. Leer los 12 bytes del resultado (Q80.16)
  uint8_t payload[12];
  Serial2.readBytes(payload, 12);

  // 5. Leer delimitador de fin 0x55
  uint8_t rx_footer = Serial2.read();
  if (rx_footer != FOOTER) {
    Serial.printf("[ERROR] Delimitador final invalido: 0x%02X\n", rx_footer);
    return;
  }

  // 6. Reconstruir el valor numerico
  // Para valores moderados (n < 65536), los primeros 64 bits cubren el resultado con holgura:
  uint64_t low64 = 0;
  for (int i = 0; i < 8; i++) {
    low64 |= ((uint64_t)payload[i]) << (i * 8);
  }

  // Separar parte entera y parte decimal (Q.16: 16 bits inferiores)
  uint64_t int_part = low64 >> 16;
  uint16_t frac_part = low64 & 0xFFFF;
  double fpga_real = (double)low64 / 65536.0;

  double theoretical = (double)n_val * (double)n_val * sqrt((double)n_val);

  Serial.printf("Resultado FPGA:    %.4f\n", fpga_real);
  Serial.printf("Parte Entera:      %llu\n", int_part);
  Serial.printf("Parte Fraccionaria: %u / 65536 (aprox. 0.%04u)\n", frac_part, (uint32_t)((frac_part * 10000ULL) / 65536ULL));
  Serial.printf("Valor Teorico:     %.4f\n", theoretical);
  Serial.printf("Error Absoluto:    %.6f\n", fabs(fpga_real - theoretical));
}

void loop() {
  uint32_t test_values[] = {10, 50, 85, 144};
  
  for (int i = 0; i < 4; i++) {
    send_and_receive(test_values[i]);
    delay(2000);
  }

  Serial.println("\nCiclo de pruebas completado. Esperando 10 segundos...\n");
  delay(10000);
}
