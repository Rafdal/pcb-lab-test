#include <Arduino.h>
//Funciones:
void serialEvent();

// UART por software para GPS en pines 2 (RX) y 3 (TX)
UART GPS(2, 3);

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 segundo

void setup() {
  Serial.begin(115200);  // Consola Serial
  while (!Serial);

  Serial1.begin(115200); // UART física integrada
  GPS.begin(115200);     // UART por software (GPS)

  Serial.println("Iniciando prueba de UART a 115200 baudios...");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial1.print("HOLA_UART1\n");
    GPS.print("HOLA_GPS\n");
  }
  serialEvent();

}

void serialEvent() {
  
 // Lectura desde UART física integrada
  if (Serial1.available()) {
    Serial.println(Serial1.read());
  }
  // Lectura desde UART GPS
  if (GPS.available()) {
    Serial.println(GPS.read());
  }
}