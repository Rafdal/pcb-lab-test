#include <Arduino.h>

#include "pin_defs.h"
#include <mbed.h>
#include <pinDefinitions.h>
#include <TinyGPS++.h>


mbed::DigitalOut *gpio_pin = nullptr;   // gpio pin for time measurement purposes

UART SerialGPS(D2_TX, D3_RX);
TinyGPSPlus gps;

void setup() {
  Serial.begin(115200);          // USB serial for debugging
  SerialGPS.begin(9600);       // Custom UART for GPS on D2 (TX) and D3 (RX)
	while (!Serial);
	//while(!SerialGPS);

  
	Serial.println("Initializing GPS on D2 (TX), D3 (RX)...");
	delay(500); // Wait for GPS to initialize

  // Check if the GPS is connected
  if (SerialGPS) {
	Serial.println("GPS initialized successfully.");
  } else {
	Serial.println("Failed to initialize GPS.");
  }

}


void loop() {
  // Read from GPS UART and feed to TinyGPS++
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }

  // Print location info when available
  if (gps.satellites.value() != 0) {
    String data = "Latitude: " + String(gps.location.lat(), 6)
                + ", Longitude: " + String(gps.location.lng(), 6)
                + ", Altitude: " + String(gps.altitude.meters(), 2) + "m"
                + ", Satellites: " + String(gps.satellites.value())
                + ", HDOP: " + String(gps.hdop.hdop());

    Serial.println(data);
  }

  if (millis() > 5000 && gps.satellites.value() == 0) {
    Serial.println("Waiting for satellite fix...");
  }
}
