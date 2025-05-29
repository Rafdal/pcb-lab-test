#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

// Simple magnetometer calibration sketch
// Press the serial monitor's send button (send 'c') to start calibration.
// Rotate the sensor slowly around all axes until calibration completes.

// Calibration parameters (to be computed)
float offsetX = 0, offsetY = 0;
float scaleX = 1, scaleY = 1;

// Raw data extremes
float minX =  1e6, maxX = -1e6;
float minY =  1e6, maxY = -1e6;
bool calibrating = true;
unsigned long calibStart = 0;
const unsigned long calibDuration = 30000; // calibrate for 30 seconds

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialize IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU");
    while (1);
  }
  IMU.setContinuousMode();

  Serial.println("Mag Calibration Ready. Send 'c' to start.");
}

void loop() {
  // Check for start command
  if (!calibrating && Serial.available()) {
    char c = Serial.read();
    if (c == 'c' || c == 'C') {
      calibrating = true;
      calibStart = millis();
      minX = 1e6; maxX = -1e6;
      minY = 1e6; maxY = -1e6;
      Serial.println("Calibration started. Rotate module...");
    }
  }

  // Read magnetometer
  float mx, my, mz;
  if (IMU.readMagneticField(mx, my, mz)) {
    if (calibrating) {
      // Track min/max
      minX = min(minX, mx);
      maxX = max(maxX, mx);
      minY = min(minY, my);
      maxY = max(maxY, my);

      // Check duration
      if (millis() - calibStart >= calibDuration) {
        calibrating = false;
        // Compute offsets and scales
        offsetX = (maxX + minX) / 2.0;
        offsetY = (maxY + minY) / 2.0;
        float rx = (maxX - minX) / 2.0;
        float ry = (maxY - minY) / 2.0;
        float rAvg = (rx + ry) / 2.0;
        scaleX = rAvg / rx;
        scaleY = rAvg / ry;

        // Print results
        Serial.println("Calibration complete.");
        Serial.print("offsetX = "); Serial.println(offsetX, 6);
        Serial.print("offsetY = "); Serial.println(offsetY, 6);
        Serial.print("scaleX  = "); Serial.println(scaleX, 6);
        Serial.print("scaleY  = "); Serial.println(scaleY, 6);
        Serial.println("Use these values in your main sketch.");
      }
    } else {
      // If not calibrating, apply calibration and print heading
      float mx_c = (mx - offsetX) * scaleX;
      float my_c = (my - offsetY) * scaleY;
      float heading = atan2(my_c, mx_c) * 180.0 / M_PI + 90.0; // Adjust for North being 180 degrees
      if (heading < 0) heading += 360.0;
      Serial.print("Heading: ");
      Serial.println(heading, 2);
      delay(200);
    }
  }
}
