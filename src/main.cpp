#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>
#include <math.h>

#include <TimedProcess.h>
#include <mbed.h>
#include <pinDefinitions.h>
#include "pin_defs.h"

mbed::DigitalOut *gpio_test = nullptr;
TimedProcessMillis loop_1s;

static float xMag, yMag;

float offsetX = -38.500000;
float offsetY = 23.500000;


float computeHeading(float x, float y) {
  float heading = atan2(y-offsetY, x-offsetX)*180.0 / M_PI + 90; // In radians
        if (heading < 0) heading += 360.0;
  return heading;
}

void setup() {
  Serial.begin(1000000);

  gpio_test = new mbed::DigitalOut(digitalPinToPinName(CS_E), LOW);
  Wire1.setClock(400000);
  Wire1.begin();

  IMU.setContinuousMode();
  if (!IMU.begin()) {
    while (1) Serial.println("Error initializing IMU");
  }

  loop_1s.set(10, [](){
    gpio_test->write(1);

    float heading = 0;
    float mx, my, mz;

    if (IMU.readMagneticField(mx, my, mz)) {
      heading = computeHeading(mx, my);
    }

    gpio_test->write(0);

    uint8_t tx_buffer[16];
    tx_buffer[0] = 0xFF;
    tx_buffer[1] = 0x00;

    memcpy(&tx_buffer[2], &heading, sizeof(heading));
    Serial.write(tx_buffer, sizeof(tx_buffer));
  });
}

void loop() {
  loop_1s.run();
}
