#include "magnetometerCalib.h"
#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

static float offsetX = 0, offsetY = 0;
static float scaleX = 1, scaleY = 1;

float getOffsetX() { return offsetX; }
float getOffsetY() { return offsetY; }
float getScaleX()  { return scaleX; }
float getScaleY()  { return scaleY; }

void Calibrar() {
    float minX =  1e6, maxX = -1e6;
    float minY =  1e6, maxY = -1e6;
    const unsigned long calibDuration = 30000; // 30 segundos
    unsigned long calibStart = millis();

    while (millis() - calibStart < calibDuration) {
        float mx, my, mz;
        if (IMU.readMagneticField(mx, my, mz)) {
            minX = min(minX, mx);
            maxX = max(maxX, mx);
            minY = min(minY, my);
            maxY = max(maxY, my);
        }
        delay(10); // Pequeño delay para no saturar el bus
    }

    offsetX = (maxX + minX) / 2.0;
    offsetY = (maxY + minY) / 2.0;
    float rx = (maxX - minX) / 2.0;
    float ry = (maxY - minY) / 2.0;
    float rAvg = (rx + ry) / 2.0;
    scaleX = rAvg / rx;
    scaleY = rAvg / ry;

    // Imprime en una sola línea los resultados de calibración
    Serial.print("offsetX: ");
    Serial.print(offsetX, 6);
    Serial.print(", offsetY: ");
    Serial.print(offsetY, 6);
    Serial.print(", scaleX: ");
    Serial.print(scaleX, 6);
    Serial.print(", scaleY: ");
    Serial.println(scaleY, 6);
}
