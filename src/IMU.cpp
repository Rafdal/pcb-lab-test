#include "IMU.hpp"

#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h>

static float xAcc, yAcc, zAcc;
static float xGyro, yGyro, zGyro;
static float xMag, yMag, zMag;

void IMU_begin()
{
    if(!IMU.begin())
    {
        while(1) Serial.println("Error al inicializar el IMU");
    }
    Serial.println("IMU inicializado");

    // IMU.setContinuousMode();
}

void IMU_read()
{
    if (IMU.accelerationAvailable()){
        IMU.readAcceleration(xAcc, yAcc, zAcc);
    }
    if (IMU.gyroscopeAvailable()){
        IMU.readGyroscope(xGyro, yGyro, zGyro);
    }
    if(IMU.magneticFieldAvailable()){
        IMU.readMagneticField(xMag, yMag, zMag);
    }
}