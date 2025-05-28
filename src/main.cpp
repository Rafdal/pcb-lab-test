#include <Arduino.h>

#include <TimedProcess.h>

#include "pin_defs.h"	// Definiciones de pines

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines
mbed::DigitalOut *gpio_test = nullptr;

TimedProcessMillis loop_1s;


#include <Arduino_BMI270_BMM150.h>

static float xAcc, yAcc, zAcc;
static float xGyro, yGyro, zGyro;
static float xMag, yMag, zMag;

static float zGyro_acc = 0;

void setup()
{
	Serial.begin(1000000);

	mbed::DigitalInOut* gpio = digitalPinToGpio(CS_E);
	if (gpio != NULL) {
		delete gpio;
	}
	gpio_test = new mbed::DigitalOut(digitalPinToPinName(CS_E), LOW);

	auto imu_int1 = digitalPinToGpio(CS_E);
	// if(gpio_test->is_connected())
	// 	Serial.println("CS_E conectado");
	// else
	// 	Serial.println("CS_E no conectado");

	// Serial.println("Setup completo");

	Wire1.setClock(400000);
	Wire1.begin();

	IMU.setContinuousMode();
	if(!IMU.begin())
    {
        while(1) Serial.println("Error al inicializar el IMU");
    }
	IMU.setContinuousMode();
    // Serial.println("IMU inicializado");

	loop_1s.set(10, [](){
		// if (IMU.accelerationAvailable()){
        // IMU.readAcceleration(xAcc, yAcc, zAcc);
		// }
		gpio_test->write(1);
		// if (IMU.gyroscopeAvailable()){
		IMU.readGyroscope(xGyro, yGyro, zGyro);
		// }
		gpio_test->write(0);

		uint8_t tx_buffer[16];
		tx_buffer[0] = 0xFF; // Start byte
		tx_buffer[1] = 0x00;
		// memcpy(&tx_buffer[2], &xGyro, sizeof(xGyro));
		// memcpy(&tx_buffer[6], &yGyro, sizeof(yGyro));
		// memcpy(&tx_buffer[10], &zGyro, sizeof(zGyro));
		zGyro_acc += zGyro * 0.01; // Integrate gyroscope data for 10 ms intervals
		memcpy(&tx_buffer[2], &zGyro_acc, sizeof(zGyro_acc));
		Serial.write(tx_buffer, sizeof(tx_buffer));
		// Serial.print('#');
		// Serial.print(xGyro, 2);
		// Serial.print(',');
		// Serial.print(yGyro, 2);
		// Serial.print(',');
		// Serial.println(zGyro, 2);
		// if(IMU.magneticFieldAvailable()){
		// 	IMU.readMagneticField(xMag, yMag, zMag);
		// }
	});
}

void loop()
{
	loop_1s.run();
}

