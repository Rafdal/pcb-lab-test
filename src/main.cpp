#include <Arduino.h>

#include <TimedProcess.h>

#include "gimbal_drv.hpp"

#include "pin_defs.h"	// Definiciones de pines

#include <NRF52_MBED_TimerInterrupt.h>

NRF52_MBED_TimerInterrupt timerInterrupt;

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines
mbed::DigitalOut *gpio_tip_pa = nullptr;

TimedProcessMillis loop_1s;

#include <SPI.h>
SPISettings mySPISettings(500000, MSBFIRST, SPI_MODE1); // 500kHz, MSB first, mode 1
MagneticSensorAS5048A encoder(CS_E, true, mySPISettings);

void setup()
{
	Serial.begin(1000000);
	while (!Serial);

	delay(500);
	Serial.println("Iniciando...");

	mbed::DigitalInOut* gpio = digitalPinToGpio(TIP_PA);
	if (gpio != NULL) {
		delete gpio;
	}
	gpio_tip_pa = new mbed::DigitalOut(digitalPinToPinName(TIP_PA), LOW);
	if(gpio_tip_pa->is_connected())
		Serial.println("TIP_PA conectado");
	else
		Serial.println("TIP_PA no conectado");

	gimbal_begin(IN1_M, IN2_M, IN3_M, ENA_M);

	encoder.init();

	if (timerInterrupt.setFrequency(1000, []() { gimbal_run(); }))
		Serial.println("Gimbal timer set");
	else
		Serial.println("Gimbal timer error");
	
	loop_1s.set(100, []() {
		// Serial.println("Loop 1s");
		encoder.update();
		if(encoder.isErrorFlag())
			Serial.print("aE ");
		else
			Serial.print("aO ");
		Serial.println(encoder.getCurrentAngle(),2);
	});
}

void loop()
{
	loop_1s.run();
	// gpio_tip_pa->write(1);
	// gpio_tip_pa->write(0);
	
	gimbal_read();
}

