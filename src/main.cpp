#include <Arduino.h>

#include <TimedProcess.h>

#include "pin_defs.h"	// Definiciones de pines

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines
mbed::DigitalOut *gpio_tip_pa = nullptr;

// Pulsos de Baja Frecuencia
TimedProcessMillis loop_1s;

#include <BARO_ASYNC.h> //libreria para el sensor barometrico LPS22HB

uint8_t baro_prints = 3;

void setup()
{
	Serial.begin(1000000);
	while (!Serial) ;

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


	Wire1.setClock(400000);
	Wire1.begin();

	// if (!BARO_ASYNC.begin_default())
	// 	Serial.println("Failed to initialize pressure sensor!");
	uint8_t attempts = 0;
	while (attempts < 5)
	{
		if (BARO_ASYNC.begin_continuous(500) == 1)
		{
			Serial.println("Pressure sensor initialized!");
			break;
		}
		else
		{
			Serial.println("Failed to initialize pressure sensor! Trying again...");
		}
		attempts++;
		delay(500);
	}
	BARO_ASYNC.dump_registers(Serial);

	BARO_ASYNC.on_error([](uint8_t error){
		Serial.print("BARO_ASYNC Err:");
		Serial.println(error);
	});


	BARO_ASYNC.on_pressure_data_ready([](float p) {
		if (baro_prints > 0)
		{
			baro_prints--;
			Serial.print("P:");
			Serial.println(p, 2);
			Serial.print("Alt:");
			Serial.println(BARO_ASYNC.kpaToMeters(p), 1);
		}
	});

	BARO_ASYNC.on_temp_data_ready([](float t) {
		if (baro_prints > 0)
		{
			Serial.print("T:");
			Serial.println(t, 2);
		}
	});

	loop_1s.set(1000, [](){
		// Serial1.println("Serial1");
	});
}

void loop()
{
	loop_1s.run();
	// if (BARO_ASYNC.test_continuous_CPU_usage())
		// gpio_tip_pa->write(1);
	BARO_ASYNC.run_continuous();
	// gpio_tip_pa->write(0);
}

