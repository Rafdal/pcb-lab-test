#include <Arduino.h>

#include "IMU.hpp"

#include <TimedProcess.h>

#include "serial_flash_spi.hpp"
// BASADO EN EL DRIVER DE UN CHABON
// https://github.com/sparkfun/SparkFun_SPI_SerialFlash_Arduino_Library

#include <RPM_sensors.h>
// #include "gimbal_drv.hpp"

#include "pin_defs.h"	// Definiciones de pines

#include <SPI.h>		// Libreria SPI

#include <NRF52_MBED_TimerInterrupt.h>

NRF52_MBED_TimerInterrupt timerInterrupt;

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines
mbed::DigitalOut *gpio_tip_pa = nullptr;

// Pulsos de Baja Frecuencia
TimedProcessMillis pcen;
TimedProcessMillis gcen;
TimedProcessMillis tip_ag;
TimedProcessMillis tip_pa;
TimedProcessMillis spi_2;
TimedProcessMillis loop_1s;

UART GPS(D2_TX, D3_RX);

inline void togglePin(uint8_t pin) { digitalWrite(pin, !digitalRead(pin)); }

SFE_SPI_FLASH flash;
#define SPI_FREQ 4000000

#include <BARO_ASYNC.h> //libreria para el sensor barometrico LPS22HB

uint8_t baro_prints = 3;

void setup()
{
	Serial.begin(1000000);
	while (!Serial) ;

	delay(500);
	Serial.println("Iniciando...");

	// Serial1.begin(1000000);
	// GPS.begin(1000000);

	// if ( flash.begin(CS_M, SPI_FREQ, SPI, SPI_MODE0) ) Serial.println("Flash inicializado");
	// else Serial.println("Error al inicializar el flash");
	
	// PINES
	// pinMode(PCEN, OUTPUT);
	// pinMode(GCEN, OUTPUT);
	// pinMode(TIP_AG, OUTPUT);
	// pinMode(TIP_PA, OUTPUT);
	// digitalWrite(TIP_AG, LOW);
	// digitalWrite(TIP_PA, LOW);

	mbed::DigitalInOut* gpio = digitalPinToGpio(TIP_PA);
	if (gpio != NULL) {
		delete gpio;
	}
	gpio_tip_pa = new mbed::DigitalOut(digitalPinToPinName(TIP_PA), LOW);
	if(gpio_tip_pa->is_connected())
		Serial.println("TIP_PA conectado");
	else
		Serial.println("TIP_PA no conectado");

	// digitalWrite(PCEN, LOW);
	// digitalWrite(GCEN, LOW);
	
	//INTERRUPT
	RPM_sensors_begin(S1_A1, S2_A2, RPM_SENSOR_MICROS);
	// gimbal_begin(IN1_M, IN2_M, IN3_M, ENA_M);
	
	Serial.println("Setup completo");

	Wire1.setClock(400000);
	Wire1.begin();

	// if (!BARO_ASYNC.begin_default())
	// 	Serial.println("Failed to initialize pressure sensor!");
	if (!BARO_ASYNC.begin_continuous(500))
		Serial.println("Failed to initialize pressure sensor!");

	BARO_ASYNC.dump_registers(Serial);

	BARO_ASYNC.on_error([](uint8_t error){
		Serial.print("BARO_ASYNC Err:");
		Serial.println(error);
	});

	// IMU_begin();

	// pcen.set(720, []() { togglePin(PCEN); });
	// gcen.set(1320, []() { togglePin(GCEN); });
	// tip_ag.set(1630, []() { togglePin(TIP_AG); });
	// tip_pa.set(1180, []() { togglePin(TIP_PA); });

	// SPI.begin();

	// spi_2.set(10, [](){
	// 	SPI.beginTransaction(SPISettings(SPI_FREQ, MSBFIRST, SPI_MODE0));
	// 	digitalWrite(CS_E, LOW);
	// 	uint8_t buffer[4] = {'H', 'E', 'L', 'L'}; // ACA
	// 	SPI.transfer(buffer, 4);    // ACA
	// 	digitalWrite(CS_E, HIGH);
	// 	SPI.endTransaction();
	// });

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
		// GPS.println("GPS");
		// float t = HS300x.readTemperature();
		// RPM_Data_t s1, s2;
		// RPM_sensors_read(&s1, &s2);
		// Serial.print("S2: ");
		// Serial.print(s2.dps);
		// Serial.print("dps, ");
		// Serial.print(s2.rpm);
		// Serial.println("rpm, ");
	});
		

	// if (timerInterrupt.setFrequency(1000, []() {
	// 	gimbal_run();
	// }))
	// Serial.println("Gimbal timer set");
	// else
	// Serial.println("Gimbal timer error");
	
}

void loop()
{
	loop_1s.run();

	// spi_2.run();
	
	// if (BARO_ASYNC.test_continuous_CPU_usage())
		// gpio_tip_pa->write(1);
	BARO_ASYNC.run_continuous();
	// gpio_tip_pa->write(0);

	// 1 - UART x2
	// while (Serial1.available()) Serial.write((uint8_t)Serial1.read());
	// while (GPS.available()) Serial.write((uint8_t)GPS.read());
	
	// 2 - Salidas de baja frecuencia
	// pcen.run();
	// gcen.run();
	// tip_ag.run();
	// tip_pa.run();
	
	// 3 - SPI x2
	// flash.getJEDEC();
	
	// 4 IRQ
	RPM_sensors_run();
	
	// 5 - I2C (sensores) // TODO

	// 6 - PWM para Control FOC
	// gimbal_read();
}

