#include <Arduino.h>

#include <Arduino_LPS22HB.h> //libreria para el sensor barometrico LPS22HB
#include <Arduino_HS300x.h>	 //libreria para el sensor de temperatura y humedad HS300x

#include <TimedProcess.h>

#include "serial_flash_spi.hpp"
//BASADO EN EL DRIVER DE UN CHABON
// https://github.com/sparkfun/SparkFun_SPI_SerialFlash_Arduino_Library

#include "rpm_sensors.hpp"
#include "gimbal_drv.hpp"

#include "pin_defs.h"

// Pulsos de Baja Frecuencia
TimedProcessMillis pcen;
TimedProcessMillis gcen;
TimedProcessMillis tip_ag;
TimedProcessMillis tip_pa;

void loop_1s_callback();


UART GPS(D2_TX, D3_RX);

inline void togglePin(uint8_t pin) { digitalWrite(pin, !digitalRead(pin)); }

TimedProcessMillis loop_1s;


// // Pines para el bus SPI
// const int CS1 = 10;
// const int CS2 = 9;

void setup()
{
	Serial.begin(115200);
	while (!Serial);
	Serial.println("Iniciando...");
	delay(500);

	Serial1.begin(115200);
	GPS.begin(115200);
	// SPI
	SPI.begin(); // Inicializa el bus SPI
	// pinMode(CS1, OUTPUT);
	// pinMode(CS2, OUTPUT);
	// digitalWrite(CS1, HIGH);
	// digitalWrite(CS2, HIGH);
	// PINES
	pinMode(PCEN, OUTPUT);
	pinMode(GCEN, OUTPUT);
	pinMode(TIP_AG, OUTPUT);
	pinMode(TIP_PA, OUTPUT);
	digitalWrite(TIP_AG, LOW);
	digitalWrite(TIP_PA, LOW);
	digitalWrite(PCEN, HIGH);
	digitalWrite(GCEN, HIGH);

	pcen.set(720, []() { togglePin(PCEN); });
	gcen.set(1320, []() { togglePin(GCEN); });
	tip_ag.set(1630, []() { togglePin(TIP_AG); });
	tip_pa.set(1180, []() { togglePin(TIP_PA); });
	loop_1s.set(1000, loop_1s_callback);
	
	//INTERRUPT
	rpm_sensors_begin(S1_A1, S2_A2);
	gimbal_begin(IN1_M, IN2_M, IN3_M, ENA_M, CS_E);

	Serial.println("Setup completo");

	// if (!BARO.begin())
	// { // inicializo
	// 	Serial.println("Failed to initialize pressure sensor!");
	// 	while (1)
	// 		;
	// }
	// if (!HS300x.begin())
	// {
	// 	Serial.println("Failed to initialize humidity temperature sensor!");
	// }
}

void loop()
{
	pcen.run();
	gcen.run();
	tip_ag.run();
	tip_pa.run();
	loop_1s.run();

	gimbal_run();
	
	// while (Serial1.available())
	// 	Serial.write((uint8_t)Serial1.read());
	// while (GPS.available())
	// 	Serial.write((uint8_t)GPS.read());
}


void loop_1s_callback()
{
	Serial1.println("HOLA_UART1");
	GPS.println("HOLA_GPS");
	// unsigned long s1_delta, s2_delta;
	// rpm_sensors_read(&s1_delta, &s2_delta);
	// Serial.print("S1: ");
	// Serial.print(s1_delta);
	// Serial.print(" S2: ");
	// Serial.println(s2_delta);

	// testSPI(); // Testea cada 1s
	// testI2();  // Testea cada 1s
}



