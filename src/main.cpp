#include <Arduino.h>

#include <TimedProcess.h>

#include "serial_flash_spi.hpp"
// BASADO EN EL DRIVER DE UN CHABON
// https://github.com/sparkfun/SparkFun_SPI_SerialFlash_Arduino_Library


#include "pin_defs.h"	// Definiciones de pines

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines

TimedProcessMillis loop_1s;


SFE_SPI_FLASH flash;
#define SPI_FREQ 4000000

void setup()
{
	Serial.begin(1000000);
	while (!Serial) ;

	delay(500);
	Serial.println("Iniciando...");


	if ( flash.begin(CS_M, SPI_FREQ, SPI, SPI_MODE0) ) Serial.println("Flash inicializado");
	else Serial.println("Error al inicializar el flash");
	
	Serial.println("Setup completo");

	loop_1s.set(1000, [](){
		// flash.getJEDEC();
	});
}

void loop()
{
	loop_1s.run();
	// 3 - SPI x2
}

