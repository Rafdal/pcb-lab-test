#include <Arduino.h>

#include "serial_flash_spi.hpp"

//BASADO EN EL DRIVER DE UN CHABON
// https://github.com/sparkfun/SparkFun_SPI_SerialFlash_Arduino_Library

// Funciones:
void testInterrupts();

void interruptHandler_D15();
void interruptHandler_D16();

// INTERRUPT
const uint8_t interruptPin_D15 = 15; // Pin de interrupción
const uint8_t interruptPin_D16 = 16; // Pin de interrupción

volatile bool interruptFlag_D15_state = false; // Indicador de interrupción // TRUE: RISE FALSE: FALL
volatile bool interruptFlag_D16_state = false;

// Variables para almacenar tiempos
volatile unsigned long startMillis_D15 = 0;
volatile unsigned long endMillis_D15 = 0;
volatile unsigned long startMillis_D16 = 0;
volatile unsigned long endMillis_D16 = 0;

// FLASH SPI
const uint8_t flashPin_CS = 14;	  // Pin de Flash SPI
SFE_SPI_FLASH flash; // Instancia de la clase SFE_SPI_FLASH

void setup()
{
	Serial.begin(115200); // Consola Serial
	while (!Serial)
		;
	// UART
	Serial1.begin(115200); // UART física integrada

	// FLASH
	if (flash.begin(flashPin_CS, 8000000, SPI, SPI_MODE0))
	{
		Serial.println("Flash SPI inicializado correctamente.");
	}
	else
	{
		Serial.println("Error al inicializar Flash SPI.");
	}

	sfe_flash_manufacturer_e mfgID = flash.getManufacturerID();
	if (mfgID != SFE_FLASH_MFG_UNKNOWN)
	{
		Serial.print(F("Manufacturer: "));
		Serial.println(flash.manufacturerIDString(mfgID));
	}
	else
	{
		uint8_t unknownID = flash.getRawManufacturerID(); // Read the raw manufacturer ID
		Serial.print(F("Unknown manufacturer ID: 0x"));
		if (unknownID < 0x10)
			Serial.print(F("0")); // Pad the zero
		Serial.println(unknownID, HEX);
	}

	Serial.print(F("Device ID: 0x"));
	Serial.println(flash.getDeviceID(), HEX);



	// INTERRUPT
	pinMode(interruptPin_D15, INPUT); // Pin de interrupción
	pinMode(interruptPin_D16, INPUT); // Pin de interrupción
	// Configuración de interrupciones
	attachInterrupt(digitalPinToInterrupt(interruptPin_D15), interruptHandler_D15, CHANGE);
	attachInterrupt(digitalPinToInterrupt(interruptPin_D16), interruptHandler_D16, CHANGE);
}

void loop()
{
	testInterrupts(); // Testea interrupciones
}

void testInterrupts()
{
	// Manejo de interrupciones para el pin D15
	if (interruptFlag_D15_state)
	{
		Serial.print("D15: ");
		Serial.print(startMillis_D15);
		Serial.print(" - ");
		Serial.println(endMillis_D15);
		interruptFlag_D15_state = false; // Reinicia el indicador de interrupción
	}
	if (interruptFlag_D16_state)
	{
		Serial.print("D16: ");
		Serial.print(startMillis_D16);
		Serial.print(" - ");
		Serial.println(endMillis_D16);
		interruptFlag_D16_state = false; // Reinicia el indicador de interrupción
	}
}

void interruptHandler_D15()
{
	// Manejo de interrupción para el pin D15
	if (digitalRead(interruptPin_D15) == HIGH)
	{
		startMillis_D15 = millis(); // Captura el tiempo de inicio
		interruptFlag_D15_state = true;
	}
	else
	{
		endMillis_D15 = millis(); // Captura el tiempo de finalización
		interruptFlag_D15_state = false;
	}
}

void interruptHandler_D16()
{
	// Manejo de interrupción para el pin D16
	if (digitalRead(interruptPin_D16) == HIGH)
	{
		startMillis_D16 = millis(); // Captura el tiempo de inicio
		interruptFlag_D16_state = true;
	}
	else
	{
		endMillis_D16 = millis(); // Captura el tiempo de finalización
		interruptFlag_D16_state = false;
	}
}
