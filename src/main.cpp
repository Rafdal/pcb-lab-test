#include <Arduino.h>

#include "pin_defs.h"


#include <mbed.h>
#include <pinDefinitions.h>
mbed::DigitalOut *gpio_pin = nullptr;   // gpio pin for time measurement purposes

// UART gps(D3_RX, D2_TX); // GPS Serial
// UART gps(D2_TX, D3_RX); // GPS Serial

void setup()
{
	Serial.begin(115200); // Consola Serial
	while (!Serial);

	mbed::DigitalInOut* tmpgpio = digitalPinToGpio(S1_A1);
	if (tmpgpio != NULL)
		delete tmpgpio;
	gpio_pin = new mbed::DigitalOut(digitalPinToPinName(S1_A1), LOW);

	delay(500);

	gpio_pin->write(1);
	// PONER CODIGO PARA MEDIR TIEMPO ACA (SEA EN SETUP O EN LOOP)
	gpio_pin->write(0);
}

void loop()
{
	// Leer GPS ?
}