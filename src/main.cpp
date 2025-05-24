#include <Arduino.h>

#include "pin_defs.h"


#include <mbed.h>
#include <pinDefinitions.h>
mbed::DigitalOut *gpio_pin = nullptr;   // gpio pin for time measurement purposes

// UART gps(D3_RX, D2_TX); // GPS Serial
// UART gps(D2_TX, D3_RX); // GPS Serial

#include "gps_test_data.h"
uint8_t index_connected = 0;
uint8_t index_disconnected = 0;
unsigned long last_millis = 0;

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
	if( millis() - last_millis > 1000 )
	{
		gpio_pin->write(1);
		if (index_connected < GPS_CONNECTED_TEST_DATA_LEN)
		{
			Serial.print(gps_test_data_connected[index_connected]);
			index_connected++;
		}
		else if (index_disconnected < GPS_DISCONNECTED_TEST_DATA_LEN)
		{
			Serial.print(gps_test_data_disconnected[index_disconnected]);
			index_disconnected++;
		}
		else
		{
			Serial.println("\nFIN DEL TEST\n");
			delay(2000);
			index_connected = 0;
			index_disconnected = 0;
		}
		gpio_pin->write(0);
		last_millis = millis();
	}
	// Leer GPS ?
}