#include <Arduino.h>

#include "pin_defs.h"


#include <mbed.h>
#include <pinDefinitions.h>
mbed::DigitalOut *gpio_pin = nullptr;   // gpio pin for time measurement purposes

UART gps_uart(D3_RX, D2_TX); // GPS UART
#include <TinyGPSPlus.h>
TinyGPSPlus gps; // GPS instance

uint8_t change_gps_baudrate[] = {0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xD0,0x08,0x00,0x00,0x00,0x84,0x03,0x00,0x07,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x84,0xE8};
uint8_t save_gps_config[] = {0xB5,0x62,0x06,0x09,0x0D,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x1D,0xAB};

void setup()
{
	Serial.begin(1000000); // Consola Serial
	while (!Serial);

	gps_uart.begin(9600); // GPS UART at 9600 baud

	gps_uart.write(change_gps_baudrate, sizeof(change_gps_baudrate)); // Change GPS baud rate
	delay(100); // Wait for the GPS to change baud rate
	gps_uart.write(save_gps_config, sizeof(save_gps_config)); // Save GPS configuration
}

String gata;

void loop()
{
	while (gps_uart.available()) 
	{
		char c = gps_uart.read();
			gps.encode(c);
		if (gps.location.isUpdated()) 
		{
			data = "Lat: " + String(gps.location.lat(), 6) + ", Lng: " + String(gps.location.lng(), 6);
		}
		else
		{
			data = "No GPS fix";
		}
	}
    Serial.println(data);
	data.
}