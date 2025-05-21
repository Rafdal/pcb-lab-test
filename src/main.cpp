#include <Arduino.h>

#include "serial_flash_spi.hpp"

//BASADO EN EL DRIVER DE UN CHABON
// https://github.com/sparkfun/SparkFun_SPI_SerialFlash_Arduino_Library

#include "pin_defs.h"

// FLASH SPI
const uint8_t flashPin_CS = CS_M;	  // Pin de Flash SPI
SFE_SPI_FLASH flash; // Instancia de la clase SFE_SPI_FLASH

#include <mbed.h>
#include <pinDefinitions.h>
mbed::DigitalOut *gpio_pin = nullptr;   // gpio pin for time measurement purposes

void prettyPrintBIN(uint8_t val)
{
    for (int i = 7; i >= 0; i--)
    {
        if (val & (1 << i))
            Serial.write((uint8_t)'1');
        else
            Serial.write((uint8_t)'0');
    }
    Serial.println();
}

auto settings = SPISettings(500000, MSBFIRST, SPI_MODE0);
uint8_t flash_buf[255+4];

// SRP0 | BP4 | BP3 | BP2 | BP1 | BP0 | WEL (Write Enabled) | BUSY
#define BUSY_STATUS_FLAG 0x01
uint8_t readStatus1()
{
	flash_buf[0] = 0x05; // Read Status Register
	flash_buf[1] = 0x00;
	SPI.beginTransaction(settings);
	digitalWrite(CS_M, LOW);
	SPI.transfer(flash_buf, 2);
	digitalWrite(CS_M, HIGH);
	SPI.endTransaction();
	return flash_buf[1];
}

enum {ERASE_4KB=0x20, ERASE_32KB=0x52, ERASE_64KB=0xD8};

bool blockErase(uint16_t page, uint8_t block_size_opcode = ERASE_4KB)
{
	if ((readStatus1() & 0x01) || page > (uint16_t)0x7FF)
		return false; // Device is busy
	SPI.beginTransaction(settings);
	digitalWrite(CS_M, LOW);
	SPI.transfer(0x06); // Write Enable
	digitalWrite(CS_M, HIGH);
	SPI.endTransaction();

	flash_buf[0] = block_size_opcode; // Block Erase command
	flash_buf[1] = (page >> 8) & 0xFF;	// Address byte MSB
	flash_buf[2] = page & 0xFF;	// Address byte LSB
	flash_buf[3] = 0x0;	// Address byte LSB
	SPI.beginTransaction(settings);
	digitalWrite(CS_M, LOW);
	SPI.transfer(flash_buf, 4);
	digitalWrite(CS_M, HIGH);
	SPI.endTransaction();
	return true;
}

bool writeData(uint16_t page, uint8_t low, uint8_t *data, uint8_t size)
{
	if ((readStatus1() & 0x01) || page > (uint16_t)0x7FF)
		return false; // Device is busy
	SPI.beginTransaction(settings);
	digitalWrite(CS_M, LOW);
	SPI.transfer(0x06); // Write Enable
	digitalWrite(CS_M, HIGH);
	SPI.endTransaction();

	flash_buf[0] = 0x02; // Byte/Page program
	flash_buf[1] = (page >> 8) & 0xFF;	// Address byte MSB
	flash_buf[2] = page & 0xFF;	// Address byte LSB
	flash_buf[3] = low;	// Address byte LSB
	memcpy(&flash_buf[4], data, size);
	SPI.beginTransaction(settings);
	digitalWrite(CS_M, LOW);
	SPI.transfer(flash_buf, size + 4);
	digitalWrite(CS_M, HIGH);
	SPI.endTransaction();
	return true;
}

void readData(uint16_t page, uint8_t low, uint8_t *data, uint8_t size)
{
	flash_buf[0] = 0x03; // Read Array
	flash_buf[1] = (page >> 8) & 0xFF;	// Address byte MSB
	flash_buf[2] = page & 0xFF;	// Address byte LSB
	flash_buf[3] = low;	// Address byte LSB
	SPI.beginTransaction(settings);
	digitalWrite(CS_M, LOW);
	SPI.transfer(flash_buf, size + 4);
	digitalWrite(CS_M, HIGH);
	SPI.endTransaction();
	memcpy(data, &flash_buf[4], size);
}

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
	// FLASH
	bool flash_status = flash.begin(flashPin_CS, 500000, SPI, SPI_MODE0);
	gpio_pin->write(0);

	delay(10);
	if (flash_status)
	{
		Serial.println("Flash OK");
	}
	else
	{
		Serial.println("Flash ERROR");
	}


	// SPI.beginTransaction(settings);
	// buf[0] = 0x9F; // Read Manufacturer ID
	// buf[1] = 0x00;
	// buf[2] = 0x00;
	// buf[3] = 0x00;
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(buf, 4);
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();


	// SPI.beginTransaction(settings);
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(0x06); // Write Enable
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();

	// delayMicroseconds(10);
	// SPI.beginTransaction(settings);
	// buf[0] = 0x05; // Read Status Register
	// buf[1] = 0x00;
	// buf[2] = 0x00;
	// buf[3] = 0x00;
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(buf, 4);
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();


	// delayMicroseconds(10);
	// SPI.beginTransaction(settings);
	// buf[0] = 0x02; // Byte/Page program
	// buf[1] = 0x00;	// Address byte MSB
	// buf[2] = 0x00;	// Address byte MMSB
	// buf[3] = 0x00;	// Address byte LSB
	// buf[4] = 'H';	// Data
	// buf[5] = 'O';	// Data
	// buf[6] = 'L';	// Data
	// buf[7] = 'A';	// Data
	// buf[8] = 0x00;	// Data
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(buf, 9);
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();

	// delayMicroseconds(10);
	// SPI.beginTransaction(settings);
	// buf[0] = 0x05; // Read Status Register
	// buf[1] = 0x00;
	// buf[2] = 0x00;
	// buf[3] = 0x00;
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(buf, 4);
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();


	// delayMicroseconds(10);
	// SPI.beginTransaction(settings);
	// buf[0] = 0x03; // Read Array
	// buf[1] = 0x00;	// Address byte MSB
	// buf[2] = 0x00;	// Address byte MMSB
	// buf[3] = 0x00;	// Address byte LSB
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(buf, 10);
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();


	// Serial.println("Status2");
	// SPI.beginTransaction(settings);
	// buf[0] = 0x35; // Read Status Register
	// buf[1] = 0x00;
	// buf[2] = 0x00;
	// buf[3] = 0x00;
	// digitalWrite(CS_M, LOW);
	// SPI.transfer(buf, 4);
	// digitalWrite(CS_M, HIGH);
	// SPI.endTransaction();
	// for (uint8_t i = 0; i < 3; i++)
	// 	prettyPrintBIN(buf[i+1]);
	
	gpio_pin->write(1);
	blockErase(0x000);
	gpio_pin->write(0);
	// delay(1);

	char write_buf[] = "HOLA QUE TAL PAPA";
	uint8_t* ptr = (uint8_t*) &write_buf[0];

	gpio_pin->write(1);
	while(readStatus1() & BUSY_STATUS_FLAG);
	gpio_pin->write(0);

	gpio_pin->write(1);
	writeData(0x000, 0x00, ptr, sizeof(write_buf));
	gpio_pin->write(0);

	uint8_t read_buf[32];
	memset(read_buf, 0, 32);

	gpio_pin->write(1);
	readData(0x000, 0x00, read_buf, sizeof(read_buf));
	gpio_pin->write(0);

	for (uint8_t i = 0; i < 32; i++)
	{
		Serial.print(read_buf[i], HEX);
		Serial.print(" ");
	}
	Serial.println();
	for (uint8_t i = 0; i < 32; i++)
	{
		Serial.write(read_buf[i]);
		Serial.print(" ");
	}
	Serial.println();
}

void loop()
{

}