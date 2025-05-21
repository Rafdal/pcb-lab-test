#include <Arduino.h>
#include <mbed.h>
#include <pinDefinitions.h>
#include "AT25SF041B_defs.h"
#include "FlashMemory.h"


bool FlashMemory::begin(uint8_t CS_pin, uint32_t spi_freq, uint8_t spi_mode, SPIClass &spiPort)
{
    mbed::DigitalInOut* tmpgpio = digitalPinToGpio(CS_pin);
	if (tmpgpio != NULL)
		delete tmpgpio;
	cs_pin = new mbed::DigitalOut(digitalPinToPinName(CS_pin), HIGH);
    cs_pin->write(1);

    _spiPort = &spiPort;

    settings = SPISettings(spi_freq, MSBFIRST, spi_mode);

	buf[0] = GET_FLASH_ID; // Read Manufacturer ID
	buf[1] = 0x00;
	buf[2] = 0x00;
	buf[3] = 0x00;
    _spiPort->beginTransaction(settings);
	cs_pin->write(0);
	_spiPort->transfer(buf, 4);
	cs_pin->write(1);
	_spiPort->endTransaction();
    uint8_t tmp = buf[1] ^ MANUFACTURER_ID;
    tmp |= (buf[2] ^ DEVICE_ID1);
    tmp |= (buf[3] ^ DEVICE_ID2);
    return (tmp == 0); // Check if the device is the expected one
}

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

bool FlashMemory::busy()
{
	return (readStatus1() & BUSY_STATUS_FLAG);
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

	flash_buf[0] = 0x02;                // Byte/Page program
	flash_buf[1] = (page >> 8) & 0xFF;	// Address byte MSB
	flash_buf[2] = page & 0xFF;	        // Address byte LSB
	flash_buf[3] = low;	                // Address byte LSB
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