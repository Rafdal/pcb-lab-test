#ifndef FLASH_MEMORY_H
#define FLASH_MEMORY_H

#include <Arduino.h>
#include <mbed.h>
#include <pinDefinitions.h>
#include <SPI.h>

class FlashMemory
{
public:
    FlashMemory();

    bool begin(uint8_t CS_pin, uint32_t spi_freq, uint8_t spi_mode = SPI_MODE0, SPIClass &spiPort = SPI);

	// 90 ms	(A11 - A0 ignored)
	void erase_4KB(uint16_t page);
	// 210 ms	(A14 - A0 ignored)
	void erase_32KB(uint16_t page);
	// 360 ms	(A15 - A0 ignored)
	void erase_64KB(uint16_t page);
	// 3 s
	void erase_full();

	bool write_data(uint16_t page, uint8_t low, uint8_t *data, uint8_t size);
	void read_data(uint16_t page, uint8_t low, uint8_t *data, uint8_t size);

	bool busy();

private:

// SRP0 | BP4 | BP3 | BP2 | BP1 | BP0 | WEL (Write Enabled) | BUSY
	uint8_t readStatus1();

    uint8_t buf[255+4]; // Buffer for SPI transfer

    mbed::DigitalOut *cs_pin = nullptr;   // gpio pin for time measurement purposes
	SPIClass *_spiPort = nullptr; // Pointer to the SPI port
	SPISettings settings;
}

#endif // FLASH_MEMORY_H