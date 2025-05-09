#include <Arduino.h>
#include <SPI.h>
#include <Arduino_LPS22HB.h> //libreria para el sensor barometrico LPS22HB
#include <Arduino_HS300x.h>	 //libreria para el sensor de temperatura y humedad HS300x


// Funciones:
void TestUARTGPS();
void testSPI();
void testI2();
void testPINES(unsigned long currentMillis);
void testInterrupts();

void interruptHandler_D15();
void interruptHandler_D16();


// UART por software para GPS en pines 2 (RX) y 3 (TX)
UART GPS(2, 3);
// PINES DE BAJA FRECUENCIA
unsigned long prevMillisD5 = 0;
unsigned long prevMillisD4 = 0;
unsigned long prevMillisD20 = 0;
unsigned long prevMillisD21 = 0;
bool stateD5 = LOW;
bool stateD4 = LOW;
bool stateD20 = LOW;
bool stateD21 = LOW;
// MILIS
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 segundo

//INTERRUPT
const uint8_t interruptPin_D15 = 15; // Pin de interrupción
const uint8_t interruptPin_D16 = 16; // Pin de interrupción

volatile bool interruptFlag_D15_state = false; // Indicador de interrupción // TRUE: RISE FALSE: FALL
volatile bool interruptFlag_D16_state = false;

// Variables para almacenar tiempos
volatile unsigned long startMillis_D15 = 0;
volatile unsigned long endMillis_D15 = 0;
volatile unsigned long startMillis_D16 = 0;
volatile unsigned long endMillis_D16 = 0;





// Pines para el bus SPI
const int CS1 = 10;
const int CS2 = 9;

void setup()
{
	Serial.begin(115200); // Consola Serial
	while (!Serial)
		;
	// UART
	Serial1.begin(115200); // UART física integrada
	// GPS
	GPS.begin(115200); // UART por software (GPS)
	// SPI
	SPI.begin(); // Inicializa el bus SPI
	pinMode(CS1, OUTPUT);
	pinMode(CS2, OUTPUT);
	digitalWrite(CS1, HIGH);
	digitalWrite(CS2, HIGH);
	// PINES
	pinMode(5, OUTPUT);	 // D5
	pinMode(4, OUTPUT);	 // D4
	pinMode(20, OUTPUT); // D20
	pinMode(21, OUTPUT); // D21
						 // I2C

	//INTERRUPT
	pinMode(interruptPin_D15, INPUT_PULLUP); // Pin de interrupción
	pinMode(interruptPin_D16, INPUT_PULLUP); // Pin de interrupción
    	// Configuración de interrupciones
    attachInterrupt(digitalPinToInterrupt(interruptPin_D15), interruptHandler_D15, CHANGE);
    attachInterrupt(digitalPinToInterrupt(interruptPin_D16), interruptHandler_D16, CHANGE);


	if (!BARO.begin())
	{ // inicializo
		Serial.println("Failed to initialize pressure sensor!");
		while (1)
			;
	}
	if (!HS300x.begin())
	{
		Serial.println("Failed to initialize humidity temperature sensor!");
	}
}

void loop()
{
	unsigned long currentMillis = millis();

	if (currentMillis - previousMillis >= interval)
	{
		previousMillis = currentMillis;
		Serial1.print("HOLA_UART1\n");
		GPS.print("HOLA_GPS\n");
		testSPI(); // Testea cada 1s
		testI2();  // Testea cada 1s
	}
	TestUARTGPS(); // REAL TIME
	testPINES(currentMillis);
	testInterrupts(); // Testea interrupciones
}

void TestUARTGPS()
{

	// Lectura desde UART física integrada
	if (Serial1.available())
	{
		Serial.println(Serial1.read());
	}
	// Lectura desde UART GPS
	if (GPS.available())
	{
		Serial.println(GPS.read());
	}
}
void testSPI()
{
	byte response1 = 0;
	byte response2 = 0;

	digitalWrite(CS1, LOW);
	response1 = SPI.transfer(0x111A); // 4378 en Hexa
	digitalWrite(CS1, HIGH);

	digitalWrite(CS2, LOW);
	response2 = SPI.transfer(0x30A); // 778 en hexa
	digitalWrite(CS2, HIGH);

	Serial.println(response1, HEX); // Imprime el valor en hexadecimal CS1
	Serial.println(response2, HEX); // imprime el valor en hexadecimal CS2
}

void testI2()
{

	float pressure = BARO.readPressure(); // lectura de presion
	float altitude = 44330 * (1 - pow(pressure / 101.325, 1 / 5.255));
	// print the sensor value
	Serial.print(altitude);
	Serial.println(" m");

	Serial.println(HS300x.readTemperature()); // lectura de temperatura
	Serial.println(HS300x.readHumidity());	  // lectura de humedad
}

void testPINES(unsigned long currentMillis)
{
	// D5
	if (currentMillis - prevMillisD5 >= 1300)
	{
		prevMillisD5 = currentMillis;
		stateD5 = !stateD5;
		digitalWrite(5, stateD5);
	}
	// D4
	if (currentMillis - prevMillisD4 >= 1500)
	{
		prevMillisD4 = currentMillis;
		stateD4 = !stateD4;
		digitalWrite(4, stateD4);
	}
	// D20
	if (currentMillis - prevMillisD20 >= 2700)
	{
		prevMillisD20 = currentMillis;
		stateD20 = !stateD20;
		digitalWrite(20, stateD20);
	}
	// D21
	if (currentMillis - prevMillisD21 >= 3500)
	{
		prevMillisD21 = currentMillis;
		stateD21 = !stateD21;
		digitalWrite(21, stateD21);
	}
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
	}
	if (interruptFlag_D16_state)
	{
		Serial.print("D16: ");
		Serial.print(startMillis_D16);
		Serial.print(" - ");
		Serial.println(endMillis_D16);
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

