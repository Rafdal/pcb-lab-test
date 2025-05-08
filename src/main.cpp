#include <Arduino.h>
#include <SPI.h>
#include <Arduino_LPS22HB.h> //libreria para el sensor barometrico LPS22HB
#include <Arduino_HS300x.h>//libreria para el sensor de temperatura y humedad HS300x
//Funciones:
void TestUARTGP();
void testSPI();
void testI2();
void testPINES();
// UART por software para GPS en pines 2 (RX) y 3 (TX)
UART GPS(2, 3);
//PINES DE BAJA FRECUENCIA
unsigned long prevMillisD5 = 0;
unsigned long prevMillisD4 = 0;
unsigned long prevMillisD20 = 0;
unsigned long prevMillisD21 = 0;
bool stateD5 = LOW;
bool stateD4 = LOW;
bool stateD20 = LOW;
bool stateD21 = LOW;
//MILIS
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 segundo

// Pines para el bus SPI
const int CS1 = 10;
const int CS2 = 9;

void setup() {
	Serial.begin(115200);  // Consola Serial
	while (!Serial);
	//UART
	Serial1.begin(115200); // UART física integrada
	//GPS
	GPS.begin(115200);     // UART por software (GPS)
	//SPI
	SPI.begin(); // Inicializa el bus SPI
	pinMode(CS1, OUTPUT);
	pinMode(CS2, OUTPUT);
	digitalWrite(CS1, HIGH);
	digitalWrite(CS2, HIGH);
	//PINES
	pinMode(5, OUTPUT);    // D5
  	pinMode(4, OUTPUT);    // D4
  	pinMode(20, OUTPUT);   // D20
  	pinMode(21, OUTPUT);   // D21
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial1.print("HOLA_UART1\n");
    GPS.print("HOLA_GPS\n");
	testSPI();//Testea cada 1s
	testI2();//Testea cada 1s
  }
  TestUARTGP();//REAL TIME
  testPINES(currentMillis); 
  
}

void TestUARTGPS() {
  
 // Lectura desde UART física integrada
  if (Serial1.available()) {
    Serial.println(Serial1.read());
  }
  // Lectura desde UART GPS
  if (GPS.available()) {
    Serial.println(GPS.read());
  }
}
void testSPI() {
  byte response1 = 0;
  byte response2 = 0;

  digitalWrite(CS1, LOW);
  response1 = SPI.transfer(0x111A);  //4378 en Hexa
  digitalWrite(CS1, HIGH);

  digitalWrite(CS2, LOW);
  response2 = SPI.transfer(0x30A);  //778 en hexa
  digitalWrite(CS2, HIGH);

  Serial.println(response1, HEX);// Imprime el valor en hexadecimal CS1
  Serial.println(response2, HEX); //imprime el valor en hexadecimal CS2
}

void testI2(){
if (!BARO.begin()) { //inicializo
    Serial.println("Failed to initialize pressure sensor!");
    while (1);
}
if (!HS300x.begin()) {
  Serial.println("Failed to initialize humidity temperature sensor!");
}
float pressure = BARO.readPressure();//toma de datos

Serial.print("Pressure = ");
Serial.print(pressure);
Serial.println(HS300x.readTemperature());
Serial.println(HS300x.readHumidity());

BARO.end();
}

void testPINES(unsigned long currentMillis) {
  // D5
  if (currentMillis - prevMillisD5 >= 1300) {
	prevMillisD5 = currentMillis;
	stateD5 = !stateD5;
	digitalWrite(5, stateD5);
  }
  // D4
  if (currentMillis - prevMillisD4 >= 1500) {
	prevMillisD4 = currentMillis;
	stateD4 = !stateD4;
	digitalWrite(4, stateD4);
  }
  // D20
  if (currentMillis - prevMillisD20 >= 2700) {
	prevMillisD20 = currentMillis;
	stateD20 = !stateD20;
	digitalWrite(20, stateD20);
  }
  // D21
  if (currentMillis - prevMillisD21 >= 3500) {
	prevMillisD21 = currentMillis;
	stateD21 = !stateD21;
	digitalWrite(21, stateD21);
  }
}