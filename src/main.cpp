#include <Arduino.h>
#include <Arduino_BMI270_BMM150.h> // filepath: c:\Users\Usuario\Desktop\Santino\pcb-lab-test\src\main.cpp

#include <TimedProcess.h>

#include "pin_defs.h"	// Definiciones de pines

// #include <NRF52_MBED_TimerInterrupt.h>
// NRF52_MBED_TimerInterrupt timerInterrupt;

#include <mbed.h>		// Libreria mbed
#include <pinDefinitions.h>	// Libreria para la conversion de pines
mbed::DigitalOut *gpio_tip_pa = nullptr;


#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <encoders/as5048a/MagneticSensorAS5048A.h>

#include "Orientation.h"	// Orientacion del CANSAT
#include "magnetometerCalib.h"
BLDCMotor motor = BLDCMotor(7); // BLDC motor instance (7 pole pairs)
BLDCDriver3PWM driver(IN1_M, IN2_M, IN3_M, ENA_M);
MagneticSensorAS5048A encoder(CS_E, true); // Magnetic sensor instance
// instantiate the commander   (FOR DEBUG ONLY)
Commander command = Commander(Serial);
// static void doMotor(char *cmd) { command.motor(&motor, cmd); }

float target_angle = 0.0f; // target angle for the motor

// heading en grados [0, 360]°
float heading = 0;
// Componentes cartesianos del vector B terrestre
float mx, my, mz;

// Variables de almacenamiento de offset (soft and hard-iron)
float offsetX = -38.500000;
float offsetY = 23.500000;

void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }

void setup()
{
	Serial.begin(1000000);
	while (!Serial);

	delay(500);
	Serial.println("Iniciando...");

	mbed::DigitalInOut* gpio = digitalPinToGpio(TIP_PA);
	if (gpio != NULL) {
		delete gpio;
	}
	gpio_tip_pa = new mbed::DigitalOut(digitalPinToPinName(TIP_PA), LOW);
	if(gpio_tip_pa->is_connected())
		Serial.println("TIP_PA conectado");
	else
		Serial.println("TIP_PA no conectado");

	// use monitoring with serial
	SimpleFOCDebug::enable(&Serial);

	encoder.init();
	motor.linkSensor(&encoder); // link the motor to the encoder

	// motor.sensor_direction = Direction::CW; // set the sensor direction

	// driver config
	driver.voltage_power_supply = 8;
	driver.init();
	motor.linkDriver(&driver);

	// aligning voltage [V]
	motor.voltage_sensor_align = 3;  //  QUE ONDA ESTO ???  (No tengo sensor de voltage)

	motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // set modulation type

	// set motion control loop to be used
	motor.controller = MotionControlType::angle; // set the controller to angle control

	motor.PID_velocity.P = 0.2f;
	motor.PID_velocity.I = 20;
	motor.PID_velocity.D = 0;

	motor.LPF_velocity.Tf = 0.01f;
	// angle P controller
	motor.P_angle.P = 20;
	// maximal velocity of the position control
	motor.velocity_limit = 50;

	// default voltage_power_supply
	motor.voltage_limit = 2; // Volts

	// comment out if not needed
	motor.useMonitoring(Serial);

	// initialize motor
	motor.init();
	// align encoder and start FOC
	motor.initFOC();

	if(!motor.pp_check_result) {
		Serial.println(F("Motor pole pairs check failed!"));
		Serial.println(F("Please check the motor wiring and configuration."));
		// while(1); // stop execution
	}

	// add target command M
	// command.add('M', doMotor, "motor");
	command.add('T', doTarget, "target angle");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target angle using serial terminal:"));

	// Inicializacion de la IMU
	IMU.setContinuousMode();
	if (!IMU.begin()) {
		while (1) Serial.println("Error initializing IMU");
	}
	Serial.println("IMU Ready");

	// Funcion de calibracion de la IMU
	Calibrar(); // Calibrar el magnetometro

	// Funcion de seteo de posicion inicial del motor
	//TODO:

	_delay(1000);
}

void loop()
{
	// gpio_tip_pa->write(1);
	// gpio_tip_pa->write(0);

	// CANSAT tiene seteado Norte a 180°.
	// El motor debe estar inicialmente ORIENTADO IGUALMENTE QUE EL ARDUINO NANO
	// Por lo que si el arduino Nano mide 170° por ejemplo,
	// Entonces el motor debe rotar 10° en el sentido de las agujas del reloj
	// Es decir, por cada rotacion -theta que tenga el arduino Nano,
	// el motor debe rotar theta.

	// EL TEST ASUME QUE LA REFERENCIA ABOSLUTA (O UN ANGULO CONOCIDO DESDE ESTA) DEL MOTOR (angle = 0° + offset conocido)
	// ES IGUAL A LA ORIENTACION DEL ARDUINO NANO
	// ES DECIR, Ambos alineados inicialmente 

	if (IMU.readMagneticField(mx, my, mz)) {
      heading = computeHeading(mx, my, offsetX, offsetY);
    }

	target_angle = 180 - heading;

	motor.move(target_angle);

	//command.run();

	motor.loopFOC();





}

