#include <Arduino.h>

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

BLDCMotor motor = BLDCMotor(7); // BLDC motor instance (7 pole pairs)
BLDCDriver3PWM driver(IN1_M, IN2_M, IN3_M, ENA_M);
MagneticSensorAS5048A encoder(CS_E, true); // Magnetic sensor instance
// instantiate the commander   (FOR DEBUG ONLY)
Commander command = Commander(Serial);
// static void doMotor(char *cmd) { command.motor(&motor, cmd); }

float target_angle = 0.0f; // target angle for the motor

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

	motor.sensor_direction = Direction::CW; // set the sensor direction

	// driver config
	driver.voltage_power_supply = 8;
	driver.init();
	motor.linkDriver(&driver);

	// aligning voltage [V]
	motor.voltage_sensor_align = 3;  //  QUE ONDA ESTO ???  (No tengo sensor de voltage)

	// motor.foc_modulation = FOCModulationType::SpaceVectorPWM; // set modulation type

	// set motion control loop to be used
	motor.controller = MotionControlType::angle;

	motor.PID_velocity.P = 0.2f;
	motor.PID_velocity.I = 20;
	motor.PID_velocity.D = 0;

	motor.LPF_velocity.Tf = 0.01f;
	// angle P controller
	motor.P_angle.P = 20;
	// maximal velocity of the position control
	motor.velocity_limit = 20;

	// default voltage_power_supply
	motor.voltage_limit = 2; // Volts

	// comment out if not needed
	motor.useMonitoring(Serial);

	// initialize motor
	motor.init();
	// align encoder and start FOC
	motor.initFOC();

	// add target command M
	// command.add('M', doMotor, "motor");
	command.add('T', doTarget, "target angle");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target angle using serial terminal:"));
	_delay(1000);
}

void loop()
{
	// gpio_tip_pa->write(1);
	// gpio_tip_pa->write(0);
	motor.move(target_angle);

	command.run();

	motor.loopFOC();
}

