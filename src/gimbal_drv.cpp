
#include <SimpleFOC.h>

#include <SPI.h>

// BLDC motor & driver instance
// 						  PPF
BLDCMotor *motor = nullptr; // BLDC motor instance
BLDCDriver3PWM *driver = nullptr; // BLDC driver instance

//HALL sensor instance
MagneticSensorAS5048A *hall = nullptr; // AS5048A sensor instance


//                                     A,  B,  C, EN
// BLDCDriver3PWM driver = BLDCDriver3PWM(6, 10, 9, 8); // mini v1.1

// instantiate the commander
Commander command = Commander(Serial);
void doMotor(char *cmd) { command.motor(motor, cmd); }

void gimbal_begin(int phA, int phB, int phC, int en)
{

	// create motor and driver instance
	motor = new BLDCMotor(7); // 7 pole pairs
	driver = new BLDCDriver3PWM(phA, phB, phC, en); // mini v1.1
	
	// use monitoring with serial
	// enable more verbose output for debugging
	// comment out if not needed
	SimpleFOCDebug::enable(&Serial);



	// driver config
	// power supply voltage [V]
	driver->voltage_power_supply = 8;
	driver->init();
	// link the motor and the driver
	motor->linkDriver(driver);

	// aligning voltage [V]
	motor->voltage_sensor_align = 3;

	// set motion control loop to be used
	motor->controller = MotionControlType::velocity_openloop;

	// default voltage_power_supply
	motor->voltage_limit = 2; // Volts

	// comment out if not needed
	motor->useMonitoring(Serial);

	// initialize motor
	motor->init();
	// align encoder and start FOC
	motor->initFOC();

	// add target command M
	command.add('M', doMotor, "motor");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target velocity using serial terminal:"));

	char buffer[200];
	sprintf(buffer, "M%d", motor->target);

	motor->target = 4; // initial target velocity 1 rad/s
	Serial.println("Target velocity: 1 rad/s");
	Serial.println("Voltage limit 2V");
	_delay(1000);
}

void gimbal_run()
{
	// main FOC algorithm function
	// the faster you run this function the better
	// Arduino UNO loop  ~1kHz
	// Bluepill loop ~10kHz
	motor->loopFOC();

	// Motion control function
	// velocity, position or voltage (defined in motor.controller)
	// this function can be run at much lower frequency than loopFOC() function
	// You can also use motor.move() and set the motor.target in the code
	motor->move();

	// function intended to be used with serial plotter to monitor motor variables
	// significantly slowing the execution down!!!!
	// motor.monitor();

	// user communication
	command.run();
}