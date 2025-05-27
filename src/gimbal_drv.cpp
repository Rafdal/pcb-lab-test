#include "gimbal_drv.hpp"

// BLDC motor & driver instance
// 						  PPF
static BLDCMotor *motor_ptr = nullptr; // BLDC motor instance
static BLDCDriver3PWM *driver = nullptr; // BLDC driver instance

//HALL sensor instance
// MagneticSensorAS5048A *hall = nullptr; // AS5048A sensor instance


//                                     A,  B,  C, EN
// BLDCDriver3PWM driver = BLDCDriver3PWM(6, 10, 9, 8); // mini v1.1

// instantiate the commander   (FOR DEBUG ONLY)
static Commander command = Commander(Serial);
static void doMotor(char *cmd) { command.motor(motor_ptr, cmd); }

void gimbal_begin(int phA, int phB, int phC, int en)
{
	// create motor and driver instance
	motor_ptr = new BLDCMotor(7); // 7 pole pairs
	driver = new BLDCDriver3PWM(phA, phB, phC, en); // mini v1.1
	
	// use monitoring with serial
	SimpleFOCDebug::enable(&Serial);

	// driver config
	driver->voltage_power_supply = 8;
	driver->init();
	motor_ptr->linkDriver(driver);

	// aligning voltage [V]
	motor_ptr->voltage_sensor_align = 3;  //  QUE ONDA ESTO ???  (No tengo sensor de voltage)

	// set motion control loop to be used
	motor_ptr->controller = MotionControlType::velocity_openloop;

	// default voltage_power_supply
	motor_ptr->voltage_limit = 2; // Volts

	// comment out if not needed
	motor_ptr->useMonitoring(Serial);

	// initialize motor
	motor_ptr->init();
	// align encoder and start FOC
	motor_ptr->initFOC();

	// add target command M
	command.add('M', doMotor, "motor");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target velocity using serial terminal:"));

	char buffer[200];
	sprintf(buffer, "M%f", motor_ptr->target);

	motor_ptr->target = 1; // initial target velocity 1 rad/s
	Serial.println("Target velocity: 1 rad/s");
	Serial.println("Voltage limit 2V");
	_delay(1000);
}

void gimbal_read()
{
	// Motion control function
	// velocity, position or voltage (defined in motor.controller)
	// this function can be run at much lower frequency than loopFOC() function
	// You can also use motor.move() and set the motor.target in the code
	motor_ptr->move();
	
	// function intended to be used with serial plotter to monitor motor variables
	// significantly slowing the execution down!!!!
	// motor.monitor();
	
	// user communication
	command.run();
}

void gimbal_run()
{
	// main FOC algorithm function
	// the faster you run this function the better
	// Arduino UNO loop  ~1kHz
	// Bluepill loop ~10kHz
	motor_ptr->loopFOC();
}