
#include <SimpleFOC.h>

#include <SPI.h>

// BLDC motor & driver instance
// 						  PPF
BLDCMotor *motor = nullptr; // BLDC motor instance
BLDCDriver3PWM *driver = nullptr; // BLDC driver instance

//HALL sensor instance
MagneticSensorSPI *sensor = nullptr; // AS5048A sensor instance



float target_angle = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_angle, cmd); }



void gimbal_begin(int phA, int phB, int phC, int en, int AS5048_SPI_cs)
{

	// create motor and driver instance
	motor = new BLDCMotor(7); // 7 pole pairs
	driver = new BLDCDriver3PWM(phA, phB, phC, en); // mini v1.1

	// create sensor instance
	sensor = new MagneticSensorSPI(AS5048_SPI, 2); // AS5048A sensor instance
	
	// use monitoring with serial
	// enable more verbose output for debugging
	// comment out if not needed
	SimpleFOCDebug::enable(&Serial);

	// initialize magnetic sensor hardware
	sensor->init();
	// link the motor to the sensor
	motor->linkSensor(sensor);



	// driver config
	// power supply voltage [V]
	driver->voltage_power_supply = 8;
		// default voltage_power_supply
	motor->voltage_limit = 2; // Volts
	driver->init();
	// link the motor and the driver
	motor->linkDriver(driver);

	  // choose FOC modulation (optional)
  	motor->foc_modulation = FOCModulationType::SpaceVectorPWM;

  	// set motion control loop to be used
  	motor->controller = MotionControlType::angle;

	// velocity PI controller parameters
	motor->PID_velocity.P = 0.2f;
	motor->PID_velocity.I = 20;
	motor->PID_velocity.D = 0;

	// set motion control loop to be used
	motor->controller = MotionControlType::velocity_openloop;
	// default voltage_power_supply
	motor->voltage_limit = 2; // Volts

	  // velocity low pass filtering time constant
 	 // the lower the less filtered
  	motor->LPF_velocity.Tf = 0.01f;


	  // angle P controller
  	motor->P_angle.P = 20;
  	// maximal velocity of the position control
  	motor->velocity_limit = 20;
  

	// comment out if not needed
	motor->useMonitoring(Serial);

	// initialize motor
	motor->init();
	// align encoder and start FOC
	motor->initFOC();

	// add target command T
	command.add('T', doTarget, "target angle");

	Serial.println(F("Motor ready."));
	Serial.println(F("Set the target angle using serial terminal:"));
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
 	motor->move(target_angle);

	// function intended to be used with serial plotter to monitor motor variables
	// significantly slowing the execution down!!!!
	// motor.monitor();

	// user communication
	command.run();
}