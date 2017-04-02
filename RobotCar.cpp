/*
	RobotCar.h - Library for controlling the movement of a robot car with two motors,
	an accelerometer and a distance sensor.

	Created by Nick Kotzalas, December 30, 2016
	Released into the public domain
*/

#include "Arduino.h"
#include "RobotCar.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

int debugEnabled = false;

/**
 * The accelerometer sensor definition
 */
Adafruit_LSM303_Mag_Unified mag;

int _distanceSensorTrigPin = 7;
int _distanceSensorEchoPin = 4;

//TODO: Use the FourWRobot library

/**
 * Constructor of the movement handler
 */
RobotCar::RobotCar(bool debug)
{
	debugEnabled = debug;
	
	// Assign a unique ID to this sensor at the same time
	mag = Adafruit_LSM303_Mag_Unified(10001);
}

/**
 * Logs the provided title and message to the serial monitor
 */
void logMessage(String title, String message) {
	if (debugEnabled) {
		Serial.print(title);
		Serial.println(message);
	}
}

void logMessage(String title, int message) {
	if (debugEnabled) {
		Serial.print(title);
		Serial.println(message);
	}
}

void _resetDistanceSensor() {
  digitalWrite(_distanceSensorTrigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(_distanceSensorTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_distanceSensorTrigPin, LOW);
}

/**
 * Disengage the brakes and move to the requested direction (LOW: backwards, 
 * HIGH: forward), with the requested speed
 */
void _performMovement(int movementDirection, int movementSpeed, struct MotorSetup motor) {
  digitalWrite(motor.brakePin, LOW);
  digitalWrite(motor.directionPin, movementDirection);
  analogWrite(motor.speedPin, movementSpeed);
}

/**
 * Engage the brakes for the provided motor
 */
void _brakeMovement(struct MotorSetup motor) {
  digitalWrite(motor.brakePin, HIGH);
  analogWrite(motor.speedPin, 0);
}

/* Initialize the components */
void RobotCar::initializeAccelerometer()
{
	logMessage("INFO: ", "Initializing the accelerometer...");
	delay(100);
	int attempts = 0;
	while (!mag.begin()) {
		attempts++;
		logMessage("ERROR: ", "Failed to detect the accelerometer!");
		if (attempts < 5) {
			logMessage("INFO: ", "Acceleremeter initialization will be retried in 5 seconds");
			delay(5000);
		} else {
			while(1);
		}
	}

	logMessage("INFO: ", "The accelerometer has been initialized");
}

void RobotCar::initializeDistanceSensor(int echoPin, int trigPin)
{
	logMessage("INFO: ", "Initializing the distance sensor...");
	_distanceSensorEchoPin = echoPin;
	_distanceSensorTrigPin = trigPin;

	pinMode(echoPin, INPUT);
	pinMode(trigPin, OUTPUT);
	logMessage("INFO: ", "Distance sensor initialized");
}

void RobotCar::initializeLeftMotor(int directionPin, int speedPin, int brakePin, int currentPin)
{
	logMessage("INFO: ", "Initializing the left motors...");
	_leftMotor.directionPin = directionPin;
	_leftMotor.speedPin = speedPin;
	_leftMotor.brakePin = brakePin;
	_leftMotor.currentPin = currentPin;

	// Initiate the motor of the left wheels (Channel A pin)
	pinMode(_leftMotor.directionPin, OUTPUT);
	// Initiate the brake of the left wheels (Channel A pin)
	pinMode(_leftMotor.brakePin, OUTPUT);

	logMessage("INFO: ", "Left motors initialized");
}

void RobotCar::initializeRightMotor(int directionPin, int speedPin, int brakePin, int currentPin)
{
	logMessage("INFO: ", "Initializing the right motors...");
	_rightMotor.directionPin = directionPin;
	_rightMotor.speedPin = speedPin;
	_rightMotor.brakePin = brakePin;
	_rightMotor.currentPin = currentPin;

	// Initiate the motor of the right wheels (Channel A pin)
	pinMode(_rightMotor.directionPin, OUTPUT);
	// Initiate the brake of the right wheels (Channel A pin)
	pinMode(_rightMotor.brakePin, OUTPUT);

	logMessage("INFO: ", "Right motors initialized");
}

/*
 * Moves forward with the provided speed (speed is an integer between 0 and 
 * 255 that defines how fast the robot will move)
 */
void RobotCar::moveForward(int speed)
{
	_performMovement(HIGH, speed, _rightMotor);
	_performMovement(LOW, speed, _leftMotor);
}

/*
 * Moves backwards with the provided speed (speed is an integer between 0 and 
 * 255 that defines how fast the robot will move)
 */
void RobotCar::moveBackwards(int speed)
{
	_performMovement(LOW, speed, _rightMotor);
	_performMovement(HIGH, speed, _leftMotor);
}

/*
 * Turn right with the provided speed (speed is an integer between 0 and 255 
 * that defines how fast the robot will move)
 */
void RobotCar::turnRight(int speed)
{
	_performMovement(HIGH, speed, _rightMotor);
	_performMovement(HIGH, speed, _leftMotor);
}

/*
 * Turn left with the provided speed (speed is an integer between 0 and 255 
 * that defines how fast the robot will move)
 */
void RobotCar::turnLeft(int speed)
{
	_performMovement(LOW, speed, _rightMotor);
	_performMovement(LOW, speed, _leftMotor);
}

/*
 * Stop all movement
 */
void RobotCar::brake()
{
	_brakeMovement(_rightMotor);
	_brakeMovement(_leftMotor);
}

/*
 * Get A calculation of the orientation from the accelerometer. The outcome 
 * is a normalized range from 0 to 360
 */
float RobotCar::calculateOrientation()
{
	// Get a new calculation from the accelerometer
	sensors_event_t accelerometerEvent; 
	mag.getEvent(&accelerometerEvent);

	float Pi = 3.14159;

	// Calculate the angle of the vector y,x
	float heading = (atan2(accelerometerEvent.magnetic.y,accelerometerEvent.magnetic.x) * 180) / Pi;

	// Normalize to 0-360
	if (heading < 0) {
		heading = 360 + heading;
	}

	return heading;
}

/*
 * Calculates the distance to the next obstacle
 */
long RobotCar::calculateDistance()
{
	// Reset the values of the distance sensor
	_resetDistanceSensor();

	long duration = pulseIn(_distanceSensorEchoPin, HIGH);

	// Calculate the distance in cms
	long distance = (duration/2) / 29.1;

	return distance;
}
