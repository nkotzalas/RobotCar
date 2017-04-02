/*
	RobotCar.h - Library for controlling the movement of a robot car with two motors,
	an accelerometer and a distance sensor.

	Created by Nick Kotzalas, December 30, 2016
	Released into the public domain
*/
#ifndef RobotCar_h
#define RobotCar_h

#include "Arduino.h"

/**
 * Structure that represents the pins of a motor shield 
 * that correspond to a dc motor
 */
struct MotorSetup {
  int directionPin;
  int speedPin;
  int brakePin;
  int currentPin;
};

class RobotCar
{
	public:
		RobotCar(bool debug);

		/* Initialize the components */
		void initializeAccelerometer();
		void initializeDistanceSensor(int echoPin, int trigPin);
		void initializeRightMotor(int directionPin, int speedPin, int brakePin, int currentPin);
		void initializeLeftMotor(int directionPin, int speedPin, int brakePin, int currentPin);

		/*
		 * Moves forward with the provided speed (speed is an integer between 0 and 
		 * 255 that defines how fast the robot will move)
		 */
		void moveForward(int speed);

		/*
		 * Moves backwards with the provided speed (speed is an integer between 0 and 
		 * 255 that defines how fast the robot will move)
		 */
		void moveBackwards(int speed);

		/*
		 * Turn right with the provided speed (speed is an integer between 0 and 255 
		 * that defines how fast the robot will move)
		 */
		void turnRight(int speed);

		/*
		 * Turn left with the provided speed (speed is an integer between 0 and 255 
		 * that defines how fast the robot will move)
		 */
		void turnLeft(int speed);

		/*
		 * Stop all movement
		 */
		void brake();

		/*
		 * Get A calculation of the orientation from the accelerometer. The outcome 
 		 * is a normalized range from 0 to 360
		 */
		float calculateOrientation();

		/*
		 * Calculates the distance to the next obstacle
		 */
		long calculateDistance();

	private:
		int _distanceSensorEchoPin;
		int _distanceSensorTrigPin;

		struct MotorSetup _rightMotor;
		struct MotorSetup _leftMotor;
};

#endif