/* Example sketch that uses the Robot movement library */
#include <DistanceSensor.h>
#include <AccelerometerSensor.h>
#include <4WRobot.h>

#define MIN_OBSTACLE_DISTANCE 20

// Debug logs are enabled and test mode is active (switch to false to disable)
const bool debug = false;

/****************************/
/* PIN WIRING MAPPING START */
/****************************/
// Left motor pins
const int LEFT_DIRECTION = 13;
const int LEFT_SPEED = 11;
const int LEFT_BRAKE = 8;
const int LEFT_CURRENT = 1;

// Right motor pins
const int RIGHT_DIRECTION = 12;
const int RIGHT_SPEED = 3;
const int RIGHT_BRAKE = 9;
const int RIGHT_CURRENT = 0;

// Distance sensor pins
const int ECHO_PIN = 4;
const int TRIG_PIN = 7;

// The accelerometer is using the SCL and SDA pins of the Arduino
/*************************/
/* PIN WIRING MAPPING END*/
/*************************/

// Create the instance of the DistanceSensor
DistanceSensor distanceSensor = DistanceSensor(debug, TRIG_PIN, ECHO_PIN);

// Create the instance of the accelerometerSensor
AccelerometerSensor accelerometerSensor = AccelerometerSensor(debug);

// Create the instance of the 4WRobot
4WRobot robot = 4WRobot(debug); 

// Keeps the orientation of the car before performing the next action
float originalOrientation = 0;

void setup() {
  Serial.begin(9600);

  printMessage("SETUP: ", "Robot car booting...");

  // Initialize the accelerometer sensor
  accelerometerSensor.setup();

  // Initialize the distance sensor
  distanceSensor.setup();

  // Initialize the motors
  robot.setupLeftMotor(LEFT_DIRECTION, LEFT_SPEED, LEFT_BRAKE, LEFT_CURRENT);
  robot.setupRightMotor(RIGHT_DIRECTION, RIGHT_SPEED, RIGHT_BRAKE, RIGHT_CURRENT);

  // Resolve the original orientation of the robot
  originalOrientation = accelerometerSensor.calculateOrientation();
  printMessage("Original orientation: ", originalOrientation);
}

void loop() {
  if (debug) {
    testModeLoop();
  } else {
    dummyMovementLoop();
  }
}

void dummyMovementLoop() {
  long distance = distanceSensor.calculateDistance();
  
  int speed = 150;
  if (distance > 200) {
    speed = 200;  
  } else if (distance < 60) {
    speed = 100;
  }

  robot.moveForward(speed);

  printMessage("INFO: Distance: ", distance);
  
  if (distance <= MIN_OBSTACLE_DISTANCE) {
    printMessage("INFO: " ,"Close to obstacle");
    avoidObstacle(distance);      
  }

  delay(100);  
}

void avoidObstacle(long originalDistance) {
  printMessage("INFO:", "Avoiding obstacle: ");
  
  // Stop moving
  robot.brake();
  delay(200);

  long distance = originalDistance;

  long reverseStart = millis();
  // Move backwards until distance from obstacle is at least 40cm or until moving backwards for more than 3 seconds
  printMessage("INFO: ", "Moving backwards until distance is < 40 cm");
  while (distance < 40 && (millis() - reverseStart < 3000)) {
    robot.moveBackwards(100);
    
    distance = distanceSensor.calculateDistance();
    printMessage("INFO: Distance: ", distance);
  }

  // Stop moving backwards
  robot.brake();
  delay(1000);

  int newObstacleFound = 0;

  long turnStart = millis();

  float orientation = accelorometerSensor.calculateOrientation();
  float targetOrientation = (float) (((int) orientation + 90) % 360);
  float difference = calculateOrientationDifference(targetOrientation, orientation);
  
  printMessage("INFO: ", "Turning right ");
  while (difference > 5 && (millis() - turnStart < 4000)) {
    robot.turnRight(250);
    
    // Resolve the current and target orientations
    orientation = accelerometerSensor.calculateOrientation();
    difference = calculateOrientationDifference(targetOrientation, orientation);

    distance = distanceSensor.calculateDistance();
    if (distance < MIN_OBSTACLE_DISTANCE) {
      printMessage("INFO:", "New obstacle detected");
      newObstacleFound = 1;
      break;
    }
  }

  if (newObstacleFound > 0) {
    avoidObstacle(distance);
  } else {
    printMessage("INFO:", "Obstacle avoided!");
  }
}

float calculateOrientationDifference(float targetOrientation, float orientation) {
  float result = 0.0;
  if (targetOrientation < orientation) {
    result = (360 - targetOrientation) + orientation;
  } else {
    result = targetOrientation - orientation;
  }

  return result;
}

void testModeLoop() {
  if (Serial.available() > 0) {
      // read the incoming byte:
      int incomingByte = Serial.read();
      if (incomingByte == 'N') {
        Serial.println("Forward");
        robot.moveForward(100);
      } else if (incomingByte == 'S') {
        Serial.println("Reverse");
        robot.moveBackwards(100);
      } else if (incomingByte == 'E') {
        Serial.println("Left");
        robot.turnLeft(100);
      } else if (incomingByte == 'W') {
        Serial.println("Right");
        robot.turnRight(100);
      } else if (incomingByte == 'B') {
        Serial.println("Brake");
        robot.brake();
      } else if (incomingByte == 'D') {
        Serial.println("Distance");
        Serial.println(distanceSensor.calculateDistance());
      } else if (incomingByte == 'A') {
        Serial.println("Orientation");
        Serial.println(accelerometerSensor.calculateOrientation());
      }
  }  
}

void printMessage(String title, String message) {
  Serial.print(title);
  Serial.println(message);
}

void printMessage(String title, float number) {
  Serial.print(title);
  Serial.println(number);
}


