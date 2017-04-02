## Table of Contents

  * [Description](#Description)
  * [Required libraries/Components](#RequiredLibrariesComponents)
  * [Assembly steps](#AssemblySteps)
  
  < br/>
  
## Description  <a id="Description"></a>

**RobotCar** is an arduino library that leverages creating sketches for a custom robotic car with four wheels.
  
 <br/>
  
## Required libraries and components <a id="RequiredLibrariesComponents"></a> 
The following libraries and the corresponding components are required in order to build the robot car:

* [LSM303Accelerometer.h](https://github.com/nkotzalas/LSM303Accelerometer)
* [DistanceSensor.h](https://github.com/nkotzalas/DistanceSensor)
* [FourWRobot.h](https://github.com/nkotzalas/FourWRobot)

 <br/>

## Assembly steps <a id="AssemblySteps"></a> 

* Follow the steps of described in each of the above libraries/components and construct all required components.
* Connect the distance sensor and the accelerometer to the Arduino.
* Load the example sketch to the Arduino IDE and update the pin wiring map accoring to the wiring the you performed (make sure that the "debug" variable of the sketch is set to "true").
* Connect the Arduino with your computer using a USB cable and upload the updated sketch.
* Open the Serial monitor (baud rate must be 9600). 
<ol>
	<li>Write the character "N" and verify that all wheels are moving to the forward direction.</li>
	<li>Write the character "S" and verify that all wheels are moving to the backwards direction.</li>
	<li>Write the character "W" and verify that the right wheels are moving forward and the left wheels are moving backwards.</li>
	<li>Write the character "E" and verify that the left wheels are moving forward and the right wheels are moving backwards.</li>
	<li>Write the character "B" and verify that all wheels stop moving.</li>
	<li>Write the character "D" and verify that the distance calculated by the distance sensor is correct (it will be displayed in the serial monitor).</li>
	<li>Write the character "O" and verify that the orientation calculated by the accelerometer is correct (it will be displayed in the serial monitor).</li>
</ol>
* Switch the "debug" variable of the example sketch to "false".
* Upload the updated sketch to the Arduino.
* Put the robot car on the floor and see it perform the movement described in the "dummyMovement" function of the sketch (i.e. moving forward and turning around when an obstacle is in front of the robot).