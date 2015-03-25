#pragma config(Sensor, S1,     lightSensor,    sensorLightActive)
#pragma DebuggerWindows("joystickSimple")

#include "JoystickDriver.c"
#include "motion_functions.h"

// Button values
// Button 1 = 1
// Button 2 = 2
// Button 3 = 4
// Button 4 = 8
// L1 = 16
// L2 = 64
// R1 = 32
// R2 = 128
// RtxL = 8
// RtxR = 2

#define L1 16
#define L2 64
#define LD 1024 // Left stick pressed
#define R1 32
#define R2 128
#define B1 1
#define B2 2
#define B3 4
#define B4 8

// Line following constants
#define LIGHT_SENSOR_THRESHOLD 30
#define PIVOT_SPEED 10
#define DEAD_ZONE_OFFSET 0.01
#define MAX_SPEED 20
#define MIN_SPEED 15


bool goToDesiredPosition = false;
int desiredPosition = 0;
int gearConfiguration = -1;

task goToDesiredPositionMotorA() {
	while (true) {
		int currentPosition = nMotorEncoder[motorA];
		int difference = currentPosition - desiredPosition;
		if (difference < -180) difference += 360;
		else if (difference > 180) difference -= 360;

		if (difference > 2) motor[motorA] = -75;
		else if (difference < -2) motor[motorA] = 75;
		else motor[motorA] = 0;
		wait1Msec(10);
	}
}

void turn_left() {
	motor[motorB] = 50;
	motor[motorC] = -50;
}

void turn_right() {
	motor[motorB] = -50;
	motor[motorC] = 50;
}



void followLine() {
	while (joystick.joy1_Buttons == B1) {
	  int sensor_reading = SensorValue[lightSensor];

		if (sensor_reading < LIGHT_SENSOR_THRESHOLD) {
			motor[motorB] = -50;
			motor[motorC] = 25;
		} else {
			motor[motorB] = 25;
			motor[motorC] = -50;
		}

		wait10Msec(1);
		getJoystickSettings(joystick);
	}
}



task main()
{
	nPowerDownDelayMinutes = 525600;
	nMotorEncoder[motorA] = 0;
	//startTask(goToDesiredPositionMotorA);
	while (true) {


		//motor[motorA] = -50; // Can only go backwards
		getJoystickSettings(joystick);

		nxtDisplayTextLine(0, "%d", joystick.joy1_x1);
		nxtDisplayTextLine(1, "%d", joystick.joy1_y1);
		nxtDisplayTextLine(2, "%d", joystick.joy1_Buttons);
		nxtDisplayTextLine(3, "%d", joystick.joy2_Buttons);

		if (joystick.joy1_Buttons == R1) turn_right();
		else if (joystick.joy1_Buttons == L1) turn_left();
		else if (joystick.joy1_Buttons == B1) followLine();
		else {
			float speedFactor = 1.0;
			// Very slow
			if (joystick.joy1_Buttons == L1 + L2) speedFactor = 0.2; // 10
			// Fast
			else if (joystick.joy1_Buttons == L1) speedFactor = 2; // 75
			// Slow
			else if (joystick.joy1_Buttons == L2) speedFactor = 0.5; // 25

			motor[motorB] = (joystick.joy1_y1/127.0 * 50 + joystick.joy1_x1/127.0 * 50) * speedFactor * gearConfiguration;
			motor[motorC] = (joystick.joy1_y1/127.0 * 50 - joystick.joy1_x1/127.0 * 50) * speedFactor * gearConfiguration;
		}

		float speedFactor = 1.0;
		// Very slow
		if (joystick.joy1_Buttons == L1 + L2) speedFactor = 0.2; // 10
		// Fast
		else if (joystick.joy1_Buttons == L1) speedFactor = 2.0; // 75
		// Slow
		else if (joystick.joy1_Buttons == L2) speedFactor = 0.5; // 25

		motor[motorB] = (joystick.joy1_y1/127.0 * 50 + joystick.joy1_x1/127.0 * 50) * speedFactor * gearConfiguration;
		motor[motorC] = (joystick.joy1_y1/127.0 * 50 - joystick.joy1_x1/127.0 * 50) * speedFactor * gearConfiguration;

		/*
		if (joystick.joy1_x1 != -1 && joystick.joy1_y1 == 1) { // Right trigger is pressed left or right
			// Turn
			if (joystick.joy1_x1 < -1) { // Turn left
				motor[motorB] = -25 * gearConfiguration * speedFactor;
				motor[motorC] = 25 * gearConfiguration * speedFactor;
			} else { // Turn right
				motor[motorB] = 25 * gearConfiguration * speedFactor;
				motor[motorC] = -25 * gearConfiguration * speedFactor;
			}
		}

		else if (joystick.joy1_y1 != 1 && joystick.joy1_x1 == -1) { // Go forwards/backwards
			// 50 is default speed
			motor[motorB] = joystick.joy1_y1/127 * 50 * speedFactor * gearConfiguration;
			motor[motorC] = joystick.joy1_y1/127 * 50 * speedFactor * gearConfiguration;
		}
		else {
			motor[motorB] = 0;
			motor[motorC] = 0;
		}*/

		wait10Msec(1);
	}
}
