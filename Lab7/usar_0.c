#pragma config(Sensor, S4,     lightSensor,    sensorLightActive)
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
#define B1 = 1
#define B2 = 2
#define B3 = 4
#define B4 = 8

// Line following constants
#define LIGHT_SENSOR_BLACK 37 // 37 for black
#define LIGHT_SENSOR_WHITE 55 // 60 for white
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

void turn_180_degrees() {
	motor[motorB] = -50;
	motor[motorC] = 50;
	wait1Msec(1825);
}





void followLine() {
  int sensor_reading = SensorValue[lightSensor];

	if (sensor_reading < LIGHT_SENSOR_BLACK) {
		pivot_left(PIVOT_SPEED);
	}
	else if (sensor_reading > LIGHT_SENSOR_WHITE) {
		pivot_right(PIVOT_SPEED);
	} else { // Proportional control
		// normalize sensor reading to a value between 0.0 and 1.0, where 0.0 is black and 1.0 is white
		float normalized_reading = (float)(sensor_reading - LIGHT_SENSOR_BLACK) / (float)(LIGHT_SENSOR_WHITE - LIGHT_SENSOR_BLACK);
		if (normalized_reading < 0.5 - DEAD_ZONE_OFFSET) {
			// turn right proportionally
			int off_center_amount = 0.5 - normalized_reading; // 0.0 to 0.5
			int adjust_speed = off_center_amount * 2 * (MAX_SPEED - MIN_SPEED);
			motor[motorB] = MAX_SPEED;
			motor[motorC] = (MAX_SPEED - adjust_speed);
		} else if (normalized_reading > 0.5 + DEAD_ZONE_OFFSET) {
			// turn left proportionally
			int off_center_amount = normalized_reading - 0.5; // 0.0 to 0.5
			int adjust_speed = off_center_amount * 2 * (MAX_SPEED - MIN_SPEED);
			motor[motorB] = (MAX_SPEED - adjust_speed);
			motor[motorC] = MAX_SPEED;
		}
		else { // dead zone, go straight ahead
			motor[motorB] = MAX_SPEED;
			motor[motorC] = MAX_SPEED;
		}
	}
}



task main()
{
	nMotorEncoder[motorA] = 0;
	//startTask(goToDesiredPositionMotorA);
	while (true) {


		//motor[motorA] = -50; // Can only go backwards
		getJoystickSettings(joystick);

		if (joystick.joy1_Buttons == B1) followLine();
		if (joystick.joy1_Buttons == R1) turn_180_degrees();

		/*
		if (joystick.joy1_Buttons == R1) {
			wait1Msec(100);
			if (goToDesiredPosition) {
				nxtDisplayTextLine(4, "Yeah");
				goToDesiredPosition = false;
				stopTask(goToDesiredPositionMotorA);
			} else {
				nxtDisplayTextLine(4, "No");
				goToDesiredPosition = true;
				startTask(goToDesiredPositionMotorA);
			}
		}
		*/

		nxtDisplayTextLine(0, "%d", joystick.joy1_x1);
		nxtDisplayTextLine(1, "%d", joystick.joy1_y1);
		nxtDisplayTextLine(2, "%d", joystick.joy1_Buttons);
		nxtDisplayTextLine(3, "%d", joystick.joy2_Buttons);

		/*
		if (joystick.joy1_Buttons == R1) motor[motorA] = -10;
		else if (joystick.joy1_Buttons == R2) motor[motorA] = 10;
		else motor[motorA] = 0;
		*/

		/*
		if (joystick.joy1_Buttons == 4) {
			stopTask(goToDesiredPositionMotorA);
			goToDesiredPosition = false;
			motor[motorA] = 20;
			nMotorEncoder[motorA] = 0;
		}
		else if (joystick.joy1_Buttons == 1) {
			stopTask(goToDesiredPositionMotorA);
			goToDesiredPosition = false;
			motor[motorA] = -20;
			nMotorEncoder[motorA] = 0;
		}
		else {
			motor[motorA] = 0;
			if (!goToDesiredPosition) {
				startTask(goToDesiredPositionMotorA);
				goToDesiredPosition = true;
			}
		}
		*/



		/*
		if (joystick.joy1_Buttons == 8 || joystick.joy1_Buttons == 2 || joystick.joy1_Buttons == 24 || joystick.joy1_Buttons == 18) { // Right trigger is pressed left or right
			// Turn
			if (joystick.joy1_Buttons == 8) { // Turn left
				motor[motorB] = -25 * gearConfiguration;
				motor[motorC] = 25 * gearConfiguration;
			} else if (joystick.joy1_Buttons == 24) { // 24 = 8 + 16 Turn left slowly
				motor[motorB] = -10 * gearConfiguration;
				motor[motorC] = 10 * gearConfiguration;
			} else if (joystick.joy1_Buttons == 2) { // Turn right
				motor[motorB] = 25 * gearConfiguration;
				motor[motorC] = -25 * gearConfiguration;
			} else { // Turn right slowly
				motor[motorB] = 10 * gearConfiguration;
				motor[motorC] = -10 * gearConfiguration;
			}
		}
		*/

		float speedFactor = 1.0;
		// Very slow
		if (joystick.joy1_Buttons == L1 + L2) speedFactor = 0.2; // 10
		// Fast
		else if (joystick.joy1_Buttons == L1) speedFactor = 1.5; // 75
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

		wait1Msec(10);
	}
}
