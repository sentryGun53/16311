#pragma config(Sensor, S1,     lightSensor,    sensorLightActive)
#pragma config(Sensor, S2,     nCompass,            sensorI2CHiTechnicCompass)
#pragma DebuggerWindows("joystickSimple")

#include "JoystickDriver.c"
#include "motion_functions.h"

// Button values
// Button 1 = 1
// Button 2 = 2
// Button 3 = 4
// Button 4 = 8
// Button A = 1
// Button B = 2
// Button X = 4
// Button Y = 8
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

void goToDesiredPositionMotorA() {
	while (true) {
		int currentPosition = nMotorEncoder[motorA];
		int difference = currentPosition - desiredPosition;
		if (difference < -180) difference += 360;
		else if (difference > 180) difference -= 360;

		if (difference > 1) motor[motorA] = -25;
		else if (difference < -1) motor[motorA] = 25;
		else {
			motor[motorA] = 0;
		}
		wait1Msec(10);
	}
}

task deploy() {
	// Go to desired position and stay there
	while (true) {
		int currentPosition = nMotorEncoder[motorA];
		int difference = currentPosition - desiredPosition;
		//if (difference < -180) difference += 360;
		//else if (difference > 180) difference -= 360;

		if (difference > 1) motor[motorA] = -25;
		else if (difference < -1) motor[motorA] = 25;
		else {
			motor[motorA] = 0;
		}
		wait1Msec(10);
	}
}

task deployWeak() {
	// Go to desired position and stay there
	while (true) {
		int currentPosition = nMotorEncoder[motorA];
		int difference = currentPosition - desiredPosition;
		//if (difference < -180) difference += 360;
		//else if (difference > 180) difference -= 360;

		if (difference > 1) motor[motorA] = -25;
		else if (difference < -1) motor[motorA] = 25;
		else {
			motor[motorA] = 0;
		}
		wait1Msec(10);
	}
}

task undeploy() {
	// Go to desired position and quit
	stopTask(deploy);
	stopTask(deployWeak);
	while (true) {
		int currentPosition = nMotorEncoder[motorA];
		int difference = currentPosition - desiredPosition;
		//if (difference < -180) difference += 360;
		//else if (difference > 180) difference -= 360;

		if (difference > 1) motor[motorA] = -25;
		else if (difference < -1) motor[motorA] = 25;
		else {
			motor[motorA] = 0;
			break;
		}
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
	while (joystick.joy1_Buttons == 128) {
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


task playHeading() {
	while (true) {
		int nCompassHeading = SensorValue[nCompass];
		nxtDisplayTextLine(4, "%d", nCompassHeading);
		wait1Msec(50);
	}
}

void reverseTwentyDegrees() {
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;

	while (nMotorEncoder[motorB] < 25) {
		motor[motorB] = 20;
		motor[motorC] = 20;
	}
	motor[motorB] = 0;
	motor[motorC] = 0;
}

task main()
{
	nPowerDownDelayMinutes = 525600;
	nMotorEncoder[motorA] = 0;

	//startTask(playHeading);

	while (true) {
		getJoystickSettings(joystick);

		nxtDisplayTextLine(0, "%d", joystick.joy1_Buttons);
		nxtDisplayTextLine(1, "%d", joystick.joy1_y1);
		nxtDisplayTextLine(2, "%d", joystick.joy1_Buttons);
		nxtDisplayTextLine(3, "%d", joystick.joy2_Buttons);

		//if (joystick.joy1_Buttons ==
		if (joystick.joy1_Buttons == R1) turn_right();
		else if (joystick.joy1_Buttons == L1) turn_left();
		else if (joystick.joy1_Buttons == 128) {
			//playSound(soundShortBlip);
			followLine();
		}
		else if (joystick.joy1_Buttons == B3) {
			desiredPosition = (desiredPosition == 0) ? -95 : 0;
			if (desiredPosition != 0) {
				stopTask(undeploy);
				stopTask(deploy);
				startTask(deployWeak);
			}
			else {
				stopTask(deploy);
				stopTask(deployWeak);
				startTask(undeploy);
			};
			wait1Msec(500); // ignore button presses for 0.5s
		}
		else if (joystick.joy1_Buttons == B2) {
			desiredPosition = (desiredPosition == 0) ? -170 : 0;
			if (desiredPosition != 0) {
				stopTask(undeploy);
				stopTask(deploy);
				startTask(deploy);
			}
			else {
				stopTask(deploy);
				stopTask(deployWeak);
				startTask(undeploy);
			};
			wait1Msec(500); // ignore button presses for 0.5s
		}
		else if (joystick.joy1_Buttons == B4) {
			reverseTwentyDegrees();
		}
		else if (joystick.joy1_x2 != 0) { // One-motor-only turns
			if (joystick.joy1_x2 < 0) motor[motorB] = -50 * joystick.joy1_x2/-128;
			else motor[motorC] = -50 * joystick.joy1_x2/128;
		} else { // Move based on left joystick values
			float speedFactor = 1.0;
			// Fast
			if (joystick.joy1_Buttons == B1) speedFactor = 2; // 75
			// Slow
			//else if (joystick.joy1_Buttons == L2) speedFactor = 0.5; // 25

			motor[motorB] = (joystick.joy1_y1/127.0 * 50 + joystick.joy1_x1/127.0 * 50) * speedFactor * gearConfiguration;
			motor[motorC] = (joystick.joy1_y1/127.0 * 50 - joystick.joy1_x1/127.0 * 50) * speedFactor * gearConfiguration;

		}

		wait10Msec(1);
	}
}
