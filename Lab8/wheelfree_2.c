#pragma DebuggerWindows("joystickSimple")

#include "JoystickDriver.c"

#define L1 16
#define L2 64
#define LD 1024 // Left stick pressed
#define R1 32
#define R2 128
#define B1 1
#define B2 2
#define B3 4
#define B4 8




int gearConfiguration = -1;

void standUp() { // Once you're here you can never leave
	nSyncedMotors = synchBC;
	while (true) {
		if (nMotorEncoder[motorB] < 34*90/24 - 2) {
			motor[motorB] = 80;
			//motor[motorC] = 30;
		} else if (nMotorEncoder[motorB] > 34*90/24 + 2) {
			motor[motorB] = -80;
			//motor[motorC] = -30;
		} else {
			motor[motorB] = 0;
			//motor[motorC] = 0;
			motor[motorA] = 100;
		}
		wait1Msec(1);
	}
}

void pivotLeft() {
	/*
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorB] = 38*180/24;
	nMotorEncoderTarget[motorC] = 38*180/24;
	*/

	motor[motorB] = -50;
	motor[motorC] = 50;

	wait1Msec(600);

	//while (nMotorRunState[motorC] != runStateIdle)
	//while (nMotorRunState[motorB] != runStateIdle)

	motor[motorB] = 0;
	motor[motorC] = 0;

}

void pivotRight() {
	/*
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;

	nMotorEncoderTarget[motorB] = -38*180/24;
	nMotorEncoderTarget[motorC] = -38*180/24;
	*/

	motor[motorB] = 50;
	motor[motorC] = -50;

	//while (nMotorRunState[motorC] != runStateIdle)
	//while (nMotorRunState[motorB] != runStateIdle)
	wait1Msec(600);

	motor[motorB] = 0;
	motor[motorC] = 0;

}

task main()
{
	nMotorEncoder[motorA] = 0;

	while (true) {
		getJoystickSettings(joystick);

		int speedFactor = 1;

		if (joystick.joy1_Buttons == B1) {
			//standUp();
			motor[motorA] = 10;
		} else if (joystick.joy1_Buttons == B2) {
			motor[motorA] = -10;
		} else {
			motor[motorA] = 0;
		}

		if (joystick.joy1_x2 != 0) { // Pivot
			// Old code
			motor[motorB] = 50 * joystick.joy1_x2/128;
			motor[motorC] = -50 * joystick.joy1_x2/128;


			/*
			if (joystick.joy1_x2 > 0) {
				// Turn left
				pivotLeft();
			} else {
				// Turn right
				pivotRight();
			}
			*/
		}
		else {
			 // Ignore miscalibration
			if (abs(joystick.joy1_y1) < 5) {
				motor[motorB] = 0;
				motor[motorC] = 0;
			} else {
				motor[motorB] = (joystick.joy1_y1/127.0 * 80 * speedFactor * gearConfiguration);
				motor[motorC] = (joystick.joy1_y1/127.0 * 80 * speedFactor * gearConfiguration);
			}
		}

		wait1Msec(10);
	}
}
