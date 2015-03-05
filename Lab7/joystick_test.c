#include "JoystickDriver.c"
#pragma DebuggerWindows("joystickSimple")

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

task main()
{
	while (true) {
		//motor[motorA] = -50; // Can only go backwards
		getJoystickSettings(joystick);


		nxtDisplayTextLine(0, "%d", joystick.joy1_x1);
		nxtDisplayTextLine(1, "%d", joystick.joy1_y1);
		nxtDisplayTextLine(2, "%d", joystick.joy1_Buttons);

		/*
		// Flipper idea
		if (joystick.joy1_Buttons == R1) motor[motorA] = -50;
		else if (joystick.joy1_Buttons == R2) motor[motorA] = 50;
		else motor[motorA] = 0;
		*/

		if (joystick.joy1_Buttons == 8 || joystick.joy1_Buttons == 2 || joystick.joy1_Buttons == 24 || joystick.joy1_Buttons == 18) { // Right trigger is pressed left or right
			// Turn
			if (joystick.joy1_Buttons == 8) { // Turn left
				motor[motorB] = -25;
				motor[motorC] = 25;
			} else if (joystick.joy1_Buttons == 24) { // 24 = 8 + 16 Turn left slowly
				motor[motorB] = -10;
				motor[motorC] = 10;
			} else if (joystick.joy1_Buttons == 2) { // Turn right
				motor[motorB] = 25;
				motor[motorC] = -25;
			} else { // Turn right slowly
				motor[motorB] = 10;
				motor[motorC] = -10;
			}
		}

		else { // Go forwards/backwards
			// 50 is default speed
			double speedFactor = 1.0;
			if (joystick.joy1_Buttons == LD) speedFactor = 1.5; // 75
			else if (joystick.joy1_Buttons == L1) speedFactor = 0.5; // 25
			else if (joystick.joy1_Buttons == L2) speedFactor = 0.2; // 10

			motor[motorB] = joystick.joy1_y1/127 * 50 * speedFactor;
			motor[motorC] = joystick.joy1_y1/127 * 50 * speedFactor;
		}

		wait1Msec(10);
	}
}
