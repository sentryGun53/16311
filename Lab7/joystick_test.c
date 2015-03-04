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

task main()
{
	while (true) {
		getJoystickSettings(joystick);


		nxtDisplayTextLine(0, "%d", joystick.joy1_x1);
		nxtDisplayTextLine(1, "%d", joystick.joy1_y1);
		nxtDisplayTextLine(2, "%d", joystick.joy1_Buttons);


		if (joystick.joy1_Buttons == 8 || joystick.joy1_Buttons == 2) { // Right trigger is pressed left or right
				// Turn
			// Todo: check for whether L1 is pressed
			if (joystick.joy1_Buttons == 8) {
				motor[motorB] = -25;
				motor[motorC] = 25;
			} else {
				motor[motorB] = 25;
				motor[motorC] = -25;
			}
		}

		else { // Go forwards/backwards
			if (joystick.joy1_Buttons == LD) {
				motor[motorB] = joystick.joy1_y1/127 * 75;
				motor[motorC] = joystick.joy1_y1/127 * 75;
			} else if (joystick.joy1_Buttons == L1) {
				motor[motorB] = joystick.joy1_y1/127 * 25;
				motor[motorC] = joystick.joy1_y1/127 * 25;
			} else {
				motor[motorB] = joystick.joy1_y1/127 * 50;
				motor[motorC] = joystick.joy1_y1/127 * 50;
			}
		}

		wait1Msec(10);
	}
}
