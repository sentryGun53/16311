// Motor A goes backward to make theta1 increase
// Motor B goes forward to make theta2 increase
// 11 rounds of the motor is 90 degrees of the arm
// Or 44 rounds for a full circle

// Globals, input goes here
float xA = 3;
float yA = 3;
float xB = 6.25;
float yB = 0;

float theta1A;
float theta2A;
float theta1B;
float theta2B;

float numberOfRoundsForFullCircle = 42;

void calculateAngles() {
	// Calculate angles for position A
	float theta2 = acos((xA*xA + yA*yA - 3.75*3.75 - 2.5*2.5) / (2*3.75*2.5));
	float theta1 = atan2(yA,xA) - asin(2.5*sin(theta2)/sqrt(xA*xA + yA*yA));
  float theta4 = -theta2;
  float theta3 = atan2(yA,xA) - asin(2.5*sin(theta4)/sqrt(xA*xA + yA*yA));

  if (PI < theta1 || theta1 < 0) { // Invalid position
  	theta1A = theta3;
  	theta2A = theta4;
  } else {
  	theta1A = theta1;
  	theta2A = theta2;
  }

	// Calculate angles for position B
  theta2 = acos((xB*xB + yB*yB - 3.75*3.75 - 2.5*2.5) / (2*3.75*2.5));
	theta1 = atan2(yB,xB) - asin(2.5*sin(theta2)/sqrt(xB*xB + yB*yB));
  theta4 = -theta2;
  theta3 = atan2(yB,xB) - asin(2.5*sin(theta4)/sqrt(xB*xB + yB*yB));

  if (PI < theta1 || theta1 < 0) { // Invalid position
  	theta1B = theta3;
  	theta2B = theta4;
  } else {
  	theta1B = theta1;
  	theta2B = theta2;
  }

  nxtDisplayTextLine(0, "%d,%d", theta1A/PI*180, theta2A/PI*180);
  nxtDisplayTextLine(1, "%d,%d", theta1B/PI*180, theta2B/PI*180);
}

float theta1ToEncoderValue(float theta1) {
	return - theta1 / (2 * PI) * numberOfRoundsForFullCircle * 360; // Negative
}

float theta2ToEncoderValue(float theta2) {
	return theta2 / (2 * PI) * numberOfRoundsForFullCircle * 360; // Positive
}

void moveToAnglesLinearly(float theta1, float theta2) {
	float desiredEncoderA = theta1ToEncoderValue(theta1);
	float desiredEncoderB = theta2ToEncoderValue(theta2);

	float differenceA = desiredEncoderA - nMotorEncoder[motorA];
	float differenceB = desiredEncoderB - nMotorEncoder[motorB];

  nxtDisplayTextLine(2, "%d", desiredEncoderA);
  nxtDisplayTextLine(3, "%d", desiredEncoderB);

	if (abs(differenceA) < abs(differenceB)) { // B goes at full speed, A goes at a fraction of that
		while (abs(nMotorEncoder[motorB]) < abs(differenceB)) {
			motor[motorA] = abs(differenceA)/abs(differenceB) * sgn(differenceA) * 75;
			motor[motorB] = 75 * sgn(differenceB);
		}
	} else { // A goes at full speed, B goes at a fraction of that
		while (abs(nMotorEncoder[motorA]) < abs(differenceA)) {
			motor[motorB] = abs(differenceB)/abs(differenceA) * sgn(differenceB) * 75;
			motor[motorA] = 75 * sgn(differenceA);
		}
	}

	motor[motorA] = 0;
	motor[motorB] = 0;
}

void moveToFirstPosition() {
	//nMotorEncoderTarget[motorA] = -720;
	//nMotorEncoderTarget[motorB] = 360;

	/*
	motor[motorA] = -50;
	motor[motorB] = 25;
	*/

	//while (nMotorRunState[motorA] != runStateIdle)
	//while (nMotorRunState[motorB] != runStateIdle)

	// Calculate desired motor encoder values for both A and B to get to desired


	while (nMotorEncoder[motorA] > -3960) {
		motor[motorA] = -50;
		motor[motorB] = 0;
	}

	motor[motorA] = 0;
	motor[motorB] = 0;
}

void moveToSecondPosition() {
}

void moveToThirdPosition() {
}

task main()
{
	// Reset motor encoders
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorB] = 0;

	// Do calculations, set to theta1/2A/B;
	calculateAngles();
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	//moveToFirstPosition()

	moveToAnglesLinearly(theta1A, theta2A);
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	moveToAnglesLinearly(theta1B, theta2B);
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	/*
	moveToAnglesLinearly(theta1A, theta2A);
	while (nNxtButtonPressed != kEnterButton) {}
	*/

	// Move to first position (A) and pause, wait for button press
	//moveToFirstPosition();

	// Move to second position (B) and pause
	//moveToSecondPosition();

	// Move to third position (A) and pause
	//moveToThirdPosition();

	// Reset
	moveToAnglesLinearly(0,0);
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

}
