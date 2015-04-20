// Motor A goes backward to make theta1 increase
// Motor B goes forward to make theta2 increase
// 10 rounds of the motor is 90 degrees of the arm
// Or 40 rounds for a full circle

// Globals, input goes here
float xA = 0;
float yA = 6.25;
float xB = 4;
float yB = 4;

float theta1A;
float theta2A;
float theta1B;
float theta2B;
float theta1WaypointA;
float theta2WaypointA;
float theta1WaypointB;
float theta2WaypointB;

float numberOfRoundsForFullCircle = 40;

void calculateAnglesAndNearestWaypoints() {
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

  // Waypoints: (0.49, 0.78), (pi/2, pi), (pi-0.49, 2*pi-0.78), (2.16, 0.95), (pi-2.16, 2*pi-0.95)
  // Calculate nearest waypoint for pos A
  float min = sqrt((0.49-theta1A)*(0.49-theta2A) + (0.78-theta2A)*(0.78-theta2A));
  theta1WaypointA = 0.49;
  theta2WaypointA = 0.78;
  float test = sqrt((PI/2-theta1A)*(PI/2-theta2A) + (PI-theta2A)*(PI-theta2A));
  if (test < min) { min = test; theta1WaypointA = PI/2; theta2WaypointA = PI; }
  test = sqrt((PI-0.49-theta1A)*(PI-0.49-theta2A) + (2*PI-0.78-theta2A)*(2*PI-0.78-theta2A));
  if (test < min) { min = test; theta1WaypointA = PI/2; theta2WaypointA = PI; }
  test = sqrt((2.16-theta1A)*(2.16-theta2A) + (0.95-theta2A)*(0.95-theta2A));
  if (test < min) { min = test; theta1WaypointA = 2.16; theta2WaypointA = 0.95; }
  test = sqrt((PI-2.16-theta1A)*(PI-2.16-theta2A) + (2*PI-0.95-theta2A)*(2*PI-0.95-theta2A));
  if (test < min) { min = test; theta1WaypointA = PI-2.16; theta2WaypointA = 2*PI-0.95; }

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

  // Calculate nearest waypoint for pos B
  min = sqrt((0.49-theta1B)*(0.49-theta2B) + (0.78-theta2B)*(0.78-theta2B));
  theta1WaypointB = 0.49;
  theta2WaypointB = 0.78;
  test = sqrt((PI/2-theta1B)*(PI/2-theta2B) + (PI-theta2B)*(PI-theta2B));
  if (test < min) { min = test; theta1WaypointB = PI/2; theta2WaypointB = PI; }
  test = sqrt((PI-0.49-theta1B)*(PI-0.49-theta2B) + (2*PI-0.78-theta2B)*(2*PI-0.78-theta2B));
  if (test < min) { min = test; theta1WaypointB = PI/2; theta2WaypointB = PI; }
  test = sqrt((2.16-theta1B)*(2.16-theta2B) + (0.95-theta2B)*(0.95-theta2B));
  if (test < min) { min = test; theta1WaypointB = 2.16; theta2WaypointB = 0.95; }
  test = sqrt((PI-2.16-theta1B)*(PI-2.16-theta2B) + (2*PI-0.95-theta2B)*(2*PI-0.95-theta2B));
  if (test < min) { min = test; theta1WaypointB = PI-2.16; theta2WaypointB = 2*PI-0.95; }

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

  nxtDisplayTextLine(2, "%d", differenceA);
  nxtDisplayTextLine(3, "%d", differenceB);

	if (abs(differenceA) < abs(differenceB)) { // B goes at full speed, A goes at a fraction of that
		if (differenceB < 0) { // desiredEncoderB < nMotorEncoder[motorB]
			while (desiredEncoderB < nMotorEncoder[motorB]) {
				motor[motorA] = abs(differenceA)/abs(differenceB) * sgn(differenceA) * 75;
				motor[motorB] = 75 * sgn(differenceB);
			}
		} else { // desiredEncoderB > nMotorEncoder[motorB]
			while (desiredEncoderB > nMotorEncoder[motorB]) {
				motor[motorA] = abs(differenceA)/abs(differenceB) * sgn(differenceA) * 75;
				motor[motorB] = 75 * sgn(differenceB);
			}
		}
	} else { // A goes at full speed, B goes at a fraction of that
		if (differenceA < 0) { // desiredEncoderA < nMotorEncoder[motorA]
			while (desiredEncoderA < nMotorEncoder[motorA]) {
				motor[motorB] = abs(differenceB)/abs(differenceA) * sgn(differenceB) * 75;
				motor[motorA] = 75 * sgn(differenceA);
			}
		} else { // desiredEncoderA > nMotorEncoder[motorA]
			while (desiredEncoderA > nMotorEncoder[motorA]) {
				motor[motorB] = abs(differenceB)/abs(differenceA) * sgn(differenceB) * 75;
				motor[motorA] = 75 * sgn(differenceA);
			}
		}
	}

	motor[motorA] = 0;
	motor[motorB] = 0;
}

void moveToFirstPosition() {
	// Move to nearest waypoint
	moveToAnglesLinearly(0.49, 0.78);
	// Move to central waypoint
	moveToAnglesLinearly(PI/2, PI);
	// Move to furthest waypoint
	moveToAnglesLinearly(theta1WaypointA, theta2WaypointA);
	// Move to goal
	moveToAnglesLinearly(theta1A, theta2A);
}

void moveToSecondPosition() {
	// Move to nearest waypoint
	moveToAnglesLinearly(theta1WaypointA, theta2WaypointA);
	// Move to central waypoint
	moveToAnglesLinearly(PI/2, PI);
	// Move to furthest waypoint
	moveToAnglesLinearly(theta1WaypointB, theta2WaypointB);
	// Move to goal
	moveToAnglesLinearly(theta1B, theta2B);
}

void moveToThirdPosition() {
	// Move to nearest waypoint
	moveToAnglesLinearly(theta1WaypointB, theta2WaypointB);
	// Move to central waypoint
	moveToAnglesLinearly(PI/2, PI);
	// Move to furthest waypoint
	moveToAnglesLinearly(theta1WaypointA, theta2WaypointA);
	// Move to goal
	moveToAnglesLinearly(theta1A, theta2A);
}

task main()
{
	// Reset motor encoders
	nMotorEncoder[motorA] = 0;
	nMotorEncoder[motorB] = 0;

	// Do calculations, set to theta1/2A/B and theta1/2WaypointA/B;
	calculateAnglesAndNearestWaypoints();
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	moveToFirstPosition();
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	moveToSecondPosition();
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	moveToThirdPosition();
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	/*
	moveToAnglesLinearly(theta1A, theta2A);
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	moveToAnglesLinearly(theta1B, theta2B);
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

	moveToAnglesLinearly(theta1A, theta2A);
	while (nNxtButtonPressed != kEnterButton) {}
	*/

	// Reset to origin
	moveToAnglesLinearly(0,0);
	while (nNxtButtonPressed != kEnterButton) {}
  wait1Msec(300);

}
