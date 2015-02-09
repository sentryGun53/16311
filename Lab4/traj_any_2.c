#include "measurements.h"

#define PI 3.14159265359


/*
typedef struct {
	float x;
	float y;
} Coordinate;
*/

int option = 1; // Change this to change which curve is drawn
/*
Coordinate positionAtTime(float t) {
	// Takes in time t in seconds
	float x, y;

	switch (option) {
		case 1:
			x = 50 * cos(t/10) * sin(t/10);
			y = 20 * sin(t/10) * sin(t/5);
			break;
		case 3:
			x = 20 * cos(t/10) * cos(t/5);
			y = 20 * cos(3*t/10) * sin(t/10);
			break;
	}

	Coordinate position = {x,y};
	return position;
}
*/

float xPositionAtTime(float t) {
	float x;

	switch (option) {
		case 1:
			x = 50 * cos(t/10) * sin(t/10);
			break;
		case 2:
			x = 20 * sin(3*t/5);
			break;
		case 3:
			x = 20 * cos(t/10) * cos(t/5);
			break;
		case 4:
			x = 20 * (0.5 * cos(3*t/10) - 0.75 * cos(t/5));
			break;
		case 5:
			x = 10 * (-2 * pow(cos(t/5),2) - sin(t/10)+ 1) * sin(t/5);
			break;
		case 6:
			x = 10 * (2 * pow(cos(t/12),3) + 1) * sin(t/4);
			break;
		case 7:
			x = 4 * (5 * cos(9*t/20) - 4 * cos(t/4));
			break;
	}

	return x;
}

float yPositionAtTime(float t) {
	float y;
	switch (option) {
		case 1:
			y = 20 * sin(t/10) * sin(t/5);
			break;
		case 2:
			y = 20 * cos(2*(t/5 + PI/4));
			break;
		case 3:
			y = 20 * cos(3*t/10) * sin(t/10);
			break;
		case 4:
			y = 20 * (-0.75 * sin(t/5) - 0.5 * sin(3*t/10));
			break;
		case 5:
			y = 10 * cos(t/5) * (-2 * pow(cos(t/5),3) - sin(t/10) + 1);
			break;
		case 6:
			y = 10 * cos(t/4) * (1 - 2 * pow(sin(t/4),4));
			break;
		case 7:
			y = 4 * (-4 * sin(t/4) - 5 * sin(9*t/20));
			break;
	}

	return y;
}

// Global variables
//Coordinate initialMarkerPosition = positionAtTime(0.0);
//float robot_X = xPositionAtTime(0.0) - MARKERDISTANCE; // cm
//float robot_Y = yPositionAtTime(0.0); // cm
float robot_X, robot_Y, markerX, markerY;
float robot_TH = 0.0; // radians
int velocityUpdateInterval = 5; // ms
float timeOffset = 0; // handles overflow, in seconds
float stopTime = 0;
//int PIDUpdateInterval = 2;
//int inputB[3] = {0,0,0};
//int inputC[3] = {0,0,0};

//159, -159 is 720 degrees (2 rounds)

float degrees_to_velocity = ((PI/180.0) / (velocityUpdateInterval/1000.0) * R);
// units of cm/s
int thetaBPrev = 0, thetaCPrev = 0;





/*****************************************
 * this part remains important and relevant so the robot knows where it is
 *****************************************/
task trajectory_following()
{
	while(timeOffset + time1[T1]/1000.0 < stopTime)
	{
		/*
		// Code that plots the robot's current position and also prints it out
		// as text
		nxtDisplayTextLine(0, "X: %f", robot_X);
		nxtDisplayTextLine(1, "Y: %f", robot_Y);
		nxtDisplayTextLine(2, "t: %f", 57.2958 * robot_TH);
		nxtDisplayTextLine(3, "mX: %f", markerX);
		nxtDisplayTextLine(5, "mY: %f", markerY);
		wait1Msec(10000);
		*/

		int thetaBNow = nMotorEncoder[motorB];
		int thetaCNow = nMotorEncoder[motorC];

		int deltaThetaB = thetaBNow - thetaBPrev;
		int deltaThetaC = thetaCNow - thetaCPrev;

	 	// Be careful here! Here we are assuming B is the left motor
		// Swap deltaThetaB and deltaThetaC if motors are the other way round
		float vLeft = deltaThetaB * degrees_to_velocity;
		float vRight = deltaThetaC * degrees_to_velocity;
		float v = (vLeft + vRight)/2; // cm/s
		float omega = (vRight - vLeft)/L; // radians/s

		// Runge-Kutta approximation
		float t = velocityUpdateInterval/1000.0;
		float k00 = v * cos(robot_TH);
		float k01 = v * sin(robot_TH);

		float k10 = v * cos(robot_TH + t/2 * omega);
		float k11 = v * sin(robot_TH + t/2 * omega);

		float k30 = v * cos(robot_TH + t * omega);
		float k31 = v * sin(robot_TH + t * omega);

		robot_X += t/6 * (k00 + 4*k10 + k30);
		robot_Y += t/6 * (k01 + 4*k11 + k31);
		robot_TH += t * omega;

		thetaBPrev = thetaBNow;
		thetaCPrev = thetaCNow;




		// Now we know where we are, we want to calculate where we should go,
		// what linear and angular speed we need to go at to get there
		// and what velocities to set motors B and C to to achieve those speeds

		float t2;
		if (time1[T1] > 30000) { //handle overflow
			timeOffset += time1[T1]/1000.0;
			time1[T1] = 0; // reset
		}

		t2 = timeOffset + time1[T1]/1000.0;

		//Coordinate goal = positionAtTime(t2+0.005); // Get position 5 ms from now
		float goalX = xPositionAtTime(t2+0.005);
		float goalY = yPositionAtTime(t2+0.005);

		markerX = robot_X + MARKERDISTANCE * cos(robot_TH);
		markerY = robot_Y + MARKERDISTANCE * sin(robot_TH);

		float xDiff = goalX - markerX;
		float yDiff = goalY - markerY;

		/*
		// Code that plots the robot's current position and also prints it out
		// as text
		nxtDisplayTextLine(0, "X: %f", robot_X);
		nxtDisplayTextLine(1, "Y: %f", robot_Y);
		nxtDisplayTextLine(2, "t: %f", 57.2958 * robot_TH);
		nxtDisplayTextLine(3, "mX: %f", markerX);
		nxtDisplayTextLine(5, "mY: %f", markerY);
		nxtDisplayTextLine(4, "dX: %f", goalX);
		nxtDisplayTextLine(6, "dY: %f", goalY);
		nxtDisplayTextLine(7, "t: %f", t2);
		wait1Msec(60000);
		*/

		// Proportional constant
		float kp = 15; // tweak this  (15 Bob 2-07-14 - was originally 10)
		float kd = 20; // can be used for omega desired  (20 Bob 2-07-14 - was originally 20)
		float vDesired = kp*(cos(robot_TH) * xDiff + sin(robot_TH) * yDiff);
		float omegaDesired = kd*((cos(robot_TH) * yDiff - sin(robot_TH) * xDiff) / L);

		motor[motorB] = 9.0/PI * (2*vDesired - L*omegaDesired) / R;
		motor[motorC] = 9.0/PI * (2*vDesired + L*omegaDesired) / R;

		// Code that plots the robot's current position and also prints it out
		// as text
		nxtSetPixel(50 + (int)(100.0 * robot_X), 32 + (int)(100.0 * robot_Y));
		nxtDisplayTextLine(0, "X: %f", robot_X);
		nxtDisplayTextLine(1, "Y: %f", robot_Y);
		nxtDisplayTextLine(2, "t: %f", 57.2958 * robot_TH);
		nxtDisplayTextLine(3, "mX: %f", markerX);
		nxtDisplayTextLine(5, "mY: %f", markerY);
		nxtDisplayTextLine(4, "dX: %f", goalX);
		nxtDisplayTextLine(6, "dY: %f", goalY);
		nxtDisplayTextLine(7, "t: %f", t2);

		wait1Msec(velocityUpdateInterval);
	}
  motor[motorB] = 0;
	motor[motorC] = 0;
}

/*****************************************
 * Function that draws a grid on the LCD
 * for easier readout of whatever is plot
 *****************************************/
void draw_grid()
{
	for(int i = 0; i < 65; i++)
	{
		nxtSetPixel(50, i);
		int grid5 = (i - 32) % 5;
		int grid10 = (i - 32) % 10;
		if(!grid5 && grid10)
		{
			for(int j = -2; j < 3; j++)
			{
				nxtSetPixel(50 + j, i);
			}
		}
		else if(!grid10)
		{
			for(int j = -4; j < 5; j++)
			{
				nxtSetPixel(50 + j, i);
			}
		}
	}
	for(int i = 0; i < 101; i++)
	{
		nxtSetPixel(i, 32);
		int grid5 = (i - 100) % 5;
		int grid10 = (i - 100) % 10;
		if(!grid5 && grid10)
		{
			for(int j = -2; j < 3; j++)
			{
				nxtSetPixel(i, 32 + j);
			}
		}
		else if(!grid10)
		{
			for(int j = -4; j < 5; j++)
			{
				nxtSetPixel(i, 32 + j);
			}
		}
	}
}

/*****************************************
 * Function that fills in the inputB and
 * inputC arrays with values inputted
 * through motorB
 *****************************************/

void getInput()
{
	nMotorEncoder[motorB] = 0;
	nNxtButtonTask = 0;

	while(nNxtButtonPressed != kEnterButton)
	{
		option = (int)(nMotorEncoder[motorB] / 10.0);
		if (option > 7) {
			option = 7;
	} else if (option < 1) {
			option = 1;
		}
		nxtDisplayTextLine(2 * 0, "B : %d", option);
		wait1Msec(10);
	}
	wait1Msec(300);

	robot_X = xPositionAtTime(0.0) - MARKERDISTANCE; // cm
	robot_Y = yPositionAtTime(0.0); // cm
	markerX = robot_X + MARKERDISTANCE * cos(robot_TH);
	markerY = robot_Y + MARKERDISTANCE * sin(robot_TH);

	if (option == 1 || option == 3 || option == 4 || option == 5) stopTime = 62.83;
	else if (option == 2) stopTime = 31.42;
	else if (option == 6) stopTime = 75.4;
	else if (option == 7) stopTime = 125.66;

	for(int j = 0; j < 8; j++)
		nxtDisplayClearTextLine(j);
}


/*****************************************
 * Main function - it is not necessary to
 * modify this
 *****************************************/
task main()
{
  /* Reset encoders and turn on PID control */
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	nMotorPIDSpeedCtrl[motorB] = mtrSpeedReg;
	nMotorPIDSpeedCtrl[motorC] = mtrSpeedReg;
  	//nPidUpdateInterval = PIDUpdateInterval;

	getInput();
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;

	draw_grid();


	time1[T1] = 0; // reset timer

	// This tells us where we are and where we're facing, setting global
	// variables robot_X, robot_Y, robot_TH
	startTask(trajectory_following);

	while(nNxtButtonPressed != kExitButton) {}
}
