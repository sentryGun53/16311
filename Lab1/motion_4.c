// motion_2.c: Takes in input using motorB

//#include "measurements_inches.h"
#define L 4.154 // inches (4.1 Adriel 2-13-15) (4.201 Adriel 2-10-15)
#define R 1.077 // inches (1.076285 Adriel 2-13-15) (1.0776 Adriel 2-10-15)
#include "math_functions.h"


//#define PI 3.14159265358979323846264338327

int velocityUpdateInterval = 5; // ms
int PIDUpdateInterval = 2;
float startPosition[3] = {0, 0, 0}; // (x,y,theta)
float robotX = startPosition[0], robotY = startPosition[1], robotTH = startPosition[2]; // starting position?
float endPosition[2] = {0, 0}; // input desired goal here (x,y)

float degrees_to_velocity = ((PI/180.0) / (velocityUpdateInterval/1000.0) * R); // units of inches/s
int thetaBPrev = 0, thetaCPrev = 0;

int nWaypoints = 8;
float waypoints[8][2] = {{18.2,39.2},{40.2,39.8},{52.4,41.1},{64.6,25.7},{40.9,7.2},{28.7,7.1},{14.8,7.7},{80.0,10.6}};
//float neighbourList[8] = {{1,},    {0,2},      {1,3},      {2,4,7},    {3,5},     {4,6},     {5,},      {3,}};



/*****************************************
 * continuously updates the robot's position
 *****************************************/
task dead_reckoning()
{
    while(true)
    {
        //
        //Fill in code for numerical integration / position estimation here
        //

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
        float k00 = v * cos(robotTH);
        float k01 = v * sin(robotTH);

        float k10 = v * cos(robotTH + t/2 * omega);
        float k11 = v * sin(robotTH + t/2 * omega);

        float k30 = v * cos(robotTH + t * omega);
        float k31 = v * sin(robotTH + t * omega);

        robotX += t/6 * (k00 + 4*k10 + k30);
        robotY += t/6 * (k01 + 4*k11 + k31);
        robotTH += t * omega;

        thetaBPrev = thetaBNow;
        thetaCPrev = thetaCNow;

        //Code that plots the robot's current position and also prints it out as text
        //nxtSetPixel(50 + (int)(100.0 * robotX), 32 + (int)(100.0 * robotY)); // This may be useful
        nxtDisplayTextLine(0, "X: %f", robotX);
        nxtDisplayTextLine(1, "Y: %f", robotY);
        nxtDisplayTextLine(2, "t: %f", 57.2958 * robotTH);

        wait1Msec(velocityUpdateInterval);
    }
}


// To implement
/*****************************************
 * Function that fills in the startPosition and endPosition arrays with values inputted
 * through motorA
 *****************************************/
void getInput()
{
    nMotorEncoder[motorA] = 0;
    nNxtButtonTask = 0;

    while(nNxtButtonPressed != kEnterButton)
    {
        startPosition[0] = max(min((int)(nMotorEncoder[motorA] / 10.0),96),0);
        nxtDisplayTextLine(0, "Start x : %d", startPosition[0]);
        wait1Msec(10);
    }
    nMotorEncoder[motorA] = 0;
    wait1Msec(300);

    while(nNxtButtonPressed != kEnterButton)
    {
        startPosition[1] = max(min((int)(nMotorEncoder[motorA] / 10.0),48),0);
        nxtDisplayTextLine(1, "Start y : %d", startPosition[1]);
        wait1Msec(10);
    }
    nMotorEncoder[motorA] = 0;
    wait1Msec(300);

    while(nNxtButtonPressed != kEnterButton)
    {
    		int tDegrees = max(min(((nMotorEncoder[motorA] / 10.0),360),0);
        startPosition[2] = tDegrees/57.2958;
        nxtDisplayTextLine(2, "Start t : %d", startPosition[2] * 57.2958);
        wait1Msec(10);
    }
    nMotorEncoder[motorA] = 0;
    wait1Msec(300);

    while(nNxtButtonPressed != kEnterButton)
    {
        endPosition[0] = max(min((int)(nMotorEncoder[motorA] / 10.0),96),0);
        nxtDisplayTextLine(3, "End x : %d", endPosition[0]);
        wait1Msec(10);
    }
    nMotorEncoder[motorA] = 0;
    wait1Msec(300);

    while(nNxtButtonPressed != kEnterButton)
    {
        endPosition[1] = max(min((int)(nMotorEncoder[motorA] / 10.0),48),0);
        nxtDisplayTextLine(4, "End y : %d", endPosition[1]);
        wait1Msec(10);
    }
    nMotorEncoder[motorA] = 0;
    wait1Msec(300);




    // Clear screen
    for(int j = 0; j < 8; j++)
        nxtDisplayClearTextLine(j);

    // Reset to 0!
    nMotorEncoder[motorB] = 0;
    nMotorEncoder[motorC] = 0;
}






float calculateDistance(float x, float y) {
    return sqrt(pow(x,2) + pow(y,2));
}

float calculateBearing(float x, float y) {
    if (0 <= x && 0 <= y) {
        return atan(y/x);
    } else if (x < 0 && 0 <= y) {
        return PI + atan(y/x);
    } else if (x < 0 && y < 0) {
        return atan(y/x) - PI;
    } else if (0 <= x && y < 0) {
        return atan(y/x);
    }
    return 0;
}










void moveToPosition(float x, float y) {

    nxtDisplayTextLine(3, "Goal: (%.1f,%.1f)",x,y);

    float distanceThreshold = 0.1; // Not too small or will overshoot!

    float dx = x - robotX;
    float dy = y - robotY;
    float distanceToGoal = calculateDistance(dx,dy);

    while(distanceToGoal > distanceThreshold) {
    		playTone((40-distanceToGoal)/40*700+300, 3);

        // Calculate bearing
        float bearing = calculateBearing(dx, dy);
        nxtDisplayTextLine(4, "Bearing: %.2f", bearing * 57.2957795);

        float difference = bearing - robotTH;
        if (difference > PI) difference -= 2*pi;
        else if (difference < -PI) difference += 2*pi;
        nxtDisplayTextLine(4, "Difference: %.2f", difference * 57.2957795);

        // These angles can be tweaked
        float hardTurnThreshold = 0.1; // radians
        float slightTurnThreshold = 0.035; // radians

        int maxPower = 30;
        if (difference > hardTurnThreshold) { // Correct bearing is to the left, turn left
            motor[motorB] = -maxPower/2;
            motor[motorC] = maxPower/2;
        } else if (difference < -hardTurnThreshold) { // Correct bearing is to the right, turn right
            motor[motorB] = maxPower/2;
            motor[motorC] = -maxPower/2;
        } else if (difference > slightTurnThreshold) {
            motor[motorB] = maxPower-5;
            motor[motorC] = maxPower;
        } else if (difference < -slightTurnThreshold ) {
            motor[motorB] = maxPower;
            motor[motorC] = maxPower-5;
        } else {
            motor[motorB] = maxPower;
            motor[motorC] = maxPower;
        }

        // Might wanna have
        wait1Msec(25); // here

        dx = x - robotX;
        dy = y - robotY;
        distanceToGoal = calculateDistance(dx,dy);


    }
    //motor[motorB] = 0;
    //motor[motorC] = 0;
}




int findNearestWaypoint(float x, float y) {
    // Returns index of waypoint nearest to (x,y)
    float minDistance = 107; // Distances shouldn't exceed this
    float result = 0;

    for (int i=0; i < nWaypoints; i++) {
        float dx = x - waypoints[i][0];
        float dy = y - waypoints[i][1];
        float d = calculateDistance(dx,dy);
        if (d < minDistance) {
            minDistance = d;
            result = i;
        }
    }

    return result;
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
    nPidUpdateInterval = PIDUpdateInterval;

    getInput();
    // reset encoder values
    nMotorEncoder[motorB] = 0;
    nMotorEncoder[motorC] = 0;
		robotX = startPosition[0];
		robotY = startPosition[1];
		robotTH = startPosition[2];

    // Decide waypoint path
    int firstWaypointIndex = findNearestWaypoint(robotX,robotY);
    int lastWaypointIndex = findNearestWaypoint(endPosition[0], endPosition[1]);
    // Print results, wait for 5s
    nxtDisplayTextLine(0, "1st:%d (%.1f,%.1f)", firstWaypointIndex, waypoints[firstWaypointIndex][0], waypoints[firstWaypointIndex][1]);
    nxtDisplayTextLine(1, "Last:%d (%.1f,%.1f)", lastWaypointIndex, waypoints[lastWaypointIndex][0], waypoints[lastWaypointIndex][1]);


    time1[T1] = 0;
    startTask(dead_reckoning);

    int nWaypointsAlongPath = lastWaypointIndex - firstWaypointIndex;
    if (nWaypointsAlongPath == 0) { // Travel to waypoint, then to goal
        moveToPosition(waypoints[firstWaypointIndex][0], waypoints[firstWaypointIndex][1]);

    } else {

        // Handle edge case of waypoint 7
        bool travelToWaypoint7 = false;
        if (firstWaypointIndex == 7) {
            // Move to 3, then carry on
            moveToPosition(waypoints[7][0], waypoints[7][1]);
            firstWaypointIndex = 3;
        } else if (lastWaypointIndex == 7) {
            travelToWaypoint7 = true;
            lastWaypointIndex = 3;
        }

        // Then do normal stuff
        if (firstWaypointIndex < lastWaypointIndex) { // Go from small to big
            for (int i=firstWaypointIndex; i < lastWaypointIndex+1; i++) {
                moveToPosition(waypoints[i][0], waypoints[i][1]);
            }
        } else if (lastWaypointIndex < firstWaypointIndex) { // Go from big to small
            for (int i=firstWaypointIndex; i > lastWaypointIndex-1; i--) {
                moveToPosition(waypoints[i][0], waypoints[i][1]);
            }
        }

        // Handling edge case
        if (travelToWaypoint7) {
            moveToPosition(waypoints[7][0], waypoints[7][1]);
        }
    }

    // Finally, move to goal
    moveToPosition(endPosition[0], endPosition[1]);


    motor[motorB] = 0;
    motor[motorC] = 0;
    nNxtButtonTask  = 0;


    //while(nNxtButtonPressed != kExitButton) {}
}
