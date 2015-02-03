#pragma config(Sensor, S3,     lightSensor,    sensorLightActive)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

#include "motion_functions.h"

#define LIGHT_SENSOR_BLACK 36 // originally 37
#define LIGHT_SENSOR_WHITE 59 // originally 53

#define PIVOT_SPEED 18
#define DEAD_ZONE_OFFSET 0.1
#define MAX_SPEED 40
#define MIN_SPEED 30

// 47 seconds regularly, 44.7 seconds one time

task main()
{
	while(true) {
		int sensor_reading = SensorValue[lightSensor];
		if (sensor_reading < LIGHT_SENSOR_BLACK) {
			pivot_left(PIVOT_SPEED);
		}
		else if (sensor_reading > LIGHT_SENSOR_WHITE) {
			pivot_right(PIVOT_SPEED);
		}
		else { // proportional control
			// normalize sensor reading to a value between 0.0 and 1.0, where 0.0 is black and 1.0 is white
			float normalized_reading = (float)(sensor_reading - LIGHT_SENSOR_BLACK) / (float)(LIGHT_SENSOR_WHITE - LIGHT_SENSOR_BLACK);
			if (normalized_reading < 0.5 - DEAD_ZONE_OFFSET) {
				// turn right proportionally
				motor[motorB] = MAX_SPEED;
				motor[motorC] = (MAX_SPEED - MIN_SPEED) * ((normalized_reading) / 0.5) + MIN_SPEED;
				} else if (normalized_reading > 0.5 + DEAD_ZONE_OFFSET) {
				// turn left proportionally
				motor[motorB] = -(MAX_SPEED - MIN_SPEED) * ((normalized_reading) / 0.5) + 2*MAX_SPEED - MIN_SPEED;
				motor[motorC] = MAX_SPEED;
			}
			else { // dead zone, go straight ahead
				motor[motorB] = MAX_SPEED;
				motor[motorC] = MAX_SPEED;
			}
		}
	}
}