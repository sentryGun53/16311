#pragma config(Sensor, S4,     lightSensor,    sensorLightActive)

#include "motion_functions.h"

#define LIGHT_SENSOR_BLACK 45 // 45 for black
#define LIGHT_SENSOR_WHITE 60 // 60 for white

#define PIVOT_SPEED 20
#define DEAD_ZONE_OFFSET 0.01
#define MAX_SPEED 30
#define MIN_SPEED 20

task main()
{
	while (true) {
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
}
