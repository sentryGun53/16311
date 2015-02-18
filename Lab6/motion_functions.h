// motion_functions.h


void pivot_left(int speed) {
	motor[motorB] = -speed;
	motor[motorC] = speed;
}

void pivot_right(int speed) {
	motor[motorB] = speed;
	motor[motorC] = -speed;
}
