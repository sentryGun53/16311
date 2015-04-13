// motion_functions.h


void pivot_left(int speed) {
	motor[motorB] = -speed * 1.5;
	motor[motorC] = speed * 0.5;
}

void pivot_right(int speed) {
	motor[motorB] = speed * 0.5;
	motor[motorC] = -speed * 1.5;
}
