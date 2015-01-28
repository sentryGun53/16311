// motion_functions.h


void pivot_left(int speed) {
	motor[motorB] = speed;
	motor[motorC] = -speed*1.1;
}

void pivot_right(int speed) {
	motor[motorB] = -speed*1.1;
	motor[motorC] = speed;
}
