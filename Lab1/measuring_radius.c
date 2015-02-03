task main()
{
	// Go forwards (run both motors) for x rounds
	int nRounds = 10;
	nMotorEncoderTarget[motorB] = nRounds * 360;
	nMotorEncoderTarget[motorC] = nRounds * 360;

	motor[motorB] = 25;
	motor[motorC] = 25;

	while(nMotorRunState[motorB] != runStateIdle || nMotorRunState[motorC] != runStateIdle) {
		wait1Msec(1);
	}
}
