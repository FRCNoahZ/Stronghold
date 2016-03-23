#include "WPILib.h"

#include "DriveTrain.h"
#include "Constants.h"
#include "Shooter.h"
#include "Position.h"
#include "Aimer.h"
#include "Arm.h"

#include <math.h>
#include <thread>
#include <fstream>

#ifndef SRC_ROBOT_H_
#define SRC_ROBOT_H_

class Robot : public SampleRobot {
	DriveTrain driveTrain;
	Joystick driveStick;
	Joystick operatorStick;
	Shooter shooter;
	Position position;
	Aimer aimer;
	Arm arm;
	Servo servo;

	//navx-mxp PID related stuff
	//
	//
	Timer timer;
	AHRS *ahrs;                         // navX-MXP
	PIDController *turnController;      // PID Controller
	double rotateToAngleRate;           // Current rotation rate
	//
	//  constants for the navx library PID
			const static double kP;
			const static double kI;
			const static double kD;
			const static double kF;
			double minShooterLimit = 30;

			/* This tuning parameter indicates how close to "on target" the    */
			/* PID Controller will attempt to get.                             */

			double kToleranceDegrees;
			bool runTJCode;

public:
	Robot();
	void OperatorControl();
	void Autonomous();
};

#endif /* SRC_ROBOT_H_ */
