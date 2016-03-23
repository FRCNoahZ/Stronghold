#include "Robot.h"

void updateThreadFunction(bool *keepRunning, Joystick *driveStick, Position *position) { //will need to be updated if drivesticks are updated
	//TODO: Incorporate the cantalon to be able to sent the ticks
	bool movingForward = false;
	while (*keepRunning == true)
	{
		if (driveStick->GetY() > 0)
		{
			movingForward = true;
		}
		if (!position->IsTurning())
		{
			//position->Update(movingForward);
			Wait(.02);
		}
	}
}

void autoUpdateThreadFunction(bool *keepRunning, Position *position) { //will need to be updated if drivesticks are updated
	//TODO: Incorporate the cantalon to be able to sent the ticks
	if (!position->IsTurning())
	{
		//position->Update(movingForward);
		Wait(.02);
	}
}

Robot::Robot() :
	driveTrain(Constants::driveLeftMasterID, Constants::driveLeftSlaveID, Constants::driveRightMasterID, Constants::driveRightSlaveID, &position),
	driveStick(Constants::driveJoystickChannel),
	operatorStick(1),
	shooter(Constants::shooterLeftTalonID, Constants::shooterRightTalonID, Constants::shooterAimTalonID, &position),
	position(),
	aimer(),
	arm(Constants::armTalonPin),
	servo(0)
{
	driveTrain.SetExpiration(0.1); // safety feature
	CameraServer::GetInstance()->SetQuality(50);
	CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	CameraServer::GetInstance()->StartAutomaticCapture("cam1");
}

void Robot::OperatorControl() //teleop code
{
	driveTrain.SetSafetyEnabled(false);
	bool updateThreadRun = true;
	//std::thread updateThread(updateThreadFunction, &updateThreadRun, &driveStick, &position);
	float throttle;
	float leftMoveValue;
	float rightMoveValue;
	float rotateValue;
	float distToTower;
	float angleToShoot;
	float angleToTower;
	bool readyToShoot = false;
	bool shooterPreparing = false;

	shooter.Enable();
	driveTrain.Enable();
	arm.Enable();

	while (IsOperatorControl() && IsEnabled())
	{
		throttle = (((driveStick.GetRawAxis(Constants::driveL2)) + 1.0)/4.0) + 0.5; //[1, .5]
		leftMoveValue = throttle * driveStick.GetRawAxis(1);
		rightMoveValue = -throttle * driveStick.GetRawAxis(5);

		SmartDashboard::PutNumber("Throttle Value", throttle);
		SmartDashboard::PutNumber("Left Move Value", leftMoveValue);
		SmartDashboard::PutNumber("Right Move Value", rightMoveValue);
		driveTrain.TankDrive(leftMoveValue, rightMoveValue, false);

		/*		if (shooterPreparing)
				{
				readyToShoot = (abs(shooter.WheelSpeed() - 1.0) < 0.01) && (abs(shooter.Angle() - angleToTower) < 0.1);

				if (readyToShoot)
				{
				shooterPreparing = false;
				}
				}
				*/
		if (driveStick.GetRawButton(Constants::calibrateButton))
		{
			//position.Calibrate();
		}
		/*		if (operatorStick.GetRawButton(Constants::prepareToShootButton))
				{
				if (shooter.HasBall())
				{
				shooterPreparing = true;
				shooter.PrepareShooter();	
				angleToTower = position.AngleToTower();
				driveTrain.TurnToRelativeAngle(angleToTower);
				angleToTower = aimer.GetAngleToTower();
				driveTrain.TurnToRelativeAngle(angleToTower);
				angleToShoot = aimer.GetAngleToShoot();
				distToTower = aimer.GetDistanceToTower();
				shooter.PrepareShooter(angleToShoot, 1.0);
				}
				}
				*/
		if (operatorStick.GetRawButton(Constants::shootButton))
		{
			if (readyToShoot)
			{
				readyToShoot = false;
				shooter.Shoot();
			}
		}
		if (operatorStick.GetRawButton(Constants::xButton))
		{
			shooter.SetSpeed(1);
			Wait(.5);
			shooter.Shoot();
			Wait(.5);
			shooter.SetSpeed(0);
		}
		float shooterAngleInput = -operatorStick.GetRawAxis(1);
		shooterAngleInput = abs(shooterAngleInput) > 0.005 ? shooterAngleInput : 0.0;
		shooter.Move(shooterAngleInput);

		float manualMoveBeltInput = operatorStick.GetRawAxis(5);
		manualMoveBeltInput = abs(manualMoveBeltInput) > 0.005 ? manualMoveBeltInput : 0.0;
		arm.ManualMoveBelt(manualMoveBeltInput / 5);

		if (operatorStick.GetRawButton(Constants::stopShooterWheels))
		{
			shooter.SetSpeed(0);
		}
		if (operatorStick.GetPOV() == 0)
		{
			shooter.SetSpeed(1);
		}
		else if (operatorStick.GetPOV() == 180)
		{
			shooter.LoadBall();
		}
		if (operatorStick.GetRawButton(Constants::ejectButton)) {
			shooter.SetSpeed(.4);
			Wait(.05);
			shooter.Shoot();
		}
		/*if (operatorStick.GetRawButton(5)) {
			shooter.SetAngle(20);
		}
		if (operatorStick.GetRawButton(6)) {
			shooter.SetAngle(50);
		}
		if (operatorStick.GetRawButton(3)) {
			driveTrain.TurnToAngle(30);
		}*/
		if (operatorStick.GetRawButton(3)) {
			driveTrain.TurnToAngle(30);
		}
		SmartDashboard::PutNumber("getPOV", operatorStick.GetPOV());
		SmartDashboard::PutNumber("xPos", position.GetX());
		SmartDashboard::PutNumber("yPos", position.GetY());
		SmartDashboard::PutNumber("Angle", position.GetAngleDegrees());
		SmartDashboard::PutString("Version", "0.9");
		SmartDashboard::PutBoolean("Has Ball", shooter.HasBall());
		SmartDashboard::PutNumber("Shooter Angle", shooter.Angle());
		shooter.ReadPot();
	}

	shooter.Disable();
	driveTrain.Disable();
	arm.Disable();

	updateThreadRun = false;
	//updateThread.join();

	driveTrain.SetSafetyEnabled(true);
}

void Robot::Autonomous()
{
	driveTrain.SetSafetyEnabled(false);
	driveTrain.Enable();
/*	//bool updateThreadRun = true;
	//std::thread updateThread(autoUpdateThreadFunction, &updateThreadRun, &autoPosition);
	std::fstream logfile; logfile.open("logfile.txt", std::fstream::out);
	//Timer timer;
	//timer.Start();
	//timer.Reset();
	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("datatable");
	int startPos = table->GetNumber("startPos", 3);
	float timeTo10 = table->GetNumber("timeTo10", 3);
	float over9000 = table->GetNumber("powerLevel", 0.75);
	logfile << startPos << " " << timeTo10 << " " << over9000 << std::endl;
	//  .net rules!!!!!!!!!!!!
		int failSafe = 0;
		if(startPos == 0|| timeTo10 == 0||over9000 == 0)
		{
			shooter.Disable();
				driveTrain.Disable();
				logfile << "Auto ended" << std::endl;
				logfile.close();
			logfile<<"ERROR.  RETURNING WITHOUT REGRETS BECAUSE ONE OF THE VALUES WAS SET TO 0!!!"<<std::endl;
			return;
		}
		//drive over defense
		logfile<<"Over the mountain" << std::endl;
	driveTrain.TankDrive(-.9, .885);
	Wait(1.5);
	driveTrain.TankDrive(0.0, 0.0);
	//turn 180 unless it's at the ends then turn  145
	logfile << "Twist'n, baby!"<< std::endl;
	float fraction = 1;
	if (startPos == 1 || startPos == 5)
	{
		if(startPos == 1)
			fraction = 215;
		else fraction = 145;
	}else fraction = 180;
	//timer.Reset();
	float timeToTurn = .75;
	//driveTrain.ArcadeDrive(0.0, fraction / 180, false);
	/*while(timer.Get() <= timeToTurn && failSafe < 200)
	{
		Wait(0.01);
		failSafe++;
		logfile<<"loop 1: "<<failSafe;

	}*/
/*	if (position.GetAngle() < fraction) {
		while (position.GetAngle() < fraction) {
			driveTrain.TankDrive(-.25, -.25);
		}
	} else if (position.GetAngle() > fraction) {
		while (position.GetAngle() > fraction) {
			driveTrain.TankDrive(.25, .25);
		}
	}
	//This does nothing.  Kill it.
	//driveTrain.TankDrive(0.0, 0.0);
	float output1;
	double angleToTurn = fraction; //tj
	double startAngle = position.GetAngleDegrees();
	double currentAngle = position.GetAngleDegrees() - startAngle;
	while (abs((currentAngle - startAngle) - angleToTurn) > 2 && failSafe <= 500)
	{
		SmartDashboard::PutString("Status", "Going through da loop.");
		SmartDashboard::PutNumber("Autonomous loop current angle", currentAngle);
		if (currentAngle >= 10)
		{
			output1 = 0.8;
		}
		else if (currentAngle < -10)
		{
			output1 = -0.8;
		}
		else if (currentAngle > 0 && currentAngle < 10)
		{
			output1 = currentAngle*0.06+0.2;
		}
		else if (currentAngle < 0 && currentAngle > -10)
		{
			output1 = -(currentAngle*0.06+0.2);
		}
		currentAngle = position.GetAngleDegrees();
		logfile << "Calculated Output 2:" << output1 << std::endl;
		logfile << "Azimuth 2:" << currentAngle << std::endl;
		driveTrain.TankDrive(-output1, output1);
		Wait(0.01);
		failSafe++;
		currentAngle = position.GetAngleDegrees();
		logfile << "loop 3:" << failSafe << std::endl;
		//turn
	}
	if(failSafe >= 500)
	{
		shooter.Disable();
			driveTrain.Disable();
			logfile << "Auto ended" << std::endl;
			logfile.close();
		logfile << "FAIL SAFE ACTIVATED!! WEEOO WEEOO!" << std::endl;
		return;
	}
	failSafe = 0;
	driveTrain.ArcadeDrive(0.0, 0.0, false);
	//turn to tower
	//logfile << "turning to tower..."<< std::endl;
	if(aimer.GetAge() > 3)
	{

	std::cout << "VALUES TO OLD!!!" << std::endl;
	shooter.Disable();
		driveTrain.Disable();
		logfile << "Auto ended" << std::endl;
		logfile.close();
	return;
	}
	float azimuth = aimer.GetAngleToTower();
	float output = 0;
	//  this would be so much easier in c#
	//driveTrain.TurnToRelativeAngle(azimuth);
//  if the image is fresh then turn slow so azimuth between -1 and 1
	while(abs(azimuth) > 2.0 && failSafe < 500){
		if(azimuth >= 10)
		{
			output = 0.8;
		}
		else if (azimuth < -10)
		{
			output = -0.8;
		}
		else if (azimuth > 0 && azimuth < 10)
		{
			output = azimuth*0.06+0.2;
		}
		else if (azimuth < 0 && azimuth > -10)
		{
			output = -(azimuth*0.06+0.2);
		}
		logfile << "Calculated Output 3:" << output << std::endl;
		logfile << "Azimuth 3:" << azimuth << std::endl;
		driveTrain.TankDrive(output, -output);
		Wait(0.01);
		failSafe++;
		azimuth = aimer.GetAngleToTower();
		logfile << "loop 3:" << failSafe << std::endl;
		logfile << output << std::endl;
		//  start the shooter motor and raise the shooter arm
		//  if azimuth < 0 and arm in position then fire shooter
		//  then turn 180 to point back to the home side
		//  console.println("pc > xbox");

		//  else sleep for .01 second sleep(1000) then increment failsafe
	}
	if (failSafe >= 500)
	{
		shooter.Disable();
			driveTrain.Disable();
			logfile << "Auto ended" << std::endl;
			logfile.close();
		logfile << "FAIL SAFE ACTIVATED!! WEEOO WEEOO!" << std::endl;
		return;
	}
	failSafe = 0;
	driveTrain.ArcadeDrive(0.0, 0.0, false);
	//aim at tower
	shooter.SetAngle(aimer.GetAngleToShoot());
	shooter.SetSpeed(1.0);
	shooter.Shoot();

	shooter.Disable();
	driveTrain.Disable();
	logfile << "Auto ended" << std::endl;
	logfile.close();
*/

	std::shared_ptr<NetworkTable> table = NetworkTable::GetTable("datatable");
	int startPos = table->GetNumber("StartPosition", 1);
	float timeTo10 = table->GetNumber("TimeTo10", 4);
	float over9000 = table->GetNumber("StartPower", 0.5);




	SmartDashboard::PutString("StatusUpdate", "Starting Autonomous");
	//  Drive straight with NavX
	ahrs->ZeroYaw();


	Wait(.5);
	SmartDashboard::PutString("StatusUpdate", "Zero Gyro Complete");
	//
	// Drive Straight
	//
	turnController->SetSetpoint(0);
	turnController->Enable();
	timer.Reset();
	int driveSafetyCounter = 0;
	int endTime = (int)100*timeTo10;
	while(driveSafetyCounter < endTime)
	{
	double currentRotationRate = rotateToAngleRate;

	driveTrain.ArcadeDrive(over9000, currentRotationRate, false);
	Wait(.02);
	driveSafetyCounter++;
	}
	turnController->Disable();
	SmartDashboard::PutString("StatusUpdate", "Completed Straight Drive");
	shooter.SetAngle(30);
	SmartDashboard::PutString("StatusUpdate", "initial shooter raise to 30 degrees");
	//
	//  rotate based on Starting position
	//
	double rotateAngle = -179;
	if(startPos == 1)
	{
		rotateAngle = -145;
	}else if(startPos == 5)
	{
		rotateAngle = 165;
	}
	else if(startPos == 2)
	{
		rotateAngle =  -165;
	}
	turnController->SetSetpoint(rotateAngle);
	turnController->Enable();
	timer.Reset();
	while(timer.Get() <= 5 && abs(ahrs->GetAngle() - rotateAngle) > kToleranceDegrees ){
    double currentRotationRate = rotateToAngleRate;
    driveTrain.ArcadeDrive(0, currentRotationRate, false);
    Wait(.005);
	}
	turnController->Disable();
	SmartDashboard::PutString("StatusUpdate", "Completed First Turn");

	//
	//    wait for 1 second to stabilize the robot
	//
	Wait(1);
	float targetingInformationAge = aimer.GetAge();
	float calculatedShooterAngle = aimer.GetAngleToShoot();
	float towerRelativeAngle = aimer.GetAngleToTower();
	//
	//  search for the target
	//
	timer.Reset();
	while(targetingInformationAge > 2 && timer.Get() < 7)
	{
		SmartDashboard::PutString("StatusUpdate", "Searching For Target");
		SmartDashboard::PutNumber("TargetAge", (double)targetingInformationAge);
		targetingInformationAge = aimer.GetAge();
		calculatedShooterAngle = aimer.GetAngleToShoot();
		towerRelativeAngle = aimer.GetAngleToTower();
	}
	if(targetingInformationAge > 2){
		SmartDashboard::PutString("StatusUpdate", "Target Not Found, timed out");
	}
	else
	{
		if(calculatedShooterAngle > minShooterLimit){
			SmartDashboard::PutString("StatusUpdate", "target found, starting turn");
			shooter.SetSpeed(1);
			shooter.SetAngle(calculatedShooterAngle);
			//
			//  target acquired turn to tower
			//
			turnController->SetSetpoint(aimer.GetAngleToTower());
			turnController->Enable();
			timer.Reset();
			while(timer.Get() <= 5 && abs(ahrs->GetAngle()) > kToleranceDegrees ){
			double currentRotationRate = rotateToAngleRate;
			double turnAdjustmentRate = .75;
			driveTrain.ArcadeDrive(0, currentRotationRate * turnAdjustmentRate, false);

			Wait(.01);
			}
			turnController->Disable();
			SmartDashboard::PutString("StatusUpdate", "Completed Turn To Target, final shooter adjustment");
			shooter.SetAngle(calculatedShooterAngle);
			SmartDashboard::PutString("StatusUpdate", "Completed shooter adjustment, wait .5");
			Wait(.5);
			SmartDashboard::PutString("StatusUpdate", "SHOOT, SCORE, CHEER");
			shooter.Shoot();
			SmartDashboard::PutString("StatusUpdate", "Ball AWAY, lower arm, rotate toward defenses");
			shooter.SetAngle(0);
			shooter.SetSpeed(0);

			//  stable platform  then confirm angle of shooter is set

		}
		else
		{
			SmartDashboard::PutString("StatusUpdate", "Saving the ball for teleop, 'cause we can");
		}
	}
}
START_ROBOT_CLASS(Robot);
