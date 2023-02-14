/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends IterativeRobot {

	Joystick actionStick = new Joystick(0);
	Joystick moveStick = new Joystick(1);
	Elevator mElevator = new Elevator();
	Intaker mIntaker = new Intaker();
	Base mBase = new Base();

	private static int autoState = 0;
	private Timer autoTimer = new Timer();
	private Timer initTimer = new Timer();
	private Timer raiseTimer = new Timer();

	private boolean isRight = false;
	private boolean firstRaise = true;

	@Override
	public void robotInit() {
		Climb.Off();
		CameraServer.getInstance().startAutomaticCapture();
//		initTimer.delay(1.0);
		mElevator.setInitialVertical();
	}

	@Override
	public void autonomousInit() {
		autoTimer.stop();
		autoTimer.reset();
		autoTimer.start();
		autoState = 0;
	}

	@Override
	public void autonomousPeriodic() {
		isRight = DriverStation.getInstance().getGameSpecificMessage().charAt(0) == 'R';
		mBase.velDrive(0.3, 0, false);

		mBase.run();
		mIntaker.run();
		mElevator.run();
	}

	@Override
	public void teleopInit() {
		SmartDashboard.putBoolean("WTF", false);
	}

	@Override
	public void teleopPeriodic() {

		raiseTimer.start();

		if (actionStick.getRawButtonPressed(buttons.intakerBackAngleBut)) {
			mElevator.setBackAngle();
		}
		if (actionStick.getRawButtonPressed(buttons.intakerToggleAngleBut_action)) {
			mElevator.toggleBotAngle();
		}
		mIntaker.emitCube(actionStick.getRawAxis(buttons.intakerEmitAxis));
		if (moveStick.getRawButton(buttons.MintakerEmitAxis)) {
			mIntaker.emitCube();
		}
		SmartDashboard.putNumber("timer", raiseTimer.get());
		switch (actionStick.getPOV()) {
		case 0:
			mElevator.changePos(2);
			break;
		case 90:
			mElevator.changePos(3);
			break;
		case 180:
			mElevator.changePos(0);
			break;
		case 225:
			mElevator.changePos(4);
			break;
		case 270:
			mElevator.changePos(1);
			break;
		}

		switch (moveStick.getPOV()) {
		case 0:
			mElevator.changePos(2);
			break;
		case 90:
			mElevator.changePos(3);
			break;
		case 180:
			mElevator.changePos(0);
			break;
		case 225:
			mElevator.changePos(4);
			break;
		case 270:
			mElevator.changePos(1);
			break;
		}

//		switch (moveStick.getPOV()) {
//		case 270:
//			mElevator.setAngleCalibrated();
//			break;
//		case 90:
//			mElevator.setElevatorCalibrated();
//			break;
//		}
		
		if (moveStick.getRawButtonPressed(buttons.CalibrateBut)) {
			mElevator.setAngleCalibrated();
		}
		
		if (moveStick.getRawButton(buttons.angleManual)) {
			mElevator.manualAngle(-moveStick.getRawAxis(3));
		}
		if (moveStick.getRawButtonReleased(buttons.angleManual)) {
			mElevator.disableManualAngle();
		}

		if (moveStick.getRawButton(buttons.climbBut)) {
			if (moveStick.getRawButton(buttons.dropBut)) {
				Climb.climb(0.12);
			} else {
				Climb.climb(1);
			}
		} else if (moveStick.getRawButton(buttons.dropBut)) {
			Climb.climb(-1);
		} else {
			Climb.climb(0);
		}

		mIntaker.setLargeAngle(moveStick.getRawButton(buttons.intakerLargeAngleBut));
		if (moveStick.getRawButtonPressed(buttons.intakerChangeModeBut) && (!moveStick.getRawButton(buttons.LiftBut))) {
			mIntaker.toggleMode(4);
		}
		if (moveStick.getRawButtonPressed(buttons.intaker45But)) {
			mElevator.toggleBotAngle45();
		}
		if (moveStick.getRawButtonPressed(buttons.intakerToggleAngleBut)
				&& (!moveStick.getRawButton(buttons.LiftBut))) {
			mElevator.toggleBotAngle();
		}

		if (moveStick.getRawButton(buttons.LiftBut)) {
			if (moveStick.getRawButton(buttons.intakerChangeModeBut)) {
				Climb.moveHook(0.5);
			} else if (moveStick.getRawButton(buttons.intakerToggleAngleBut)) {
				Climb.moveHook(-0.5);
			} else {
				Climb.moveHook(0);
			}
		}

		if (mIntaker.isRaiseNeeded()) {
			mElevator.toggleBotAngle();
			mIntaker.isRaiseFinished();
		}
		if (mElevator.isTop()) {
			Constants.kRawMaxVel = 800;
		} else {
			Constants.kRawMaxVel = 2000;
		}

		if (moveStick.getRawButtonReleased(buttons.slowModeBut)) {
			Constants.kRawMaxVel = 2000;
		}
		if (moveStick.getRawButtonReleased(buttons.fastModeBut)) {
			Constants.kRawMaxVel = 2000;
		}
		if (moveStick.getRawButton(buttons.fastModeBut)) {
			Constants.kRawMaxVel = 3000;
		}
		if (moveStick.getRawButton(buttons.slowModeBut)) {
			Constants.kRawMaxVel = 800;
		}
		mElevator.hasCube = mIntaker.hasCube();
		mBase.velDrive(-moveStick.getY(), -moveStick.getX() * 0.85, true);
		mBase.run();
		mIntaker.run();
		mElevator.run();

	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void testPeriodic() {
		TalonSRX ElevatorMaster = new TalonSRX(pin.ElevatorMasterID);
		VictorSPX ElevatorSlave = new VictorSPX(pin.ElevatorSlaveID);
		ElevatorMaster.setNeutralMode(NeutralMode.Brake);
		ElevatorSlave.setNeutralMode(NeutralMode.Brake);
		ElevatorMaster.setInverted(true);
		ElevatorSlave.setInverted(true);
		ElevatorSlave.follow(ElevatorMaster);
		if (actionStick.getRawButton(1)) {
			ElevatorMaster.set(ControlMode.PercentOutput, 0.3);
		} else if (actionStick.getRawButton(2)) {
			ElevatorMaster.set(ControlMode.PercentOutput, -0.3);
		}
	}

}