/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.*;
import org.usfirst.frc.team6394.robot.Constant.*;
import static java.lang.Math.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

	private Joystick m_stick = new Joystick(5);
	private Timer m_timer = new Timer();
	private ClosedLoopDrive drive= new ClosedLoopDrive();
	private static final double rotatespeed=0.45;
	private static final double basespeed=0.6;
	
	int i=0;
	
	//SensorDifferentialBase Base=new SensorDifferentialBase(LTalon,RTalon);
	
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	
	public void robotInit() {	
		drive.getAHRS().reset();
		drive.getAHRS().resetDisplacement();
	}
	
	/**
	 * This function is run once each time the robot enters autonomous mode.
	 */
	@Override
	public void autonomousInit() {
		m_timer.reset();
		m_timer.start();
		drive.resetSensor();
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		// Drive for 2 seconds
		
		
		if(i<8) {
		switch(i) {
		case 0:
			if(drive.DisDrive(1, 0, 0.2)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 1:
			if(drive.Rotate(90, rotatespeed,0.1)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 2:
			if(drive.DisDrive(1, 0, 0.2)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 3:
			if(drive.Rotate(90, rotatespeed,0.1)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 4:
			if(drive.DisDrive(1, 0, 0.2)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 5:
			if(drive.Rotate(90, rotatespeed,0.1)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 6:
			if(drive.DisDrive(1, 0, 0.2)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 7:
			if(drive.Rotate(90, rotatespeed,0.1)) {
				i++;
				drive.resetSensor();
			}
			break;
		case 8:
			drive.velDrive(0, 0);
			break;
		}
		}//square
		
		/*
		if(i<8) {
			switch(i) {
			case 0:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 1:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 2:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 3:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 4:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 5:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 6:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 7:
				if(drive.Rotate(90, rotatespeed)) {
					i++;
					drive.resetSensor();
				}
				break;
			case 8:
				drive.velDrive(0, 0);
				break;
			}
			}
			*/
		
		/*
		for(i=0;i<6;i++) {
			drive.DisDrive(2, 0, 0.6);
			drive.DisInit();
			drive.DisDrive(0.0, 0.733, 0.6);
			drive.DisInit();
		}//hexagon
		
		drive.DisDrive(4, 4.4, 0.5);//circle
		*/
		
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
	
		drive.velDrive(util.deadband(m_stick.getY(),0.1)*basespeed, util.deadband(m_stick.getX(),0.1)*basespeed);
		
		if(m_stick.getRawButtonPressed(1)) {
			drive.resetSensor();
		}
		
		
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {

	}
	
	public void Disabled() {
		while(isDisabled());
	}
	
	public void RobotInit() {
		
	}
}
