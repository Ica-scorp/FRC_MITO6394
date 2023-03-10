package org.usfirst.frc.team6394.robot;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

public class ClosedLoopDrive {
	
	TalonSRX LTalonE=new TalonSRX(Constant.LTalonEID);
	TalonSRX RTalonE=new TalonSRX(Constant.RTalonEID);
	TalonSRX RTalon=new TalonSRX(Constant.RTalonID);
	TalonSRX LTalon=new TalonSRX(Constant.LTalonID);
	
	private final AHRS ahrs = new AHRS(Port.kMXP);
	
	private double pos_X=0;
	private double pos_Y=0;

	public ClosedLoopDrive() {
		/** Initiate Closed-loop system*/
		
		//'E' stands for encoder
		TalonSRXInit(LTalonE);
		TalonSRXInit(RTalonE);
		configPID(LTalonE,0.4,0.6,0,0);
		configPID(RTalonE,0.4,0.6,0,0);

		RTalon.setInverted(true);
		RTalonE.setInverted(true);
		
		LTalon.follow(LTalonE);
		RTalon.follow(RTalonE);
		
	}
	
	public AHRS getAHRS() {return  ahrs;}
	public double getX(){return pos_X;}
	public double getY(){return pos_Y;}
	
	private void DisplayInfo() {
		/**** Displaying velocity on smart dashboard */
		
		SmartDashboard.putNumber("Info_LVel",LTalonE.getSelectedSensorVelocity(Constant.kPIDLoopIdx));
		
		SmartDashboard.putNumber("Info_LPWM",LTalonE.getMotorOutputPercent());
		SmartDashboard.putNumber("Info_RPWM",RTalonE.getMotorOutputPercent());
		
		SmartDashboard.putNumber("Info_LDis",(double)LTalonE.getSelectedSensorPosition(Constant.kPIDLoopIdx)/4096.0);
		SmartDashboard.putNumber("Info_RDis",(double)RTalonE.getSelectedSensorPosition(Constant.kPIDLoopIdx)/4096.0);
		
		SmartDashboard.putNumber("Info_Angle",(double)ahrs.getAngle());

		SmartDashboard.putNumber("Info_X",(double)pos_X);
		SmartDashboard.putNumber("Info_Y",(double)pos_Y);
	}

	public void resetSensor() {
		LTalonE.setSelectedSensorPosition(0, Constant.kPIDLoopIdx, Constant.kTimeoutMs);
		RTalonE.setSelectedSensorPosition(0, Constant.kPIDLoopIdx, Constant.kTimeoutMs);
		ahrs.reset();
		ahrs.resetDisplacement();
	}

	public void MoveToPos(double x, double y, double angle, double speed){
		
	}
	
	public void UpdateMap(){
		double last_dis;

		double this_dis=
			(LTalonE.getSelectedSensorPosition(Constant.kPIDLoopIdx)
			+RTalonE.getSelectedSensorPosition(Constant.kPIDLoopIdx))/2;

		double delta_dis=this_dis-last_dis;

		pos_X+=Math.cos(ahrs.getAngle())*delta_dis;
		pos_Y+=Math.sin(ahrs.getAngle())*delta_dis;

		last_dis=this_dis;
	}

	public void velDrive(double forward, double turn, double smoothMode=true) {
		/****** speed mode : clockwise is positive */
		
		/* Convert 500 RPM to units / 100ms.
		 * 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
		 * velocity setpoint is in units/100ms
		 */
		
		double LTempV=(forward*Math.abs(forward)+turn*Math.abs(turn))
					* Constant.r_max_V;
		double RTempV=(forward*Math.abs(forward)-turn*Math.abs(turn))
					* Constant.r_max_V;

		if(smoothMode){
			//motion profile applied
			LTalonE.set(ControlMode.Velocity,
            	util.setWithin(
                	LTempV,   //Temp velocity
                	LTalonE.getSelectedSensorVelocity(Constant.kPIDLoopIdx),    //Target velocity
                	Constant.r_min_V));   //limit
		
        	RTalonE.set(ControlMode.Velocity,
            	util.setWithin(
                	RTempV,   //Temp velocity
                	RTalonE.getSelectedSensorVelocity(Constant.kPIDLoopIdx),    //Targer velocity
                	Constant.r_min_V));   //limit
		}else{
			//no motion profile applied
			LTalonE.set(ControlMode.Velocity,LTempV); 
        	RTalonE.set(ControlMode.Velocity,RTempV); 
		}		


		DisplayInfo();
	}
	
	public void PWMDrive(double forward, double turn) {
		/****** PWM mode : clockwise is positive */
		
		LTalonE.set(ControlMode.PercentOutput,
				forward*Math.abs(forward)+turn*Math.abs(turn));
		
		RTalonE.set(ControlMode.PercentOutput,
				forward*Math.abs(forward)-turn*Math.abs(turn));	
		
		DisplayInfo();
	}
	
	public boolean Rotate(double angle,double vel, double forVel) {
		/***
		 * angle is in degree
		 */
		
		double angleLeft=angle-ahrs.getAngle();
		if(util.isWithin(angleLeft,0,1)) {
			velDrive(forVel,0,false);
			return true;
		}else if(
				util.isWithin(
					angleLeft,
					0,
					Constant.SlowDownAngle*(ahrs.getRate()/Cosntant.max_RV)
					//Different starting angles
				)
		{
			velDrive(forVel,Constant.,false);
			return false;
		}else{
			velDrive(forVel,util.equalsign(angleLeft, vel));
			return false;
		}
	}

	
	public boolean DisDrive(double dis, double angle, double speed) {
		/**** distance closed-loop mode */
		
		/* One rev=4096units
		 * Times 4096
		 */
		
		double temppos;
		boolean result=false;
		
		temppos=(dis+angle)* 4096;
		if(Math.abs(LTalonE.getSelectedSensorPosition(0))<Math.abs(temppos)) {
			LTalonE.set(ControlMode.Velocity,
					util.equalsign(temppos,
					(Constant.EndingSpeed+speed*
							Math.pow(Math.abs((temppos-LTalonE.getSelectedSensorPosition(0))/temppos),0.22)
					)* 500.0 * 4096 / 600));
			result=false;
		}else {
			LTalonE.set(ControlMode.Velocity,0);
			result=true;
		}
		
		temppos=(dis-angle)* 4096;
		if(Math.abs(RTalonE.getSelectedSensorPosition(0))<Math.abs(temppos)) {
			RTalonE.set(ControlMode.Velocity,
					util.equalsign(temppos,
					(Constant.EndingSpeed+speed*
							Math.pow(Math.abs((temppos-RTalonE.getSelectedSensorPosition(0))/temppos),0.22)
					)* 500.0 * 4096 / 600));
			return false;
		}else {
			RTalonE.set(ControlMode.Velocity,0);
			return result&true;
		}
		
		/*
		configPID(LTalon,0,0.05,0,0.02);
		configPID(RTalon,0,0.05,0,0.02);
		
		LTalon.set(ControlMode.Position,
				(dis+angle)* 4096);
		
		RTalon.set(ControlMode.Position,
				(dis-angle)* 4096);
		
		DisplayInfo();
		 */
	}

	private void TalonSRXInit(TalonSRX _talon) {
		//set up TalonSRX and closed loop
		
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				Constant.kPIDLoopIdx,Constant.kTimeoutMs);
		_talon.setSensorPhase(true);
		
		_talon.configNominalOutputForward(0, Constant.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constant.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constant.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constant.kTimeoutMs);

		_talon.setSelectedSensorPosition(0, Constant.kPIDLoopIdx, Constant.kTimeoutMs);
	}
	
	private void configPID(TalonSRX _talon,double kF, double kP,double kI, double kD) {
		_talon.config_kF(Constant.kPIDLoopIdx, kF, Constant.kTimeoutMs);
		_talon.config_kP(Constant.kPIDLoopIdx, kP, Constant.kTimeoutMs);
		_talon.config_kI(Constant.kPIDLoopIdx, kI, Constant.kTimeoutMs);
		_talon.config_kD(Constant.kPIDLoopIdx, kD, Constant.kTimeoutMs);
	}

}
