package org.usfirst.frc.team6394.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public abstract class util {

	public static double deadband(double val, double threshold) {
		return Math.abs(val)>=threshold?val:0;
	}


	public static double equalsign(double sign, double value) {
		return Math.abs(value)*Math.abs(sign)/sign;
	}

	
	public static boolean isWithin(double value, double center, double range) {
		/*** 
		 * range is in percent
		 */
		range=Math.abs(range);
		return (value<=(center+range))&&(value>=(center-range));
	}

	public static double setWithin(double value, double center, double range){
		
		range=Math.abs(range);
		if(value>center+range){
			return center+range;
		}else if(value<center-range){
			return center-range;
		}else{
			return value;
		}
	}
	
	public static void TalonSRXInit(TalonSRX _talon) {
		//set up TalonSRX and closed loop
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
				Constants.kPIDLoopIdx,Constants.kTimeoutMs);

		_talon.setSensorPhase(true);

		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

	}
	
	public static void configPID(TalonSRX _talon,double kF, double kP,double kI, double kD) {
		_talon.config_kF(Constants.kPIDLoopIdx, kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kPIDLoopIdx, kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kPIDLoopIdx, kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kPIDLoopIdx, kD, Constants.kTimeoutMs);

	}
}