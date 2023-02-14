package org.usfirst.frc.team6394.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Climb {
	
	private static TalonSRX ClimbMotorMaster=new TalonSRX(pin.ClimbMotorMasterID);
	private static TalonSRX ClimbMotorSlave=new TalonSRX(pin.ClimbMotorSlaveID);
	private static VictorSPX LiftMotor=new VictorSPX(pin.LiftMotorID);
	private static DoubleSolenoid ClimbDSol=
			new DoubleSolenoid(pin.ClimbDoubleSolenoidForwardID,pin.ClimbDoubleSolenoidBackwardID);
	
	public Climb() {
		ClimbMotorSlave.follow(ClimbMotorMaster);
	}
	
	public static void Open() {
		ClimbDSol.set(DoubleSolenoid.Value.kForward);
	}
	public static void Reverse() {
		ClimbDSol.set(DoubleSolenoid.Value.kReverse);
	}
	public static void Off() {
		ClimbDSol.set(DoubleSolenoid.Value.kOff);
	}
	public static void moveHook(double power) {
		LiftMotor.set(ControlMode.PercentOutput,power);
	}
	
	public static void climb(double power) {
		ClimbMotorMaster.set(ControlMode.PercentOutput, power);
	}
}
