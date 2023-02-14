package org.usfirst.frc.team6394.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Timer;

public class Intaker {

	/* util constants */

	private final static double LowSpeed = 0.5;
	private final static double HighSpeed = 0.8;

	/* Components */

	private VictorSPX IntakerRMotor = new VictorSPX(pin.IntakerRMotorID);
	private VictorSPX IntakerLMotor = new VictorSPX(pin.IntakerLMotorID);

	private DigitalInput IntakerRLight = new DigitalInput(pin.IntakerRLight);
	private DigitalInput IntakerLLight = new DigitalInput(pin.IntakerLLight);

	private DoubleSolenoid IntakerDSol = new DoubleSolenoid(pin.IntakerDoubleSolenoidForwardID,
			pin.IntakerDoubleSolenoidBackwardID);

	private Solenoid IntakerSSol = new Solenoid(pin.IntakerSingleSolenoidID);

	/* flags */

	private static boolean isLargeAngle = false;
	private static int Mode = 0;
	private static boolean RBonus = false;
	private static boolean hasCube = false;
	private static boolean raiseNeeded = false;

	private static Timer hasCubeTimer = new Timer();

	public Intaker() {
		IntakerLMotor.setInverted(true);
		hasCubeTimer.reset();
	}

	public void setLargeAngle(boolean value) {
		isLargeAngle = value;
	}

	public void run() {

		switch (Mode) {
		case 0:
			hasCube = false;

			if (isLargeAngle) {
				IntakerDSol.set(DoubleSolenoid.Value.kReverse);
			} else {
				IntakerDSol.set(DoubleSolenoid.Value.kForward);
			}

			IntakerSSol.set(false);
			IntakerRMotor.set(ControlMode.PercentOutput, 0);
			IntakerLMotor.set(ControlMode.PercentOutput, 0);

			if ((!IntakerRLight.get()) && (!IntakerLLight.get())) {
				hasCubeTimer.reset();
				hasCubeTimer.start();
				hasCube = true;
				Mode = 2;
			} 
			break;
		case 1:
			if (isLargeAngle) {
				IntakerDSol.set(DoubleSolenoid.Value.kReverse);
			} else {
				IntakerDSol.set(DoubleSolenoid.Value.kForward);
			}
			IntakerSSol.set(false);
			IntakerRMotor.set(ControlMode.PercentOutput, RBonus ? HighSpeed : LowSpeed);
			IntakerLMotor.set(ControlMode.PercentOutput, RBonus ? LowSpeed : HighSpeed);
			if ((!IntakerRLight.get()) && (!IntakerLLight.get())) {
				hasCubeTimer.reset();
				hasCubeTimer.start();
				hasCube = true;
				Mode = 2;
			}
			break;
		case 2:
			if (!hasCube) {
				Mode++;
			}
			if (hasCubeTimer.get() >= 0.5) {
				raiseNeeded = true;
				hasCubeTimer.reset();
				hasCubeTimer.stop();
				IntakerRMotor.set(ControlMode.PercentOutput, 0);
				IntakerLMotor.set(ControlMode.PercentOutput, 0);
			}
			IntakerDSol.set(DoubleSolenoid.Value.kForward);
			IntakerSSol.set(true);
			break;
		case 3:
			RBonus = !RBonus;
			Mode = 0;
			break;
		}
	}

	public void emitCube(double speed) {
		if (speed > 0.6) {
			IntakerDSol.set(DoubleSolenoid.Value.kForward);
			IntakerSSol.set(false);
			IntakerRMotor.set(ControlMode.PercentOutput, -speed + 0.5);
			IntakerLMotor.set(ControlMode.PercentOutput, -speed + 0.5);
			Mode = 3;
		}
	}
	
	public void emitCube() {
			IntakerDSol.set(DoubleSolenoid.Value.kForward);
			IntakerSSol.set(false);
			IntakerRMotor.set(ControlMode.PercentOutput, -0.5);
			IntakerLMotor.set(ControlMode.PercentOutput, -0.5);
			Mode = 3;
	}
	
	public boolean isRaiseNeeded() {
		return raiseNeeded;
	}

	public void isRaiseFinished() {
		raiseNeeded = false;
	}

	public boolean hasCube() {
		return hasCube;
	}

	public void cubeEjected() {
		Mode = 0;
	}

	public void toggleMode(int value) {
		/*
		 * value 0:Mode 0 1:Mode 1 2:Mode 2 3:Mode 3 4:Mode++
		 */
		if (value == 4) {
			Mode = (Mode + 1) % 4;
		} else {
			Mode = value;
		}
	}

}