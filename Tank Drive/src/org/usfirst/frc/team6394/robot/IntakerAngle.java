package org.usfirst.frc.team6394.robot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class IntakerAngle {
	
	private final static int kPIDLoopIdx = 0;
	private final static int kTimeoutMs = 10;
	private final static int kCurrentLimit = 8;
	private final static int kCurrentLimitDuration=10;
	
	private final static double maxAngle=0.3*10000;
	private final static double Angle_135=0.225*10000;
	private final static double Angle_90=0.16*10000;
	private final static double Angle_45=0.08*10000;
	private final static double minAngle=0.0*10000;
	
	
	private TalonSRX AngleMotor=new TalonSRX(pin.AngleMotorID);
	
	private static double setAngle=0;
	
	private static int pos=0;
	
	public IntakerAngle() {
		
		//Initiate AngleMotor
		TalonSRXInit(AngleMotor);
		configPID(AngleMotor,0,0.5,0,0);
		
		//Motor Safety

		AngleMotor.enableCurrentLimit(true);
		AngleMotor.configContinuousCurrentLimit(kCurrentLimit, kTimeoutMs);
		AngleMotor.configPeakCurrentLimit((int)(kCurrentLimit*1.5), kTimeoutMs);
		AngleMotor.configPeakCurrentDuration(kCurrentLimitDuration, kTimeoutMs);
		
		
	}
	
	public void run() {

		AngleMotor.set(ControlMode.Position, setAngle);
		SmartDashboard.putNumber("Info_IntakerAngle", AngleMotor.getSelectedSensorPosition(kPIDLoopIdx));
		SmartDashboard.putNumber("Info_IntakerAngleCurrent", AngleMotor.getOutputCurrent());

	}
	
	public void setOrigin() {
		setAngle=minAngle;
	}

	
	public void move() {
		pos=(pos+1)%4;
		switch(pos) {
		case 0:
			setAngle=minAngle;
			break;
		case 1:
			setAngle=Angle_45;
			break;
		case 2:
			setAngle=Angle_90;
			break;
		case 3:
			setAngle=Angle_135;
			break;
		}
		
	}
	
	public void set(double value) {
		setAngle=(maxAngle+minAngle)/2+value*(maxAngle-minAngle)/2;
	}

	private void TalonSRXInit(TalonSRX _talon){

		//set up TalonSRX and closed loop

		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
					kPIDLoopIdx,kTimeoutMs);
		_talon.setSensorPhase(true);

		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1, kTimeoutMs);
		_talon.configPeakOutputReverse(-1, kTimeoutMs);

		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

	}

	private void configPID(TalonSRX _talon,double kF, double kP,double kI, double kD) {

		_talon.config_kF(kPIDLoopIdx, kF, kTimeoutMs);
		_talon.config_kP(kPIDLoopIdx, kP, kTimeoutMs);
		_talon.config_kI(kPIDLoopIdx, kI, kTimeoutMs);
		_talon.config_kD(kPIDLoopIdx, kD, kTimeoutMs);

	}
}