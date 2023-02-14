package org.usfirst.frc.team6394.robot;

public class Constant {
	
	//measurement
	
	public final static double rotate90=1.03;
	
	//Pin
	
	public final static int LTalonEID=3;
	
	public final static int RTalonEID=1;
	
	public final static int LTalonID=4;
	
	public final static int RTalonID=2;
	
	//Constants
	
	public final static int kTimeoutMs = 10;

	public final static int kPIDLoopIdx = 0;

	public final static int kTurnTimeoutSec = 3;
	
	/****
	/* Below are constants that need to be adjusted
	/* V stands for velocity
	/* RV stands for rotation velocity
	*/
	
	public final static double r_min_V=200;		//raw encoder value
	
	public final static double r_max_V=1000.0;	//raw encoder value
	
	public final static double min_V=r_min_V/r_max_V;	//normalized min velocity when robot starts to move
	
	public final static double max_RV=300;
	
	public final static double SlowDownAngle=20;	//angle where robot starts to slow down
}
