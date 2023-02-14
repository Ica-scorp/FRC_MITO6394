package org.usfirst.frc.team6394.robot;

public enum EnumBut {
	intakerChangeModeBut(1),
	intakerToggleAngleBut(2),
	intakerLargeAngleBut(11),
	slowModeBut(8),
	medModeBut(7),
	climbBut(10),
	dropBut(9),
	LiftUpBut(5),
	LiftDownBut(6),
	angleManual(12),
	
	elevatorForceRaiseBut(3),
	elevatorForceDropBut(4),
	
	intakerBackAngleBut(5),
	elevatorRaiseBut(4),
	elevatorDropBut(1),
	
	intakerEmitAxis(3),
	intakerToggleAngleBut_action(6);
	
	
	private final int butNumber;
	private EnumBut(int number) { butNumber = number; }
	public int butNo() { return butNumber; }
}