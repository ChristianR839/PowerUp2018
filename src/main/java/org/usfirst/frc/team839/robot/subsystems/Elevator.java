package org.usfirst.frc.team839.robot.subsystems;

import org.usfirst.frc.team839.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {
	
	public static double ticksPerInch = 349.5357142857143;
	public static double getTicks(double height)
	{
		return height * ticksPerInch;
	}

	@Override
	protected void initDefaultCommand()
	{
		RobotMap.elevatorFollowerMotor.set(ControlMode.Follower,RobotMap.elevatorMotor.getDeviceID());
		RobotMap.elevatorMotor.configOpenloopRamp(.5, 0);
	}
	public void raise()
	{
		RobotMap.elevatorMotor.set(1);
	}
	public void lower()
	{
		RobotMap.elevatorMotor.set(-1);
	}
	public void stop()
	{
		RobotMap.elevatorMotor.set(0);
	}

}