package org.usfirst.frc.team839.robot.subsystems;

import org.usfirst.frc.team839.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Intake extends Subsystem {

	@Override
	protected void initDefaultCommand()
	{
		RobotMap.intakeFollowerMotor.set(ControlMode.Follower,RobotMap.intakeMotor.getDeviceID());
	}
	public void intake()
	{
		RobotMap.intakeFollowerMotor.set(.6);
		RobotMap.intakeMotor.set(.6);
	}
	public void outtake(double d)
	
	{
		RobotMap.intakeFollowerMotor.set(-d);
		RobotMap.intakeMotor.set(-d);
	}
	public void stop()
	{
		RobotMap.intakeFollowerMotor.set(0);
		RobotMap.intakeMotor.set(0);
	}
}