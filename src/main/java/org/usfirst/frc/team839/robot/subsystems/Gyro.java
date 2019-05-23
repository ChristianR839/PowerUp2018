package org.usfirst.frc.team839.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The gyro subsystem for the robot.
 * @author Matt
 */
public class Gyro extends Subsystem 
{
	private static final int refreshrate = 60;
	
	private static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

	protected void initDefaultCommand() 
	{
			
	}
	
	public double getAngle()
	{

		return gyro.getAngle();
	}
	
	public void reset()
	{
		gyro.reset();
	}
}
