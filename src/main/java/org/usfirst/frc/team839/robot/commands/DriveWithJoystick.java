package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithJoystick extends Command
{
    public DriveWithJoystick() 
    {
        requires(Robot.drivetrain);
 //       requires(Robot.gyro);
    }
    
    // Called just before this Command runs the first time
    protected void initialize() 
    {
    	//Robot.gyro.reset();
    }
    
    // Called repeatedly when this Command is scheduled to run
    protected void execute() 
    {
    	//if(Robot.oi.shiftButton.get())
        //    Robot.drivetrain.setDriveSpeeds(Robot.oi.getStrafeSpeed(), Robot.oi.getSpeed(), Robot.oi.getRotation(), Robot.gyro.getAngle());
    	//else
            Robot.drivetrain.setDriveSpeeds(Robot.oi.getStrafeSpeed(), Robot.oi.getSpeed(), Robot.oi.getRotation(), 0);

    }
        
    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() 
    {
        return false;
    }
    // Called once after isFinished returns true
    protected void end() 
    {
        Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() 
    {
        end();
    }
}
