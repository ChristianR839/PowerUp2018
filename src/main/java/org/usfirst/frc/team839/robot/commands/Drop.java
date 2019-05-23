package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Drop extends Command
{
	
    public Drop(double duration)
    {
    	this.setTimeout(1);
    	requires(Robot.intake);
    	requires(Robot.drivetrain);
    }
    
    public Drop()
    {
    	requires(Robot.intake);
    	requires(Robot.drivetrain);
    }

    protected void initialize() 
    {

    }

    protected void execute() 
    {
    	Robot.drivetrain.setDriveSpeeds(0, -0.3, 0, 0);
    }

    protected boolean isFinished() 
    {
        return this.isTimedOut();
    }

    protected void end() 
    {
    	Robot.intake.outtake(-0.1);
    	Robot.drivetrain.stop();
    }

    protected void interrupted() 
    {
    	end();
    }
}