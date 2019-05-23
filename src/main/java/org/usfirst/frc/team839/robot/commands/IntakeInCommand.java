package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Command;

public class IntakeInCommand extends Command
{
	
    double power = 1;
	double maxCurrent = 0;
    public IntakeInCommand(double duration)
    
    {
    	this.setTimeout(duration);
    	this.power = 1;
    	requires(Robot.intake);
    }
    
    public IntakeInCommand()
    {
    	requires(Robot.intake);
    }

    protected void initialize() 
    {

    }

    protected void execute() 
    {
        Robot.intake.intake();
    }

    protected boolean isFinished() 
    {
        return false;
    }

    protected void end() 
    {
    	Robot.intake.stop();
    }

    protected void interrupted() 
    {
    	end();
    }
}