package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeOutCommand extends Command
{
	private static final double DEFAULT_SPEED = 0.5;
    private static double outtakeSpeed = DEFAULT_SPEED;
	private String side;

	public IntakeOutCommand(double duration)
    {
    	this.setTimeout(duration);
    	requires(Robot.intake);
    }
    
    public IntakeOutCommand()
    {
    	requires(Robot.intake);
    }
    
    public IntakeOutCommand(double duration, String side)
    {
    	this.side = side;
    	this.setTimeout(duration);
    	requires(Robot.intake);
    	
    }

    protected void initialize() 
    {
    	if(this.side != null && Robot.fieldLayout != null)
    	{
	    	System.out.println("field layout: " + side + " " +Robot.fieldLayout.isOurSwitch(side));
	    	if(!Robot.fieldLayout.isOurSwitch(side))
	    	{
	    		outtakeSpeed = 0;
	    	}
    	}
    }

    protected void execute() 
    {
        Robot.intake.outtake(outtakeSpeed);//Robot.oi.getOuttakeSpeed());
    }

    protected boolean isFinished() 
    {
        return this.isTimedOut();
    }

    protected void end() 
    {
    	Robot.intake.stop();
    	this.side = null;
    	outtakeSpeed = DEFAULT_SPEED;
    	
    }

    protected void interrupted() 
    {
    	end();
    }
}