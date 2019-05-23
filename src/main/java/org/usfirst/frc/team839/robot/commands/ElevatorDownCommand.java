package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorDownCommand extends Command
{
    double power = -1;
	
    public ElevatorDownCommand(double duration)
    {
    	this.setTimeout(duration);
    	this.power = -1;
    	requires(Robot.elevator);
    }
    
    public ElevatorDownCommand()
    {
    	requires(Robot.elevator);
    }

    protected void initialize() 
    {

    }

    protected void execute() 
    {
        Robot.elevator.lower();
        System.out.println(RobotMap.elevatorMotor.getSelectedSensorPosition(0));
    }

    protected boolean isFinished() 
    {
        return RobotMap.elevatorMotor.getSelectedSensorPosition(0)< 100 ||this.isTimedOut();
    }

    protected void end() 
    {
    	Robot.elevator.stop();
    }

    protected void interrupted() 
    {
    	end();
    }
}