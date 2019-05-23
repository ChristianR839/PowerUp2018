package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;
import org.usfirst.frc.team839.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorUpCommand extends Command
{
    double power = 1;
	private double height = 0;
	
    public ElevatorUpCommand(double height)
    {
    	this.height = Elevator.getTicks(height);
    	this.power = 1;
    	requires(Robot.elevator);
    }
    
    public ElevatorUpCommand()
    {
    	requires(Robot.elevator);
    }

    protected void initialize() 
    {

    }

    protected void execute() 
    {
        Robot.elevator.raise();
        System.out.println(RobotMap.elevatorMotor.getSelectedSensorPosition(0));
    }

    protected boolean isFinished() 
    {
    	int encoderPosition = RobotMap.elevatorMotor.getSelectedSensorPosition(0);
        return encoderPosition >= 28500 || this.isTimedOut() || (this.height > 0 && encoderPosition >= this.height);
        
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