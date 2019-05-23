package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.commands.DrivePID;
import org.usfirst.frc.team839.robot.commands.ElevatorUpCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class ScaleDrop extends CommandGroup
{
	{
	  //addSequential (new DrivePID [Driving + Turning] / TurnToAngle [Turning Only] (#.#f [Inches] , #.#f [Target Angle]));
		addSequential (new ElevatorUpCommand());
		addSequential (new DrivePID(140.0f, 0.0f));		//140 inches
	  //addSequential (new TurnToAngle(90.0f));
	}
}