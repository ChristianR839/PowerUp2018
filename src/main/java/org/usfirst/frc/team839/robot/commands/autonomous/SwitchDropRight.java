package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.commands.DrivePID;
import org.usfirst.frc.team839.robot.commands.ElevatorUpCommand;
import org.usfirst.frc.team839.robot.commands.IntakeOutCommand;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchDropRight extends CommandGroup
{
	{
		addSequential (new ElevatorUpCommand(28f));	//9787 ticks
		addSequential (new DrivePID(97.0f, 0.0f, 2.0));	//140 inches
		addSequential (new IntakeOutCommand(1, "R"));	//1 second
	}
}