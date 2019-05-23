package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.commands.DrivePID;
import org.usfirst.frc.team839.robot.commands.ElevatorUpCommand;
import org.usfirst.frc.team839.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ZAMP extends CommandGroup
{
	{
		  addSequential (new TurnToAngle(45.00f, "angle"));
	}
}