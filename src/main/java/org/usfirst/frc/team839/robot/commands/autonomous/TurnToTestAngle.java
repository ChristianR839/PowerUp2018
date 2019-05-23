package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.commands.DrivePID;
import org.usfirst.frc.team839.robot.commands.ElevatorUpCommand;
import org.usfirst.frc.team839.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnToTestAngle extends CommandGroup
{
	{
	  //addSequential (new DrivePID [Driving + Turning] / TurnToAngle [Turning Only] (#.#f [Inches] , #.#f [Target Angle]));
		  addSequential (new TurnToAngle(90.0f, "angle"));
//		  addSequential (new TurnToAngle((float)SmartDashboard.getNumber("angle", 0), "angle"));
	}
}