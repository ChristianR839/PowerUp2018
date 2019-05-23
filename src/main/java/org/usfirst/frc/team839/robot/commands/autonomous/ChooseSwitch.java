package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.commands.DrivePID;
import org.usfirst.frc.team839.robot.commands.ElevatorUpCommand;
import org.usfirst.frc.team839.robot.commands.IntakeOutCommand;
import org.usfirst.frc.team839.robot.commands.TurnToAngle;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ChooseSwitch extends CommandGroup
{
	{
	  //addSequential (new DrivePID [Driving + Turning] / TurnToAngle [Turning Only] (#.#f [Inches] , #.#f [Target Angle]));
		addSequential (new ElevatorUpCommand(28f));	//9787 ticks
		addSequential (new DrivePID(12.0f, 0.0f));		//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new TurnToAngle(-45.0f,"R"));		//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new DrivePID(65.0f, 0.0f));	//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new TurnToAngle(45.0f,"R"));	//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new DrivePID(43.0f, 0.0f));		//13 Feet
		addSequential (new IntakeOutCommand(1, "B"));	//1 second

	  //addSequential (new TurnT1oAngle(90.0f));
	}
}















//LOG {03/26/18}
//DO NOT READ

//YOU CAN'T PROGRAM WITHOUT A PRACTICE BOT
//WHY DON'T WE HAVE THE PRACTICE BOT?
//WE NEED THE PRACTICE BOT
//WHAT IS TAKING SO LONG?
//WHY ARE THERE ONLY THREE PEOPLE WORKING ON THE PRACTICE BOT?
//I CAN'T DO ANYTHING
//PEOPLE AROUND ME ARE ANNOYING
//HELP
//THE PRACTICE BOT STILL ISN'T DONE
//WHY ISN'T IT DONE YET?
//I AM GETTING SICK OF SITTING HERE
//GOODBYE UNTIL THE PRACTICE BOT IS READY
//...
//IT'S STLL NOT FINISHED
//WHY ISN'T IT FINISHED YET?
//AAAAAAAAAA
//NOBODY WANTS TO WORK ON THE ROBOT
//THE MENTORS ARE IN THEIR MEETING, PEOPLE ARE DOING NOTHING
//I CAN'T DO ANYTHING UNTIL THE PRACTICE BOT IS READY
//WHY ISN'T IT DONE YET?
//PEOPLE NEED TO WORK FASTER
//I'M STILL SITTING HERE WAITING FOR PEOPLE IT FINISH
//I WANT TO WORK ON AUTONOMOUS
//I'M GETTING BORED
//UGH
//THIS IS ANNOYING
//HALP
//IT'S STILL NOT DONE YET
//WHY?
//LAZY PEOPLE ARE LAZY
//ANNOYING PEOPLE ARE ANNOYING
//GOODBYE
//...
//NEVERMIND, THERE IS STILL HOPE
//I'VE LOST ALL HOPE
//KIDS ALL AROUND ME
//SCREAMING
//I WANT TO GET WORK DONE
//THE MENTORS WILL GET MAD AT THE STUDENTS WHO HAVEN'T DONE ANYTHING
//TOO BAD FOR THEM, THEY'RE LAZY
//I THINK I'LL JUST KEEP TYPING FOREVER
//IT'S A GOOD OUTLET FOR MY PAIN
//THE PRACTICE BOT HAS HAD LITTLE PROGRESS
//HMMMMMMMM
//I'M GETTING TIRED
//ENOUGH IS ENOUGH
//GOODBYE
//...
//WHY IS THIS HAPPEING?
//SHOULD I CONTINUE THIS?
//I'M DONE