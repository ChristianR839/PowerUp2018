package org.usfirst.frc.team839.robot;

import org.usfirst.frc.team839.robot.commands.ElevatorDownCommand;
import org.usfirst.frc.team839.robot.commands.ElevatorUpCommand;
import org.usfirst.frc.team839.robot.commands.IntakeInCommand;
import org.usfirst.frc.team839.robot.commands.IntakeOutCommand;
import org.usfirst.frc.team839.robot.commands.Drop;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public UniversalJoystick joystickDrive = new UniversalJoystick(0);
	public UniversalJoystick joystickAccessory = new UniversalJoystick(1);

	public Button intakeInButton;
	public Button intakeOutButton;
	public Button elevatorUpButton;
	public Button elevatorDownButton;
	public Button dropButton;

	public OI() 
	{
		//Accessory buttons
    	
		//Drive buttons

    	intakeInButton = new JoystickButton(joystickAccessory, UniversalJoystick.kBtnRB);
    	intakeOutButton = new JoystickButton(joystickAccessory, UniversalJoystick.kBtnLB);
    	
    	intakeInButton.whileHeld(new IntakeInCommand());
    	intakeOutButton.whileHeld(new IntakeOutCommand());
    	
    	elevatorUpButton = new JoystickButton(joystickAccessory, UniversalJoystick.kBtnY);
    	elevatorDownButton = new JoystickButton(joystickAccessory, UniversalJoystick.kBtnX);
    	
    	elevatorUpButton.whileHeld(new ElevatorUpCommand());
    	elevatorDownButton.whileHeld(new ElevatorDownCommand());
    	
    	//dropButton = new JoystickButton(joystickDrive, UniversalJoystick.kBtnA);
    	
    	//dropButton.whenPressed(new Drop());
    	
    	//tableShutdownButton    .whenPressed(new NetworkTableSenderCommand("getDistance"));
    	//fieldOrientedButton	 .whenPressed(new ResetGyroCommand());
	}

	public UniversalJoystick getJoystickDrive() {
		return joystickDrive;
	}
	
	public double getOuttakeSpeed() {
		return (joystickDrive.leftTrigger());
	}

	public double getStrafeSpeed() {
		return deadBand(-joystickDrive.leftAxisX());
	}

	public double getSpeed() {
		return deadBand(joystickDrive.leftAxisY());
	}

	public double getRotation() {
		return deadBand(-joystickDrive.rightAxisX());// .getThrottle());
	}

	double deadBand(double axisVal) {
		if (axisVal < -0.150)// Orig: -0.250
			return axisVal;
		if (axisVal > +0.150)// Orig: +0.250
			return axisVal;
		return 0;
	}

}
