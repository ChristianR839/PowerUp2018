package org.usfirst.frc.team839.robot.subsystems;

import org.usfirst.frc.team839.robot.RobotMap;
import org.usfirst.frc.team839.robot.commands.DriveWithJoystick;


import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * The drivetrain subsystem for the robot.
 * @author Matt
 */
public class DriveTrain extends Subsystem
{

    public void initDefaultCommand() 
    {
        setDefaultCommand(new DriveWithJoystick());
        
        RobotMap.frontLeftMotor.set(ControlMode.PercentOutput, 0);
        RobotMap.frontRightMotor.set(ControlMode.PercentOutput, 0);
        RobotMap.backLeftMotor.set(ControlMode.PercentOutput, 0);
        RobotMap.backRightMotor.set(ControlMode.PercentOutput, 0);
        
        //sensorPos, pidIdx, timeoutMs
        
    }

    /**
     * Sets the drive speeds for the Mecanum drive.
     *
     * @param strafe The speed that the robot should drive in the X direction. [-1.0..1.0]
     * @param forward The speed that the robot should drive in the Y direction. [-1.0..1.0]
     * @param rotate The rate of rotation for the robot. [-1.0..1.0]
     * @param angle The current angle reading from the gyro.
     */
    public void setDriveSpeeds(double strafe, double forward, double rotate, double angle)
    {
    	RobotMap.robotDrive.driveCartesian(strafe, forward, rotate, angle);
    }
    
    public void stop()
    {
    	RobotMap.robotDrive.driveCartesian(0, 0, 0, 0);
    }

}
