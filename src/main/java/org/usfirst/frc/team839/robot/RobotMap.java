package org.usfirst.frc.team839.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.MecanumDrive;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap 
{
	private static final int frontRightMotorID = 5;
	private static final int frontLeftMotorID = 2;
	private static final int backRightMotorID = 3;
	private static final int backLeftMotorID = 4;
	
	private static final int elevatorMotorID = 13;
	private static final int elevatorFollowerMotorID = 14;
	
	private static final int intakeMotorID = 11;
	private static final int intakeFollowerMotorID = 12;

	public static final WPI_TalonSRX frontRightMotor = new WPI_TalonSRX(frontRightMotorID);
	public static final WPI_TalonSRX frontLeftMotor = new WPI_TalonSRX(frontLeftMotorID);
	public static final WPI_TalonSRX backRightMotor = new WPI_TalonSRX(backRightMotorID);
	public static final WPI_TalonSRX backLeftMotor = new WPI_TalonSRX(backLeftMotorID);
	
	public static final WPI_TalonSRX elevatorMotor = new WPI_TalonSRX(elevatorMotorID);
	public static final WPI_TalonSRX elevatorFollowerMotor = new WPI_TalonSRX(elevatorFollowerMotorID);
	
	public static final WPI_TalonSRX intakeMotor = new WPI_TalonSRX(intakeMotorID);
	public static final WPI_TalonSRX intakeFollowerMotor = new WPI_TalonSRX(intakeFollowerMotorID);





	public static final MecanumDrive robotDrive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

	public static Joystick joystickDrive;
	public static Joystick joystickAccessory;
	
	//public static AHRS gyro;
	
	private static final int refreshrate = 60;
	
	public static void init()
	{

		frontRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		frontLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		backRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		backLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);

		frontLeftMotor.setInverted(true);
		backLeftMotor.setInverted(true);

		joystickDrive = new Joystick(0);
		joystickAccessory = new Joystick(1);
		
		//gyro = new AHRS(SPI.Port.kMXP, (byte)refreshrate);
		
	}
}
