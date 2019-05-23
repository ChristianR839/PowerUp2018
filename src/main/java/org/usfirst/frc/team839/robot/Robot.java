package org.usfirst.frc.team839.robot;


import org.usfirst.frc.team839.robot.commands.autonomous.ChooseSwitch;
import org.usfirst.frc.team839.robot.commands.autonomous.CrossTheLine;
import org.usfirst.frc.team839.robot.commands.autonomous.ScaleDrop;
import org.usfirst.frc.team839.robot.commands.autonomous.SwitchDropLeft;
import org.usfirst.frc.team839.robot.commands.autonomous.SwitchDropRight;
import org.usfirst.frc.team839.robot.commands.autonomous.TurnToTestAngle;
import org.usfirst.frc.team839.robot.subsystems.DriveTrain;
import org.usfirst.frc.team839.robot.subsystems.Elevator;
import org.usfirst.frc.team839.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot 
{
	public static Intake intake = null;
	public static Elevator elevator = null;
	public static DriveTrain drivetrain;
	//public static Gyro gyro;

	public static OI oi;
	public static double defaultRampRate = .2;

    Command autonomousCommand;
    SendableChooser<Command> chooser;
	public static FieldLayout fieldLayout;
	public static float angleValue;
	public static enum Direction {
	    FORWARD, BACKWARD, STRAFE_LEFT, STRAFE_RIGHT 
	};
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() 
    {
    	drivetrain = new DriveTrain();
    	//gyro = new Gyro();
    	intake = new Intake();
    	elevator = new Elevator();
		oi = new OI();
        chooser = new SendableChooser<Command>();
        chooser.addObject("Switch Drop Right", new SwitchDropRight());
        chooser.addObject("Switch Drop Left", new SwitchDropLeft());
        chooser.addObject("Choose Switch", new ChooseSwitch());
        chooser.addObject("Scale Drop", new ScaleDrop());
        chooser.addDefault("Cross the Line", new CrossTheLine());
        chooser.addObject("Choose Switch", new ChooseSwitch());
        chooser.addObject("TurnToTestAngle", new TurnToTestAngle());
        //CameraServer.getInstance().startAutomaticCapture();
        SmartDashboard.putData("Auto mode", chooser);
        //SmartDashboard.putData("Gyro", gyro);
		//CameraServer.getInstance().setQuality(50);
		CameraServer.getInstance().startAutomaticCapture();


    }
	
	/**
     * This function is called once each time the robot enters Disabled mode.
     * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
     */
    public void disabledInit()
    {

    }
	
	public void disabledPeriodic() 
	{
		Scheduler.getInstance().run();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the getString code to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional commands to the chooser code above (like the commented example)
	 * or additional comparisons to the switch structure below with additional strings & commands.
	 */
    public void autonomousInit() 
    {
    	fieldLayout = new FieldLayout(DriverStation.getInstance().getGameSpecificMessage().toUpperCase());
        autonomousCommand = (Command) chooser.getSelected();
        SmartDashboard.putData("Field Locations", autonomousCommand);
        System.out.println(fieldLayout);
        
		angleValue = (float) SmartDashboard.getNumber("angle", 0);
        
        RobotMap.frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        RobotMap.frontRightMotor.setSelectedSensorPosition(0, 0, 0);
        RobotMap.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        RobotMap.elevatorMotor.setSelectedSensorPosition(0, 0, 0);
        
		/* String autoSelected = SmartDashboard.getString("Auto Selector", "Default");
		switch(autoSelected) {
		case "My Auto":
			autonomousCommand = new MyAutoCommand();
			break;
		case "Default Auto":
		default:
			autonomousCommand = new ExampleCommand();
			break;
		} */
    	
    	// schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() 
    {
        Scheduler.getInstance().run();
    }

    public void teleopInit() 
    {
		// This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to 
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
		RobotMap.frontRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.frontLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.backRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.backLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		
        //RobotMap.frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        //RobotMap.frontRightMotor.setSelectedSensorPosition(0, 0, 0);
        //RobotMap.elevatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
        //RobotMap.elevatorMotor.setSelectedSensorPosition(0, 0, 0);
        
        System.out.print(RobotMap.intakeMotor.getOutputCurrent());

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() 
    {
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() 
    {

    }
}
