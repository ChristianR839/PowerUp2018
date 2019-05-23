package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

public class TurnToAngle extends Command {

	PIDController turnController;
	PigeonIMU gyro = new PigeonIMU(RobotMap.frontLeftMotor);
	TurnPidListener turnListener = null;
	String direction = "R";
	private float angle = 0;
	static final double kP = 0.02;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;
	double [] ypr = new double[3];
	static final double kToleranceDegrees = 1.0f;

	public TurnToAngle(float angle, String direction) {
		this.direction = direction;
		this.angle = angle;
		this.setTimeout(5);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (!Robot.fieldLayout.isOurSwitch(this.direction))
			this.angle = -angle;
		System.out.println("Init Turn to angle");
		turnListener = new TurnPidListener(gyro);
		this.gyro.setYaw(0, 0);
		this.turnController = new PIDController(kP, kI, kD, kF, turnListener, turnListener,.02);
		this.turnController.setInputRange(-180.0f,  180.0f);
		this.turnController.setOutputRange(-0.5, 0.5);
		this.turnController.setAbsoluteTolerance(kToleranceDegrees);
		this.turnController.setContinuous(false);	//CHANGE MADE 3/26 [NOT TESTED]
		this.turnController.setSetpoint(this.angle);


	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		turnController.enable();
        double currentRotationRate = turnListener.turnRate;

        try {

    		System.out.println("Turn rate: " + currentRotationRate);
         	System.out.println("Angle ERROR: " + this.turnController.getError());
         	Robot.drivetrain.setDriveSpeeds(0, 0, currentRotationRate, 0);
         	
         	this.gyro.getYawPitchRoll(ypr); 
         	System.out.println("Current angle!!!: " + ypr[0]);

        } catch( RuntimeException ex ) {
            DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return this.turnController.onTarget()|| this.isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        RobotMap.robotDrive.driveCartesian(0, 0,0);
		RobotMap.frontRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.frontLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.backRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.backLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}

	private class TurnPidListener  implements PIDOutput, PIDSource{
		public TurnPidListener(PigeonIMU gyro) {
			this.gyro = gyro;
			resetGyro();
		}
		
		public void resetGyro(){
			gyro.setYaw(0, 0);
		}
		
		public double turnRate = 0.0f;
		private PigeonIMU gyro = null;
		private PIDSourceType pidsourceType;
		@Override
		public void pidWrite(double output) {
			this.turnRate = output;
			
		}
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			this.pidsourceType = pidSource;
			
		}
		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
		@Override
		public double pidGet() {
         	double [] ypr = new double[3];
         	this.gyro.getYawPitchRoll(ypr);
			return ypr[0];
		}
		
	}

}
