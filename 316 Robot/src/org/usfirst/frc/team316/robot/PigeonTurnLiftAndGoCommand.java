package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PigeonTurnLiftAndGoCommand extends Command {

	
	private PigeonIMU pigeonGyro;
	private DifferentialDrive drive;
	private double degrees; 
	private double turningMaxPower; 
	private double turningMinPower;
	private double bearing;
	private double twoThirdDegrees;
	private double continualError;
	private double previousError;
	private double turningKp;
	private double turningKi;
	private double turningKd;
	private double inches;
	private double driveMaxPower;
	private double driveMinPower;
	private double driveKp;
	private double driveKi;
	private double driveKd;
	private BaseMotorController sensorLeft;
	private BaseMotorController sensorRight;
	
	private int setPoint;
	private boolean turning;
	private boolean initDrive;
	private boolean lift;
	private boolean isFinish;
	private double turningKi_temp;
	private int driveTolerance;
	private double turningTolerance;
	
	
	private TalonSRX elevatorMotor;
	private DigitalInput elevatorTop;
	private DigitalInput elevatorBottom;
	private int elevatorSetPoint = -1;
	
	public PigeonTurnLiftAndGoCommand(PigeonIMU pigeonGyro, DifferentialDrive drive, double degrees, double turningMaxPower, double turningMinPower,
			double turningKp, double turningKi, double turningKd, double turningTolerance,
			BaseMotorController sensorLeft, BaseMotorController sensorRight, double inches, double driveMaxPower, double driveMinPower,
			double driveKp, double driveKi, double driveKd, int driveTolerance) {
		//pigeon gyro, drive, angle, min and max power, 
		super();
		this.pigeonGyro = pigeonGyro;
		this.drive = drive;
		this.degrees = degrees;
		this.turningMaxPower = turningMaxPower;
		this.turningMinPower = turningMinPower;
		this.driveMaxPower = driveMaxPower;
		this.driveMinPower = driveMinPower;
		this.inches = inches;
		this.sensorRight = sensorRight;
		this.sensorLeft = sensorLeft;
		this.setPoint = (int) (inches * RobotConstants.TICS_PER_INCH);
		
		this.driveKp = driveKp;
		this.driveKi = driveKi;
		this.driveKd = driveKd;
		this.driveTolerance = driveTolerance;
		
		this.turningKp = turningKp;
		this.turningKi_temp = turningKi;
		this.turningKd = turningKd;
		this.turningTolerance = turningTolerance;
	}
	
	public PigeonTurnLiftAndGoCommand(PigeonIMU pigeonGyro, DifferentialDrive drive, double degrees, double turningMaxPower, double turningMinPower,
			double turningKp, double turningKi, double turningKd, double turningTolerance,
			BaseMotorController sensorLeft, BaseMotorController sensorRight, double inches, double driveMaxPower, double driveMinPower,
			double driveKp, double driveKi, double driveKd, int driveTolerance,
			TalonSRX motor, DigitalInput top, DigitalInput bottom, int setPoint) {
		//pigeon gyro, drive, angle, min and max power, 
		super();
		this.pigeonGyro = pigeonGyro;
		this.drive = drive;
		this.degrees = degrees;
		this.turningMaxPower = turningMaxPower;
		this.turningMinPower = turningMinPower;
		this.driveMaxPower = driveMaxPower;
		this.driveMinPower = driveMinPower;
		this.inches = inches;
		this.sensorRight = sensorRight;
		this.sensorLeft = sensorLeft;
		this.setPoint = (int) (inches * RobotConstants.TICS_PER_INCH);
		
		this.driveKp = driveKp;
		this.driveKi = driveKi;
		this.driveKd = driveKd;
		this.driveTolerance = driveTolerance;
		
		this.turningKp = turningKp;
		this.turningKi_temp = turningKi;
		this.turningKd = turningKd;
		this.turningTolerance = turningTolerance;
		
		this.elevatorMotor = motor;
		this.elevatorTop = top;
		this.elevatorBottom = bottom;
		this.elevatorSetPoint = setPoint;
		requires(Robot.elevatorSubsystem);
	}
	
	
	
	@Override
	protected void initialize() {		
		//set bearing to the pigeon angle plus desired angle
		double deg = getAngle();
		this.bearing = deg + this.degrees;
		this.turning = true;
		this.isFinish = false;
		this.initDrive = false;
		this.lift=false;
		
		this.twoThirdDegrees = deg + (this.degrees*2.0/3.0);
	}
	
	private double getAngle() {
		double[] ypr = new double[3];
		 pigeonGyro.getYawPitchRoll(ypr);
		 return ypr[0];
	}
	
	private int getSensorPosition() {
		return (sensorLeft.getSelectedSensorPosition(0) + (-1*sensorRight.getSelectedSensorPosition(0)))/2;
	}
	
	
	@Override
	protected void execute() {
		
		
		//turn to desired angle
		//turn using the PID loop with error being the difference between the current angle and bearing
		
		//if turning 
		if(turning) {
			//get angle
			double d = getAngle();
		
			//calculate error based off angle
			double error = d - this.bearing;
		
			//check if we are done within tolerance if so move to next phase
			if(Math.abs(error) < this.turningTolerance ) {
				turning = false;
				initDrive = true;
			}else {
			
				//calculate continual and change in error for PID
				continualError = continualError + error;
				double changeInError = error - previousError;
				previousError = error;
		
				//calculate power using PID
				double power = (turningKp * (double)error + turningKi * (double)continualError + turningKd * (double)changeInError);
				
				//check that the power is with min and max power
				if(power<-turningMaxPower) {
					power=-turningMaxPower;
				} else if(power>turningMaxPower) {
					power = turningMaxPower;
				} 
				
				
				if(power< 0 && power>-turningMinPower) {
					power=-turningMinPower;
				} else if(power> 0 && power<turningMinPower) {
					power = turningMinPower;
				} 
					
				//if we are within 2/3 degree start applying kP
				if(Math.abs(d)>Math.abs(this.twoThirdDegrees)) {
					this.turningKi = this.turningKi_temp;
				} 
				
				//reset continual error if the 0 is crossed, this will help prevent integral wine-up
				if((this.continualError < 0 && error > 0) || (this.continualError > 0 && error < 0)) {
					this.continualError = 0; 
				}
				
				//apply power to motors
				drive.tankDrive(power, -power);
				SmartDashboard.putString("Stage:", "Turning");
			}
		} else if(initDrive) {
			//reset the sensors for driving straight
			this.sensorRight.setSelectedSensorPosition(0, 0, 10);
			this.sensorLeft.setSelectedSensorPosition(0, 0, 10);	
			initDrive = false;
			lift = true;
			SmartDashboard.putString("Stage:", "InitDrive");
		} else if(lift) {
			if(this.elevatorSetPoint!=-1) {
				elevatorMotor.set(ControlMode.Position, elevatorSetPoint);
			}
			lift=false;
		} else {
			SmartDashboard.putString("Stage:", "Drive");
			//get position
			int position = getSensorPosition();
			
			//calculate error
			int error = setPoint - position;
			
			//check if we are done within tolerance
			if(Math.abs(error)< this.driveTolerance) {
				drive.arcadeDrive(1.0, 0.0);
				drive.arcadeDrive(0.0, 0.0);
				this.isFinish = true;
				SmartDashboard.putString("Stage:", "Finished");
				return;
			}
			
			//calculate continual and change in error for PID
			continualError = continualError + error;
			double changeInError = error - previousError;
			previousError = error;
			
			//calculate power using PID
			double power = -1 * (driveKp * (double)error + driveKi * (double)continualError + driveKd * (double)changeInError);
			
			SmartDashboard.putString("Pre Power", power + "");
			
			//check that the power is with min and max power
			if(power<-driveMaxPower) {
				power=-driveMaxPower;
			} else if(power>driveMaxPower) {
				power = driveMaxPower;
			} 
			
			SmartDashboard.putString("Mid Power", power + "");
			
			if(power< 0 && power>-driveMinPower) {
				power=-driveMinPower;
			} else if(power> 0 && power<driveMinPower) {
				power = driveMinPower;
			}
			
			SmartDashboard.putString("Post Power", power + "");
			
			//auto correct for going straight based on the bearing we are heading
			double aError = bearing - getAngle();
			
			double correct = 0.02 * aError;
			
			double powerRight = power + correct;
			double powerLeft = power - correct;

			SmartDashboard.putString("Power L R", powerLeft + " " + powerRight);
			
			//apply power to the motors
			drive.tankDrive(powerLeft, powerRight);
			
		}
		
		
	}
	
	@Override
	protected boolean isFinished() {
		return this.isFinish;
	}

}
