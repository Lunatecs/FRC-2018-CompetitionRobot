package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PigeonMoveCommand extends Command {

	private boolean finished = false;
	private BaseMotorController sensorRight;
	private BaseMotorController sensorLeft;
	private DifferentialDrive drive;
	private int setPoint;
	private int closeToSetPoint;
	private int continualError = 0;
	private int previousError = 0;
	
	private PigeonIMU pigeonGyro;
	
	
	//private static final double Kp = 0.0000375;
	private double Kp = 0.00002556634;
	private double Ki = 0.0000003;
	private double Kd = 0.000003;
	
	private int tolerance = 100;
	private double maxPower = 1.0;
	private double minPower = 0.0;

	private double bearing = 0.0;
	
	public PigeonMoveCommand(PigeonIMU pigeonGyro, BaseMotorController sensorLeft, BaseMotorController sensorRight, DifferentialDrive drive, double inches, double maxPower, double minPower) {
		super();
		this.sensorRight = sensorRight;
		this.sensorLeft = sensorLeft;
		this.drive = drive;
		this.setPoint = (int) (inches * RobotConstants.TICS_PER_INCH);
		
		
		this.Kp = .5*(2.5/((double)Math.abs(setPoint)));
		this.Ki = this.Kp * 0.0007;
		this.Kd = this.Kp * .1;
		
		SmartDashboard.putString("KP", this.Kp + "");
		this.maxPower = maxPower;
		this.minPower = minPower;
		
		this.pigeonGyro = pigeonGyro;
	}
	
	
	public PigeonMoveCommand(PigeonIMU pigeonGyro, BaseMotorController sensorLeft, BaseMotorController sensorRight, DifferentialDrive drive, double inches, double maxPower, double minPower, double Kp, double Ki, double Kd, int tolerance) {
		super();
		this.sensorRight = sensorRight;
		this.sensorLeft = sensorLeft;
		this.drive = drive;
		this.setPoint = (int) (inches * RobotConstants.TICS_PER_INCH);
		
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		
		this.tolerance = tolerance;
		
		this.maxPower = maxPower;
		this.minPower = minPower;
		
		this.pigeonGyro = pigeonGyro;
	}
	
	private int getSensorPosition() {
		return (sensorLeft.getSelectedSensorPosition(0) + (-1*sensorRight.getSelectedSensorPosition(0)))/2;
	}
	
	protected void initialize() {
		this.sensorRight.setSelectedSensorPosition(0, 0, 10);
		this.sensorLeft.setSelectedSensorPosition(0, 0, 10);	
		this.bearing = getAngle();
	}
	
	@Override
	protected void execute() {
		SmartDashboard.putString("Set Point", setPoint + "");
		int position = getSensorPosition();
		
		int error = setPoint - position;

		SmartDashboard.putString("Error", error + "");	

		
		if(Math.abs(error)<tolerance) {
			drive.arcadeDrive(1.0, 0.0);
			drive.arcadeDrive(0.0, 0.0);
			this.finished = true;
			return;
		}
		
		
		continualError = continualError + error;
		int changeInError = error - previousError;
		previousError = error;
		
		double power = -1 * (Kp * (double)error + Ki * (double)continualError + Kd * (double)changeInError);
		
		SmartDashboard.putString("Pre Power", power + "");
		
		if(power<-maxPower) {
			power=-maxPower;
		} else if(power>maxPower) {
			power = maxPower;
		} 
		
		if(power< 0 && power>-minPower) {
			power=-minPower;
		} else if(power> 0 && power<minPower) {
			power = minPower;
		}
		
		SmartDashboard.putString("Post Power", power + "");
		
		/*
		int left = sensorLeft.getSelectedSensorPosition(0);
		int right = -1*sensorRight.getSelectedSensorPosition(0);
		
		int average = (left + right)/2;
		
		int diffRight = average - right;
		int diffLeft = average - left;
		double correcting = .00005;
		
		double powerLeft = power + (correcting * (double)diffRight);
		double powerRight = power + (correcting * (double)diffLeft);
		*/
		double aError = bearing - getAngle();
		
		double correct = 0.02 * aError;
		
		double powerRight = power + correct;
		double powerLeft = power - correct;
		
		//SmartDashboard.putString("Diff L R", diffLeft + " " + diffRight);
		SmartDashboard.putString("Power L R", powerLeft + " " + powerRight);
		
		drive.tankDrive(powerLeft, powerRight);
		
	}
	
	private double getAngle() {
		double[] ypr = new double[3];
		 pigeonGyro.getYawPitchRoll(ypr);
		 return ypr[0];
	}
	
	@Override
	protected boolean isFinished() {
		SmartDashboard.putBoolean("Finished", this.finished);
		return finished;
	}
	
}

