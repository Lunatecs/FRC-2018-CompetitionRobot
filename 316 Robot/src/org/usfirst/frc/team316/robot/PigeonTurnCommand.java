package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PigeonTurnCommand extends Command {

	private boolean finished = false;
	private PigeonIMU pigeonGyro;
	private DifferentialDrive drive;
	private double degrees;
	private double goalDegrees;
	private double twoThirdDegrees;
	private double continualError = 0;
	private double previousError = 0;
	
	
	//private static final double Kp = 0.0000375;
	private double Kp = 0.0;
	private double Ki = 0.0;
	private double Kd = 0.0;
	private double Ki_temp = 0.0;
	
	private double tolerance = .2;
	
	private double maxPower = 1.0;
	private double minPower = 0.0;
	
	public PigeonTurnCommand(PigeonIMU pigeonGyro, DifferentialDrive drive, double degrees, double maxPower, double minPower) {
		super();
		this.pigeonGyro = pigeonGyro;
		this.drive = drive;
		this.degrees = degrees;
		
		
		//this.Kp = 0.012;
		//this.Kp = 0.007;
		this.Kp = 0.02;
		this.Ki = this.Kp * 0;//0.01;
		this.Kd = this.Kp * 0;//0.1;
		
				
		this.maxPower = maxPower;
		this.minPower = minPower;
	}

	public PigeonTurnCommand(PigeonIMU pigeonGyro, DifferentialDrive drive, double degrees, double maxPower, double minPower, double Kp, double Ki, double Kd) {
		super();
		this.pigeonGyro = pigeonGyro;
		this.drive = drive;
		this.degrees = degrees;
		
		
		//this.Kp = 0.012;
		//this.Kp = 0.007;
		this.Kp = Kp;
		this.Ki_temp = Ki;//0.01;
		this.Kd = Kd;//0.1;
		
				
		this.maxPower = maxPower;
		this.minPower = minPower;
	}
	
	@Override
	protected void initialize() {
		//gyro.calibrate();
		double deg = getAngle();
		this.goalDegrees = deg + this.degrees;
		this.twoThirdDegrees = deg + (this.degrees*2.0/3.0);
	}
	
	private double getAngle() {
		double[] ypr = new double[3];
		 pigeonGyro.getYawPitchRoll(ypr);
		 return ypr[0];
	}
	
	@Override
	protected void execute() {

		
		double d = getAngle();
		
		double error = d - this.goalDegrees;

		SmartDashboard.putString("Error", error + "");	
		SmartDashboard.putString("Degrees", degrees + "");
		SmartDashboard.putString("Goal", this.goalDegrees +"");
		SmartDashboard.putNumber("Gryo Move", d);
		
		if(Math.abs(error)<Math.abs(tolerance)) {
			drive.arcadeDrive(0.0, 0.0);
			this.finished = true;
			return;
		}
		
		
		continualError = continualError + error;
		double changeInError = error - previousError;
		previousError = error;
		
		double power = (Kp * (double)error + Ki * (double)continualError + Kd * (double)changeInError);
		
		SmartDashboard.putString("Pre Power", power + "");
		
		SmartDashboard.putString("Pre Power", power +"");
		
		if(power<-maxPower) {
			power=-maxPower;
		} else if(power>maxPower) {
			power = maxPower;
		} 
		
		SmartDashboard.putString("Mid Power", power +"");
		
		if(power< 0 && power>-minPower) {
			power=-minPower;
		} else if(power> 0 && power<minPower) {
			power = minPower;
		} 
			
		SmartDashboard.putString("Post Power", power + "");
		
		SmartDashboard.putBoolean("Two Thirds", Math.abs(d)>Math.abs(this.twoThirdDegrees));
		
		if(Math.abs(d)>Math.abs(this.twoThirdDegrees)) {
			this.Ki = this.Ki_temp;
		} 
		
		if((this.continualError < 0 && error > 0) || (this.continualError > 0 && error < 0)) {
			this.continualError = 0; 
		}
		
		SmartDashboard.putString("Continue Error", this.continualError + "");
		
		SmartDashboard.putString("Third Power", power + "");
		
		drive.tankDrive(power, -power);
		
	}
	
	
	@Override
	protected boolean isFinished() {
		return finished;
	}

}
