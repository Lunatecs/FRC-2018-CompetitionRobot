package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroTurnCommand extends Command {

	private boolean finished = false;
	private Gyro gyro;
	private DifferentialDrive drive;
	private double degrees;
	private double continualError = 0;
	private double previousError = 0;
	
	
	//private static final double Kp = 0.0000375;
	private double Kp = 0.00002556634;
	private double Ki = 0.0000003;
	private double Kd = 0.000003;
	
	private double tolerance = .75;
	
	private double maxPower = 1.0;
	private double minPower = 0.0;
	
	public GyroTurnCommand(Gyro gyro, DifferentialDrive drive, double degrees, double maxPower, double minPower) {
		super();
		this.gyro = gyro;
		this.drive = drive;
		this.degrees = degrees;
		
		
		this.Kp = 0.012;
		this.Ki = this.Kp * 0.01;
		this.Kd = this.Kp * .1;
		
				
		this.maxPower = maxPower;
		this.minPower = minPower;
	}
	
	@Override
	protected void initialize() {
		//gyro.calibrate();
		gyro.reset();
	}
	
	@Override
	protected void execute() {

		
		double d = gyro.getAngle();
		
		double error = degrees - d;

		SmartDashboard.putString("Error", error + "");	
		SmartDashboard.putString("Degrees", degrees + "");
		SmartDashboard.putNumber("Gryo Move", d);
		
		if(error<tolerance && error>-tolerance) {
			drive.arcadeDrive(0.0, 0.0);
			this.finished = true;
			return;
		}
		
		
		continualError = continualError + error;
		double changeInError = error - previousError;
		previousError = error;
		
		double power = (Kp * (double)error + Ki * (double)continualError + Kd * (double)changeInError);
		
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
		
		drive.tankDrive(power, -power);
		
	}
	
	
	@Override
	protected boolean isFinished() {
		return finished;
	}

}
