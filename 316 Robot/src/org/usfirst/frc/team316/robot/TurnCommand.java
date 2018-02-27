package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnCommand extends Command {

	private boolean finished = false;
	private BaseMotorController sensorRight;
	private BaseMotorController sensorLeft;
	private DifferentialDrive drive;
	private int setPoint;
	private int continualError = 0;
	private int previousError = 0;
	
	
	//private static final double Kp = 0.0000375;
	private double Kp = 0.00002556634;
	private double Ki = 0.0000003;
	private double Kd = 0.000003;
	
	private int tolerance = 100;
	
	private double maxPower = 1.0;
	private double minPower = 0.0;
	
	public TurnCommand(BaseMotorController sensorLeft, BaseMotorController sensorRight, DifferentialDrive drive, double degrees, double maxPower, double minPower) {
		super();
		this.sensorRight = sensorRight;
		this.sensorLeft = sensorLeft;
		this.drive = drive;
		this.setPoint = (int) ((-1*degrees) * RobotConstants.TICS_PER_DEGREE);
		
		
		this.Kp = .5*(2.7/((double)Math.abs(setPoint)));
		this.Ki = this.Kp * 0.0;
		this.Kd = this.Kp * .1;
		
		SmartDashboard.putString("KP", this.Kp + "");
				
		this.maxPower = maxPower;
		this.minPower = minPower;
	}
	
	
	private int getSensorPosition() {
		return (sensorLeft.getSelectedSensorPosition(0) + sensorRight.getSelectedSensorPosition(0))/2;
	}
	
	@Override
	protected void initialize() {
		this.sensorRight.setSelectedSensorPosition(0, 0, 10);
		this.sensorLeft.setSelectedSensorPosition(0, 0, 10);	
		
	}
	
	@Override
	protected void execute() {
		SmartDashboard.putString("Set Point", setPoint + "");
		int position = getSensorPosition();
		
		int error = setPoint - position;

		SmartDashboard.putString("Error", error + "");	

		
		if(error<tolerance && error>-tolerance) {
			drive.arcadeDrive(0.0, 0.0);
			this.finished = true;
			return;
		}
		
		
		continualError = continualError + error;
		int changeInError = error - previousError;
		previousError = error;
		
		double power = -1 * (Kp * (double)error + Ki * (double)continualError + Kd * (double)changeInError);
		
		SmartDashboard.putString("Power", power + "");
		
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
	
		int left = sensorLeft.getSelectedSensorPosition(0);
		int right = -1*sensorRight.getSelectedSensorPosition(0);
		
		int average = (left + right)/2;
		
		int diffRight = average - right;
		int diffLeft = average - left;
		double correcting = 0;
		
		double powerLeft = power - (correcting * (double)diffRight);
		double powerRight = power - (correcting * (double)diffLeft);
		
		SmartDashboard.putString("Power L R", powerLeft + " " + powerRight);
		
		drive.tankDrive(powerLeft, -powerRight);
		
	}
	
	
	@Override
	protected boolean isFinished() {
		SmartDashboard.putBoolean("Finished", this.finished);
		return finished;
	}
	
}
