package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RangeFinderMoveCommand extends Command {
	private boolean finished = false;
	private Ultrasonic finder;
	private DifferentialDrive drive;
	private double setPoint;
	private double continualError = 0;
	private double previousError = 0;
	
	
	//private static final double Kp = 0.0000375;
	private double Kp = 0.00002556634;
	private double Ki = 0.0000003;
	private double Kd = 0.000003;
	
	private int tolerance = 1;
	private double maxPower = 1.0;
	private double minPower = 0.0;

	private boolean reverse = false;
	private boolean atLeast = false;
	
	public RangeFinderMoveCommand(Ultrasonic finder, DifferentialDrive drive, double inches, double maxPower, double minPower, boolean reverse) {
		super();
		this.finder = finder;
		this.drive = drive;
		
		this.reverse = !reverse;
		
		this.setPoint = inches;
		
		
		this.Kp = .0334;
		this.Ki = this.Kp * 0.001;
		this.Kd = this.Kp * .1;
		
		this.maxPower = maxPower;
		this.minPower = minPower;
	}
	
	public RangeFinderMoveCommand(Ultrasonic finder, DifferentialDrive drive, double inches, double maxPower, double minPower, boolean reverse, boolean atLeast) {
		super();
		this.finder = finder;
		this.drive = drive;
		
		this.reverse = !reverse;
		
		this.setPoint = inches;
		
		
		this.Kp = .0334;
		this.Ki = this.Kp * 0.001;
		this.Kd = this.Kp * .1;
		
		this.maxPower = maxPower;
		this.minPower = minPower;
		this.atLeast = atLeast;
	}
	
	
	public RangeFinderMoveCommand(Ultrasonic finder, DifferentialDrive drive, double inches, double maxPower, double minPower, double Kp, double Ki, double Kd, int tolerance, boolean reverse) {
		super();
		this.finder = finder;
		this.drive = drive;

		this.reverse = !reverse;
		
		this.setPoint = inches;
		
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		
		this.tolerance = tolerance;
		
		this.maxPower = maxPower;
		this.minPower = minPower;
	}
	

	
	protected void initialize() {
		
	}
	
	public void setAtLeast(boolean atLeast) {
		this.atLeast = atLeast;
	}
	
	@Override
	protected void execute() {
		SmartDashboard.putString("Set Point", setPoint + "");
		//finder.ping();
		double position = finder.getRangeInches();
		
		//while(!finder.isRangeValid()) {
			//finder.ping();
		//	position = finder.getRangeInches();
		//}
		
		double error = setPoint - position;

		SmartDashboard.putString("Error", error + "");	

		if(atLeast) {
			SmartDashboard.putString("Tolerance", tolerance+"");
			SmartDashboard.putBoolean("At Least", error > tolerance);
		}
		
		
		if(error<tolerance && error>-tolerance) {
			drive.arcadeDrive(0.0, 0.0);
			this.finished = true;
			return;
		//} else if(error > tolerance){
		//	drive.arcadeDrive(0.0, 0.0);
		//	this.finished = true;
		//	return;
		}
		
		
		continualError = continualError + error;
		double changeInError = error - previousError;
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
		
		if(reverse) {
			power = -power;
		}
		
		drive.tankDrive(power, power);
		
	}
	
	
	@Override
	protected boolean isFinished() {
		SmartDashboard.putBoolean("Finished", this.finished);
		return finished;
	}

}
