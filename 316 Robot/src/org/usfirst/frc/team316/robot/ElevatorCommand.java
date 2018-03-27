package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorCommand extends Command {

	private boolean isFinished = false;
	private TalonSRX motor;
	private DigitalInput top;
	private DigitalInput bottom;
	private int setPoint;
	
	
	public static final int BOTTOM_SET_POINT = 0;
	public static final int SWITCH_SET_POINT = (int)(RobotConstants.TICS_PER_ELEVATOR_INCH * 42.0);
	public static final int SCALE_UP_SET_POINT = (int)(RobotConstants.TICS_PER_ELEVATOR_INCH * RobotConstants.TOP_ELEVATOR_HEIGHT);
	public static final int SCALE_EVEN_SET_POINT = (int)(RobotConstants.TICS_PER_ELEVATOR_INCH * 65.0);
	public static final int SCALE_DOWN_SET_POINT = (int)(RobotConstants.TICS_PER_ELEVATOR_INCH * 53.0);
	
	public ElevatorCommand(TalonSRX motor, DigitalInput top, DigitalInput bottom, int setPoint) {
		this.motor = motor;
		this.top = top;
		this.bottom = bottom;
		this.setPoint = setPoint;
		requires(Robot.elevatorSubsystem);
	}
	
	protected void initialize() {
		this.isFinished = false;
	}
	
	public void setSetPoint(int setPoint) {
		this.setPoint = setPoint;
	}
	
	
	protected void end() {
	}
	
	protected void execute() {
		
		SmartDashboard.putNumber("Elevator Position:", this.setPoint);
		
		/*if(top.get() && this.setPoint == SCALE_UP_SET_POINT ) {
			motor.set(ControlMode.Position, motor.getSelectedSensorPosition(0));
			this.isFinished = true;
			return;
		}*/
		
		if(bottom.get() && this.setPoint == BOTTOM_SET_POINT ) {
			motor.setSelectedSensorPosition(0, 0, 10);
			this.isFinished = true;
			return;
		}

		
		motor.set(ControlMode.Position, setPoint);
		
		if(Math.abs(motor.getSelectedSensorPosition(0)-setPoint) < 200) {
			this.isFinished = true;
		}
		
		SmartDashboard.putBoolean("Elevator done:", isFinished);
		
	}
	
	@Override
	public boolean isInterruptible() {
		return true;
	}
	
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

}
