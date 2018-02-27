package org.usfirst.frc.team316.robot;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WaitCommand extends Command {

	private Date stopTime;
	private boolean isFinished = false;
	private int milliSeconds;
	
	public WaitCommand(int milliSeconds) {
		this.milliSeconds = milliSeconds;
	}
	
	@Override
	protected void initialize() {
		Calendar calculateDate = GregorianCalendar.getInstance();
		calculateDate.add(GregorianCalendar.MILLISECOND, milliSeconds);
		stopTime = calculateDate.getTime();
	}
	
	@Override
	protected void execute() {
		if(new Date().after(stopTime)) {
			this.isFinished = true;
		}
	}
	
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isFinished;
	}

}
