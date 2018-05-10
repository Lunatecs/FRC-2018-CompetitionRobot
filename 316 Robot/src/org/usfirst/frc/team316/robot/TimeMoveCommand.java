package org.usfirst.frc.team316.robot;

import java.util.Calendar;
import java.util.Date;
import java.util.GregorianCalendar;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class TimeMoveCommand extends Command {

	private Date stopTime;
	private boolean isFinished = false;
	private int milliSeconds;
	DifferentialDrive drive;
	private double go = -.5;
	
	public TimeMoveCommand(int milliSeconds, DifferentialDrive drive) {
		this.milliSeconds = milliSeconds;
		this.drive = drive;
	}
	
	public TimeMoveCommand(int milliSeconds, DifferentialDrive drive, double go) {
		this.milliSeconds = milliSeconds;
		this.drive = drive;
		this.go = go;
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
			drive.arcadeDrive(0, 0);
			return;
		}
		
		drive.tankDrive(go, go);
		
	}
	
	
	@Override
	protected boolean isFinished() {
		// TODO Auto-generated method stub
		return isFinished;
	}

}
