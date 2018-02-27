package org.usfirst.frc.team316.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class LowerIntakeCommand extends Command {

	private boolean isFinished = false;
	private DoubleSolenoid intake;
	
	
	public LowerIntakeCommand(DoubleSolenoid intake) {
		super();
		this.intake = intake;
	}
	
	@Override
	protected void execute() {
		this.intake.set(DoubleSolenoid.Value.kReverse);
		//this.intake.set(DoubleSolenoid.Value.kOff);
		isFinished = true;
	}
	
	
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

}
