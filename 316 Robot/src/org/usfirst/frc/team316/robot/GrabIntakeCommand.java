package org.usfirst.frc.team316.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Command;

public class GrabIntakeCommand extends Command {

	private boolean isFinished = false;
	private DoubleSolenoid intake;
	private boolean open = false;
	
	
	public GrabIntakeCommand(DoubleSolenoid intake, boolean open) {
		super();
		this.intake = intake;
		this.open = open;
	}
	
	@Override
	protected void execute() {
		if(open) {
			this.intake.set(DoubleSolenoid.Value.kReverse);
		} else {
			this.intake.set(DoubleSolenoid.Value.kForward);
		}
		isFinished = true;
	}
	
	
	@Override
	protected boolean isFinished() {
		return isFinished;
	}

}
