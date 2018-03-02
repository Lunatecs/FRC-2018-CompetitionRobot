/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team316.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project. test commit
 */
public class Robot extends IterativeRobot {
	
	public static Elevator elevatorSubsystem = new Elevator();
	
	private static final String kDefaultAuto = "Default";
	private static final String CenterSwitchAuto = "Center Switch Auto";
	private static final String TenFeetForwardAuto = "Ten Forward Auto";
	private static final String ShootCubeAuto = "Shoot Cube Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	private DifferentialDrive drive;
	private Joystick joyStick;
	private Joystick opJoyStick;
	private DriverStation station;
	
	private NeutralMode driveNeutralMode = NeutralMode.Brake;
	
	private WPI_TalonSRX leaderMiddleRightDrive;
	private WPI_TalonSRX leaderMiddleLeftDrive;
	
	private TalonSRX leaderElevator;
	
	private VictorSPX leaderIntake;
	
	private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	DigitalInput elevatorButtomLimitSwitch;
	DigitalInput elevatorTopLimitSwitch;
	DoubleSolenoid intakeSolenoid;
	

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		station = DriverStation.getInstance();
		
		m_chooser.addDefault("Default Auto - Lifts and Lowers Intake, nothing else", kDefaultAuto);
		m_chooser.addObject("Center Switch Auto", CenterSwitchAuto);
		m_chooser.addObject("Ten Feet Foward Auto", TenFeetForwardAuto);
		m_chooser.addObject("Shoot Cube Auto - Illegal", ShootCubeAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		leaderMiddleRightDrive = new WPI_TalonSRX(15);
		WPI_VictorSPX followerFrontRightDrive = new WPI_VictorSPX(16);
		WPI_VictorSPX followerBackRightDrive = new WPI_VictorSPX(14);
		
		leaderMiddleLeftDrive = new WPI_TalonSRX(2);
		WPI_VictorSPX followerFrontLeftDrive = new WPI_VictorSPX(3);
		WPI_VictorSPX followerBackLeftDrive = new WPI_VictorSPX(1);
		
		
		leaderMiddleRightDrive.configOpenloopRamp(0, 10);
		leaderMiddleLeftDrive.configOpenloopRamp(0, 10);
		
		leaderMiddleRightDrive.setNeutralMode(driveNeutralMode);
		followerFrontRightDrive.setNeutralMode(driveNeutralMode);
		followerBackRightDrive.setNeutralMode(driveNeutralMode);
		
		leaderMiddleLeftDrive.setNeutralMode(driveNeutralMode);
		followerFrontLeftDrive.setNeutralMode(driveNeutralMode);
		followerBackLeftDrive.setNeutralMode(driveNeutralMode);
		
		
		followerFrontRightDrive.follow(leaderMiddleRightDrive);
		followerBackRightDrive.follow(leaderMiddleRightDrive);
		
		followerFrontLeftDrive.follow(leaderMiddleLeftDrive);
		followerBackLeftDrive.follow(leaderMiddleLeftDrive);
		
		leaderMiddleLeftDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
		leaderMiddleRightDrive.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,0,0);
		
		 drive = new DifferentialDrive(leaderMiddleLeftDrive, leaderMiddleRightDrive);
		 
		 joyStick = new Joystick(0);
		 opJoyStick = new Joystick(1);
		 
		 gyro.calibrate();
		 
		 leaderElevator = new TalonSRX(6);
		 VictorSPX followerElevator = new VictorSPX(5);
		 
		 followerElevator.setInverted(false);
		 
		 leaderElevator.setNeutralMode(driveNeutralMode);
		 followerElevator.setNeutralMode(driveNeutralMode);
		 
		 leaderElevator.configPeakOutputForward(1, 10);
		 followerElevator.configPeakOutputForward(1, 10);
		 
		 leaderElevator.configPeakOutputReverse(-1, 10);
		 followerElevator.configPeakOutputReverse(-1, 10);
		 
		 followerElevator.follow(leaderElevator);
		 
		 leaderElevator.setSelectedSensorPosition(0, 0, 10);
		 leaderElevator.configOpenloopRamp(.25, 10);
		 
		 //leaderElevator.config_kP(0, .13 , 10);
		 
		 leaderElevator.config_kP(0, .18 , 10);
		 leaderElevator.configAllowableClosedloopError(0, 200, 10);
		 
		 leaderIntake = new VictorSPX(11);
		 VictorSPX followerIntake = new VictorSPX(12);
		 
		 leaderIntake.setNeutralMode(driveNeutralMode);
		 followerIntake.setNeutralMode(driveNeutralMode);
		 
		 followerIntake.setInverted(true);
		 
		 followerIntake.follow(leaderIntake);
		 
		 elevatorButtomLimitSwitch = new DigitalInput(0);
		 elevatorTopLimitSwitch = new DigitalInput(1);
		 
		 System.out.println("START DISPLAYING OUTPUT");
		 
		 System.out.println(elevatorButtomLimitSwitch.getAnalogTriggerTypeForRouting());
		 System.out.println(elevatorButtomLimitSwitch.getChannel());
		 System.out.println(elevatorButtomLimitSwitch.getPortHandleForRouting());
		 System.out.println(elevatorButtomLimitSwitch.isAnalogTrigger());
		 //this.elevatorButtomLimitSwitch.enableInterrupts();
		 
		 System.out.println("END DISPLAYING OUTPUT");
		 
		 this.intakeSolenoid = new DoubleSolenoid(0,1); 
		 
		 this.leaderElevator.setSelectedSensorPosition(0, 0, 10);
		 this.elevatorCommand = new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, 0);
	}

	@Override
	public void robotPeriodic() {
		 SmartDashboard.putData("Our Encoder", this.leaderMiddleRightDrive);
		 //SmartDashboard.putData("Our Gyro", gyro);
		 //SmartDashboard.putBoolean("Drive Data", drive.isAlive());
		 
		 //SmartDashboard.putString("Z Gyro", gyro.getAngle() + "");
		 
		 //SmartDashboard.putNumber("Output Voltage", leaderElevator.getMotorOutputVoltage());
		 
		 SmartDashboard.putNumber("Elevator Encoder:", leaderElevator.getSelectedSensorPosition(0));
		 
		 //SmartDashboard.putData("Our Drive", drive);
		 
		 //SmartDashboard.putBoolean("Fwd Solenoid Intake", intakeSolenoid.isFwdSolenoidBlackListed());
		 //SmartDashboard.putBoolean("Rev Solenoid Intake", intakeSolenoid.isRevSolenoidBlackListed());
		 //SmartDashboard.putString("Intake SoleNoid",intakeSolenoid.get() +"");
		 //SmartDashboard.putString("Right Postion", this.leaderMiddleRightDrive.getSelectedSensorPosition(0) + "");
		 //SmartDashboard.putString("Left Postion", this.leaderMiddleLeftDrive.getSelectedSensorPosition(0) + "");
		 //SmartDashboard.putBoolean("Elevator Buttom Sensor", elevatorButtomLimitSwitch.get());
		 //SmartDashboard.putBoolean("Elevator Top Sensor", elevatorTopLimitSwitch.get());
		 
		 if(elevatorButtomLimitSwitch.get()) {
			 this.leaderElevator.setSelectedSensorPosition(0, 0, 10);
		 }
		 
	}
	
	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		
		String gameData = station.getGameSpecificMessage();		
		char allianceSwitch = gameData.charAt(0);

		//CommandGroup firstCommand = new CommandGroup();
		//firstCommand.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));
		//firstCommand.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
		
		
		
		move = new CommandGroup();
		//move.addParallel(firstCommand);
		double minPower = .4;
		move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));
		move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
		switch(m_autoSelected) {
			case CenterSwitchAuto:
				if (allianceSwitch == 'R') {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 36.0, .75, minPower));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 45.0, .5, minPower));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 54.0, .5, .1, .0001, 0.00001, 0.0, 150));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -50.0, .5, minPower));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 40.0, .5, minPower, .00016, 0.00001, 0.0, 150));
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .5, 500));
				} else if (allianceSwitch == 'L') {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 36.0, .75, minPower));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -45.0, .5, minPower));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 84.0, .75, minPower, .00016, 0.0, 0.0, 150));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 45.0, .5, minPower));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 6.0, .75, minPower, .00016, 0.0, 0.0, 150));
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .5, 500));
				}
				break;
			case TenFeetForwardAuto:
				move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 120.0, .75, minPower, .00004, 0.0000004, 0.000004, 150));
				break;
			case ShootCubeAuto:
				//move.addParallel(new ElevatorCommand(this.leaderElevator,this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));
				move.addSequential(new WaitCommand(3000));
				move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .5, 500));
			default:
		}
		move.start();
	}

	CommandGroup move;
	boolean start = true;
	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
			
		Scheduler.getInstance().run();
		
	}
	
	@Override
	public void teleopInit() {
		this.leaderMiddleRightDrive.setSelectedSensorPosition(0, 0, 10);
		this.leaderMiddleLeftDrive.setSelectedSensorPosition(0, 0, 10);
		
	}
	
	ElevatorCommand elevatorCommand;
	boolean piston = false;
	
	
	
	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		drive(joyStick.getRawAxis(1), joyStick.getRawAxis(4));
		
		//green button
		if(this.opJoyStick.getRawButton(1)) {
			elevatorCommand.cancel();
			elevatorCommand.setSetPoint(ElevatorCommand.BOTTOM_SET_POINT);
			elevatorCommand.start();
		} 
		
		//red button
		if(this.opJoyStick.getRawButton(2)) {
			elevatorCommand.cancel();
			elevatorCommand.setSetPoint(ElevatorCommand.SWITCH_SET_POINT);
			elevatorCommand.start();
		} 
		
		//blue button
		if(this.opJoyStick.getRawButton(3)) {
			elevatorCommand.cancel();
			elevatorCommand.setSetPoint(ElevatorCommand.SCALE_EVEN_SET_POINT);
			elevatorCommand.start();
		} 

		if(this.opJoyStick.getRawButton(5)) {
			this.intakeSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if(this.opJoyStick.getRawButton(6)) {
			this.intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			this.intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		}
		
		if(this.opJoyStick.getRawAxis(2)>0) {
			this.leaderIntake.set(ControlMode.PercentOutput, opJoyStick.getRawAxis(2));
			
		} else if(this.opJoyStick.getRawAxis(3)>0) {
			this.leaderIntake.set(ControlMode.PercentOutput, -opJoyStick.getRawAxis(3));
		
		} else if(this.opJoyStick.getRawButton(4)) {
			this.leaderIntake.set(ControlMode.PercentOutput, .5);
			
		} else if(this.opJoyStick.getRawButton(8)) {
			this.leaderIntake.set(ControlMode.PercentOutput, 1.0);
			
		} else {
			this.leaderIntake.set(ControlMode.PercentOutput, 0);
		}
		
		if(Math.abs(this.opJoyStick.getRawAxis(1)) < .2 && !elevatorCommand.isRunning()) {
			this.leaderElevator.set(ControlMode.PercentOutput, 0);
		} else if(Math.abs(this.opJoyStick.getRawAxis(1)) > .2) {
			elevatorCommand.cancel();
			this.leaderElevator.set(ControlMode.PercentOutput, -this.opJoyStick.getRawAxis(1));
		}
		/*if(Math.abs(this.opJoyStick.getRawAxis(1))>.2) {
			this.leaderElevator.set(ControlMode.PercentOutput, -this.opJoyStick.getRawAxis(1));
		} else {
			this.leaderElevator.set(ControlMode.PercentOutput, 0);
		}*/
		
		Scheduler.getInstance().run();
	}

	boolean startForward = true;
	
	private void drive(double forward, double turn) {
		if(Math.abs(turn)<.2 && Math.abs(forward)>.2) {
			if(startForward) {
				this.leaderMiddleLeftDrive.setSelectedSensorPosition(0, 0, 10);
				this.leaderMiddleRightDrive.setSelectedSensorPosition(0, 0, 10);
				startForward = false;
			}
			
			int left = this.leaderMiddleLeftDrive.getSelectedSensorPosition(0);
			int right = -1*this.leaderMiddleRightDrive.getSelectedSensorPosition(0);
			
			int average = (left + right)/2;
			
			int diffRight = average - right;
			int diffLeft = average - left;
			double correcting = .00004;
			
			double powerLeft = forward + (correcting * (double)diffRight);
			double powerRight = forward + (correcting * (double)diffLeft);
			
			//SmartDashboard.putString("Diff L R", diffLeft + " " + diffRight);
			//SmartDashboard.putString("Power L R", powerLeft + " " + powerRight);
			
			drive.tankDrive(powerLeft, powerRight);
		} else {
			drive.arcadeDrive(forward, turn);
			startForward = true;
		}
		//SmartDashboard.putBoolean("start forward", startForward);
	}
	
	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
	
	@Override
	public void disabledPeriodic() {
		String gameData = station.getGameSpecificMessage();	
		if(gameData != null && gameData.length()==3) {
			SmartDashboard.putBoolean("There is game data:", true);
		} else {
			SmartDashboard.putBoolean("There is game data:", false);
		}
	}
	
	@Override
	public void disabledInit() {
		if(this.elevatorCommand!=null) {
			elevatorCommand.cancel();
			Scheduler.getInstance().run();
		}
	}
}
