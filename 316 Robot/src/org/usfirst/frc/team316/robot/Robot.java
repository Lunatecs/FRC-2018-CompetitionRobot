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
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Ultrasonic;
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
	private static final String RightSideScale = "Right Side Scale";
	private static final String LeftSideScale = "Left Side Scale";
	private static final String RightSideSwitch = "Switch Right Side";
	private static final String LeftSideSwitch = "Switch Left Side";
	private static final String LeftSideCrossScale = "Left Side Cross Scale";
	private static final String TenFeetForwardAuto = "Ten Forward Auto";
	private static final String ShootCubeAuto = "Shoot Cube Auto";
	private static final String TurnAuto = "Turn Auto";
	private static final String ForwardRange = "Forward Range";
	private static final String TestDropIntake = "Test Drop Intake";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	
	private DifferentialDrive drive;
	private Joystick joyStick;
	private Joystick opJoyStick;
	private DriverStation station;
	
	private NeutralMode driveNeutralMode = NeutralMode.Brake;
	
	private WPI_TalonSRX leaderMiddleRightDrive;
	private WPI_TalonSRX leaderMiddleLeftDrive;
	WPI_VictorSPX followerFrontRightDrive;
	WPI_VictorSPX followerBackRightDrive;
	WPI_VictorSPX followerFrontLeftDrive;
	WPI_VictorSPX followerBackLeftDrive;	
	private TalonSRX leaderElevator;
	
	private VictorSPX leaderIntake;
	private VictorSPX leaderClimber;
	
	private ADXRS450_Gyro gyro = new ADXRS450_Gyro();
	
	DigitalInput elevatorButtomLimitSwitch;
	DigitalInput elevatorTopLimitSwitch;
	//DigitalInput elevatorTopLimitSwitch2;
	DoubleSolenoid intakeSolenoid;
	DoubleSolenoid grabberSolenoid;
	
	Ultrasonic rearRangeFinder;
	//Ultrasonic frontRangeFinder;
	//AnalogGyro gyro2;
	
	//MaxbotixUltrasonic finder;
	
	//AnalogInput in;
	
	PigeonIMU pigeonGyro;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		station = DriverStation.getInstance();
		
		m_chooser.addDefault("Default Auto - Sit there like nobody's business", kDefaultAuto);
		m_chooser.addObject("Center Switch Auto", CenterSwitchAuto);
		m_chooser.addObject("Left Side Cross Scale Auto", LeftSideCrossScale);
		m_chooser.addObject("Right Side Scale", RightSideScale);
		m_chooser.addObject("Left Side Scale", LeftSideScale);
		m_chooser.addObject("Switch Right Side", RightSideSwitch);
		m_chooser.addObject("Switch Left Side", LeftSideSwitch);
		m_chooser.addObject("Ten Feet Foward Auto", TenFeetForwardAuto);
		m_chooser.addObject("Test Drop Inake", TestDropIntake);
		//m_chooser.addObject("Shoot Cube Auto - Illegal", ShootCubeAuto);
		m_chooser.addObject("Turn Auto - Gyro Turn Testing", TurnAuto);
		//m_chooser.addObject("Forward Range", ForwardRange);
		SmartDashboard.putData("Auto choices", m_chooser);
		
		leaderMiddleRightDrive = new WPI_TalonSRX(15);
		followerFrontRightDrive = new WPI_VictorSPX(16);
		followerBackRightDrive = new WPI_VictorSPX(14);
		
		leaderMiddleLeftDrive = new WPI_TalonSRX(2);
		followerFrontLeftDrive = new WPI_VictorSPX(3);
		followerBackLeftDrive = new WPI_VictorSPX(1);
		
		
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
		 
		 this.leaderClimber = new VictorSPX(4);
		 leaderClimber.setNeutralMode(driveNeutralMode);
		 
		 
		 elevatorButtomLimitSwitch = new DigitalInput(0);
		 elevatorTopLimitSwitch = new DigitalInput(1);
		 //elevatorTopLimitSwitch2 = new DigitalInput(2);
		 
		 System.out.println("START DISPLAYING OUTPUT");
		 
		 System.out.println(elevatorButtomLimitSwitch.getAnalogTriggerTypeForRouting());
		 System.out.println(elevatorButtomLimitSwitch.getChannel());
		 System.out.println(elevatorButtomLimitSwitch.getPortHandleForRouting());
		 System.out.println(elevatorButtomLimitSwitch.isAnalogTrigger());
		 //this.elevatorButtomLimitSwitch.enableInterrupts();
		 
		 System.out.println("END DISPLAYING OUTPUT");
		 
		 this.intakeSolenoid = new DoubleSolenoid(0,1);
		 this.grabberSolenoid = new DoubleSolenoid(2,3); 
		 
		 this.leaderElevator.setSelectedSensorPosition(0, 0, 10);
		 this.elevatorCommand = new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, 0);
		 
		 
		 try {
			//this.gyro2 = new AnalogGyro(0);
			//gyro2.calibrate();
			//gyro2.initGyro();
			
			//this.frontRangeFinder = new Ultrasonic(8,8);
		 } catch(RuntimeException e) {
			 //System.out.println("gyro");
			 e.printStackTrace();
		 }
		/*
		 try {
				this.frontRangeFinder = new Ultrasonic(6,7);
				
			} catch(RuntimeException e) {
				System.out.println("front ranch finder");
				System.out.println(e.getMessage());
				 e.printStackTrace();
			} 
			*/
		try {
			this.rearRangeFinder = new Ultrasonic(8,9);
			
			//this.rearRangeFinder.setAutomaticMode(true);
		} catch(RuntimeException e) {
			System.out.println("rear ranch finder");
			System.out.println(e.getMessage());
			 e.printStackTrace();
		}
		//boolean _use_units, double _min_voltage, double _max_voltage, double _min_distance, double _max_distance
		//finder = new MaxbotixUltrasonic(2, true, .293, 4.885, 11.811,196.85);  
		//in = new AnalogInput(2);
		//this.frontRangeFinder.setAutomaticMode(true);
		this.rearRangeFinder.setAutomaticMode(true);
		
		//CameraServer.getInstance().startAutomaticCapture();
		
		this.pigeonGyro = new PigeonIMU(new TalonSRX(13));
		
	}

	@Override
	public void robotPeriodic() {
		 //SmartDashboard.putData("Our Encoder", this.leaderMiddleRightDrive);
		 //SmartDashboard.putData("Our Gyro", gyro);
		 //SmartDashboard.putBoolean("Drive Data", drive.isAlive());
		 
		 //SmartDashboard.putString("Z Gyro", gyro.getAngle() + "");
		 
		 //SmartDashboard.putNumber("Output Voltage", leaderElevator.getMotorOutputVoltage());
		 
		 SmartDashboard.putNumber("Elevator Encoder:", leaderElevator.getSelectedSensorPosition(0));
		 
		 //SmartDashboard.putData("Our Drive", drive);
		 
		 //SmartDashboard.putBoolean("Fwd Solenoid Intake", intakeSolenoid.isFwdSolenoidBlackListed());
		 //SmartDashboard.putBoolean("Rev Solenoid Intake", intakeSolenoid.isRevSolenoidBlackListed());
		 //SmartDashboard.putString("Intake SoleNoid",intakeSolenoid.get() +"");
		 SmartDashboard.putString("Right Drive Postion", this.leaderMiddleRightDrive.getSelectedSensorPosition(0) + "");
		 SmartDashboard.putString("Left Drive Postion", this.leaderMiddleLeftDrive.getSelectedSensorPosition(0) + "");
		 SmartDashboard.putBoolean("Elevator Buttom Sensor", elevatorButtomLimitSwitch.get());
		 SmartDashboard.putBoolean("Elevator Top Sensor", elevatorTopLimitSwitch.get());
		 
		 double[] ypr = new double[3];
		 pigeonGyro.getYawPitchRoll(ypr);
		 
		 SmartDashboard.putString("Pigeon Gyro", "Y: " + ypr[0] + "  P: " + ypr[1] + " R: " + ypr[2]);
		 
		 if(this.rearRangeFinder!=null) {
			 //this.rearRangeFinder.ping();
			 SmartDashboard.putData("Rear Range Finder Data", this.rearRangeFinder);
			 SmartDashboard.putBoolean("Rear Good", true);
		 } else {
			 SmartDashboard.putBoolean("Rear Good", false);
		 }

		 
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
		leaderMiddleRightDrive.setNeutralMode(driveNeutralMode);
		followerFrontRightDrive.setNeutralMode(driveNeutralMode);
		followerBackRightDrive.setNeutralMode(driveNeutralMode);
		
		leaderMiddleLeftDrive.setNeutralMode(driveNeutralMode);
		followerFrontLeftDrive.setNeutralMode(driveNeutralMode);
		followerBackLeftDrive.setNeutralMode(driveNeutralMode);
		m_autoSelected = m_chooser.getSelected();
		
		String gameData = station.getGameSpecificMessage();		
		char allianceSwitch = gameData.charAt(0);
		char allianceScale = gameData.charAt(1);
		
		SmartDashboard.putString("Game Data: ", gameData);
		
		//CommandGroup firstCommand = new CommandGroup();
		//firstCommand.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));
		//firstCommand.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
		
		
		
		move = new CommandGroup();
		//move.addParallel(firstCommand);
		double minPower = .5;
		
		switch(m_autoSelected) {
			case CenterSwitchAuto:
				move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
				move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));
				if (allianceSwitch == 'R') {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 36.0, .75, minPower));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 45.0, .6, minPower));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 58.0, .5, .1, .0001, 0.00001, 0.0, 150));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -48.0, .5, minPower));
					//if(this.frontRangeFinder!=null) {
					//	move.addSequential(new RangeFinderMoveCommand(this.frontRangeFinder, this.drive, 10.0, .5, .3, .0334, .000334, .00334, 2, false));
					//} else {
					//	move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 40.0, .5, minPower, .00016, 0.00001, 0.0, 150));
					//}
					move.addSequential(new TimeMoveCommand(1000,drive));
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .5, 500));
				} else if (allianceSwitch == 'L') {
					//move.addParallel(new TimeActivateIntakeCommand(this.leaderIntake, .5, 500, 13000));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 16.0, .75, minPower));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -48.0, .6, minPower));
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 84.0, .5, minPower, .0001, 0.0, 0.0, 150));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 45.0, .5, minPower));
					//if(this.frontRangeFinder!=null) {
					//	move.addSequential(new RangeFinderMoveCommand(this.frontRangeFinder, this.drive, 10.0, .5, .3, false, true));
					//} else {
					//	move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 6.0, .75, minPower, .00016, 0.0, 0.0, 150));
					//}
					move.addSequential(new TimeMoveCommand(1000,drive));
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .5, 500));
				}
				break;
			case RightSideScale:
				if(allianceScale == 'R') {
					SmartDashboard.putString("Scale Switch", "Scale");
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 288.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -85.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 23.0, .5, .4, 0.05, .0005, .005, 1, true));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SCALE_UP_SET_POINT));			
					move.addSequential(new WaitCommand(300));
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 30.0, .5, .4, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .8, 500));
				} else if(allianceSwitch == 'R') {
					SmartDashboard.putString("Scale Switch", "Switch");
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 154.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -85.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));			
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 49.0, .5, .5, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .7, 500));
				} else {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 120.0, .75, minPower, .00004, 0.0000004, 0.000004, 150));
				}
				break;
			case LeftSideScale:
				if(allianceScale == 'L') {
					SmartDashboard.putString("Scale Switch", "Scale");
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 288.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 85.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 23.0, .5, .4, 0.05, .0005, .005, 1, true));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SCALE_UP_SET_POINT));			
					move.addSequential(new WaitCommand(300));
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 30.0, .5, .4, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .8, 500));
					/*
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 288.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 90.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 20.0, .5, .3, 0.05, .0005, .005, 1, true));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SCALE_UP_SET_POINT));			
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 30.0, .5, .3, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .7, 500));
					*/
				} else if(allianceSwitch == 'L') {
					SmartDashboard.putString("Scale Switch", "Switch");
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 154.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 85.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));			
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 49.0, .5, .3, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .7, 500));
				} else {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 120.0, .75, minPower, .00004, 0.0000004, 0.000004, 150));
				}
				break;
			case RightSideSwitch:
				if(allianceSwitch == 'R') {
					SmartDashboard.putString("Scale Switch", "Switch");
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 154.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -85.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));			
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 49.0, .5, .5, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .7, 500));
				} else {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 120.0, .75, minPower, .00004, 0.0000004, 0.000004, 150));
				}
				break;
			case LeftSideSwitch:
				if(allianceSwitch == 'L') {
					SmartDashboard.putString("Scale Switch", "Switch");
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 154.0, .75, minPower, .00007, 0.0, 0.0, 150));
					//move.addSequential(new GyroTurnCommand(gyro2, drive, -90, .4, .2));
					move.addSequential(new TurnCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 85.0, .6, .6));
					//move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, -10.0, .75, minPower));
					move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
					move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));			
					move.addSequential(new RangeFinderMoveCommand(this.rearRangeFinder, this.drive, 49.0, .5, .3, true));		
					move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .7, 500));
				} else {
					move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 120.0, .75, minPower, .00004, 0.0000004, 0.000004, 150));
				}
				break;
			case TenFeetForwardAuto:
				move.addSequential(new MoveCommand(this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 120.0, .75, minPower, .00004, 0.0000004, 0.000004, 150));
				break;
			case LeftSideCrossScale: //3/4 kP
				move.addSequential(new PigeonMoveCommand(this.pigeonGyro, this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 213.0, 1.0, .46,0.00004375,0.0,0.0,30));
				//move.addSequential(new BrakeCommand(this.drive, 500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, -89, .9, .4, 0.008, 0.0001, 0.0));
				move.addSequential(new PigeonMoveCommand(this.pigeonGyro, this.leaderMiddleLeftDrive, this.leaderMiddleRightDrive, this.drive, 188.0, 1.0, .46,0.000035,0.0,0.0,30));
				//move.addSequential(new BrakeCommand(this.drive, 500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, 90, .9, .4, 0.008, 0.0001, 0.0));
				break;
			case ShootCubeAuto:
				move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
				move.addSequential(new ElevatorCommand(this.leaderElevator, this.elevatorTopLimitSwitch, this.elevatorButtomLimitSwitch, ElevatorCommand.SWITCH_SET_POINT));				
				move.addSequential(new WaitCommand(3000));
				move.addSequential(new ActivateIntakeCommand(this.leaderIntake, .5, 500));
				break;
			case TurnAuto:
				double minP = .455;
				double maxP = .9;																//0.0002 //0.00009
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, 90, maxP, minP, 0.01, 0.0, 0.0)); //0.000225, 0.0));
				move.addSequential(new WaitCommand(500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, 45, maxP, minP, 0.0175, 0.0, 0.0)); // 0.0003, 0.0));
				move.addSequential(new WaitCommand(500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, 45, maxP, minP, 0.0175, 0.0, 0.0));
				move.addSequential(new WaitCommand(500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, -90, maxP, minP, 0.01, 0.0, 0.0));
				move.addSequential(new WaitCommand(500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, -90, maxP, minP, 0.01, 0.0, 0.0));
				move.addSequential(new WaitCommand(500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, -180, maxP, .46, 0.008, 0.0, 0.0));
				move.addSequential(new WaitCommand(500));
				move.addSequential(new PigeonTurnCommand(pigeonGyro, drive, 120, maxP, .46, 0.008, 0.0, 0.0));
				break;
			case ForwardRange:	
				//move.addSequential(new RangeFinderMoveCommand(this.frontRangeFinder, this.drive, 22.0, .5, .3, false));
			case TestDropIntake:
				move.addSequential(new LowerIntakeCommand(this.intakeSolenoid));
				break;
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
		//gyro2.calibrate();
		
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
		
		//yellow button
		if(this.opJoyStick.getRawButton(4)) {
			elevatorCommand.cancel();
			elevatorCommand.setSetPoint(ElevatorCommand.SCALE_UP_SET_POINT);
			elevatorCommand.start();
		} 

		//red button
		if(this.opJoyStick.getRawButton(3)) {
			this.grabberSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if(this.opJoyStick.getRawButton(2)) {
			this.grabberSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			this.grabberSolenoid.set(DoubleSolenoid.Value.kOff);
		}
		
		
		if(this.opJoyStick.getRawButton(5)) {
			this.intakeSolenoid.set(DoubleSolenoid.Value.kForward);
		} else if(this.opJoyStick.getRawButton(6)) {
			this.intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
		} else {
			this.intakeSolenoid.set(DoubleSolenoid.Value.kOff);
		}
		
		if(this.joyStick.getRawButton(2) && this.joyStick.getRawAxis(3)>0) {
			this.leaderClimber.set(ControlMode.PercentOutput, joyStick.getRawAxis(3));
		} else if (this.joyStick.getRawButton(8) && this.joyStick.getRawAxis(2)>0) {
			this.leaderClimber.set(ControlMode.PercentOutput, -joyStick.getRawAxis(2));
		} else {
			this.leaderClimber.set(ControlMode.PercentOutput, 0);
		}
	
		if(this.opJoyStick.getRawAxis(2)>0) {
			this.leaderIntake.set(ControlMode.PercentOutput, opJoyStick.getRawAxis(2));
			
		} else if(this.opJoyStick.getRawAxis(3)>0) {
			this.leaderIntake.set(ControlMode.PercentOutput, -opJoyStick.getRawAxis(3));
		
		} else if(this.opJoyStick.getRawButton(8)) {
			this.leaderIntake.set(ControlMode.PercentOutput, .25);
			
		} else {
			this.leaderIntake.set(ControlMode.PercentOutput, 0);
		}
		
		if(Math.abs(this.opJoyStick.getRawAxis(1)) < .2 && !elevatorCommand.isRunning() || (this.elevatorTopLimitSwitch.get() && this.opJoyStick.getRawAxis(1) < .2)) {
			this.leaderElevator.set(ControlMode.PercentOutput, 0);
		} else if(Math.abs(this.opJoyStick.getRawAxis(1)) > .2) {
			elevatorCommand.cancel();
			/*if(!this.elevatorTopLimitSwitch2.get() && this.opJoyStick.getRawAxis(1) < -.2) {
				 this.leaderElevator.set(ControlMode.Position, this.leaderElevator.getSelectedSensorPosition(0));
			 } else {
				 this.leaderElevator.set(ControlMode.PercentOutput, -this.opJoyStick.getRawAxis(1));
			 }*/
			if(this.leaderElevator.getSelectedSensorPosition(0) > ElevatorCommand.SCALE_UP_SET_POINT && this.opJoyStick.getRawAxis(1) < -.2) {
				this.leaderElevator.set(ControlMode.PercentOutput, .25);
			} else {
				this.leaderElevator.set(ControlMode.PercentOutput, -this.opJoyStick.getRawAxis(1));
			}
		}
		/*if(Math.abs(this.opJoyStick.getRawAxis(1))>.2) {
			this.leaderElevator.set(ControlMode.PercentOutput, -this.opJoyStick.getRawAxis(1));
		} else {
			this.leaderElevator.set(ControlMode.PercentOutput, 0);
		}*/
		
		Scheduler.getInstance().run();
	}

	private double getAngle() {
		double[] ypr = new double[3];
		 pigeonGyro.getYawPitchRoll(ypr);
		 return ypr[0];
	}
	
	boolean startForward = true;
	double bearing = 0;
	
	double kP = 0.02;
	double kI = 0.0002;
	double kD = 0;
	
	private void drive(double forward, double turn) {
		if(Math.abs(turn)<.2 && Math.abs(forward)>.2) {
			if(startForward) {
				this.leaderMiddleLeftDrive.setSelectedSensorPosition(0, 0, 10);
				this.leaderMiddleRightDrive.setSelectedSensorPosition(0, 0, 10);
				startForward = false;
				bearing = getAngle();
			}
			
			//double error = bearing - getAngle();
			
			//double correct = kP * error;
			/*
			double powerRight = forward + correct;
			double powerLeft = forward - correct;
			*/
			
			
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
			
			//SmartDashboard.putString("Drive Straight", "Bearing: " + bearing + "  Error: " + error + "  Left: " + powerLeft + "  Right: " + powerRight);
			
						
			drive.tankDrive(powerLeft, powerRight);
		} else {
			drive.arcadeDrive(forward, turn);
			startForward = true;
			SmartDashboard.putString("Turn",""+turn);
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
