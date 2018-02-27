/**
 * 
 */
package org.usfirst.frc.team316.robot;

/**
 * @author Wildcat
 *
 */
public class RobotConstants {

	public static final double WHEEL_DIAMETER = 6.0;
	public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
	public static final double TICS_PER_ROTATION = 4096;
	public static final double TICS_PER_INCH = TICS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
	
	public static final double ROBOT_WIDTH = 25.3125;
	public static final double ROBOT_CIRCUMFERENCE = Math.PI * ROBOT_WIDTH;
	public static final double TICS_PER_DEGREE = (ROBOT_CIRCUMFERENCE * TICS_PER_INCH)/360.0;
	
	//public static final double TOP_ELEVATOR_TICS = 27000;
	public static final double TOP_ELEVATOR_TICS = 34000;
	public static final double TOP_ELEVATOR_HEIGHT = 73;
	public static final double TICS_PER_ELEVATOR_INCH = TOP_ELEVATOR_TICS/TOP_ELEVATOR_HEIGHT;
}
