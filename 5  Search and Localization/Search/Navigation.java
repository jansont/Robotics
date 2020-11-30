package ca.mcgill.ecse211.lab5;


import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

// to make the code cleaner: not to type Lab5. each time we want to call the motors.
import static ca.mcgill.ecse211.lab5.Lab5.*;

/**
 * This class is used to make the robot travel to a specific point.
 * 
 * @author Alexander Asfar
 */
public class Navigation {

	private Odometer odometer;
	private UltrasonicPoller us;


	private static boolean navigating = false;
	
	/**
	 * Distance from sensor to next waypoint
	 */
	private static int SENSOR_TO_NEXTWAYPOINT=27;


	public Navigation(Odometer odometer, UltrasonicPoller us) {
		this.odometer=odometer;
		this.us=us;
	}

	/**
	 * Travel to the point with coordinates x and y.
	 * If there is a can on the way, it stops in front of it, scans it and take action depending on the scan.
	 * 
	 * @param: waypoint coordinates
	 * travels to waypoint
	 */
	public void travelTo(double x, double y) {

		//reset motors
		for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
			motor.stop();
			motor.setAcceleration(3000);
		}

		navigating = true;

		//compute the turn angle
		double dX = x - odometer.getX(); //remaining x distance
		double dY = y - odometer.getY(); //remaining y distance
		double turn_angle = Math.atan2(dX, dY);

		//rotate
		leftMotor.setSpeed(ROT_SPEED);
		rightMotor.setSpeed(ROT_SPEED);
		turnTo(turn_angle);
		double distance = Math.hypot(dX, dY);

		// if SearchLocalize has started
		if(inSearchLoc==true) {
			if (us.getDistance()<SENSOR_TO_NEXTWAYPOINT) { // if there's a can on the way
				leftMotor.setSpeed(40);
				rightMotor.setSpeed(40);
				while(us.getDistance()>3) { // until the robot is 3cm away
					leftMotor.forward();
					rightMotor.forward();
				}
				leftMotor.stop(true);
				rightMotor.stop(false);
				int color = SearchLocalize.findColor(); // scan can
				if (TR==color) { // if right can
					Sound.beep();
					leftMotor.rotate(-convertDistance(15), true);
					rightMotor.rotate(-convertDistance(15), false);
					avoid_obastacle();
					travelTo(URx*TILE_SIZE, URy*TILE_SIZE);
					finishedinNav=true;
					return;
				}
				else {
					Sound.twoBeeps();
					leftMotor.rotate(-convertDistance(15), true);
					rightMotor.rotate(-convertDistance(15), false);
					avoid_obastacle();
					travelTo(x, y); // go where it was supposed to go
				}
			}
		}
		//move to waypoint
		leftMotor.setSpeed(FWD_SPEED);
		rightMotor.setSpeed(FWD_SPEED);
		leftMotor.rotate(convertDistance(distance),true);
		rightMotor.rotate(convertDistance(distance),false);
	}

	/** 
	 * Takes the the new heading as input and make the robot turn to it
	 * 
	 * @param: double theta that represents an angle in radians
	 */
	public void turnTo(double theta) {

		double angle = getMinAngle(theta-odometer.getTheta());

		leftMotor.rotate(convertAngle(angle),true);
		rightMotor.rotate(-convertAngle(angle),false);
	}

	/**
	 * Gets the smallest value (between 180 and -180) of an angle
	 * Example: input: 372 --> output: 12
	 * @param angle
	 * @return
	 */
	public double getMinAngle(double angle){
		if (angle > PI) {  //Pi = 180 degrees
			angle -= 2*PI; 
		} else if (angle < -PI) {
			angle = angle + 2*PI;
		}
		return angle;
	}

	/** 
	 * @return: the status of the robot (navigating or not)
	 */
	public boolean isNavigating() {
		return navigating;
	}

	
	/**
	 * Avoid the can.
	 */
	public static void avoid_obastacle() {
		rotate_90_left();
		leftMotor.rotate(convertDistance(15), true);
		rightMotor.rotate(convertDistance(15), false);
		rotate_90_right();
		leftMotor.rotate(convertDistance(40), true);
		rightMotor.rotate(convertDistance(40), false);
		rotate_90_right();
		leftMotor.rotate(convertDistance(15), true);
		rightMotor.rotate(convertDistance(15), false);


	}

	/**
	 * Rotate 90 degrees to the left.
	 */
	public static void rotate_90_left() {
		leftMotor.setSpeed(ROT_SPEED);
		rightMotor.setSpeed(ROT_SPEED);
		leftMotor.rotate(-convertAngle(PI/2), true);
		rightMotor.rotate(convertAngle(PI/2), false);
	}

	/**
	 * Rotate 90 degrees to the right.
	 */
	public static void rotate_90_right() {
		leftMotor.setSpeed(ROT_SPEED);
		rightMotor.setSpeed(ROT_SPEED);
		leftMotor.rotate(convertAngle(PI/2), true);
		rightMotor.rotate(-convertAngle(PI/2), false);
	}

}
