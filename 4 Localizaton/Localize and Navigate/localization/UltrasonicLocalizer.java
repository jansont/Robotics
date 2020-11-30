package ca.mcgill.ecse211.lab4localization.localization;

import ca.mcgill.ecse211.lab4localization.main.LocalizationLab;
import ca.mcgill.ecse211.lab4localization.odometer.Odometer;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class UltrasonicLocalizer implements Runnable {
	//True for rising edge mode
	public boolean risingEdge;
	//Ultrasonic sensor instance
	public static SensorModes usSensor;
	private SampleProvider usDistance;
	private float[] usData;
	//Motor instances
	private static EV3LargeRegulatedMotor leftMotor = LocalizationLab.leftMotor;
	private static EV3LargeRegulatedMotor rightMotor = LocalizationLab.rightMotor;
	private static int rotateSpeed = 75;
	//Odometer instance
	public static Odometer odometer;
	//Detection of wall angles
	private double firstAngle, secondAngle;
	// Distance for wall detection
	private static int wallDistance = 25;
	// Distance k for the error offset [d - k; d + k]
	private static int offset = 10;

	public UltrasonicLocalizer(boolean rising) {
		this.risingEdge = rising;
		usSensor = LocalizationLab.usSensor;

		this.usDistance = usSensor.getMode("Distance");
		this.usData = new float[usDistance.sampleSize()];

		this.firstAngle = 0;
		this.secondAngle = 0;
		
		odometer = LocalizationLab.odometer;
	}

	
	/**
	 * Entry point of Ultrasonic localizer thread.
	 * Calls either falling edge or rising edge.
	 */
	public void run() {
		if (this.risingEdge) {
			risingEdge();
		} else {
			fallingEdge();
		}
	}

	/**
	 * fallingEdge() implements the falling edge
	 * @return null;
	 */
	public void fallingEdge() {
		// 1. Rotate until wall is no longer detected
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		rotateWheels(true);
		while (getDistance() < wallDistance + offset);
		Delay.msDelay(1500); //To avoid double detection
		//2. Rotate until falling edge is detected, and keep rotating
		while (getDistance() > wallDistance + offset);
		double enterFallingEdge = odometer.getXYT()[2];
		Sound.beep();
		//3. keep rotating into falling edge until limit. 
		while (getDistance() > wallDistance - offset);
		double leaveFallingEdge = odometer.getXYT()[2];
		Sound.beep();
		leftMotor.stop(true);
		rightMotor.stop();
		//First angle is the average of both detections
		this.firstAngle = (enterFallingEdge + leaveFallingEdge) / 2;
		//4. Switch directions, rotate until falling edge is detected
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		rotateWheels(false);
		while (getDistance() < wallDistance + offset);
		Delay.msDelay(3000);
		// 5. rotate until falling edge is detected
		while (getDistance() > wallDistance + offset);
		enterFallingEdge = odometer.getXYT()[2];
		Sound.beep();
		// 6. rotate into falling ege until limit.
		while (getDistance() > wallDistance - offset);
		leaveFallingEdge = odometer.getXYT()[2];
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();
		this.secondAngle = (enterFallingEdge + leaveFallingEdge) / 2; 
		turnToZero(); //Turn towards North

	}

	/**
	 * risingEdge() implements the rising edge.
	 * @return null
	 */
	public void risingEdge() {

		// 1. Rotate until wall is no longer detected
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		rotateWheels(true);

		while (getDistance() > wallDistance + offset);
		//2. Rotate until rising edge is detected, and keep rotating
		while (getDistance() > wallDistance - offset);
		double enterRisingEdge = odometer.getXYT()[2];
		Sound.beep();
		//3. change direction once minimum wall distance not detected
		rotateWheels(false);
		while (getDistance() < wallDistance + offset);
		double leaveRisingEdge = odometer.getXYT()[2];
		Sound.beep();
		leftMotor.stop(true);
		rightMotor.stop();
		this.firstAngle = (enterRisingEdge + leaveRisingEdge) / 2; 
		// 4. Rotate until rising edge is detected
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		rotateWheels(false);
		while (getDistance() > wallDistance + offset);
		// Step five: keep rotating until it's too close to the wall
		while (getDistance() > wallDistance - offset);
		enterRisingEdge = odometer.getXYT()[2];
		Sound.beep();
		// Step six: change direction of rotation until wall is far
		rotateWheels(true);
		while (getDistance() < wallDistance + offset);
		leaveRisingEdge = odometer.getXYT()[2];
		leftMotor.stop(true);
		rightMotor.stop();
		this.secondAngle = (enterRisingEdge + leaveRisingEdge) / 2;
		Sound.beep();

		turnToZero(); //Go to north
	}
	/**
	 * This method turns the robot to a specific heading
	 * @param angle
	 * @return	null
	 */
	public static void turnTo(double theta) {
		double robotT;
		double dTheta;
		robotT = odometer.getXYT()[2];
		//Computing change in heading
		dTheta = theta - robotT;
		//Bounding change in theta between 0 and 360
		dTheta = (dTheta + 360) % 360;
		// Computing minimal angle
		if (Math.abs(dTheta - 360) < dTheta) {
			dTheta -= 360;
		}
		// make the robot turn by changeTheta
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.rotate(convertAngle(LocalizationLab.WHEEL_RAD, LocalizationLab.TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(LocalizationLab.WHEEL_RAD, LocalizationLab.TRACK, dTheta), false);

	}
	/**
	 * This method calculate the angle that must be passed to the motor 
	 * using the radius of the wheel and the distance we want the robot to cross
	 * @param radius
	 * @param distance
	 * @return	corresponding distance
	 */
	public static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}
	/**
	 * This method calculate the angle that the motor must turn 
	 * in order for the robot to turn by an certain angle
	 * @param radius
	 * @param width
	 * @param angle
	 * @return Angle by which the wheels must turn
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
	
	
	/**
	 * This method rotates the robot to North using the two angles calculated
	 * @param radius
	 * @param distance
	 * @return	corresponding distance
	 */
	public void turnToZero() {
		double deltaTheta = getDeltaTheta(this.firstAngle, this.secondAngle);

		// turn by deltaTheta degrees, brought back to 360 degrees
		// the -2 value is a minor correction that we found necessary
		// through experimentation.
		turnTo((odometer.getXYT()[2] + deltaTheta - 2) % 360);

		odometer.setTheta(0);
	}

	/**
	 * Calculates the deltaTheta value that is the angle that the robot needs to
	 * turn by to get to 0.
	 * @param angle1 : angle at which the back wall is detected
	 * @param angle2 : angle at which the left wall is detected
	 * @return : value of deltaTheta
	 */
	private static double getDeltaTheta(double angle1, double angle2) {
		double deltaTheta = 0;

		// add the angle difference from the current heading to the calculated
		// secondAngle
		deltaTheta += (angle2 - odometer.getXYT()[2]);

		// angleDifference is half the change in heading to get from the second angle
		// heading to the first angle heading, i.e. to reach the 45 degree heading
		double angleDifference = (360 - angle2 + angle1) / 2;
		// add that value to deltaTheta
		deltaTheta += angleDifference;

		// remove 45 degrees because we are going to the origin.
		deltaTheta -= 45;

		return deltaTheta;
	}

	/**
	 * getDistance() returns the distance value as measured by the ultrasonic
	 * sensor.
	 */
	private int getDistance() {
		this.usDistance.fetchSample(usData, 0); // acquire data
		int distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
		return distance;
	}

	/**
	 * Rotates the wheels clockwise if leftIsForward is true, anti-clockwise
	 * otherwise.
	 * 
	 * @param leftIsForward
	 */
	private void rotateWheels(boolean leftIsForward) {
		if (leftIsForward) {
			leftMotor.forward();
			rightMotor.backward();
		} else {
			leftMotor.backward();
			rightMotor.forward();
		}
	}
}
