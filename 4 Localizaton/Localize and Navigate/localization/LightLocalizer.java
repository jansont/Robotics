package ca.mcgill.ecse211.lab4localization.localization;

import ca.mcgill.ecse211.lab4localization.main.LocalizationLab;
import ca.mcgill.ecse211.lab4localization.odometer.Odometer;
import ca.mcgill.ecse211.lab4localization.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.utility.Delay;

public class LightLocalizer implements Runnable {
	//Colour sensor initialization
	public static final EV3ColorSensor sensor = LocalizationLab.colorSensor;
	static float[] sampleColor = new float[sensor.sampleSize()];
	static SampleProvider myColorSample = sensor.getMode("Red");
	//Motor instances
	public static final EV3LargeRegulatedMotor leftMotor = LocalizationLab.leftMotor;
	public static final EV3LargeRegulatedMotor rightMotor = LocalizationLab.rightMotor;
	//Odometer instances
	public static final Odometer odometer = LocalizationLab.odometer;
	//distance between Final robot measuring location to the light sensor
	public static final double centerToSensor = 13;
	private static final int THRESHOLD = 130;
	//Recording location of black lines
	double blackLineDistanceX;
	double blackLineDistanceY;
	//distance between origin and robot
	double distance;

	/**
	 * Initial entry point to the light Localizer thread. Calls the lightlocalize method. .
	 */
	public void run() {
		/*
		 * Robot is facing 0 degrees from part 1. Odometer's value is around this. 
		 */
		try {
			lightLocalize();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Uses the light sensor to identify x and y axis from black lines to get to the origin. 
	 * @throws InterruptedException 
	 * @throws OdometerExceptions 
	 */
	public void lightLocalize() throws InterruptedException, OdometerExceptions {
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		//Robot moves past the negative x axis and keeps moving for half a second
		moveUntilAxis(true, 500, 1);
		//robot moves in reverse through negative x axis, records that position from the odometer and keeps moving back for 4 second. 
		moveUntilAxis(false, 4000, 1);
		// Turn 90 degrees right to repeat the process with the negative Y axis
		UltrasonicLocalizer.turnTo((odometer.getXYT()[2] + 90) % 360);
		leftMotor.setSpeed(150);
		rightMotor.setSpeed(150);
		// Cross the negative Y axis and keep moving for half a second
		moveUntilAxis(true, 500, 0);
		// Move backwards and record location of line from odometer
		moveUntilAxis(false, 4000, 0);
		//Calculate distance between current location of robot and intersection of the X and Y axis
		distance = Math.sqrt(Math.pow((odometer.getXYT()[0]-blackLineDistanceX),2)+Math.pow((odometer.getXYT()[1]-blackLineDistanceY),2));
		//correct this distance
		distance -= centerToSensor;
		//Move by 45 degrees since robot is facing East, move to origin, rotate another 45 so that the robot is facing North
		UltrasonicLocalizer.turnTo((odometer.getXYT()[2] -45) % 360);
		leftMotor.rotate(UltrasonicLocalizer.convertDistance(LocalizationLab.WHEEL_RAD, distance),true);
		rightMotor.rotate(UltrasonicLocalizer.convertDistance(LocalizationLab.WHEEL_RAD, distance),false);
		UltrasonicLocalizer.turnTo((odometer.getXYT()[2] -45) % 360);
		
	}

	/**
	 * The moveUntilAxis method moves forward or backward until a black line or axis is detected.
	 * @param forward: true to move forward, false to move backward
	 * @param delay: delay in ms to wait until after the line is detected
	 * @param direction: 0 for crossing X axis, 1 for crossing Y axis
	 * @throws InterruptedException 
	 */
	private void moveUntilAxis(boolean forward, int delay, int direction) throws InterruptedException {
		if (forward) {
			leftMotor.forward();
			rightMotor.forward();
		} else {
			leftMotor.backward();
			rightMotor.backward();
		}

		//Move forward or backwards
		while (fetch()>THRESHOLD);
		Sound.beep();
		if (!forward && direction==1) {
		blackLineDistanceY = odometer.getXYT()[1];
		}
		else if(!forward && direction== 0) {
		blackLineDistanceX = odometer.getXYT()[0];
		}


		if (direction == 0) {
			odometer.setX(0);
		} else {
			odometer.setY(0);
		}
		//Delay when moving past a black line. This avoids detecting twice in a row
		Delay.msDelay(delay);
		leftMotor.stop(true);
		rightMotor.stop();
	}
	private int fetch() throws InterruptedException {
		Thread.sleep(75);
		int sensor_data;
		myColorSample.fetchSample(sampleColor, 0);
		sensor_data=(int) (sampleColor[0] * 1000);
		
		return sensor_data;
	}
	
	
	
}



