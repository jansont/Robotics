/*
 * OdometryCorrection.java
 */
package ca.mcgill.ECSE211.odometer;
import lejos.hardware.port.*;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;

import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
	private static final long CORRECTION_PERIOD = 15;
	private Odometer odometer;
	// Value at which we now we crossed a black line
	private static final double THRESHOLD = 300;
	 //350.0 //300 for small table //220 for blue table
	// Distance between the sensor and the center of the wheels
	private static final double xOffset= 4;
	private static final double yOffset = 11.5; //10.45
	double color;
	private static final double TILE_LENGTH = 30.48;
	
	
	static Port portColor = LocalEV3.get().getPort("S1");

	// Attach instances of Color and Touch sensors to specified ports.

	static SensorModes myColor = new EV3ColorSensor(portColor);

	// Get an instance of a sample provider for each sensor. Operating
	// modes are specified in the constructor. Note that the color
	// sensor is set to return the intensity of the reflected light.

	static SampleProvider myColorSample = myColor.getMode("Red");

	// Need to allocate buffers for each sensor
	static float[] sampleColor = new float[myColor.sampleSize()];
	
	/*
	
	//private static Port SensorPort = LocalEV3.get().getPort("S1");
	static EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S1);
	
	//private static SensorModes sensor = new EV3ColorSensor(portColor);

	private static SampleProvider myColorSample = sensor.getMode("Red");

	// Need to allocate buffers for each sensor
	private static float[] sampleColor = new float[sensor.sampleSize()];
*/
	/**
	 * This is the default class constructor. An existing instance of the odometer
	 * is used. This is to ensure thread safety.
	 * 
	 * @throws OdometerExceptions
	 */
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;

	}

	/**
	 * In here we verify if we crossed a black line
	 * Depending on our direction and the numbers of lines crossed
	 * We change the values of X and Y
	 * @throws OdometerExceptions
	 * @ author Group 21
	 */
	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;
		int Ylines = 0, Xlines = 0; // Counter for the number of lines crossed in each direction

		while (true) {
			correctionStart = System.currentTimeMillis();

			// TODO Trigger correction (When do I have information to correct?)
			myColorSample.fetchSample(sampleColor, 0);
			//int detectedValue = ((EV3ColorSensor) sensor).getColorID();
			color = sampleColor[0]*1000;
			// crossed a line
			if (color < THRESHOLD) {
				Sound.beep();
				Sound.beep();
				double[] position = this.odometer.getXYT();

				if (position[2] < 10 || position[2] > 350) { // Going North

					this.odometer.setY(TILE_LENGTH * Ylines);
				//	this.odometer.setTheta(Math.atan((TILE_SIZE * Ylines - offset)/(TILE_SIZE * Xlines - offset)));
					Ylines++;

				} else if (position[2] > 80 && position[2] < 100) {// Going East
					this.odometer.setX(TILE_LENGTH * Xlines);
					Xlines++;

				} else if (position[2] > 170 && position[2] < 190) {// Going South
					//Ylines=0;
					Ylines--;
					this.odometer.setY(TILE_LENGTH * Ylines + yOffset);
					

				} else if (position[2] > 260 && position[2] < 280) {// Going West
					//Xlines = 0;
					Xlines--;
					this.odometer.setX(TILE_LENGTH * Xlines +  xOffset);
					

				}
			}
			

			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here
				}
			}
		}
	}
}
