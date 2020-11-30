package ca.mcgill.ecse211.lab5;
import lejos.hardware.lcd.LCD;

import lejos.robotics.SampleProvider;
import lejos.robotics.filter.MedianFilter;
//to make the code cleaner: not to type Lab5. each time we want to call the motors.
import static ca.mcgill.ecse211.lab5.Lab5.*;
/**
 * This class's main objective is to return the distance read from the ultrasonic sensor.
 * @author Alexander Asfar 
 *
 */
public class UltrasonicPoller extends Thread {
	private float[] usData; //float array that will store the ultrasonic's sensor data
	private int distance; //the actual distance read by sensor
	SampleProvider us;

	/**
	 * Constructor
	 * @param us
	 */
	public UltrasonicPoller() {
		us=usSensor.getMode("Distance");
		usData = new float[us.sampleSize()]; //create an array of size the number of samples you will use
	}

	/**
	 * Run method of this class that will be executed when the thread starts.
	 * It fetches sample and return the distance
	 */
	public void run() {
		//  Assuming that the mf.fetchSample
		//  methods operate in about 20mS, and that the thread sleeps for
		//  50 mS at the end of each loop, then one cycle through the loop
		//  is approximately 70 mS.  This corresponds to a sampling rate
		//  of 1/70mS or about 14 Hz.
		while (true) {
			us.fetchSample(usData, 0);// acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

			try {
				Thread.sleep(50); //sleep the thread for 50 ms
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	/**
	 * This method returns the distance from the US sensor.
	 * @return distance
	 */
	public int getDistance() {
		return distance;
	}

}