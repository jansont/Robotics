package ca.mcgill.ecse211.lab5;

import java.text.DecimalFormat;

import lejos.hardware.lcd.TextLCD;

/**
 * This class is used to display information on lcd
 * @author Alexander Asfar
 *
 */
public class OdometryDisplay extends Thread {

	//Constants
	private static final long DISPLAY_PERIOD = 250; //display period

	//Fields
	private Odometer odometer;
	private TextLCD lcd;

	/**
	 * Default constructor
	 * @param odometer
	 * @param t
	 */
	public OdometryDisplay(Odometer odometer, TextLCD t) {
		this.odometer = odometer;
		this.lcd = t;
	}

	/**
	 * Method to be executed once the Thread starts
	 */
	public void run() {

		long updateStart, updateEnd;
		double[] position = new double[3]; //array for position

		//Clear the screen before displaying
		lcd.clear();

		while (true) {
			updateStart = System.currentTimeMillis();

			// Display X, Y position and Theta orientation
			DecimalFormat numberFormat = new DecimalFormat("######0.00");
			lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
			lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
			lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);

			// Get the Odometry information
			odometer.getPosition(position, new boolean[] { true, true, true });

			// Display Odometry information
			for (int i = 0; i < 3; i++) {
				lcd.drawString(formattedDoubleToString(position[i], 2), 3, i);
			}

			// Throttle the OdometryDisplay
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < DISPLAY_PERIOD) {
				try {
					Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {}
			}
		}
	}

	/**
	 * Format double to String so that it can be displayed
	 * Inspired by StackOverflow
	 * @param x
	 * @param places
	 * @return
	 */
	private static String formattedDoubleToString(double x, int places) {
		String result = "";
		String stack = "";
		long t;

		// put in a minus sign as needed
		if (x < 0.0)
			result += "-";

		// put in a leading 0
		if (-1.0 < x && x < 1.0)
			result += "0";
		else {
			t = (long)x;
			if (t < 0)
				t = -t;

			while (t > 0) {
				stack = Long.toString(t % 10) + stack;
				t /= 10;
			}

			result += stack;
		}

		// put the decimal, if needed
		if (places > 0) {
			result += ".";

			// put the appropriate number of decimals
			for (int i = 0; i < places; i++) {
				x = Math.abs(x);
				x = x - Math.floor(x);
				x *= 10.0;
				result += Long.toString((long)x);
			}
		}

		return result;
	}

}