package ca.mcgill.ecse211.lab5;
import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

//to make the code cleaner: not to type Lab5. each time we want to call the motors.
import static ca.mcgill.ecse211.lab5.Lab5.*;

import java.util.Arrays;

/**
 * This class makes the robot search for cans in the search area and identify its color if it finds one.
 * If if it is the right color, the robot goes to the upper right corner of the search area.
 * 
 * @author Alexander Asfar
 *
 */
public class SearchLocalize extends Thread{


	private Navigation navig;
	private UltrasonicPoller uspoller;


	// means of RGB values pre-computed
	private static final double blue_µ_red=0.2751;
	private static final double blue_µ_green=0.6302;
	private static final double blue_µ_blue=0.7069;
	private static final double red_µ_red=0.9837;
	private static final double red_µ_green=0.1241;
	private static final double red_µ_blue=0.0999;
	private static final double green_µ_red=0.378;
	private static final double green_µ_green=0.7971;
	private static final double green_µ_blue=0.3643;
	private static final double asfar_µ_red=0.85;
	private static final double asfar_µ_green=0.4826;
	private static final double asfar_µ_blue=0.1422;
	
	
	/**
	 * Constructor of the class.
	 * @param navig
	 * @param uspoller
	 */
	public SearchLocalize(Navigation navig, UltrasonicPoller uspoller) {

		this.navig=navig;
		this.uspoller=uspoller;
	}

	/**
	 * The method that makes the robot search for cans, and find the color of a detected one.
	 */
	public void run() {
		inSearchLoc=true;
		// go to lower corner of the search area
		navig.travelTo(TILE_SIZE, LLy*TILE_SIZE);
		navig.travelTo(LLx*TILE_SIZE, LLy*TILE_SIZE);
		
		// for the left vertical line
		for (int i=1;i<=URy-LLy;i++) {
			leftMotor.setSpeed(FWD_SPEED);
			rightMotor.setSpeed(FWD_SPEED);
			navig.travelTo(LLx*TILE_SIZE, (LLy+i)*TILE_SIZE); // travel to first intersection
			if(finishedinNav==true) return;
			leftMotor.setSpeed(ROT_SPEED);
			rightMotor.setSpeed(ROT_SPEED);
			
			// rotate 90 degrees
			leftMotor.rotate(convertAngle(Math.PI/2), true);
			rightMotor.rotate(-convertAngle(Math.PI/2), false);
			
			//if there is a can at most two tiles away
			if (uspoller.getDistance()<65) {
				LCD.drawString("Object Detected", 0, 0);
				leftMotor.setSpeed(FWD_SPEED);
				rightMotor.setSpeed(FWD_SPEED);
				
				// go forward until you are 3cm away
				while(uspoller.getDistance()>3) {
					leftMotor.forward();
					rightMotor.forward();
				}
				leftMotor.stop(true);
				rightMotor.stop(false);
				int color = findColor(); //find color of the can
				
				// go backwards 10 cm
				leftMotor.rotate(-convertDistance(10), true);
				rightMotor.rotate(-convertDistance(10), false);

				
				// if it is the righ can, go to upper right corner of the search area and end program
				if (color == TR) {
					Sound.beep();
					navig.travelTo(LLx*TILE_SIZE, (LLy+i)*TILE_SIZE);
					navig.travelTo(LLx*TILE_SIZE, URy*TILE_SIZE);
					navig.travelTo(URx*TILE_SIZE, URy*TILE_SIZE);
					return;
					
				}
				else {
					// go back to the line
					Sound.twoBeeps();
					LCD.clear();
					navig.travelTo(LLx*TILE_SIZE, (LLy+i)*TILE_SIZE);
				}
			}
		}
		
		// for the upper horizontal line
		for (int i=2;i<=URx-LLx;i++) {
			leftMotor.setSpeed(FWD_SPEED);
			rightMotor.setSpeed(FWD_SPEED);
			navig.travelTo((LLx+i)*TILE_SIZE, URy*TILE_SIZE);
			if(finishedinNav==true) return;
			leftMotor.setSpeed(ROT_SPEED);
			rightMotor.setSpeed(ROT_SPEED);
			leftMotor.rotate(convertAngle(Math.PI/2), true);
			rightMotor.rotate(-convertAngle(Math.PI/2), false);
			if (uspoller.getDistance()<65) {
				LCD.drawString("Object Detected", 0, 0);

				leftMotor.setSpeed(FWD_SPEED);
				rightMotor.setSpeed(FWD_SPEED);
				while(uspoller.getDistance()>3) {
					leftMotor.forward();
					rightMotor.forward();
				}
				leftMotor.stop(true);
				rightMotor.stop(false);
				int color = findColor();
				leftMotor.rotate(-convertDistance(10), true);
				rightMotor.rotate(-convertDistance(10), false);
				if (color == TR) {
					Sound.beep();
					navig.travelTo((LLx+i)*TILE_SIZE, URy*TILE_SIZE);
					navig.travelTo(URx*TILE_SIZE, URy*TILE_SIZE);
					return;
				}
				else {
					Sound.twoBeeps();
					LCD.clear();
					navig.travelTo((LLx+i)*TILE_SIZE, URy*TILE_SIZE);
				}
			}
		}
		
		// for right vertical line
		for (int i=2;i<=URy-LLy;i++) {
			leftMotor.setSpeed(FWD_SPEED);
			rightMotor.setSpeed(FWD_SPEED);
			navig.travelTo(URx*TILE_SIZE, (URy-i)*TILE_SIZE);
			if(finishedinNav==true) return;
			leftMotor.setSpeed(ROT_SPEED);
			rightMotor.setSpeed(ROT_SPEED);
			leftMotor.rotate(convertAngle(Math.PI/2), true);
			rightMotor.rotate(-convertAngle(Math.PI/2), false);
			if (uspoller.getDistance()<65) {
				LCD.drawString("Object Detected", 0, 0);

				leftMotor.setSpeed(FWD_SPEED);
				rightMotor.setSpeed(FWD_SPEED);
				while(uspoller.getDistance()>3) {

					leftMotor.forward();
					rightMotor.forward();
				}
				leftMotor.stop(true);
				rightMotor.stop(false);
				int color = findColor();
				leftMotor.rotate(-convertDistance(10), true);
				rightMotor.rotate(-convertDistance(10), false);
				if (color == TR) {
					Sound.beep();
					navig.travelTo(URx*TILE_SIZE, (URy-i)*TILE_SIZE);
					navig.travelTo(URx*TILE_SIZE, URy*TILE_SIZE);
					return;
				}
				else {
					Sound.twoBeeps();
					LCD.clear();
					navig.travelTo(URx*TILE_SIZE, (URy-i)*TILE_SIZE);
				}
			}
		}
		
		// for the lower horizontal line
		for (int i=2;i<=URx-LLx;i++) {
			leftMotor.setSpeed(FWD_SPEED);
			rightMotor.setSpeed(FWD_SPEED);
			navig.travelTo((URx-i)*TILE_SIZE, LLy*TILE_SIZE);
			if(finishedinNav==true) return;
			leftMotor.setSpeed(ROT_SPEED);
			rightMotor.setSpeed(ROT_SPEED);
			leftMotor.rotate(convertAngle(Math.PI/2), true);
			rightMotor.rotate(-convertAngle(Math.PI/2), false);
			if (uspoller.getDistance()<65) {
				LCD.drawString("Object Detected", 0, 0);

				leftMotor.setSpeed(FWD_SPEED);
				rightMotor.setSpeed(FWD_SPEED);
				while(uspoller.getDistance()>3) {
					leftMotor.forward();
					rightMotor.forward();
				}
				leftMotor.stop(true);
				rightMotor.stop(false);
				int color = findColor();
				leftMotor.rotate(-convertDistance(10), true);
				rightMotor.rotate(-convertDistance(10), false);
				if (color == TR) {
					Sound.beep();
					navig.travelTo((URx-i)*TILE_SIZE, LLy*TILE_SIZE);
					navig.travelTo(URx*TILE_SIZE, LLy*TILE_SIZE);
					navig.travelTo(URx*TILE_SIZE, URy*TILE_SIZE);
					return;
				}
				else {
					Sound.twoBeeps();
					LCD.clear();
					navig.travelTo(URx*TILE_SIZE, (URy-i)*TILE_SIZE);
				}
			}
		}

	}
	
	/**
	 * This method finds the color of a can.
	 * 
	 * @return the number associated to the number of the can
	 */
	public static int findColor() {
		int tr;
		SampleProvider ls = lightSensor.getRGBMode();
		float[] lsData=new float[ls.sampleSize()];
		sensormotor.setSpeed(50);
		//arrays to store rgb values
		float[] red=new float[80];
		float[] green =new float[80];
		float[] blue =new float[80];
		
		// rotate the sensor around the can and take samples for each 20 degrees rotated.
		for (int angle=0,j=0; angle<=120 && j<80; angle+=20) {
			sensormotor.rotateTo(angle);
			for (int i=0; i<10; i++) {
				ls.fetchSample(lsData, 0);

				red[j] = Normalize(lsData[0], lsData[1], lsData[2]);
				green[j] = Normalize(lsData[1], lsData[0], lsData[2]);
				blue[j] = Normalize(lsData[2], lsData[1], lsData[0]);
				j++;
			}

		}
		sensormotor.rotateTo(0);
		sensormotor.stop();
		
		// sort the arrays
		Arrays.sort(red);
		Arrays.sort(green);
		Arrays.sort(blue);

		//take the median of each array
		float med_red= red[red.length/2];
		float med_green = green[green.length/2];
		float med_blue = blue[blue.length/2];

		// euclidean distances 
		double d_blue = calc_ecl_dist(med_red, blue_µ_red, med_green, blue_µ_green, med_blue, blue_µ_blue);
		double d_green = calc_ecl_dist(med_red, green_µ_red, med_green, green_µ_green, med_blue, green_µ_blue);
		double d_red = calc_ecl_dist(med_red, red_µ_red, med_green, red_µ_green, med_blue, red_µ_blue);
		double d_yellow = calc_ecl_dist(med_red, asfar_µ_red, med_green, asfar_µ_green, med_blue, asfar_µ_blue);

		double min_d = findMin(d_blue, d_green, d_red, d_yellow); //minimum of the distances

		if (min_d==d_blue) {
			LCD.drawString("Blue", 0, 1);
			tr=1;
			return tr;
		}

		if(min_d==d_green) {
			LCD.drawString("Green", 0, 1);
			tr=2;
			return tr;
		}

		if(min_d==d_red) {
			LCD.drawString("Red", 0, 1);
			tr=4;
			return tr;
		}

		if(min_d==d_yellow) {
			LCD.drawString("Yellow", 0, 1);
			tr=3;
			return tr;
		}

		return 0;
	}

	/**
	 * 
	 * This method calculates the euclidean distance.
	 * 
	 * @param sr sample of red
	 * @param µr mean of red
	 * @param sg sample of green
	 * @param µg mean of green
	 * @param sb sample of blue
	 * @param µb mean of blue
	 * @return the euclidean distance
	 */
	public static double calc_ecl_dist(double sr, double µr, double sg, double µg, double sb, double µb) {
		return Math.sqrt((Math.pow(sr-µr, 2))+(Math.pow(sg-µg, 2))+(Math.pow(sb-µb, 2)));

	}

	/**
	 * This method finds the minimum of 4 values.
	 * 
	 * @param d1
	 * @param d2
	 * @param d3
	 * @param d4
	 * @return the minimum
	 */
	public static double findMin(double d1, double d2, double d3, double d4) {
		double[] ds= {d1, d2, d3, d4};
		double min = ds[0];
		for (int i=0; i<ds.length;i++) {
			if (min > ds[i]) {
				min = ds[i];
			}
		}
		return min;
	}
	
	/**
	 * This method normalizes one of the rgb values (the first argument is the one being normalized).
	 * 
	 * @param red
	 * @param green
	 * @param blue
	 * @return the normalized value
	 */
	public static float Normalize(float red, float green, float blue) {
		return (float) (red/(Math.sqrt(Math.pow(red, 2)+Math.pow(green, 2)+Math.pow(blue, 2))));
	}

}
