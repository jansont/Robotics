package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to start our program and initiate all the threads needed
 * 
 * @author Group 21
 *
 */
public class Lab3 {

	// Left and right motor objects construcuted
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	private static final TextLCD lcd = LocalEV3.get().getTextLCD();
	// Lab paramaters
	public static final double WHEEL_RAD = 2.15;
	public static final double TRACK = 16.4;
	public static final double Tile_Size = 30.48;
	//Class instances
	public static Navigation navigation;
	public static Odometer odometer;
	public static Thread navigator;
	
	/**
	 * Main method: entry point to code, initializes classes, starts threads
	 * @param String[] args
	 * @return null
	 */
	public static void main(String[] args) throws OdometerExceptions {

		int mapChoice;

		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD); // TODO Complete implementation
		// constructing dipsay
		Display odometryDisplay = new Display(lcd); 

		do {
			// clear display
			lcd.clear();
			//Prompt user for map choice
			lcd.drawString("       Map1		", 0, 0);
			lcd.drawString("        |        ", 0, 1);
			lcd.drawString("Map2 <--|--> Map3", 0, 2);
			lcd.drawString("        |	    ", 0, 3);
			lcd.drawString("       Map4       ", 0, 4);
			
			mapChoice = Button.waitForAnyPress();
			
		} while (mapChoice != Button.ID_LEFT && mapChoice != Button.ID_RIGHT && mapChoice != Button.ID_UP
				&& mapChoice != Button.ID_DOWN);
		//Set map choice to the one corresponding to the map selected
		int map_num = 0;
		switch (mapChoice) {
		case (Button.ID_LEFT):
			map_num = 1;
			break;
		case (Button.ID_RIGHT):
			map_num = 2;
			break;
		case (Button.ID_UP):
			map_num = 0;
			break;
		case (Button.ID_DOWN):
			map_num = 3;
			break;
		}
		// creating and starting odometer, odometer display, and navigation threads
		Thread odoThread = new Thread(odometer);
		odoThread.start();
		Thread odoDisplayThread = new Thread(odometryDisplay);
		odoDisplayThread.start();
		navigation = new Navigation(Tile_Size, leftMotor, rightMotor, map_num);
		navigator = new Thread(navigation);
		navigator.start();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}
}
