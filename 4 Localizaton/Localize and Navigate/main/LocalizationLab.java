package ca.mcgill.ecse211.lab4localization.main;

import ca.mcgill.ecse211.lab4localization.localization.LightLocalizer;
import ca.mcgill.ecse211.lab4localization.localization.UltrasonicLocalizer;
import ca.mcgill.ecse211.lab4localization.odometer.Odometer;
import ca.mcgill.ecse211.lab4localization.odometer.OdometerExceptions;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class LocalizationLab {

	public static final Port colorPort = LocalEV3.get().getPort("S4");
	public static final Port usPort = LocalEV3.get().getPort("S1");
	public static final EV3ColorSensor colorSensor = new EV3ColorSensor(colorPort);
	public static EV3UltrasonicSensor usSensor;
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B"));
	public static final TextLCD lcd = LocalEV3.get().getTextLCD();
	public static final double WHEEL_RAD = 2.1;
	public static final double TRACK = 17;
	//odometer and display instances
	public static Odometer odometer;
	public static Display display;
	
	/**
	 * Main entry point of the code.
	 * Launches relevant threads (odometer, display, both localizers)
	 * in the right order and sets the overall behaviour.
	 * @param args unused
	 * @throws OdometerExceptions
	 */
	public static void main(String[] args) throws OdometerExceptions {
		
		try {
			usSensor = new EV3UltrasonicSensor(usPort);
		} catch (IllegalArgumentException e) {
			usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2"));
		}
		
		int buttonChoice;

		do {
			// clear the display
			lcd.clear();
			// Prompts the user for Rising or Falling edge
			lcd.drawString("<  Left  | Right >", 0, 0);
			lcd.drawString("         |        ", 0, 1);
			lcd.drawString(" Falling | Rising ", 0, 2);
			buttonChoice = Button.waitForAnyPress(); // Record choice (left or right press)
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT);

		boolean rising = (buttonChoice == Button.ID_LEFT ? false : true);
		//Starting odometer thread
		odometer = Odometer.getOdometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
		Thread odoThread = new Thread(odometer);
		odoThread.start();	
		//Starting display thread
		display = new Display(lcd);
		Thread dispThread = new Thread(display);
		dispThread.start();
		//Starting ultrasonic localizer thread
		UltrasonicLocalizer usLoc = new UltrasonicLocalizer(rising);
		Thread usLocThread = new Thread(usLoc);
		usLocThread.start();
		//Wait for the user to press a button between both parts of the lab
		waitForPress();
		//Starting light Localizer thread
		LightLocalizer lightLoc = new LightLocalizer();
		Thread lightLocThread = new Thread(lightLoc);
		lightLocThread.start();
	}
	
	/**
	 * Waits for the user to press any button
	 * @return null
	 */
	public static void waitForPress() {
		int buttonChoice = 0;
		do {
			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice == 0);
	}
}
