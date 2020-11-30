package ca.mcgill.ecse211.lab5;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;

/**
 * This class is where all threads are started.
 * 
 * 
 * 
 * @author Alexander Asfar
 *
 */
public class Lab5 {


	// Static Resources

	/**
	 * The left motor.
	 */
	public static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A")); //Left motor connected to port B

	/**
	 * The right motor.
	 */
	public static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(LocalEV3.get().getPort("B")); //Right motor connected to port D

	/**
	 * The motor that rotates the light sensor.
	 */
	public static final EV3MediumRegulatedMotor sensormotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("D"));

	/**
	 * The ultrasonic sensor.
	 */
	public static final EV3UltrasonicSensor usSensor = new EV3UltrasonicSensor(LocalEV3.get().getPort("S2")); //US sensor connected to port S1

	/**
	 * The light sensor that scans the can.
	 */
	public static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1")); //Light sensor connected to port S2

	/**
	 * The light sensor located at the left for light localization
	 */
	public static final EV3ColorSensor lightSensor2 = new EV3ColorSensor(LocalEV3.get().getPort("S3")); //Light sensor connected to port S2

	/**
	 * The light sensor located at the right for light localization
	 */
	public static final EV3ColorSensor lightSensor3 = new EV3ColorSensor(LocalEV3.get().getPort("S4")); //Light sensor connected to port S2

	private static int buttonChoice; //button to be selected

	public static boolean inSearchLoc =false;

	/**
	 * Boolean that shows if the robot found the right can on the perimeter.
	 */
	public static boolean finishedinNav = false;

	// Constants

	/**
	 * Wheel radius.
	 */
	public static final double WHEEL_RADIUS = 2.1; //Radius of wheel
	/**
	 * Track
	 */
	public static final double TRACK = 9.8; //Distance between the center of both wheels

	/**
	 * The tile's size
	 */
	public static final double TILE_SIZE=30;

	/**
	 * Forward speed
	 */
	public static final int FWD_SPEED= 150;

	/**
	 * Speed during roration
	 */
	public static final int ROT_SPEED=70;
	public static final double PI = Math.PI;

	/**
	 * Lower left corner's x coordinate
	 */
	public static final int LLx =2;

	/**
	 * Lower left corner's y coordinate
	 */
	public static final int LLy =2;

	/**
	 * Upper right corner's x coordinate
	 */
	public static final int URx =5;

	/**
	 * Upper right corner's y coordinate
	 */
	public static final int URy =5;

	/**
	 * Starting corner
	 */
	public static final int SC =0;

	/**
	 * Target can
	 */
	public static final int TR =1;

	/**
	 * Main method that starts the program and activate threads. 
	 * @param args
	 * @throws InterruptedException 
	 */
	public static void main(String[] args) throws InterruptedException {

		final TextLCD t = LocalEV3.get().getTextLCD(); //Access the screen
		Odometer odometer = new Odometer(leftMotor, rightMotor); //Create an instance of Odometer
		OdometryDisplay odometryDisplay = new OdometryDisplay(odometer, t); //Create an instance of OdometryDisplay
		UltrasonicPoller usPoller = new UltrasonicPoller(); // Create an instance of UltracsonicPoller
		Navigation navig = new Navigation(odometer,usPoller); // Create an instance of Navigation
		UltrasonicLocalizer usloc = new UltrasonicLocalizer(odometer, usPoller); // Create an instance of Ultrasonic Localizer
		LightLocalizer lightLoc = new LightLocalizer(odometer,navig); // Create an instance of LightLocalizer
		SearchLocalize sl = new SearchLocalize(navig, usPoller); // Create an instance of SearchLocalize


		do {
			// clear the display
			t.clear();

			t.drawString("   < Left | Right >", 0, 0);
			t.drawString("          |        ", 0, 1);
			t.drawString(" Lab 5    | Color Cal ", 0, 2);
			t.drawString("  ----------------   ", 0, 3);
			t.drawString("     Track Cal      ", 0, 4);

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT && buttonChoice != Button.ID_DOWN);


		if(buttonChoice == Button.ID_LEFT) { //Falling edge

			t.clear();      
			usPoller.start();
			odometer.start();
			odometryDisplay.start();
			usloc.fallingEdge();
			lightLoc.start();
			lightLoc.join();
			sl.start();


		} else if(buttonChoice == Button.ID_RIGHT) { //Rising edge

			t.clear();
			SearchLocalize.findColor();

		} else {

			t.clear();
			leftMotor.setSpeed(ROT_SPEED);
			rightMotor.setSpeed(ROT_SPEED);
			leftMotor.rotate(-convertAngle(2*PI), true);
			rightMotor.rotate(convertAngle(2*PI), false);
		}


		while (Button.waitForAnyPress() != Button.ID_ESCAPE);
		System.exit(0);
	}





	/**
	 * This method allows the conversion of a distance to the total rotation of each wheel need to
	 * cover that distance.
	 * 
	 * @param distance
	 * @return
	 */
	public static int convertDistance(double distance){
		return (int) (360*distance/(2*PI*WHEEL_RADIUS));
	}
	/**
	 * This method takes an angle as an input and returns the total rotation that each wheel 
	 * has to do to rotate the robot by the angle inputed.
	 * 
	 *  @param: angle in radians
	 */
	public static int convertAngle(double angle){
		return convertDistance(TRACK*angle/2);
	}

}
