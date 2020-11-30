package ca.mcgill.ecse211.navigation;

import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
/**
 * Navigation class. Selects map of waypoint that the robot will follow. 
 * Travels to each of these waypoints by turning towards them and moving forwards. 
 * Avoids obstacles along the way. 
 * @author Group 21
 *
 */
public class Navigation implements Runnable {
	
	public static EV3LargeRegulatedMotor rightMotor;
	public static EV3LargeRegulatedMotor leftMotor;
	// Initializing the sensor, and sensor port and sensor's motor. 
	public static final EV3MediumRegulatedMotor sensorMotor = new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static SensorModes usSensor = new EV3UltrasonicSensor(usPort); // usSensor is the instance
	private static SampleProvider usDistance = usSensor.getMode("Distance"); // usDistance provides samples from
	// Instance of sensor data
	private static float[] usData = new float[usDistance.sampleSize()];
	// instance of odometer
	private static Odometer odometer;
	private static double robotX;
	private static double robotY;
	private static double robotT;
	private static double dTheta;
	//True when the robot is moving
	private static boolean traveling = false;
	// Speeds for the motors
	private static final int fwdSpeedHigh = 250;
	private static final int rotateSpeed = 140;
	// initializing four map options
	private static final int[][] map1 = new int[][] { { 0, 2 }, { 1, 1 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final int[][] map2 = new int[][] { { 1, 1 }, { 0, 2 }, { 2, 2 }, { 2, 1 }, { 1, 0 } };
	private static final int[][] map3 = new int[][] { { 1, 0 }, { 2, 1 }, { 2, 2 }, { 0, 2 }, { 1, 1 } };
	private static final int[][] map4 = new int[][] { { 0, 1 }, { 1, 2 }, { 1, 0 }, { 2, 1 }, { 2, 2 } };
	private static final int[][][] map_set = {map1, map2, map3, map4};
	// Variable initialization and declaration
	private static int distance; //distance to waypoint
	private static final int crashDistance=7; //distance to start avoiding abstacle
	public double Tile_Size;
	private static int map;
	
	/**
	 * Navigation class constructor
	 * @param Tile_Size
	 * @param leftMotor1
	 * @param rightMotor1
	 * @param num: map index depending on user input
	 */
	public Navigation(double Tile_Size, EV3LargeRegulatedMotor leftMotor1, EV3LargeRegulatedMotor rightMotor1, int num) {
		this.Tile_Size = Tile_Size;
		leftMotor = leftMotor1;
		rightMotor = rightMotor1;
		rightMotor.stop();
		leftMotor.stop();
		map=num;
		System.out.println(map);

	}
	/**
	 * The run method is used to select a map and travel to each waypoint
	 * during each iteration. 
	 * @return void
	 */
	public void run() {
		//set the waypoints to map chosen by the user
		int[][] waypoints = map_set[map];
		// Travel to every point one after the other
		for (int i = 0; i < 5; i++) {

			try {
				TravelTo(Tile_Size * waypoints[i][0], Tile_Size * waypoints[i][1]);
			} catch (OdometerExceptions e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}

	}
	/**
	 * Method TravelTo makes the robot travel to a waypoint. 
	 * @param wayPointX
	 * @param wayPointY
	 * @throws OdometerExceptions
	 * @throws InterruptedException
	 * @return void
	 */
	public static void TravelTo(double wayPointX, double wayPointY) throws OdometerExceptions, InterruptedException {
		// Get current robot position from odometer
		odometer = Lab3.odometer;
		wayPointX = odometer.getXYT()[0];
		wayPointY = odometer.getXYT()[1];
		//Travel to a point within error of 2
		while (calculateDistance(wayPointX, wayPointY, robotX, robotY)>2) {
			traveling = true;
			//updating robot's x and y position
			robotX = odometer.getXYT()[0];
			robotY = odometer.getXYT()[1];
			//calling on turn to turn the robot in the direction it should be facing
			turnTo(getTheta(wayPointX, wayPointY, robotX, robotY)*180/Math.PI);
			// move forward by the required distance
			leftMotor.setSpeed(fwdSpeedHigh);
			rightMotor.setSpeed(fwdSpeedHigh);
			//Euclindean distance to waypoint
			double dist = calculateDistance(wayPointX, wayPointY, robotX, robotY);
			leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, dist), true);
			rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, dist), true);
			//check for obstacles
			while (calculateDistance(robotX, robotY, wayPointX, wayPointY)>2) {
				// update robot's X and Y
				robotX = odometer.getXYT()[0];
				robotY = odometer.getXYT()[1];
				//Getting sensor data
				usSensor.fetchSample(usData, 0); 
				distance = (int) (usData[0] * 100.0); 
				//If distance to an obstacle is less than a crash distance, avoid it
				if (distance < crashDistance) {
					Avoid();
					break;
				}

				try {
					Thread.sleep(50);
				} catch (Exception e) {
				}
			}
			
		}
		//Arrived to wayPoint
		traveling = false;
		

	}
	/**
	 * This method makes the robot turn by theta degrees
	 * @param theta
	 */
	public static void turnTo(double theta) {
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
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, dTheta), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, dTheta), false);

	}
	/**
	 * This predicate method returns travleing status of robot
	 * @return	status 
	 */
	public static boolean isNavigating() {
		return traveling;
	}
	/**
	 * This method calculate the angle that must be passed to the motor 
	 * using the radius of the wheel and the distance we want the robot to cross
	 * @param radius
	 * @param distance
	 * @return	corresponding distance
	 */
	private static int convertDistance(double radius, double distance) {
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
	 * This method return whether or not the robot reached to desired position
	 * @param x1
	 * @param y1
	 * @param x2
	 * @param y2
	 * @return distance 
	 */
	private static double calculateDistance(double x1, double y1, double x2, double y2) {
		double distance = Math.sqrt(Math.pow((x1 - x2), 2) + Math.pow((y1 - y2), 2));
		return distance;
	}
	/**
	 * This method make the robot avoid the obstacle detected
	 * @return void
	 */
	public static void Avoid() {
		//make the robot turn 90 degrees to the right
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
		leftMotor.setSpeed(fwdSpeedHigh);
		rightMotor.setSpeed(fwdSpeedHigh);
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 26), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 26 ), false);
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
		rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
		leftMotor.setSpeed(fwdSpeedHigh);
		rightMotor.setSpeed(fwdSpeedHigh);
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 40), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 40 ), false);
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
		rightMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
		leftMotor.setSpeed(fwdSpeedHigh);
		rightMotor.setSpeed(fwdSpeedHigh);
		leftMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 25), true);
		rightMotor.rotate(convertDistance(Lab3.WHEEL_RAD, 25 ), false);
		leftMotor.setSpeed(rotateSpeed);
		rightMotor.setSpeed(rotateSpeed);
		leftMotor.rotate(convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), true);
		rightMotor.rotate(-convertAngle(Lab3.WHEEL_RAD, Lab3.TRACK, 90), false);
		leftMotor.setSpeed(fwdSpeedHigh);
		rightMotor.setSpeed(fwdSpeedHigh);
	}
	/**
	 * Calculates heading off of north to next waypoint
	 * @param wayPointX
	 * @param wayPointY
	 * @param robotX
	 * @param robotY
	 * @return theta (correct heading)
	 */
	private static double getTheta(double wayPointX, double wayPointY, double robotX, double robotY) {
		double theta = 0;
		if (wayPointY == robotY) {
			if (wayPointY > robotX) {
				theta = -Math.PI/2;
			} else if (robotX < robotX) {
				theta = Math.PI/2;
			}
		else if (wayPointX == robotX) {
			if (wayPointY > robotY) {
				theta = Math.PI;
			} else if (wayPointY < robotY) {
				theta = 0;
			}
		} else {
			theta = Math.atan((robotX - wayPointX) / (robotY - wayPointY));
			if (robotY > wayPointY) {
				theta += Math.PI;
			}
		}
	}
		return theta;		
	}

}
