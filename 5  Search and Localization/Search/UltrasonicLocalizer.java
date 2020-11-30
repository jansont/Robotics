package ca.mcgill.ecse211.lab5;


import lejos.hardware.lcd.LCD;
//to make the code cleaner: not to type Lab5. each time we want to call the motors.
import static ca.mcgill.ecse211.lab5.Lab5.*;


/**
 * This method orient the robot in the positive y direction
 * 
 * @author Alexander Asfar
 *
 */
public class UltrasonicLocalizer extends Thread {


  private static final int mindist = 24; //This is the threshold value (distance from wall) for the US sensor to detect
  private static final int margin_error = 2; //This is the value of the interval that checks whether or not we entered the noise margin or if we detected a wall
  private Odometer odometer;
  private UltrasonicPoller us;

  
  /**
   * constructor
   * @param odo : the odometer
   * @param us : Ultrasonic Poller 
   */
  public UltrasonicLocalizer(Odometer odo, UltrasonicPoller us) {
    this.odometer = odo;
    this.us = us;
  }


  /**
   * This method rotates the robot until it finds a falling edge, then set theta to 0, then rotates again in the other
   * direction until it finds a second falling edge then record theta.
   * The robot rotates in the other direction by theta/2 + 45
   */
  public void fallingEdge() {
    double theta_1 = 0, theta_2 = 0,theta_3=0;
    leftMotor.setSpeed(77);
    rightMotor.setSpeed(77);

    /*
     * if the robot is facing the wall, rotate until it faces away
     */
    while(us.getDistance()<50) {
      leftMotor.forward();
      rightMotor.backward();
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    try {
      Thread.sleep(100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    while(true) {

      leftMotor.backward();//rotate the robot
      rightMotor.forward();//rotate the robot
      
      if(underThreshold()) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        break;
      }
    }
    leftMotor.stop(true);
    rightMotor.stop(false);
    odometer.setTheta(0); // set theta to 0 at first falling edge to make computations easier
    try {
      Thread.sleep(800);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    odometer.setTheta(0); //second time in case first time is not precise


    

    try {
      leftMotor.forward();
      rightMotor.backward();
      UltrasonicPoller.sleep(1500); // pause the sensor to not directly detect the second falling edge
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }
    /*
     * After detecting the first falling edge
     */
    while(true) {
      //Rotate right
      Lab5.leftMotor.forward();
      Lab5.rightMotor.backward();

      /*
       *  the robot can't get theta 1 because the sensor passes the margin to quickly
       *  we added it because that's what its supposed to do
       */
      if(inNoiseMargin()) {
        theta_1 = odometer.getTheta();
      }
      if(underThreshold()) {
        leftMotor.stop(true);
        rightMotor.stop(false);
        theta_2 = odometer.getTheta(); // this the angle that the robot rotated from the first falling edge to the second
        break;
      }
    }

    theta_3 = 0.5 *theta_2; // the angle the robot needs to turn to be at 45 degrees 
    //rotate right by theta_3 + 45 degrees
    leftMotor.rotate(-Lab5.convertAngle(theta_3+Math.toRadians(45)), true);
    rightMotor.rotate(Lab5.convertAngle(theta_3+Math.toRadians(45)), false);

    leftMotor.stop(true);
    leftMotor.stop(false);
    odometer.setTheta(0);// the robot will be at the real 0 degrees at the end of the method so we set theta to 0
               // this way the reading of the robot will be synchronized with the real life values
    try {
      Thread.sleep(200);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }



  }

  /**
   * This method rotates the robot until it finds a rising edge, then set theta to 0, then rotates again in the other
   * direction until it finds a second rising edge then record theta.
   * The robot rotates in the other direction by theta/2 + 135
   */

  public void risingEdge() {
    double theta_1 = 0, theta_2 = 0, theta_3 = 0;
    Lab5.leftMotor.setSpeed(77);
    Lab5.rightMotor.setSpeed(77);

    /*
     * if the robot is facing away of the wall, rotate until it faces it
     */
    while(us.getDistance() > 3) {
      Lab5.leftMotor.backward();
      Lab5.rightMotor.forward();
    }

    while(true) {

      //Rotate right
      Lab5.leftMotor.forward();
      Lab5.rightMotor.backward();

      if(aboveTheshold()) {
        Lab5.leftMotor.stop(true);
        Lab5.rightMotor.stop(false);
        break;
      }
    }

    //set odometer to zero here to make computations easier
    odometer.setTheta(0);
    try {
      Thread.sleep(800);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    odometer.setTheta(0); //second time in case first time is not precise

    try {
      Lab5.leftMotor.backward();
      Lab5.rightMotor.forward();
      UltrasonicPoller.sleep(2000); // pause the sensor to not directly detect the second rising edge
    } catch (InterruptedException e1) {
      e1.printStackTrace();
    }

    while(true) {
      //Rotate left
      Lab5.leftMotor.backward();
      Lab5.rightMotor.forward();

      /*
       *  the robot can't get theta 1 because the sensor passes the margin to quickly
       *  we added it because that's what its supposed to do
       */
      if(inNoiseMargin()) {
        theta_1 = odometer.getTheta();
      }
      if(aboveTheshold()) {
        Lab5.leftMotor.stop(true);
        Lab5.rightMotor.stop(false);
        theta_2 = odometer.getTheta(); // this is the angle the robot rotated from the first rising edge to the second
        break;
      }
    }

    theta_3 = 0.5 *theta_2; 
    //rotate right by theta_3 + 135 degrees
    Lab5.leftMotor.rotate(Lab5.convertAngle(theta_3+Math.toRadians(170)), true);
    Lab5.rightMotor.rotate(-Lab5.convertAngle(theta_3+Math.toRadians(170)), false);


    Lab5.leftMotor.stop(true);
    Lab5.leftMotor.stop(false);
    odometer.setTheta(0);

    try {
      Thread.sleep(20);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

  }
  
  /**
   * @return if the robot is under the distance threshold or not
   */
  private boolean underThreshold() {
    return (us.getDistance() < mindist - margin_error) ;
  }
  
  /**
   * 
   * @return return if the robot is above threshold or not
   */
  private boolean aboveTheshold() {
    return (us.getDistance() > mindist + margin_error);
  }

  /**
   * 
   * @return return if the robot is in the noise margin or not
   */
  private boolean inNoiseMargin() {
    return (us.getDistance() >= mindist - margin_error && us.getDistance() <= mindist + margin_error);
  }

}
