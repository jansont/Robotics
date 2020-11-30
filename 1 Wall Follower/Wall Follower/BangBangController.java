package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

  private final int bandCenter; //Center of a proper course
  private final int bandwidth;  // Radius of proper course
  private final int motorLow;   
  private final int motorHigh;
  private int distance;
  private int filterControl;
  private static final int FILTER_OUT = 3; //Number of polls under which distance is ignored
  private static final int  CONSTANT=3; 


//Constructor
  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    this.filterControl = 0;

//Code commented out so that poller and motor start at the same time
    /*
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    */
    
  }

  @Override
  public void processUSData(int distance) {
	  
	  this.distance = distance
	  int offset=0;

// rudimentary filter - toss out invalid samples corresponding to null signal.

    if (distance >= 120 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
      distance = bandwidth //remain on proper course

    } else if (distance >= 120 ) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;

    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
	  

//bang bang controller 

    int distError=distance-bandCenter; // Compute error

//if the robot is on proper course, keep motor's constant
    if (Math.abs(distError) <= bandwidth) { // Within limits, same speed
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start moving forward
    	WallFollowingLab.rightMotor.setSpeed(motorHigh);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
 // Too far from the wall, turn inwards
    else if (distError > 0) {
   	    WallFollowingLab.leftMotor.setSpeed(motorHigh); 
	    WallFollowingLab.rightMotor.setSpeed(motorLow);
	    WallFollowingLab.leftMotor.forward();
	    WallFollowingLab.rightMotor.forward();
    }
// Too close to the wall, turn outwards
//Multipliers determined through trial and error
    else if (distError < 0 && distance >= 7) { 
    	WallFollowingLab.leftMotor.setSpeed(motorLow);
    	WallFollowingLab.rightMotor.setSpeed(motorHigh*2); 
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
     }

//Extreme value, reverse robot 
   else if (distance<7) { 
    	// too close to the wall
    	// Immediately turn right
    	WallFollowingLab.leftMotor.setSpeed(motorHigh); 
 	    WallFollowingLab.rightMotor.setSpeed(motorLow);
	    WallFollowingLab.leftMotor.backward();
 	    WallFollowingLab.rightMotor.forward();
    
 
    } 
    
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }
}
