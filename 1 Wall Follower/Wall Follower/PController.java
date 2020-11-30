package ca.mcgill.ecse211.wallfollowing;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 200; //normal motor speed
  private static final int FILTER_OUT = 3; //Number of polls under which distance is ignored
  private static final int  CONSTANT=3; //Proportionality constant

  private final int bandCenter; //Center of proper course
  private final int bandWidth; //radius of proper course
  private int distance;
  private int filterControl;

//constructor
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter+10;
    this.bandWidth = bandwidth - 1 ;
    this.filterControl=0;

 //Code below is commented out so that poller and motor start simultaneously  
 /*
    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
    */
  }

  @Override
  public void processUSData(int distance) {

 	 int offset=0; //Difference between bandwidth and distance
	  int delta=0; //Speed increase promotional to distance 

    // rudimentary filter - toss out invalid samples corresponding to null signal.
	  
    if (distance >= 75 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
      offset = bandWidth; //Remain on a proper course, ignore polling value

    } else if (distance >= 75 ) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;

    } else {
      // distance went below 255: reset filter and leave distance alone.
      filterControl = 0;
      this.distance = distance;
    }

    offset = bandCenter - distance;
    
//If the robot is on proper course
  if (Math.abs(offset) <= bandWidth) { 
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
//If the robot is too far from the wall, get closer by turning inward. 
    else if(offset < 0) { 
    	delta= deltaSpeedCalc(offset);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED+delta); 
    	WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-delta); 
    	WallFollowingLab.leftMotor.forward();
    	WallFollowingLab.rightMotor.forward();
    }
//If the robot is extremely close to the wall, reverse. 
//Speed multipliers determined through trial and error
    else if(distance<10) { //extreme value
    	delta= deltaSpeedCalc(offset);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED*2+75);  //MOTOR_SPEED*2+75
 	    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED-MOTOR_SPEED/2);  
 	    WallFollowingLab.leftMotor.backward();
 	    WallFollowingLab.rightMotor.forward();
    }
//If the robot is too close to the wall, but not yet in an extreme position, turn away  from the wall.
//Multipliers determined by trial and error 
    else {
    	delta= deltaSpeedCalc(offset);
    	WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED-delta/2); //MOTOR_SPEED-delta/2
 	    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED+delta*5/4); //MOTOR_SPEED+delta*5/4
 	    WallFollowingLab.leftMotor.forward();
 	    WallFollowingLab.rightMotor.forward();
    } 
    
  }

//Calculation of the delta constant. Offset multiplied by a constant. 
//Delta is capped at 100, so 100<motorSpeed<300. Overly large distance values are bounded. 
  public int deltaSpeedCalc(int offset) {
	  int delta=0;
	  delta= (Math.abs(offset)*CONSTANT);
	  if(delta>=100) {
		  delta=100;
	  }
	  return delta;
  }

  @Override
  public int readUSDistance() {
    return this.distance;
  }

}
