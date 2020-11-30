package ca.mcgill.ecse211.lab5;
/**
 * @author Alexander Asfar and Dhriti Rudresh
 */

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.robotics.SampleProvider;

//to make the code cleaner: not to type Lab5. each time we want to call the motors.
import static ca.mcgill.ecse211.lab5.Lab5.*;

/**
 * This class is used to correct the odometer values.
 * 
 * @author Alexander Asfar
 *
 */

public class LightLocalizer extends Thread {

  private SampleProvider slsensor = lightSensor2.getMode("Red");
  private SampleProvider slsensor2 = lightSensor3.getMode("Red");

  private float[] colordata = new float[slsensor.sampleSize()]; //readings of the sensor
  private float[] colordata2 = new float[slsensor2.sampleSize()]; //readings of the sensor

  private Odometer odometer;
  private Navigation navig;
  
  /**
   * color read by the left light sensor
   */
  private float leftColor;
  
  /**
   * color read by right light sensor
   */
  private float rightColor;
  
  /**
   * Boolean that shows if the left motor is stopped or not
   */
  private boolean isLeftStopped=false;
  
  /**
   * Boolean that shows if the right motor is stopped or not
   */
  private boolean isRightStopped=false;

  
  /**
   * Constructor
   * @param odometer
   * @param navig
   */
  public LightLocalizer(Odometer odometer, Navigation navig) {
    this.odometer=odometer;
    this.navig=navig;
  }
  
  /**
   * This method makes the robot identify its real position
   */

  public void run() {
	  
	leftMotor.setSpeed(40);
	rightMotor.setSpeed(40);
	
    lightSensor2.setFloodlight(true); //turn the sensor's light on
    lightSensor3.setFloodlight(true); //turn the sensor's light on
    

    slsensor.fetchSample(colordata, 0);  //fetch the color sample from the sensor 
    float lastcolorLeft = colordata[0]*1000;    //color that sensor reads (*1000 because the original value is too small)
    slsensor2.fetchSample(colordata, 0);  //fetch the color sample from the sensor 
    float lastcolorRight = colordata[0]*1000;    //color that sensor reads (*1000 because the original value is too small)

    leftMotor.forward();
    rightMotor.forward();
    while (true) {

      slsensor.fetchSample(colordata, 0); 
      leftColor = colordata[0]*1000;  
      slsensor2.fetchSample(colordata2, 0);   
      rightColor = colordata2[0]*1000;
      
      /*
       *  if black line detected by left sensor
       *  Differential filter for light sensor
       */
      if(leftColor-lastcolorLeft<-100) {
        lastcolorLeft=leftColor; //we update the last color
        leftMotor.stop();
        isLeftStopped=true;
      }
        
      /*
       *  if black line detected by right sensor
       *  Differential filter for light sensor
       */
      if (rightColor-lastcolorRight<-100) {
    	  lastcolorRight=rightColor;
    	  rightMotor.stop();
    	  isRightStopped=true;
      }
      
      // if both motor stopped
      if (isLeftStopped==true && isRightStopped==true) {
    	  odometer.setY(TILE_SIZE);
    	  odometer.setTheta(0);
    	  break;
      }
      
    }  
    
    LCD.clear();
    
    //go backwards 12cm
    leftMotor.rotate(-Lab5.convertDistance(12), true);
    rightMotor.rotate(-Lab5.convertDistance(12), false);
    isLeftStopped=false;
    isRightStopped=false;
    // rotate 90 degrees to the left
	leftMotor.rotate(convertAngle(Math.PI/2), true);
	rightMotor.rotate(-convertAngle(Math.PI/2), false);
	
	//reset the value of the last colors
	 slsensor.fetchSample(colordata, 0);  
	 lastcolorLeft = colordata[0]*1000;    
	 slsensor2.fetchSample(colordata, 0); 
	 lastcolorRight = colordata[0]*1000;    
	

		leftMotor.forward();
	    rightMotor.forward();
	while (true) {

	      slsensor.fetchSample(colordata, 0);  //fetch the color sample from the sensor 
	      leftColor = colordata[0]*1000;  
	      slsensor2.fetchSample(colordata2, 0);  //fetch the color sample from the sensor 
	      rightColor = colordata2[0]*1000;//color that sensor reads (*1000 because the original value is too small)

	      
	      // if black line detected by left sensor

	      if(leftColor-lastcolorLeft< -100) {
	        lastcolorLeft=leftColor; //we update the last color
	        leftMotor.stop();
	        isLeftStopped=true;
	      }
	      // if black line detected by right sensor
	      if (rightColor-lastcolorRight< -100) {
	    	  lastcolorRight=rightColor;
	    	  rightMotor.stop();
	    	  isRightStopped=true;
	      }
	      
	      // if both motor stopped
	      if (isLeftStopped==true && isRightStopped==true) {
	    	  odometer.setX(TILE_SIZE);
	    	  break;
	      }
	      
	    }  

    
    navig.travelTo(TILE_SIZE, TILE_SIZE); // go to (1,1)
    Lab5.leftMotor.stop(true);
    Lab5.rightMotor.stop(false);

  }
  
}
