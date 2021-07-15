//Standard libraries
#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>
#include <String>

//BMP280 libraries
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

//Servo library
#include <Servo.h>

//BNO libraries
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//FRAM library
#include "Adafruit_FRAM_I2C.h"

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "Full_Flight_Program.h"

using namespace std; //Declares steandard namespace usage / standard name terminology

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //Specific value for time in between BNO data samples. Should be same as time step in PID header file
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29); //Sets i2c address for BNO

Adafruit_BMP280 bmp;// i2c recognition for BMP

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C(); // Activates FRAM chip

short servoPin1=6; //Declares the Arduino pin we want for servo1 -- Rocket gimbal y axis
short servoPos1=90; //Initial value for servo1 angle

short servoPin2=5; //Declares the Arduino pin we want for servo2 -- Rocket gimbal z axis
short servoPos2=90; //Initial value for servo2 angle

short servoPin3=10; //Declares the Arduino pin we want for servo3 -- Parachute deployment system arm
short servoPos3=90; //Initial value for servo3 angle

Servo myServo1; //Declares servo1 as a servo via header file command
Servo myServo2; //Declares servo2 as a servo via header file command
Servo myServo3; //Declares servo3 as a servo via header file command


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//BNO printEvent
void printEvent(sensors_event_t* event) { //Function to tell the BNO sensor what to print
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}



//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Ground Idle Stage Function -- System connections and funcitonality testing before flight
void groundIdle() {
  
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //backup restatement of bno sample rate

  Serial.begin(115200); //Opens serial monitor at 115200 baud
  delay(500);
  
  //BMP Test
  Serial.println(F("BMP280 Sensor Test"));
  delay(100);
  
  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude());
  Serial.println(" m");

  Serial.println();
  delay(200);


  //BNO Test
  Serial.println(F("BNO055 Sensor Test"));
  delay(100);  
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);

  bool bnoCalFlag = true; //Flag to mark whether or not the loop keeps going
  while (bnoCalFlag == true) { // BNO Calibration loop
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);
    
    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);
    
    int8_t boardTemp = bno.getTemp();
    Serial.println();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);
    
    uint8_t system, gyro, accel, mag = 0;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.println();
    Serial.print("Calibration: Sys=");
    Serial.print(system);
    Serial.print(" Gyro=");
    Serial.print(gyro);
    Serial.print(" Accel=");
    Serial.print(accel);
    Serial.print(" Mag=");
    Serial.println(mag);
    
    Serial.println("--");
    delay(BNO055_SAMPLERATE_DELAY_MS);

    if (gyro == 3) {
      bnoCalFlag = false;
      break;
    }
  }
  Serial.println(F("BNO055 Fully Calibrated"));
  delay(100);

  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51); //IF statement to print whether the Arduino has found the FRAM chip or not
    Serial.println("Found I2C FRAM");
  } 
  else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }


  //Servo Setup
  myServo1.attach(servoPin1);
  myServo2.attach(servoPin2);
  myServo3.attach(servoPin3);
  
  Serial.println("Testing Servo 1");
  delay(500);

  myServo1.write(servoPos1);
  delay(500);
  myServo1.write(135);
  delay(250);
  myServo1.write(45);
  delay(250);
  myServo1.write(90);
  delay(1000);

  
  Serial.println("Testing Servo 2");
  delay(500);
  
  myServo2.write(servoPos2);
  delay(500);
  myServo2.write(135);
  delay(250);
  myServo2.write(45);
  delay(250);
  myServo2.write(90);
  delay(1000);

  Serial.println("Systems Testing Protocol Complete");
  delay(1000);

  //Function to automatically move parachute servo to assist parachute packing in nose cone
  chutePacking();
  
  Serial.println("Powered Flight Starting in 5");
  delay(2000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  Serial.println("Initiating Powered Flight");

}



//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Function questioning whether the parachute has been packed correctly during ground idle stage
void chutePacking() {

  Serial.println("Parachute Packing Sequence");
  Serial.println("Does the parachute need to be packed? (yes/no)");
  
  bool inputFlag1 = false;
  bool inputFlag2 = false;
  bool inputFlag3 = false;
  
  while (inputFlag1 == false) {
    while (Serial.available()==0){}
    String userInput1 = Serial.readString();
    delay(100);

    if (userInput1 == "yes" or userInput1 == "Yes") {
      delay(100);
      myServo3.write(170);
      Serial.println("Parachute Deployment System Lock Disengaged");
      Serial.println("Are you ready to engage the Parachute Deployment System Lock? (type 'yes' when ready to proceed)");
      while (inputFlag2 == false) {
        while (Serial.available()==0){}
        String userInput2 = Serial.readString();
        delay(100);
        if (userInput2 == "yes" or userInput2 == "Yes") {
          delay(100);
          myServo3.write(90);
          Serial.println("Parachute Deployment System Lock Engaged");
          inputFlag2 = true;
          break;
        }
        else {
          Serial.println("Please type 'yes' when you are ready to proceed");
          inputFlag2 = false;
        }
      }
      Serial.println("Has the parachute been packed correctly inside the nose cone? (type 'yes' when ready to proceed)");
      while (inputFlag3 == false) {
        while (Serial.available()==0){}
        String userInput3 = Serial.readString();
        delay(100);
        if (userInput3 == "yes" or userInput3 == "Yes") {
          delay(100);
          myServo3.write(90);
          Serial.println("Continuing to next flight stage in 10 seconds");
          delay(10000);
          inputFlag3 = true;
          break;
        }
        else {
          Serial.println("Please type 'yes' when you are ready to proceed");
          inputFlag3 = false;
        }
      }
      inputFlag1 = true;
      break;
      break;
    }
    else if (userInput1 == "no" or userInput1 == "No") {
      Serial.println("Continuing to next Flight Stage in 10 seconds");
      delay(10000);
      inputFlag1 = true;
      break;
    }
    else {
      Serial.println("Please respond only with either 'yes' or 'no'");
      inputFlag1 = false;
    }
  }
}



//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  
//Function that waits to start powered flight stage until the rocket recognizes an upward acceleration
void poweredFlightStall() {
  
  bool flightStallFlag = true;
  short flightStallCount = 0;
  while (flightStallFlag = true) {
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);\

    float accelY = accel.y();
    
//    Serial.print(F("accelX= "));
//    Serial.print(accelX);
//    Serial.print(F(" accelY= "));
//    Serial.print(accelY);
//    Serial.print(F(" accelZ= "));
//    Serial.print(accelZ);

    if (accelY > 1){
      flightStallCount++;
    }
    else flightStallCount = 0;

    if (flightStallCount > 3){     
      break;
    }
    //Serial.print(F(" Count= "));
    //Serial.println(flightStallCount);
  }
}


//Powered Flight Stage Function -- Includes PID algorithm and recording of data
void poweredFlight() {

  //Function to stall PID from running until the rocket detects vertical acceleration while sitting on the launch pad
  //poweredFlightStall();

  flightStage = 2; //changes flight stage marker to 2
  
  Serial.begin(115200); //Setup command to begin sending information to the Serial monitor at the baudrate specified
  
  myServo1.write(servoPos1); //Sets servo1 angle to initial value
  myServo2.write(servoPos2); //Sets servo2 angle to initial value

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bno.setExtCrystalUse(true); //Sets the timing source for the BNO chip as an external source (the Arduino) instead of the internal crystal inside the chip -- this is to measure everything at the same time through the arduino


//PID gains----------------------------------
  float KP = 0.8; //PID Proportional Gain
  float KI = 0.03; //PID Integral Gain
  float KD = 0.2; //PID Derivative Gain
//-------------------------------------------


  for (short n = 0; n < 2; n++) { //Loop to set initial theta values for ease of use in simpler formulas in PID loop

    runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed
      
    thetaY[2] = thetaY[1]; //Array rearrangements for Y axis
    thetaY[1] = thetaY[0];

    gimbal_angleY[1] = gimbal_angleY[0];

    thetaZ[2] = thetaZ[1]; //Array rearrangements for Z axis
    thetaZ[1] = thetaZ[0];

    gimbal_angleZ[1] = gimbal_angleZ[0];

    //BNO READINGS AND HEX SEPARATION
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement (must be done every time BNO makes a measurement)

    thetaX = euler.x(); //Sets thetaX equal to the most recent x-axis orientation measurement
    thetaY[0] = euler.y() + thetaY_offset; //Sets first element in thetaY array equal to most recent y-axis orientation measurement
    thetaZ[0] = euler.z() + thetaZ_offset; //Sets first element in thetaZ array equal to most recent z-axis orientation measurement -- (-90 included to zero the sensor to earth's normal) 
    
    if (thetaX >= 0) { //If statement to separate the X-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntX = floor(thetaX); //Separates integer value of X-axis orientation measurement
      thetaDecimalX = 100*(thetaX-floor(thetaX)); //Separates decimal value of X-axis orientation measurement
      thetaFlagX = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntX = -1*ceil(thetaX); //Separates the ineger value of the X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalX = -100*(thetaX-ceil(thetaX)); //Separates decimal value of X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagX = 1; //Flag set to 1 to recognize as negative number
    }
    if (thetaY[0] >= 0) { //If statement to separate the Y-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntY = floor(thetaY[0]); //Separates integer value of Y-axis orientation measurement
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0])); //Separates decimal value of Y-axis orientation measurement
      thetaFlagY = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]); //Separates the ineger value of the Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0])); //Separates decimal value of Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagY = 1; //Flag set to 1 to recognize as negative number
    }      
    if (thetaZ[0] >= 0) { //If statement to separate the Z-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntZ = floor(thetaZ[0]); //Separates integer value of Z-axis orientation measurement
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement
      thetaFlagZ = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]); //Separates the ineger value of the Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagZ = 1; //Flag set to 1 to recognize as negative number
    }

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeFirst = floor(altitude/100); //Separates first 2 digits of altitude measurement
    altitudeMid = (floor(altitude) - 100*altitudeFirst); //Separates middle 2 digits of altitude measurement
    altitudeLast = 100*(altitude - floor(altitude)); //Separates decimal digits of altitude measurement

    pressureFirst = floor(pressure/100); //Separates first 2 digits of pressure measurement
    pressureMid = (floor(pressure) - 100*pressureFirst); //Separates middle 2 digits of pressure measurement
    pressureLast = 100*(pressure - floor(pressure)); //Separates decimal digits of pressure measurement
    
      
    //Writing values to FRAM chip -- 22 values stored per measurement loop
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
    
    //ORIENTATION
    fram.write8(universalIndex*22+1, thetaIntX); //Integer value X-axis
    fram.write8(universalIndex*22+2, thetaDecimalX); //Decimal value X-axis
    fram.write8(universalIndex*22+3, thetaFlagX); //Positive / Negative Flag X-axis
    fram.write8(universalIndex*22+4, thetaIntY); //Integer value Y-axis
    fram.write8(universalIndex*22+5, thetaDecimalY); //Decimal value Y-axis
    fram.write8(universalIndex*22+6, thetaFlagY); //Positive / Negative Flag Y-axis
    fram.write8(universalIndex*22+7, thetaIntZ); //Integer value Z-axis
    fram.write8(universalIndex*22+8, thetaDecimalZ); //Decimal value Z-axis
    fram.write8(universalIndex*22+9, thetaFlagZ); //Positive / Negative Flag Z-axis

    //GIMBAL ANGLES
    fram.write8(universalIndex*22+10, gimbal_angleIntY); //Integer value Y-axis
    fram.write8(universalIndex*22+11, gimbal_angleDecimalY); //Decimal value Y-axis
    fram.write8(universalIndex*22+12, gimbal_angleIntZ); //Integer value Z-axis
    fram.write8(universalIndex*22+13, gimbal_angleDecimalZ); //Decimal value Z-axis

    //SERVO ANGLES
    fram.write8(universalIndex*22+14, servoPos1); //Y-axis
    fram.write8(universalIndex*22+15, servoPos2); //Z-axis

    //ALTITUDE
    fram.write8(universalIndex*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(universalIndex*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+21, pressureLast); //Digits in the Tenths and Hundredths place
      

    // Print Statement for all relevant information while for loop is active ----- Can be commented out during actual flights to reduce lag between measurements
    Serial.print(F("n= "));
    Serial.println(universalIndex);
//    Serial.print(F(" runtime= "));
//    Serial.print(runtime[n]);
//    Serial.print(F(" ThetaX= "));
//    Serial.print(thetaX);
//    Serial.print(F(" thetaIntX= "));
//    Serial.print(thetaIntX);
//    Serial.print(F(" thetaDecimalX= "));
//    Serial.print(thetaDecimalX);
//    Serial.print(F(" thetaFlagX= "));
//    Serial.println(thetaFlagX);

//    Serial.print(F("thetaY= "));
//    Serial.print(thetaY[0]);
//    Serial.print(F(" thetaIntY= "));
//    Serial.print(thetaIntY);
//    Serial.print(F(" thetaDecimalY= "));
//    Serial.print(thetaDecimalY);
//    Serial.print(F(" thetaFlagY= "));
//    Serial.print(thetaFlagY);
//    Serial.print(F(" thetaZ= "));
//    Serial.print(thetaZ[0]);
//    Serial.print(F(" thetaIntZ= "));
//    Serial.print(thetaIntZ);
//    Serial.print(F(" thetaDecimalZ= "));
//    Serial.print(thetaDecimalZ);
//    Serial.print(F(" thetaFlagZ= "));
//    Serial.println(thetaFlagZ);
      
//    Serial.print(F("gimbal_angleY= "));
//    Serial.print(gimbal_angleY[0]);
//    Serial.print(F(" gimbal_angleIntY= "));
//    Serial.print(gimbal_angleIntY);
//    Serial.print(F(" gimbal_angleDecimalY= "));
//    Serial.print(gimbal_angleDecimalY);
//    Serial.print(F(" gimbal_angleZ= "));
//    Serial.print(gimbal_angleZ[0]);
//    Serial.print(F(" gimbal_angleIntZ= "));
//    Serial.print(gimbal_angleIntZ);
//    Serial.print(F(" gimbal_angleDecimalZ= "));
//    Serial.print(gimbal_angleDecimalZ);
//    Serial.print(F(" Servo angleY= "));
//    Serial.print(servoPos1);
//    Serial.print(F(" Servo angleZ= "));
//    Serial.println(servoPos2);

//    Serial.print(F("altitude= "));
//    Serial.println(altitude);
//    Serial.print(F(" altitudeLast= "));
//    Serial.print(altitudeLast);
//    Serial.print(F(" altitudeMid= "));
//    Serial.print(altitudeMid);
//    Serial.print(F(" altitudeFirst= "));
//    Serial.println(altitudeFirst);
//
//    Serial.print(F("pressure= "));
//    Serial.print(pressure);
//    Serial.print(F(" pressureFirst= "));
//    Serial.print(pressureFirst);
//    Serial.print(F(" pressureMid= "));
//    Serial.print(pressureMid);
//    Serial.print(F(" pressureLast= "));
//    Serial.println(pressureLast);
    //----------------------------------------------------------------------------

    universalIndex++; //Adds +1 to measurement counting index universal between all functions
    //delay(BNO055_SAMPLERATE_DELAY_MS); //Delay for selected BNO sample rate
    
  }
  

  for (int n = 2; n <= motor_time+1; n++) { //Loop for actual PID algorithm math

    runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed (just like in the other for loop but for the remainder of the runtime)

    thetaY[2] = thetaY[1]; //Array rearrangements for Y axis
    thetaY[1] = thetaY[0];

    gimbal_angleY[1] = gimbal_angleY[0];

    errorY[1] = errorY[0];

    thetaZ[2] = thetaZ[1]; //Array rearrangements for Z axis
    thetaZ[1] = thetaZ[0];

    gimbal_angleZ[1] = gimbal_angleZ[0];

    errorZ[1] = errorZ[0];
      
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x(); //Sets thetaX equal to the most recent X-axis orientation measurement
    thetaY[0] = euler.y() + thetaY_offset; //Sets first element in thetaY array equal to most recent Y-axis orientation measurement
    thetaZ[0] = euler.z() + thetaZ_offset; //Sets first element in thetaZ array equal to most recent Z-axis orientation measurement -- (-90 included to zero the sensor to earth's normal)
      
    if (thetaX >= 0) { //If statement to separate the X-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntX = floor(thetaX); //Separates integer value of X-axis orientation measurement
      thetaDecimalX = 100*(thetaX-floor(thetaX)); //Separates decimal value of X-axis orientation measurement
      thetaFlagX = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntX = -1*ceil(thetaX); //Separates the ineger value of the X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalX = -100*(thetaX-ceil(thetaX)); //Separates decimal value of X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagX = 1; //Flag set to 1 to recognize as negative number
    }
    if (thetaY[0] >= 0) { //If statement to separate the Y-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntY = floor(thetaY[0]); //Separates integer value of Y-axis orientation measurement
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0])); //Separates decimal value of Y-axis orientation measurement
      thetaFlagY = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]); //Separates the ineger value of the Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0])); //Separates decimal value of Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagY = 1; //Flag set to 1 to recognize as negative number
    }      
    if (thetaZ[0] >= 0) { //If statement to separate the Z-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntZ = floor(thetaZ[0]); //Separates integer value of Z-axis orientation measurement
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement
      thetaFlagZ = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]); //Separates the ineger value of the Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagZ = 1; //Flag set to 1 to recognize as negative number
    }

    //PID control 
    errorY[0] = theta0 - thetaY[0];
    errorZ[0] = theta0 - thetaZ[0];

    proportional_errorY = errorY[0]; //Proportional error for Y axis - difference between current measured angle and the desired value (perfectly vertical in this case)
    integral_errorY += errorY[0] * time_step; //Integral error for Y axis - equal to the running sum of rocket angles for Y axis
    derivative_errorY = (errorY[0] - errorY[1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

    proportional_errorZ = errorZ[0]; //Proportional error for Z axis - difference between current measured angle and the desired value (perfectly vertical in this case)
    integral_errorZ += errorZ[0] * time_step; //Integral error for Z axis - equal to the running sum of rocket angles for Z axis
    derivative_errorZ = (errorZ[0] - errorZ[1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

    outputY = + KP*proportional_errorY + KI*integral_errorY + KD*derivative_errorY; //Actual math for PID equation in Y axis
    outputZ = + KP*proportional_errorZ + KI*integral_errorZ + KD*derivative_errorZ; //Actual math for PID equation in Z axis

    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    if ((outputY - gimbal_angleY[1]) > maxRotationPerStep) { //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Y axis
      gimbal_angleY[0] = gimbal_angleY[1] + maxRotationPerStep;
    }
    else if ((gimbal_angleY[1] - outputY) > maxRotationPerStep) {
      gimbal_angleY[0] = gimbal_angleY[1] - maxRotationPerStep;
    }
    else {
      gimbal_angleY[0] = outputY;
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
    if (gimbal_angleY[0] > max_angle) { //If statement to keep the maximum angle value for servos within their mobility range for Y axis
      gimbal_angleY[0] = max_angle;
    }
    if (gimbal_angleY[0] < - max_angle){
      gimbal_angleY[0] = - max_angle;
    }
    //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    if ((outputZ - gimbal_angleZ[1]) > maxRotationPerStep) { //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Z axis
      gimbal_angleZ[0] = gimbal_angleZ[1] + maxRotationPerStep;
    }
    else if ((gimbal_angleZ[1] - outputZ) > maxRotationPerStep) { 
      gimbal_angleZ[0] = gimbal_angleZ[1] - maxRotationPerStep;
    }
    else {
      gimbal_angleZ[0] = outputZ;
    }
    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    if (gimbal_angleZ[0] > max_angle) { //If statement to keep the maximum angle value for servos within their mobility range for Z axis
      gimbal_angleZ[0] = max_angle;
    }
    if (gimbal_angleZ[0] < - max_angle){
      gimbal_angleZ[0] = - max_angle;
    }
    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    if (gimbal_angleY[0] >= 0) { //If statement to separate the Y-axis gimbal angle calculation into 2, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      gimbal_angleIntY = floor(gimbal_angleY[0]); //Separates integer value of Y-axis gimbal angle calculation
      gimbal_angleDecimalY = 100*(gimbal_angleY[0]-floor(gimbal_angleY[0])); //Separates decimal value of Y-axis gimbal angle calculation
    }
    else {
      gimbal_angleIntY = -1*ceil(gimbal_angleY[0]); //Separates decimal value of Y-axis gimbal angle calculation -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      gimbal_angleDecimalY = 100-100*(gimbal_angleY[0]-ceil(gimbal_angleY[0])); //Separates decimal value of Y-axis gimbal angle calculation -- Temporarily adds 100 to the stored data value to signify being a negative integer. Reverted back in the memory-reading program
    }
    if (gimbal_angleZ[0] >= 0) { //If statement to separate the Z-axis gimbal angle into 2, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      gimbal_angleIntZ = floor(gimbal_angleZ[0]); //Separates integer value of Z-axis orientation measurement
      gimbal_angleDecimalZ = 100*(gimbal_angleZ[0]-floor(gimbal_angleZ[0])); //Separates decimal value of Z-axis gimbal angle calculation
    }
    else {
      gimbal_angleIntZ = -1*ceil(gimbal_angleZ[0]); //Separates decimal value of Z-axis gimbal angle calculation -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      gimbal_angleDecimalZ = 100-100*(gimbal_angleZ[0]-ceil(gimbal_angleZ[0])); //Separates decimal value of Z-axis gimbal angle calculation -- Temporarily adds 100 to the stored data value to signify being a negative integer. Reverted back in the memory-reading program
    }
    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    servoPos1 = 90+findBottomAngleRatio(gimbal_angleY[0])*gimbal_angleY[0]; //Calculates the servo angle based on the given gimbal angle for servo1
    myServo1.write(servoPos1); //Applies the calculated angle for servo1

    servoPos2 = 90+findTopAngleRatio(gimbal_angleZ[0])*gimbal_angleZ[0]; //Calculates the servo angle based on the given gimbal angle for servo2
    myServo2.write(servoPos2); //Applies the calculated angle for servo2

    //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeLast = 100*(altitude - floor(altitude)); //Separates first 2 digits of altitude measurement
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100)); //Separates middle 2 digits of altitude measurement
    altitudeFirst = floor(floor(altitude)/100); //Separates decimal digits of altitude measurement

    pressureLast = 100*(pressure-floor(pressure)); //Separates first 2 digits of pressure measurement
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100)); //Separates middle 2 digits of pressure measurement
    pressureFirst = floor(floor(pressure)/100); //Separates decimal digits of pressure measurement

    //Writing values to FRAM chip -- 22 values stored per measurement loop
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
    
    //ORIENTATION
    fram.write8(universalIndex*22+1, thetaIntX); //Integer value X-axis
    fram.write8(universalIndex*22+2, thetaDecimalX); //Decimal value X-axis
    fram.write8(universalIndex*22+3, thetaFlagX); //Positive / Negative Flag X-axis
    fram.write8(universalIndex*22+4, thetaIntY); //Integer value Y-axis
    fram.write8(universalIndex*22+5, thetaDecimalY); //Decimal value Y-axis
    fram.write8(universalIndex*22+6, thetaFlagY); //Positive / Negative Flag Y-axis
    fram.write8(universalIndex*22+7, thetaIntZ); //Integer value Z-axis
    fram.write8(universalIndex*22+8, thetaDecimalZ); //Decimal value Z-axis
    fram.write8(universalIndex*22+9, thetaFlagZ); //Positive / Negative Flag Z-axis

    //GIMBAL ANGLES
    fram.write8(universalIndex*22+10, gimbal_angleIntY); //Integer value Y-axis
    fram.write8(universalIndex*22+11, gimbal_angleDecimalY); //Decimal value Y-axis
    fram.write8(universalIndex*22+12, gimbal_angleIntZ); //Integer value Z-axis
    fram.write8(universalIndex*22+13, gimbal_angleDecimalZ); //Decimal value Z-axis

    //SERVO ANGLES
    fram.write8(universalIndex*22+14, servoPos1); //Y-axis
    fram.write8(universalIndex*22+15, servoPos2); //Z-axis

    //ALTITUDE
    fram.write8(universalIndex*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(universalIndex*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+21, pressureLast); //Digits in the Tenths and Hundredths place
 

    // Print Statement for all relevant information while PID loop is active ----- Can be commented out during actual flights to reduce lag between measurements
        
    Serial.print(F("n= "));
    Serial.println(universalIndex);
//    Serial.print(F(" runtime= "));
//    Serial.print(runtime[n]);
//    Serial.print(F(" ThetaX= "));
//    Serial.print(thetaX);
//    Serial.print(F(" thetaIntX= "));
//    Serial.print(thetaIntX);
//    Serial.print(F(" thetaDecimalX= "));
//    Serial.print(thetaDecimalX);
//    Serial.print(F(" thetaFlagX= "));
//    Serial.println(thetaFlagX);

//    Serial.print(F("thetaY= "));
//    Serial.print(thetaY[0]);
//    Serial.print(F(" thetaIntY= "));
//    Serial.print(thetaIntY);
//    Serial.print(F(" thetaDecimalY= "));
//    Serial.print(thetaDecimalY);
//    Serial.print(F(" thetaFlagY= "));
//    Serial.print(thetaFlagY);
//    Serial.print(F(" thetaZ= "));
//    Serial.print(thetaZ[0]);
//    Serial.print(F(" thetaIntZ= "));
//    Serial.print(thetaIntZ);
//    Serial.print(F(" thetaDecimalZ= "));
//    Serial.print(thetaDecimalZ);
//    Serial.print(F(" thetaFlagZ= "));
//    Serial.println(thetaFlagZ);
      
//    Serial.print(F("gimbal_angleY= "));
//    Serial.print(gimbal_angleY[0]);
//    Serial.print(F(" gimbal_angleIntY= "));
//    Serial.print(gimbal_angleIntY);
//    Serial.print(F(" gimbal_angleDecimalY= "));
//    Serial.print(gimbal_angleDecimalY);
//    Serial.print(F(" gimbal_angleZ= "));
//    Serial.print(gimbal_angleZ[0]);
//    Serial.print(F(" gimbal_angleIntZ= "));
//    Serial.print(gimbal_angleIntZ);
//    Serial.print(F(" gimbal_angleDecimalZ= "));
//    Serial.print(gimbal_angleDecimalZ);
//    Serial.print(F(" Servo angleY= "));
//    Serial.print(servoPos1);
//    Serial.print(F(" Servo angleZ= "));
//    Serial.println(servoPos2);

//    Serial.print(F("altitude= "));
//    Serial.println(altitude);
//    Serial.print(F(" altitudeLast= "));
//    Serial.print(altitudeLast);
//    Serial.print(F(" altitudeMid= "));
//    Serial.print(altitudeMid);
//    Serial.print(F(" altitudeFirst= "));
//    Serial.println(altitudeFirst);
//
//    Serial.print(F("pressure= "));
//    Serial.print(pressure);
//    Serial.print(F(" pressureFirst= "));
//    Serial.print(pressureFirst);
//    Serial.print(F(" pressureMid= "));
//    Serial.print(pressureMid);
//    Serial.print(F(" pressureLast= "));
//    Serial.println(pressureLast);
    //---------------------------------------------------------------------------

    universalIndex++; //Adds +1 to measurement counting index universal between all functions
    //delay(BNO055_SAMPLERATE_DELAY_MS);//Delay for selected BNO sample rate  
  }
  
}



//----------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Unpowered Flight Stage Function
void unpoweredFlight() {

  flightStage = 3; //changes flight stage marker to 3
  
  altitudePrev = altitude; //Variable to keep track of and compare the current altitude measurement to the previous one
  pressurePrev = pressure; //Variable to keep track of and compare the current pressure measurement to the previous one

  bool unpoweredFlag = true; //Flag to mark whether or not the loop keeps going
  while (unpoweredFlag == true) { //While loop to record orientation, altitude, and pressure data until the rocket begins descending
    
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x(); //Sets thetaX equal to the most recent X-axis orientation measurement
    thetaY[0] = euler.y() + thetaY_offset; //Sets first element in thetaY array equal to most recent Y-axis orientation measurement
    thetaZ[0] = euler.z() + thetaZ_offset; //Sets first element in thetaZ array equal to most recent Z-axis orientation measurement -- (-90 included to zero the sensor to earth's normal)
      
    if (thetaX >= 0) { //If statement to separate the X-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntX = floor(thetaX); //Separates integer value of X-axis orientation measurement
      thetaDecimalX = 100*(thetaX-floor(thetaX)); //Separates decimal value of X-axis orientation measurement
      thetaFlagX = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntX = -1*ceil(thetaX); //Separates the ineger value of the X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalX = -100*(thetaX-ceil(thetaX)); //Separates decimal value of X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagX = 1; //Flag set to 1 to recognize as negative number
    }
    if (thetaY[0] >= 0) { //If statement to separate the Y-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntY = floor(thetaY[0]); //Separates integer value of Y-axis orientation measurement
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0])); //Separates decimal value of Y-axis orientation measurement
      thetaFlagY = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]); //Separates the ineger value of the Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0])); //Separates decimal value of Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagY = 1; //Flag set to 1 to recognize as negative number
    }      
    if (thetaZ[0] >= 0) { //If statement to separate the Z-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntZ = floor(thetaZ[0]); //Separates integer value of Z-axis orientation measurement
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement
      thetaFlagZ = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]); //Separates the ineger value of the Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagZ = 1; //Flag set to 1 to recognize as negative number
    }

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeLast = 100*(altitude - floor(altitude)); //Separates first 2 digits of altitude measurement
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100)); //Separates middle 2 digits of altitude measurement
    altitudeFirst = floor(floor(altitude)/100); //Separates decimal digits of altitude measurement

    pressureLast = 100*(pressure-floor(pressure)); //Separates first 2 digits of pressure measurement
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100)); //Separates middle 2 digits of pressure measurement
    pressureFirst = floor(floor(pressure)/100); //Separates decimal digits of pressure measurement

    //Writing values to FRAM chip for Unpowered Flight stage
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
      
    //ORIENTATION
    fram.write8(universalIndex*22+1, thetaIntX); //Integer value X-axis
    fram.write8(universalIndex*22+2, thetaDecimalX); //Decimal value X-axis
    fram.write8(universalIndex*22+3, thetaFlagX); //Positive / Negative Flag X-axis
    fram.write8(universalIndex*22+4, thetaIntY); //Integer value Y-axis
    fram.write8(universalIndex*22+5, thetaDecimalY); //Decimal value Y-axis
    fram.write8(universalIndex*22+6, thetaFlagY); //Positive / Negative Flag Y-axis
    fram.write8(universalIndex*22+7, thetaIntZ); //Integer value Z-axis
    fram.write8(universalIndex*22+8, thetaDecimalZ); //Decimal value Z-axis
    fram.write8(universalIndex*22+9, thetaFlagZ); //Positive / Negative Flag Z-axis
    
    //ALTITUDE
    fram.write8(universalIndex*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(universalIndex*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+21, pressureLast); //Digits in the Tenths and Hundredths place

    //No need to record gimbal or servo angles on fram chip anymore because they are no longer in use

    if (altitude < altitudePrev){ //and pressure > pressurePrev) { //Condition to end the loop and progress to the next flight stage
      Serial.println("Unpowered Flight Loop Broken");
      unpoweredFlag = false;
      break;
    }    

    altitudePrev = altitude; //Sets the previous altitude measurement equal to the current one in preparation to take a new measurement
    pressurePrev = pressure; //Sets the previous pressure measurement equal to the current one in preparation to take a new measurement

    universalIndex++; //Adds +1 to measurement counting index universal between all functions
    delay(BNO055_SAMPLERATE_DELAY_MS);//Delay for selected BNO sample rate
  }
}



//--------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Ballistic Descent Stage Function
void ballisticDescent() {

  flightStage = 4; //changes flight stage marker to 4
    
  bool ballisticFlag = true; //Flag to mark whether or not the loop keeps going
  while (ballisticFlag == true) { //While loop to record orientation, altitude, and pressure data until the rocket descends to an altitude where it needs to deploy the parachute

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x(); //Sets thetaX equal to the most recent X-axis orientation measurement
    thetaY[0] = euler.y() + thetaY_offset; //Sets first element in thetaY array equal to most recent Y-axis orientation measurement
    thetaZ[0] = euler.z() + thetaZ_offset; //Sets first element in thetaZ array equal to most recent Z-axis orientation measurement -- (-90 included to zero the sensor to earth's normal)
      
    if (thetaX >= 0) { //If statement to separate the X-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntX = floor(thetaX); //Separates integer value of X-axis orientation measurement
      thetaDecimalX = 100*(thetaX-floor(thetaX)); //Separates decimal value of X-axis orientation measurement
      thetaFlagX = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntX = -1*ceil(thetaX); //Separates the ineger value of the X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalX = -100*(thetaX-ceil(thetaX)); //Separates decimal value of X-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagX = 1; //Flag set to 1 to recognize as negative number
    }
    if (thetaY[0] >= 0) { //If statement to separate the Y-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntY = floor(thetaY[0]); //Separates integer value of Y-axis orientation measurement
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0])); //Separates decimal value of Y-axis orientation measurement
      thetaFlagY = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]); //Separates the ineger value of the Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0])); //Separates decimal value of Y-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagY = 1; //Flag set to 1 to recognize as negative number
    }      
    if (thetaZ[0] >= 0) { //If statement to separate the Z-axis orientation into 3, single-byte values to be recorded on the FRAM chip -- Statements separated by whether the angle is positive or negative
      thetaIntZ = floor(thetaZ[0]); //Separates integer value of Z-axis orientation measurement
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement
      thetaFlagZ = 0; //Flag set to 0 to recognize as positive number
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]); //Separates the ineger value of the Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0])); //Separates decimal value of Z-axis orientation measurement -- Temporarily changes stored data value to a positive integer. Reverted back to negative value in the memory-reading program
      thetaFlagZ = 1; //Flag set to 1 to recognize as negative number
    }


    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeLast = 100*(altitude - floor(altitude)); //Separates first 2 digits of altitude measurement
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100)); //Separates middle 2 digits of altitude measurement
    altitudeFirst = floor(floor(altitude)/100); //Separates decimal digits of altitude measurement

    pressureLast = 100*(pressure-floor(pressure)); //Separates first 2 digits of pressure measurement
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100)); //Separates middle 2 digits of pressure measurement
    pressureFirst = floor(floor(pressure)/100); //Separates decimal digits of pressure measurement

    //Writing values to FRAM chip for Ballistic Descent stage
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
      
    //ORIENTATION
    fram.write8(universalIndex*22+1, thetaIntX); //Integer value X-axis
    fram.write8(universalIndex*22+2, thetaDecimalX); //Decimal value X-axis
    fram.write8(universalIndex*22+3, thetaFlagX); //Positive / Negative Flag X-axis
    fram.write8(universalIndex*22+4, thetaIntY); //Integer value Y-axis
    fram.write8(universalIndex*22+5, thetaDecimalY); //Decimal value Y-axis
    fram.write8(universalIndex*22+6, thetaFlagY); //Positive / Negative Flag Y-axis
    fram.write8(universalIndex*22+7, thetaIntZ); //Integer value Z-axis
    fram.write8(universalIndex*22+8, thetaDecimalZ); //Decimal value Z-axis
    fram.write8(universalIndex*22+9, thetaFlagZ); //Positive / Negative Flag Z-axis
    
    //ALTITUDE
    fram.write8(universalIndex*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(universalIndex*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+21, pressureLast); //Digits in the Tenths and Hundredths place

    if (altitude < 350){ //m above sea level -- altitude threshold for parachute deployment -- PICK BETTER ALTITUDE THRESHOLD FOR PARACHUTE TO BE DEPLOYED ONCE ACTUAL TEST FLIGHTS START
      Serial.println(F("Ballistic Descent Loop Broken"));
      ballisticFlag = false;
      break;
    }
    
    universalIndex++; //Adds +1 to measurement counting index universal between all functions
    delay(BNO055_SAMPLERATE_DELAY_MS); //Delay for selected BNO sample rate
  } 
  
}



//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//Chute Descent Stage Function -- Includes parachute deployment commands
void chuteDescent() {

  myServo3.write(160); //angle for parachute system servo arm to release latch and deploy parachute -- POSSIBLE NEED TO CHANGE ANGLE AFTER TESTING

  flightStage = 5; //changes flight stage marker to 4
  
  altitudePrev = altitude; //Once again using this variable to keep track of and compare the current altitude measurement to the previous one
  pressurePrev = pressure; //Once again using this variable to keep track of and compare the current pressure measurement to the previous one

  bool chuteFlag = true; //Flag to mark whether or not the loop keeps going
  short chuteIndex = 0; //Index to keep track of how long the rocket has been stationary
  while (chuteFlag == true) {//While loop to record altitude and pressure data until the rocket is stationary on the ground again for at least 2 seconds

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude(); //Altitude measurement
    pressure = bmp.readPressure()-90000; //Pressure measurement -- subtracts 90,000Pa from the pressure measurements for easier storage on FRAM chip. Added back in chip reading program

    altitudeLast = 100*(altitude - floor(altitude)); //Separates first 2 digits of altitude measurement
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100)); //Separates middle 2 digits of altitude measurement
    altitudeFirst = floor(floor(altitude)/100); //Separates decimal digits of altitude measurement

    pressureLast = 100*(pressure-floor(pressure)); //Separates first 2 digits of pressure measurement
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100)); //Separates middle 2 digits of pressure measurement
    pressureFirst = floor(floor(pressure)/100); //Separates decimal digits of pressure measurement

    //Writing values to FRAM chip for Chute Descent stage
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE

    //ALTITUDE
    fram.write8(universalIndex*22+16, altitudeFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+17, altitudeMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+18, altitudeLast); //Digits in the Tenths and Hundredths place

    //PRESSURE
    fram.write8(universalIndex*22+19, pressureFirst); //Digits in the hundreds and thousands place
    fram.write8(universalIndex*22+20, pressureMid); //Digits in the Tens and Ones place
    fram.write8(universalIndex*22+21, pressureLast); //Digits in the Tenths and Hundredths place

    //No need to record orientation data after parachute deploys

    if (altitude == altitudePrev) { //If statement to add 1 to the index if consecutive altitude measurements are equal to one another
      chuteIndex++;
    }
    else {
      chuteIndex = 0; //Restarts the index if measurements are not equal
    }

    if (chuteIndex == 20) { //If statement to end chute descent loop after 20 consecutive, identical altitude measurements (2 seconds worth of measurements)
      Serial.println(F("Chute Descent Loop Broken"));
      chuteFlag = false;
      break;
    }

    altitudePrev = altitude; //Sets the previous altitude measurement equal to the current one in preparation to take a new measurement

    universalIndex++; //Adds +1 to measurement counting index universal between all functions
    delay(BNO055_SAMPLERATE_DELAY_MS); //Delay for selected BNO sample rate
    
  }
}


//Future Function to possibly incorporate powered landing instead of chute descent
void landing() {
}


//Code to be run by the arduino once, includes all functions above in order
void setup() {
  
  groundIdle();
  poweredFlight();
  //unpoweredFlight();
  //ballisticDescent();
  //chuteDescent();
 
}


//Code to be run by the arduino indefinitely, not in use for this program
void loop() {
}
