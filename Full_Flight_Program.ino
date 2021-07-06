#include <fstream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdio.h>

//BMP
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

//Servos
#include <Servo.h>

//BNO
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

//FRAM
#include "Adafruit_FRAM_I2C.h"

/*INCLUDE FULL FILE PATHWAY IF THE PROGRAM CANNOT FIND THE ARDUINOPID.h FILE*/
#include "Full_Flight_Program.h"

using namespace std; //Declares steandard namespace usage / standard name terminology

uint16_t BNO055_SAMPLERATE_DELAY_MS = 10; //Specific value for time in between BNO data samples. Should be same as time step in PID header file
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29); //Sets i2c address for BNO

Adafruit_BMP280 bmp;// I2C

Adafruit_FRAM_I2C fram     = Adafruit_FRAM_I2C(); // Activates FRAM chip

short servoPin1=6; //Declares the Arduino pin we want for servo1
short servoPos1=90; //Initial value for servo1 angle

short servoPin2=5; //Declares the Arduino pin we want for servo2
short servoPos2=90; //Initial value for servo2 angle

short servoPin3=10; //Declares the Arduino pin we want for servo3
short servoPos3=90; //Initial value for servo2 angle

Servo myServo1; //Declares servo1 as a servo via header file command
Servo myServo2; //Declares servo2 as a servo via header file command
Servo myServo3; //Declares servo2 as a servo via header file command


//BNO printEvent
void printEvent(sensors_event_t* event) {
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



void groundIdle() {
  
  uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

  Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);


  Serial.begin(115200);
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
    Serial.print(bmp.readAltitude()); /* Adjusted to local forecast! */
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

  bool bnoCalFlag = true;
  while (bnoCalFlag = true) {
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
      break;
    }
  }
  Serial.println(F("BNO055 Fully Calibrated"));
  delay(100);


  //Servo Setup
  myServo1.attach(servoPin1);
  myServo2.attach(servoPin2);

  
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

  Serial.println("Gimbal Full Mobility Test");
  delay(500);

  myServo1.write(135);
  delay(250);
  myServo2.write(135);
  delay(100);
  myServo1.write(45);
  delay(100);
  myServo2.write(45);
  delay(100);
  myServo1.write(135);
  delay(100);
  myServo2.write(135);
  delay(100);
  myServo1.write(45);
  delay(100);
  myServo2.write(45);
  delay(100);
  myServo1.write(135);
  delay(100);
  myServo2.write(90);
  delay(250);
  myServo1.write(90);
  delay(250);

  Serial.println("Systems Testing Protocol Complete");
  delay(1000);


  //REMEMBER TO ADD LOOP TO WAIT UNTIL ROCKET READS ACCELERATION BEFORE MOVING TO NEXT FLIGHT STAGE
}


void poweredFlight() {

  flightStage = 2;
  
  Serial.begin(115200); //Setup command to begin sending information to the Serial monitor at the baudrate specified
  
  myServo1.attach(servoPin1); //Activates servo1 on the pin specified in the command earlier
  myServo1.write(servoPos1); //Sets servo1 angle to initial value
  myServo2.attach(servoPin2); //Activates servo2 on the pin specified in the command earlier
  myServo2.write(servoPos2); //Sets servo2 angle to initial value

  if (fram.begin()) {  // you can stick the new i2c addr in here, e.g. begin(0x51); //IF statement to print whether the Arduino has found the FRAM chip or not
    Serial.println("Found I2C FRAM");
  } 
  else {
    Serial.println("I2C FRAM not identified ... check your connections?\r\n");
    Serial.println("Will continue in case this processor doesn't support repeated start\r\n");
  }
  
  if(!bno.begin()) //IF statement to print whether the Arduino has found the BNO chip or not
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                    "try a different address!"));
    while (1) delay(10);
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bno.setExtCrystalUse(true); //Sets the timing source for the BNO chip as an external source (the Arduino) instead of the internal crystal inside the chip -- this is to measure everything at the same time through the arduino


    //F15 PID gains------
  float KP = 0.7; //Proportional Gain
  float KI = 0.005; //Integral Gain
  float KD = .11; //Derivative Gain


  for (short n = 0; n < 2; n++) { //Loop to set initial theta values for ease of use in simpler formulas in PID loop

    runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed
      
    thetaY[2] = thetaY[1]; //Array rearrangement
    thetaY[1] = thetaY[0];

    thetaZ[2] = thetaZ[1];
    thetaZ[1] = thetaZ[0];

    gimbal_angleY[1] = gimbal_angleY[0];
    gimbal_angleZ[1] = gimbal_angleZ[0];

    //BNO READINGS AND HEX SEPARATION
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement (must be done every time BNO makes a measurement)

    thetaX = euler.x();
    thetaY[0] = euler.y();
    thetaZ[0] = euler.z()-90;
    
    if (thetaX >= 0) {
      thetaIntX = floor(thetaX);
      thetaDecimalX = 100*(thetaX-floor(thetaX));
      thetaFlagX = 0;
    }
    else {
      thetaIntX = -1*ceil(thetaX);
      thetaDecimalX = -100*(thetaX-ceil(thetaX));
      thetaFlagX = 1;
    }
    if (thetaY[0] >= 0) {
      thetaIntY = floor(thetaY[0]);
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0]));
      thetaFlagY = 0;
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]);
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0]));
      thetaFlagY = 1;
    }      
    if (thetaZ[0] >= 0) {
      thetaIntZ = floor(thetaZ[0]);
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0]));
      thetaFlagZ = 0;
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]);
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0]));
      thetaFlagZ = 1;
    }

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude();
    pressure = bmp.readPressure()-90000;

    altitudeFirst = floor(altitude/100);
    altitudeMid = (floor(altitude) - 100*altitudeFirst);
    altitudeLast = 100*(altitude - floor(altitude));

    pressureFirst = floor(pressure/100);
    pressureMid = (floor(pressure) - 100*pressureFirst);
    pressureLast = 100*(pressure - floor(pressure));
    
      
    //Writing values to FRAM
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
      
    fram.write8(universalIndex*22+1, thetaIntX); //ORIENTATION
    fram.write8(universalIndex*22+2, thetaDecimalX);
    fram.write8(universalIndex*22+3, thetaFlagX);
    fram.write8(universalIndex*22+4, thetaIntY);
    fram.write8(universalIndex*22+5, thetaDecimalY);
    fram.write8(universalIndex*22+6, thetaFlagY);
    fram.write8(universalIndex*22+7, thetaIntZ);
    fram.write8(universalIndex*22+8, thetaDecimalZ);
    fram.write8(universalIndex*22+9, thetaFlagZ);

    fram.write8(universalIndex*22+10, gimbal_angleIntY); //GIMBAL ANGLE
    fram.write8(universalIndex*22+11, gimbal_angleDecimalY);
    fram.write8(universalIndex*22+12, gimbal_angleIntZ);
    fram.write8(universalIndex*22+13, gimbal_angleDecimalZ);

    fram.write8(universalIndex*22+14, servoPos1); //SERVO ANGLE
    fram.write8(universalIndex*22+15, servoPos2);

    fram.write8(universalIndex*22+16, altitudeFirst); //ALTITUDE
    fram.write8(universalIndex*22+17, altitudeMid);
    fram.write8(universalIndex*22+18, altitudeLast);

    fram.write8(universalIndex*22+19, pressureFirst); //PRESSURE
    fram.write8(universalIndex*22+20, pressureMid);
    fram.write8(universalIndex*22+21, pressureLast);
      

        // Print Statement for all relevant information while for loop is active -------------------------------------------------------------------------------------------------------------------------------------------------------------------
      
    Serial.print(F("n= "));
    Serial.print(universalIndex);
    Serial.print(F(" runtime= "));
    Serial.print(runtime[n]);
    Serial.print(F(" ThetaX= "));
    Serial.print(thetaX);
    Serial.print(F(" thetaIntX= "));
    Serial.print(thetaIntX);
    Serial.print(F(" thetaDecimalX= "));
    Serial.print(thetaDecimalX);
    Serial.print(F(" thetaFlagX= "));
    Serial.println(thetaFlagX);

    Serial.print(F("thetaY= "));
    Serial.print(thetaY[0]);
    Serial.print(F(" thetaIntY= "));
    Serial.print(thetaIntY);
    Serial.print(F(" thetaDecimalY= "));
    Serial.print(thetaDecimalY);
    Serial.print(F(" thetaFlagY= "));
    Serial.print(thetaFlagY);
    Serial.print(F(" thetaZ= "));
    Serial.print(thetaZ[0]);
    Serial.print(F(" thetaIntZ= "));
    Serial.print(thetaIntZ);
    Serial.print(F(" thetaDecimalZ= "));
    Serial.print(thetaDecimalZ);
    Serial.print(F(" thetaFlagZ= "));
    Serial.println(thetaFlagZ);
      
    Serial.print(F("gimbal_angleY= "));
    Serial.print(gimbal_angleY[0]);
    Serial.print(F(" gimbal_angleIntY= "));
    Serial.print(gimbal_angleIntY);
    Serial.print(F(" gimbal_angleDecimalY= "));
    Serial.print(gimbal_angleDecimalY);
    Serial.print(F(" gimbal_angleZ= "));
    Serial.print(gimbal_angleZ[0]);
    Serial.print(F(" gimbal_angleIntZ= "));
    Serial.print(gimbal_angleIntZ);
    Serial.print(F(" gimbal_angleDecimalZ= "));
    Serial.print(gimbal_angleDecimalZ);
    Serial.print(F(" Servo angleY= "));
    Serial.print(servoPos1);
    Serial.print(F(" Servo angleZ= "));
    Serial.println(servoPos2);

    Serial.print(F("altitude= "));
    Serial.print(altitude);
    Serial.print(F(" altitudeLast= "));
    Serial.print(altitudeLast);
    Serial.print(F(" altitudeMid= "));
    Serial.print(altitudeMid);
    Serial.print(F(" altitudeFirst= "));
    Serial.println(altitudeFirst);

    Serial.print(F("pressure= "));
    Serial.print(pressure);
    Serial.print(F(" pressureFirst= "));
    Serial.print(pressureFirst);
    Serial.print(F(" pressureMid= "));
    Serial.print(pressureMid);
    Serial.print(F(" pressureLast= "));
    Serial.println(pressureLast);

      
    delay(BNO055_SAMPLERATE_DELAY_MS);
    //------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    universalIndex++;
    
  }

  float runningSumY = thetaY[0]+thetaY[1]; //Declares runningSum variable to keep track of rocket's total angle offset in Y axis
  float runningSumZ = thetaZ[0]+thetaZ[1]; //Declares runningSum variable to keep track of rocket's total angle offset in Z axis


  for (int n = 2; n <= motor_time+1; n++) { //Loop for actual PID algorithm math

    runtime[n] = n*10; //Sets the index value in the runtime equal to the number of milliseconds that have elapsed (just like in the other for loop but for the remainder of the runtime)

    thetaY[2] = thetaY[1]; //Array rearrangement
    thetaY[1] = thetaY[0];

    thetaZ[2] = thetaZ[1];
    thetaZ[1] = thetaZ[0];

    gimbal_angleY[1] = gimbal_angleY[0];
    gimbal_angleZ[1] = gimbal_angleZ[0];
      
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x();
    thetaY[0] = euler.y();
    thetaZ[0] = euler.z()-90;
      
    if (thetaX >= 0) {
      thetaIntX = floor(thetaX);
      thetaDecimalX = 100*(thetaX-floor(thetaX));
      thetaFlagX = 0;
    }
    else {
      thetaIntX = -1*ceil(thetaX);
      thetaDecimalX = -100*(thetaX-ceil(thetaX));
      thetaFlagX = 1;
    }
    if (thetaY[0] >= 0) {
      thetaIntY = floor(thetaY[0]);
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0]));
      thetaFlagY = 0;
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]);
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0]));
      thetaFlagY = 1;
    }      
    if (thetaZ[0] >= 0) {
      thetaIntZ = floor(thetaZ[0]);
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0]));
      thetaFlagZ = 0;
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]);
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0]));
      thetaFlagZ = 1;
    }

    //PID control
    runningSumY = runningSumY + (thetaY[0]-theta0); //Adds new value to running sum in Y axis rocket angle for current index
    runningSumZ = runningSumZ + (thetaZ[0]-theta0); //Adds new value to running sum in Z axis rocket angle for current index

    proportional_errorY = thetaY[0]-theta0; //Proportional error for Y axis - difference between current measured angle and the desired value (perfectly vertical in this case)
    integral_errorY = runningSumY; //Integral error for Y axis - equal to the running sum of rocket angles for Y axis
    derivative_errorY = (thetaY[0]-thetaY[1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

    proportional_errorZ = thetaZ[0]-theta0; //Proportional error for Z axis - difference between current measured angle and the desired value (perfectly vertical in this case)
    integral_errorZ = runningSumZ; //Integral error for Z axis - equal to the running sum of rocket angles for Z axis
    derivative_errorZ = (thetaZ[0]-thetaZ[1])/time_step; //Derivative error for Y axis - Equal to the change in Y axis rocket angle for current time step only

    outputY = - KP*proportional_errorY - KI*integral_errorY - KD*derivative_errorY; //Actual PID equation math or Y axis
    outputZ = - KP*proportional_errorZ - KI*integral_errorZ - KD*derivative_errorZ; //Actual PID equation math or Z axis

    
    //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Y axis
    if ((outputY - gimbal_angleY[1]) > maxRotationPerStep) { 
      gimbal_angleY[0] = gimbal_angleY[1] + maxRotationPerStep;
    }
    else if ((gimbal_angleY[1] - outputY) > maxRotationPerStep) {
      gimbal_angleY[0] = gimbal_angleY[1] - maxRotationPerStep;
    }
    else {
      gimbal_angleY[0] = outputY;
    }
    //--------------------------------------------------------------------------------------------------------------
        

    //If statement to keep the maximum angle value for servos within their mobility range for Y axis
    if (gimbal_angleY[0] > max_angle) { 
      gimbal_angleY[0] = max_angle;
    }
    if (gimbal_angleY[0] < - max_angle){
      gimbal_angleY[0] = - max_angle;
    }
    //----------------------------------------------------------------------------------------------

    //If statement to keep the output value rate of change from exceeding the maximum turn speed for servo in Z axis
    if ((outputZ - gimbal_angleZ[1]) > maxRotationPerStep) { 
      gimbal_angleZ[0] = gimbal_angleZ[1] + maxRotationPerStep;
    }
    else if ((gimbal_angleZ[1] - outputZ) > maxRotationPerStep) { 
      gimbal_angleZ[0] = gimbal_angleZ[1] - maxRotationPerStep;
    }
    else {
      gimbal_angleZ[0] = outputZ;
    }
    //--------------------------------------------------------------------------------------------------------------
        

    //If statement to keep the maximum angle value for servos within their mobility range for Z axis
    if (gimbal_angleZ[0] > max_angle) { 
      gimbal_angleZ[0] = max_angle;
    }
    if (gimbal_angleZ[0] < - max_angle){
      gimbal_angleZ[0] = - max_angle;
    }
    //----------------------------------------------------------------------------------------------


    if (gimbal_angleY[0] >= 0) {
      gimbal_angleIntY = floor(gimbal_angleY[0]);
      gimbal_angleDecimalY = 100*(gimbal_angleY[0]-floor(gimbal_angleY[0]));
    }
    else {
      gimbal_angleIntY = -1*ceil(gimbal_angleY[0]);
      gimbal_angleDecimalY = 100-100*(gimbal_angleY[0]-ceil(gimbal_angleY[0]));
    }
    if (gimbal_angleZ[0] >= 0) {
      gimbal_angleIntZ = floor(gimbal_angleZ[0]);
      gimbal_angleDecimalZ = 100*(gimbal_angleZ[0]-floor(gimbal_angleZ[0]));
    }
    else {
      gimbal_angleIntZ = -1*ceil(gimbal_angleZ[0]);
      gimbal_angleDecimalZ = 100-100*(gimbal_angleZ[0]-ceil(gimbal_angleZ[0]));
    }

    //----------------------------------------------------------------------------------------------


    servoPos1 = 90+findBottomAngleRatio(gimbal_angleY[0])*gimbal_angleY[0]; //Calculates the servo angle based on the given gimbal angle for servo1
    myServo1.write(servoPos1); //Applies the calculated angle for servo1

    servoPos2 = 90+findTopAngleRatio(gimbal_angleZ[0])*gimbal_angleZ[0]; //Calculates the servo angle based on the given gimbal angle for servo2
    myServo2.write(servoPos2); //Applies the calculated angle for servo2

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude();
    pressure = bmp.readPressure()-90000;

    altitudeLast = 100*(altitude - floor(altitude));
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100));
    altitudeFirst = floor(floor(altitude)/100);

    pressureLast = 100*(pressure-floor(pressure));
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100));
    pressureFirst = floor(floor(pressure)/100);

    //Writing values to FRAM
    fram.write8(n*22, flightStage); //FLIGHT STAGE
      
    fram.write8(universalIndex*22+1, thetaIntX); //ORIENTATION
    fram.write8(universalIndex*22+2, thetaDecimalX);
    fram.write8(universalIndex*22+3, thetaFlagX);
    fram.write8(universalIndex*22+4, thetaIntY);
    fram.write8(universalIndex*22+5, thetaDecimalY);
    fram.write8(universalIndex*22+6, thetaFlagY);
    fram.write8(universalIndex*22+7, thetaIntZ);
    fram.write8(universalIndex*22+8, thetaDecimalZ);
    fram.write8(universalIndex*22+9, thetaFlagZ);

    fram.write8(universalIndex*22+10, gimbal_angleIntY); //GIMBAL ANGLE
    fram.write8(universalIndex*22+11, gimbal_angleDecimalY);
    fram.write8(universalIndex*22+12, gimbal_angleIntZ);
    fram.write8(universalIndex*22+13, gimbal_angleDecimalZ);

    fram.write8(universalIndex*22+14, servoPos1); //SERVO ANGLE
    fram.write8(universalIndex*22+15, servoPos2);

    fram.write8(universalIndex*22+16, altitudeFirst); //ALTITUDE
    fram.write8(universalIndex*22+17, altitudeMid);
    fram.write8(universalIndex*22+18, altitudeLast);

    fram.write8(universalIndex*22+19, pressureFirst); //PRESSURE
    fram.write8(universalIndex*22+20, pressureMid);
    fram.write8(universalIndex*22+21, pressureLast);
 

    // Print Statement for all relevant information while PID loop is active ------------------------------------------------------------------------------------------------------------------------------------------------------------------
        
    Serial.print(F("n= "));
    Serial.print(universalIndex);
    Serial.print(F(" runtime= "));
    Serial.print(runtime[n]);
    Serial.print(F(" ThetaX= "));
    Serial.print(thetaX);
    Serial.print(F(" thetaIntX= "));
    Serial.print(thetaIntX);
    Serial.print(F(" thetaDecimalX= "));
    Serial.print(thetaDecimalX);
    Serial.print(F(" thetaFlagX= "));
    Serial.println(thetaFlagX);

    Serial.print(F("thetaY= "));
    Serial.print(thetaY[0]);
    Serial.print(F(" thetaIntY= "));
    Serial.print(thetaIntY);
    Serial.print(F(" thetaDecimalY= "));
    Serial.print(thetaDecimalY);
    Serial.print(F(" thetaFlagY= "));
    Serial.print(thetaFlagY);
    Serial.print(F(" thetaZ= "));
    Serial.print(thetaZ[0]);
    Serial.print(F(" thetaIntZ= "));
    Serial.print(thetaIntZ);
    Serial.print(F(" thetaDecimalZ= "));
    Serial.print(thetaDecimalZ);
    Serial.print(F(" thetaFlagZ= "));
    Serial.println(thetaFlagZ);
      
    Serial.print(F("gimbal_angleY= "));
    Serial.print(gimbal_angleY[0]);
    Serial.print(F(" gimbal_angleIntY= "));
    Serial.print(gimbal_angleIntY);
    Serial.print(F(" gimbal_angleDecimalY= "));
    Serial.print(gimbal_angleDecimalY);
    Serial.print(F(" gimbal_angleZ= "));
    Serial.print(gimbal_angleZ[0]);
    Serial.print(F(" gimbal_angleIntZ= "));
    Serial.print(gimbal_angleIntZ);
    Serial.print(F(" gimbal_angleDecimalZ= "));
    Serial.print(gimbal_angleDecimalZ);
    Serial.print(F(" Servo angleY= "));
    Serial.print(servoPos1);
    Serial.print(F(" Servo angleZ= "));
    Serial.println(servoPos2);

    Serial.print(F("altitude= "));
    Serial.print(altitude);
    Serial.print(F(" altitudeLast= "));
    Serial.print(altitudeLast);
    Serial.print(F(" altitudeMid= "));
    Serial.print(altitudeMid);
    Serial.print(F(" altitudeFirst= "));
    Serial.println(altitudeFirst);

    Serial.print(F("pressure= "));
    Serial.print(pressure);
    Serial.print(F(" pressureFirst= "));
    Serial.print(pressureFirst);
    Serial.print(F(" pressureMid= "));
    Serial.print(pressureMid);
    Serial.print(F(" pressureLast= "));
    Serial.println(pressureLast);
        
    delay(BNO055_SAMPLERATE_DELAY_MS);
    //-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    universalIndex++;
    
  }
}



void unpoweredFlight() {

  flightStage = 3;
  
  altitudePrev = altitude;
  pressurePrev = pressure;

  bool unpoweredFlag = true;
  
  while (unpoweredFlag = true) {

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x();
    thetaY[0] = euler.y();
    thetaZ[0] = euler.z()-90;
      
    if (thetaX >= 0) {
      thetaIntX = floor(thetaX);
      thetaDecimalX = 100*(thetaX-floor(thetaX));
      thetaFlagX = 0;
    }
    else {
      thetaIntX = -1*ceil(thetaX);
      thetaDecimalX = -100*(thetaX-ceil(thetaX));
      thetaFlagX = 1;
    }
    if (thetaY[0] >= 0) {
      thetaIntY = floor(thetaY[0]);
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0]));
      thetaFlagY = 0;
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]);
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0]));
      thetaFlagY = 1;
    }      
    if (thetaZ[0] >= 0) {
      thetaIntZ = floor(thetaZ[0]);
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0]));
      thetaFlagZ = 0;
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]);
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0]));
      thetaFlagZ = 1;
    }

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude();
    pressure = bmp.readPressure()-90000;

    altitudeLast = 100*(altitude - floor(altitude));
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100));
    altitudeFirst = floor(floor(altitude)/100);

    pressureLast = 100*(pressure-floor(pressure));
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100));
    pressureFirst = floor(floor(pressure)/100);

    //Writing values to FRAM
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
      
    fram.write8(universalIndex*22+1, thetaIntX); //ORIENTATION
    fram.write8(universalIndex*22+2, thetaDecimalX);
    fram.write8(universalIndex*22+3, thetaFlagX);
    fram.write8(universalIndex*22+4, thetaIntY);
    fram.write8(universalIndex*22+5, thetaDecimalY);
    fram.write8(universalIndex*22+6, thetaFlagY);
    fram.write8(universalIndex*22+7, thetaIntZ);
    fram.write8(universalIndex*22+8, thetaDecimalZ);
    fram.write8(universalIndex*22+9, thetaFlagZ);
    
    fram.write8(universalIndex*22+16, altitudeFirst); //ALTITUDE
    fram.write8(universalIndex*22+17, altitudeMid);
    fram.write8(universalIndex*22+18, altitudeLast);

    fram.write8(universalIndex*22+19, pressureFirst); //PRESSURE
    fram.write8(universalIndex*22+20, pressureMid);
    fram.write8(universalIndex*22+21, pressureLast);

    if (altitude < altitudePrev){ //and pressure > pressurePrev) { //Condition to end the loop
      Serial.println("Unpowered Flight Loop Broken");
      break;
    }    

    altitudePrev = altitude;
    pressurePrev = pressure;

    universalIndex++;
    delay(BNO055_SAMPLERATE_DELAY_MS);
  }
}


void ballisticDescent() {

  flightStage = 4;
    
  bool ballisticFlag = true;
  
  while (ballisticFlag = true) {

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER); //Sets the object "euler" equal to the 3-dimensional euler (position) vector from each BNO measurement

    thetaX = euler.x();
    thetaY[0] = euler.y();
    thetaZ[0] = euler.z()-90;
      
    if (thetaX >= 0) {
      thetaIntX = floor(thetaX);
      thetaDecimalX = 100*(thetaX-floor(thetaX));
      thetaFlagX = 0;
    }
    else {
      thetaIntX = -1*ceil(thetaX);
      thetaDecimalX = -100*(thetaX-ceil(thetaX));
      thetaFlagX = 1;
    }
    if (thetaY[0] >= 0) {
      thetaIntY = floor(thetaY[0]);
      thetaDecimalY = 100*(thetaY[0]-floor(thetaY[0]));
      thetaFlagY = 0;
    }
    else {
      thetaIntY = -1*ceil(thetaY[0]);
      thetaDecimalY = -100*(thetaY[0]-ceil(thetaY[0]));
      thetaFlagY = 1;
    }      
    if (thetaZ[0] >= 0) {
      thetaIntZ = floor(thetaZ[0]);
      thetaDecimalZ = 100*(thetaZ[0]-floor(thetaZ[0]));
      thetaFlagZ = 0;
    }
    else {
      thetaIntZ = -1*ceil(thetaZ[0]);
      thetaDecimalZ = -100*(thetaZ[0]-ceil(thetaZ[0]));
      thetaFlagZ = 1;
    }

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude();
    pressure = bmp.readPressure()-90000;

    altitudeLast = 100*(altitude - floor(altitude));
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100));
    altitudeFirst = floor(floor(altitude)/100);

    pressureLast = 100*(pressure-floor(pressure));
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100));
    pressureFirst = floor(floor(pressure)/100);

    //Writing values to FRAM
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE
      
    fram.write8(universalIndex*22+1, thetaIntX); //ORIENTATION
    fram.write8(universalIndex*22+2, thetaDecimalX);
    fram.write8(universalIndex*22+3, thetaFlagX);
    fram.write8(universalIndex*22+4, thetaIntY);
    fram.write8(universalIndex*22+5, thetaDecimalY);
    fram.write8(universalIndex*22+6, thetaFlagY);
    fram.write8(universalIndex*22+7, thetaIntZ);
    fram.write8(universalIndex*22+8, thetaDecimalZ);
    fram.write8(universalIndex*22+9, thetaFlagZ);
    
    fram.write8(universalIndex*22+16, altitudeFirst); //ALTITUDE
    fram.write8(universalIndex*22+17, altitudeMid);
    fram.write8(universalIndex*22+18, altitudeLast);

    fram.write8(universalIndex*22+19, pressureFirst); //PRESSURE
    fram.write8(universalIndex*22+20, pressureMid);
    fram.write8(universalIndex*22+21, pressureLast);

    if (altitude < 350){ //m above sea level -- PICK BETTER ALTITUDE THRESHOLD FOR PARACHUTE TO BE DEPLOYED ONCE ACTUAL TEST FLIGHTS START
      Serial.println(F("Ballistic Descent Loop Broken"));
      break;
    }
    
    universalIndex++;
    delay(BNO055_SAMPLERATE_DELAY_MS);
  } 
}

void chuteDescent() {

  myServo3.write(45); //CHANGE ANGLE AFTER TESTING

  flightStage = 5;
  
  altitudePrev = altitude;
  pressurePrev = pressure;

  bool chuteFlag = true;

  short chuteIndex = 0;
  
  while (chuteFlag = true) {
    
    altitudePrev = altitude;

    //BMP READINGS AND HEX SEPARATION
    altitude = bmp.readAltitude();
    pressure = bmp.readPressure()-90000;

    altitudeLast = 100*(altitude - floor(altitude));
    altitudeMid = 100*(floor(altitude)/100 - floor(floor(altitude)/100));
    altitudeFirst = floor(floor(altitude)/100);

    pressureLast = 100*(pressure-floor(pressure));
    pressureMid = 100*(floor(pressure)/100 - floor(floor(pressure)/100));
    pressureFirst = floor(floor(pressure)/100);

    //Writing values to FRAM
    fram.write8(universalIndex*22, flightStage); //FLIGHT STAGE

    fram.write8(universalIndex*22+16, altitudeFirst); //ALTITUDE
    fram.write8(universalIndex*22+17, altitudeMid);
    fram.write8(universalIndex*22+18, altitudeLast);

    fram.write8(universalIndex*22+19, pressureFirst); //PRESSURE
    fram.write8(universalIndex*22+20, pressureMid);
    fram.write8(universalIndex*22+21, pressureLast);

    if (altitude == altitudePrev) {
      chuteIndex++;
    }
    else {
      chuteIndex = 0;
    }

    if (chuteIndex == 20) {
      Serial.println(F("Chute Descent Loop Broken"));
      break;
    }

    universalIndex++;
    delay(BNO055_SAMPLERATE_DELAY_MS);
    
  }
}


void landing() {
}



void setup() {

  groundIdle();

  poweredFlight();

  unpoweredFlight();

  ballisticDescent();

  chuteDescent();
 
}



void loop() {
}
