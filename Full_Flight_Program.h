#include <cmath>
#include <math.h>

short universalIndex = 0;

short motor_time = 1000;
short flightStage = 0;

float thetaX; //NOT USED FOR PID SO NOT AN ARRAY
float thetaY[3]; //Rocket pitch angle Y axis
float thetaZ[3]; //Rocket pitch angle Z axis
short runtime[1000];

float gimbal_angleY[2]; //Gimbal Angle Y axis
float gimbal_angleZ[2]; //Gimbal Angle Z axis

double pi = atan(1)*4;

float max_angle = 8.0; //max angle for gimbal 
float time_step = 0.01; //update rate on BNO055 IMU ~10ms
float maxRotationPerStep = 20/0.1*time_step;  //TEMPORARY VALUE - 100ms/60deg - TO BE REPLACED WITH MEASURED VALUE

float theta0 = -0*pi/180; //0 degree pitch target

float altitude = 0;
float pressure = 0;

float altitudePrev = 0;
float pressurePrev = 0;

//VARIABLES FOR RECORDING DATA

short altitudeFirst = 0;
short altitudeMid = 0;
short altitudeLast = 0;

short pressureFirst = 0;
short pressureMid = 0;
short pressureLast = 0;

short thetaIntX = 0;
short thetaIntY = 0;
short thetaIntZ = 0;

short thetaDecimalX = 0;
short thetaDecimalY = 0;
short thetaDecimalZ = 0;

short thetaFlagX = 0; //NEG/POS MARKERS FOR THETA ANGLES
short thetaFlagY = 0;
short thetaFlagZ = 0;

short gimbal_angleIntY = 0;
short gimbal_angleIntZ = 0;

short gimbal_angleDecimalY = 0;
short gimbal_angleDecimalZ = 0;

//flightStage, euler int(3), euler decimal (3), euler pos/neg flags (3), gimbalAngle int(2), gimbalAngle decimal (2), pressure int, pressure decimal, altitude decimal 1, altitude decimal 2, altitude int, servo angles (2)


int findTopAngleRatio(float angle) {

    if (angle < 0.9) {
        return 5.55556;
    }
    else if (angle <= 2.2) {
        return 4.54545;
    }
    else if (angle <= 3.4) {
        return 4.41176;
    }
    else if (angle <= 4.5) {
        return 4.44444;
    }
    else if (angle <= 6.6) {
        return 4.54545;
    }
    else if (angle <= 7.5) {
        return 4.66667;
    }
    else {
        return 4.70588;
    }
}

int findBottomAngleRatio(float angle) {

    if (angle <= 5) {
        return 5;
    }
    else if (angle <= 5.9) {
        return 5.08474;
    }
    else if (angle <= 6.6) {
        return 5.30303;
    }
    else if (angle <= 7.4) {
        return 5.40541;
    }
    else {
        return 5.48780;
    }
}

float proportional_errorY = 0.0; //Declares proportional_error variable for Y axis
float integral_errorY = 0.0; //Declares integral_error variable for Y axis
float derivative_errorY = 0.0; //Declares derivative_error variable for Y axis
float outputY = 0.0; //Declares PID output variable for Y axis

float proportional_errorZ = 0.0; //Declares proportional_error variable for Z axis
float integral_errorZ = 0.0; //Declares integral_error variable for Z axis
float derivative_errorZ = 0.0; //Declares derivative_error variable for Y axis
float outputZ = 0.0; //Declares PID output variable for Z axis