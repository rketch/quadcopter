/**
 * @file quad_firmware.ino
 *
 * @mainpage Quadcopter
 *
 * @section description Description
 * The C++ Arduino firmware for a quadcopter made as part of UCSD class CSE 176e
 *
 * @section hardware Hardware
 *  Custom PCB utilizing the ATmega128RFA1.
 * - Quadcopter hardware by Thomas Stuart and Robert Ketchum. Documentation and EAGLE board files available here: https://github.com/rketch/quadcopter
 * - Remote PCB made by UCSD Professor Steven Swanson.
 *
 * @section libraries Libraries
 * All libraries available at: https://github.com/rketch/quadcopter
 * - Wire
 *   - Arduino I2C 
 * - Arduino_LSM9DS1
 *   - Onboard IMU for quadcopter orientation.
 * - Adafruit_Simple_AHRS
 *   - Attitude and heading reference system for LSM
 * - radio
 *   - Radio library. Modified to include data structure sent via radio
 * - quad_remote
 *   - Custom CSE 176e library for the remote
 * - EEPROM
 *   - To save PID and trim calibration values to Non-volatile memory
 * - RotaryEncoder
 *   - To use the knob and button
 *
 *
 * @section authors Authors
 * Created by Thomas Stuart and Robert Ketchum, April 2019.
 * Modified by Robert Ketchum, May 2021.
 *
 */

#include "radio.h"                // Radio library. Modified to include data structure sent via radio
#include <Wire.h>                 // Arduino I2C
#include <Adafruit_Simple_AHRS.h> // Attitude and heading reference system for LSM
#include <Adafruit_LSM9DS1.h>     // Accelerometer/Gyroscope

#define RADIO_CH 17       ///< Radio Channel
#define M1 35             ///< Pin 35 is mapped to motor 1
#define M2 34             ///< Pin 34 is mapped to motor 2
#define M3 8              ///< Pin 8  is mapped to motor 3
#define M4 9              ///< Pin 9  is mapped to motor 4
#define FILT_RATIO 0.91   ///< Complementary filter ratio for a time constant of 10 Hz = 0.91
#define MS_TO_S 1000      ///< Conversion ratio
#define NEUTRAL_PITCH 43  ///< Neutral pitch gimbal position
#define NEUTRAL_ROLL 60   ///< Neutral roll gimbal position
#define NEUTRAL_YAW 60    ///< Neutral yaw gimbal position
#define MAX_ANGLE_FROM_NEUTRAL 10 ///< Maximum pitch or roll from neutral
#define MAX_TRIM_ANGLE 30         ///< Maximum pitch or roll trim angle
#define YAW_MAX_ANG_VEL 60        ///< Maximum yaw angular velocity (deg/sec)
#define TRIM_HISTORY 3            ///< Number of median filtered trim values

/*------------------
 * Section 1:
 * Declare global variables
 * -----------------*/

// Current time and previous step time stamp for controller
unsigned long time;  ///< Current time since start
unsigned int last = millis(); ///< Last loop time since start
double  timeDiff = 0.000000; ///< Implemented in complementary filter and controller


struct Data dataFromRemote; ///< Data structure received over radio

bool resetFlag = false; ///< For zeroing off orientation offset when armed.

// Values needed for the complimentary and median filters
quad_data_t orientation;    ///< Data structure with quadcopter orientation
float accHistoryArr[3][2];  ///< pitch, roll accelerometer reading history
float accHistoryArrCopy[3]; ///< For median filter
float rawGyro[2];           ///< Raw gyro pitch, roll rate
float gyroAngle[2];         ///< Integrated gyro angles
float rawAcc[2];            ///< Raw accelerometer angles
float filteredAngle[2];     ///< Complementary filtered angle
float filtGyroAngle[2];     ///< Partially filtered gyro angle
float filtAccelAngle[2];    ///< Partially filtered accelerometer angle
float gyroAngleStepBack[2]; ///< Previous time step filtered gyroscope angle. Needed for complementary filter
float angleOffset[2];       ///< Accelerometer offset from neutral. Measured when armed.
float trimAngle[2];         ///< Trimm offsets
float trimHistoryArr[3][4]; ///< Pitch, roll trim floats.
float trimHistoryArrCopy[3];///< Trim History for median filter
float medOutTrimArray[4];   ///< Pitch, roll median filter outputs

// PID variables
float   PID_output[3];      ///< PID output
float   remoteSetAngle[3];  ///< Set angle
float   currentAngle[3];    ///< Current quadcopter angle
float   error[3];           ///< Error angle
float   errorSum[3];        ///< Integrated error
float   lastError[3];       ///< previous error (for derivative) 


float MotorValue[4]; ///< Implemented motor thrust variables

/*------------------
 * Section 2:
 * Initialize IMU
 * -----------------*/

.
//  lib file | instance name | constructor in lib file
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); ///< Create LSM9DS1 board instance
// Create simple AHRS algorithm using the LSM9DS1 instance's accelerometer, gyroscope, and magnetometer.
// lib file     | instance name | instance name for parameter pointers
Adafruit_Simple_AHRS ahrs(&lsm.getAccel(), &lsm.getMag(), &lsm.getGyro());

/** 
 *  set up our instance of the sensor with the wanted register values
 */

void setupLSM(){
    // GYRO [first register]   // Set data rate for G and XL.  Set G low-pass cut off.  (Section 7.12)
    lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG1_G,  ODR_119 | G_BW_G_11 );

    // Enable the accelerometer X,Y,Z directions (Section 7.23)
    lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG5_XL, XL_ENABLE_X | XL_ENABLE_Y | XL_ENABLE_Z);
   
    // GYRO [second register]
    lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG2_G,  G_OUTSEL_HP );
    //lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG2_G,  G_OUTSEL_RAW ); //0.0 added
  
    // GYRO [third register]
    lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_G, G_HP_CUT_0000 | G_HP_EN ); 

    // Set low-pass XL filter frequency divider (Section 7.25)
    lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_9 );
    //lsm.write8(XGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG7_XL, HR_MODE | XL_LP_ODR_RATIO_50 ); //0.0 replaced
 
    // Magnotometer third register. Enable mag continuous (Section 8.7)
    lsm.write8(MAGTYPE, Adafruit_LSM9DS1::LSM9DS1_REGISTER_CTRL_REG3_M, B00000000);

    // Set up acc, gyro, mag ranges to stop clipping
    lsm.setupAccel(Adafruit_LSM9DS1::LSM9DS1_ACCELRANGE_2G); // 2, 4, 6, 8, or 16 Gs
    lsm.setupMag(Adafruit_LSM9DS1::LSM9DS1_MAGGAIN_4GAUSS);  // 2, 4, 6, 8, or 16 Gauss
    lsm.setupGyro(Adafruit_LSM9DS1::LSM9DS1_GYROSCALE_2000DPS); // 245, 500, or 2000 DPS
}

/*--------------------
 * Section 3:
 * Setup the quadcopter
 * ------------------*/

/**
 * Setup the quadcopter by initializing the radio, motors, IMU registers, and start clock
 */

void setup() {
    //Serial.begin(9600);     // used for consoling out written data
    Serial.begin(115200); // used for reading IMU data on a graph.
    rfBegin(RADIO_CH);      // Initialize radio
    initializeMotors();     // set motors as outputs
    while(!lsm.begin()){    //If no LSM9DS1 detected, send error message to user
        Serial.println(F("Ooops, no LSM9DS1 detected ... Check your wiring or I2C address!"));
        delay(1000);
        }
    setupLSM();   // Setup LSM. Helps with noise over defulat option.
    time = millis();             // start clock
    //dataFromRemote.arm = 0;    // if Reset is hit, restart
    resetFlag = true;            // Prevent motors from spinning on reset
    }

/*------------------
 * Section 4:
 * Loop forever
 * -----------------*/

/**
 * Loop forever. Receive radio data, find current quadcopter orientation, compute control system values, 
 * find current throttle corrections,write throttle values to motors if armed.
 */

void loop() {
    unsigned int now = millis();
    timeDiff = float(now - last)/1000;  // update time in seconds
    verifyRadioData();                  // receive and verify radio data 
    ComplimentaryFilter(timeDiff);      // Find the current quad angle
    PID(timeDiff);                      // Call the controller function
    mixer();                            // MIX the user throttle values with the controller offsets
    if ((dataFromRemote.arm == 1)&&(resetFlag == true)){       // if just armed quad, zero accelerometer
        calibrateAccelerometer();       // Zero Accelerometer
        resetFlag = false;
        }
    else if ((dataFromRemote.arm == 1)&&(resetFlag == false)){ // If armed, write command to motor
        engageMotors();  // run motors
        }
    else{                                                      // if disarmed, shut down motors 
        disableMotors(); // prevent motors from running
        }
    last = now;                       //update the time step
    }

/*---------------------------
 * Section 5:
 * Verify Received Radio Data
 * -------------------------*/

/**
* Read radio data received from the remote. If it is verified, save the data in a structure
*/

void verifyRadioData(){
    uint8_t buff[256];
    int len;
    if (len = rfAvailable()){                // If the quad receives radio signal from the controller
        rfRead(buff, len);                   // read the data sent, save buffer as "buff", length as "len"
        if(  (buff[54] == 154) && (buff[55] == 2) && (len == 126) ){  // Radio data checksum.
            dataFromRemote = *(Data*)buff;   // save the received data as a new instance
            }
        }
    }

/*------------------------------
 * Section 6:
 * Complementary Filter Function
 * ----------------------------*/

/**
* Combine the gyroscope and accelerometer to get an accurate heading of the quadcopter
* @param  dt The time difference between calling the function (in ms)
*/

void ComplimentaryFilter( double dt ){
    if (ahrs.getQuadOrientation(&orientation) ){  // if we have a new valid sensor reading
      // /*activate median filter for accelerometer readings
        for (int j=0; j<2; j++){ // for pitch, roll
            for (int i=0; i<2; i++){ // update acc steps
                accHistoryArr[i][j] = accHistoryArr[i+1][j];
                }
            accHistoryArr[2][0] = orientation.pitch; // get new acc reading
            accHistoryArr[2][1] = orientation.roll;  // get new acc reading
            }
        for (int i=0; i<2; i++){                     // for pitch, roll
            for (int j=0; j<3;j++){                  // make copies of the previous steps to send to median filter
                accHistoryArrCopy[j] = accHistoryArr[j][i];
                }
            rawAcc[i] = median(accHistoryArrCopy, 3);
            }
        // */ end median filter for accelerometer readings
        //rawAcc[0] = orientation.pitch;     // to use raw readings instead of median
        //rawAcc[1] = orientation.roll;      // to use raw readings instead of median
        rawGyro[0] = orientation.pitch_rate; // raw gyroscope angular velocity
        rawGyro[1] = orientation.roll_rate;  // raw gyroscope angular velocity
        for (int i=0; i<2; i++){             // for pitch, roll
            gyroAngleStepBack[i] = gyroAngle[i];         // previous gyro value for high pass filter
            gyroAngle[i] = gyroAngle[i] + rawGyro[i]*dt; // Integrate raw gyro value
            filtGyroAngle[i] = FILT_RATIO * (filtGyroAngle[i] + gyroAngle[i] - gyroAngleStepBack[i]); // high pass the gyroscope reading
            filtAccelAngle[i] = FILT_RATIO * filtAccelAngle[i] + (1 - FILT_RATIO) * rawAcc[i];        // low pass the accelerometer reading
            filteredAngle[i] = filtGyroAngle[i] + filtAccelAngle[i];                                  // complementary filter output
            }
        }
    }

/*------------------------------
 * Section 7:
 * PID Controller Function
 * ----------------------------*/

/**
* Utilize PID controller to ensure stable quadcopter flight
* @param  dt The time difference between calling the function (in ms)
*/

void PID(double dt ){
    findGimbalOffsets(); // Find the remote set angles from the gimbal positions
    findTrimOffsets();   // Find current trim values
    //current angle is obtained by reading the on board orientation data and subtracting off the trim and offset
    currentAngle[0] = -filteredAngle[0] - angleOffset[0] - trimAngle[0]; // pitch
    currentAngle[1] = -filteredAngle[1] - angleOffset[1] - trimAngle[1]; // roll
    currentAngle[2] = -orientation.yaw_rate;                             // yaw rate (unfiltered)
    //  i = 0 ( pitch )  , i = 1 ( roll ) , i = 2 ( yaw )   
    for( int i = 0; i < 3; i++){                                    // For Pitch, Roll, Yaw
        error[i] = remoteSetAngle[i] - currentAngle[i];             // Current Error
        errorSum[i] = errorSum[i] + error[i];                       // Integrated Error
        Coefficients k =   dataFromRemote.coefficientsContainer[i]; // Load Pitch, Yaw, or Roll PID values
        PID_output[i] = (k.proportional * error[i] ) + (k.integral * ( errorSum[i] * dt ) ) + ( k.derivative * ( (error[i] - lastError[i])/dt) );
        //               Proportional                    Integral                              Derivative
        lastError[i] = error[i];                                    // For error derivative
        }
    }

/*------------------------------
 * Section 8:
 * Function findGimbalOffsets
 * ----------------------------*/

/**
* Find the remote set angles from the gimbal positions
*/

void findGimbalOffsets(){
    //PITCH
    if (dataFromRemote.pitch > NEUTRAL_PITCH * 2){ // limit gimbal reading to accurate values
        dataFromRemote.pitch = NEUTRAL_PITCH * 2;
        }
    remoteSetAngle[0]     = float(dataFromRemote.pitch) * (MAX_ANGLE_FROM_NEUTRAL * 2) / (NEUTRAL_PITCH * 2) - MAX_ANGLE_FROM_NEUTRAL;
    //ROLL
    if (dataFromRemote.roll > NEUTRAL_ROLL * 2){ // limit gimbal reading to accurate values
        dataFromRemote.roll = NEUTRAL_ROLL * 2;
        }
    remoteSetAngle[1]     = -float(dataFromRemote.roll) * (MAX_ANGLE_FROM_NEUTRAL * 2) / (NEUTRAL_ROLL * 2) + MAX_ANGLE_FROM_NEUTRAL;
    //YAW
    if (dataFromRemote.yaw > NEUTRAL_YAW * 2){ // limit gimbal reading to accurate values
        dataFromRemote.yaw = NEUTRAL_YAW * 2;
        }
    remoteSetAngle[2]     = 2*(float(dataFromRemote.yaw) - YAW_MAX_ANG_VEL);
    }

/*------------------------------
 * Section 9:
 * Function findTrimOffsets
 * ----------------------------*/

/**
* Median the trim data and convert to a float angle.
*/

void findTrimOffsets(){
    if(dataFromRemote.pitchTrimSign){ // if negative
        trimAngle[0] = -float(dataFromRemote.pitchTrimMagnitude) * float(MAX_TRIM_ANGLE) / 255;
        }
    else{                   // if positive
        trimAngle[0] = float(dataFromRemote.pitchTrimMagnitude) * float(MAX_TRIM_ANGLE) / 255;
        }
    if(dataFromRemote.rollTrimSign){ // if negative
        trimAngle[1] = -float(dataFromRemote.rollTrimMagnitude) * float(MAX_TRIM_ANGLE) / 255;
        }
    else{                   // if positive
        trimAngle[1] = float(dataFromRemote.rollTrimMagnitude) * float(MAX_TRIM_ANGLE) / 255;
        }
    }

/*------------------------------
 * Section 10:
 * Median Filter Function
 * ----------------------------*/

/**
* Find the middle value of three raw inputs to filter out pertubations. Thanks geeksforgeeks
* @param  medArray An array containing the last n sensed values
* @param  n The length of medArray
* @return The median value
*/

float median(float medArray[], int n){
     sort(medArray, n);
     if (n % 2 != 0){ // if an odd number of values in array
         return (float)medArray[n / 2];
         }
     else{ // if an even number
         return (float)(medArray[(n - 1) / 2] + medArray[n / 2]) / 2.0;
         }
    }

/*------------------------------
 * Section 11:
 * Sort Array Function
 * ----------------------------*/

/**
* Sort an array into increasing numerical values. Thanks tsbrownie on youtube.
* @param unsortedArray An array containing the last n sensed values
* @param n The length of medianFinder
*/

void sort(float unsortedArray[], int n){
    for(int x = 0; x < n; x++){
        for(int y = 0; y < n-1; y++){
            if(unsortedArray[y] > unsortedArray[y+1]){
                float tempVal = unsortedArray[y+1];
                unsortedArray[y+1] = unsortedArray[y];
                unsortedArray[y] = tempVal;
                }
            }
        }
    }

/*------------------------------
 * Section 12:
 * Calibrate Accelerometer Funciton
 * ----------------------------*/

/**
* Calculate the angle offset of the accelerometer
*/

void calibrateAccelerometer(){
    for (int i=0; i<2; i++){ // pitch, roll
        angleOffset[i] = -filteredAngle[i];
        }
    }

/*------------------------------
 * Section 13:
 * Throttle Mixing Function
 * ----------------------------*/

/**
* Mix the user throttle value and the PID controller output
*/

void mixer(){
    if(dataFromRemote.throt <= 1){ // if throttle is off
        for (int i=0; i<4; i++){   // Don't spin motor
            MotorValue[i] = 0;
            }
        for (int i; i<3; i++){     // Reset Integral Error
            errorSum[i] = 0;
            }
        }
    else{
        //  i = 0 ( pitch )  , i = 1 ( roll ) , i = 2 ( yaw ) 
        MotorValue[0] = (float)dataFromRemote.throt + PID_output[0] - PID_output[1] + PID_output[2]; // front starboard
        MotorValue[3] = (float)dataFromRemote.throt + PID_output[0] + PID_output[1] - PID_output[2]; // front port
        MotorValue[1] = (float)dataFromRemote.throt - PID_output[0] - PID_output[1] - PID_output[2]; // back starboard
        MotorValue[2] = (float)dataFromRemote.throt - PID_output[0] + PID_output[1] + PID_output[2]; // back port
        }
    for (int i=0; i<4; i++){ // Prevent motor values from exceeding PWM limits
        if (MotorValue[i] < 0){
            MotorValue[i] = 0;
            }
        if (MotorValue[i] > 255){
            MotorValue[i] = 255;
            }
        }
    }

/*------------------------------
 * Section 14:
 * Initialize Motors Function
 * ----------------------------*/

/**
* Initialize motor pins
*/

void initializeMotors(){
    pinMode(M1, OUTPUT); 
    pinMode(M2, OUTPUT);
    pinMode(M3, OUTPUT);
    pinMode(M4, OUTPUT);
    pinMode(LED_BUILTIN,OUTPUT);
    }

/*------------------------------
 * Section 15:
 * Disable Motors Function
 * ----------------------------*/

/**
* Ensure that the motors do not run when the quad is disarmed
*/

void disableMotors(){
    digitalWrite(LED_BUILTIN,LOW);
    analogWrite(M1, 0);
    analogWrite(M2, 0);
    analogWrite(M3, 0);
    analogWrite(M4, 0);
    }

/*------------------------------
 * Section 14:
 * Engage Motors Function
 * ----------------------------*/

/**
* Write the motor commands to the respective motors
*/

void engageMotors(){
    digitalWrite(LED_BUILTIN,HIGH);
    analogWrite(M1, MotorValue[1]);      // back starboard
    analogWrite(M2, MotorValue[0]);      // front starboard
    analogWrite(M3, MotorValue[3]);      // front port
    analogWrite(M4, MotorValue[2]);      // back port
    }
