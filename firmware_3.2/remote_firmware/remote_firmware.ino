#include "quad_remote.h"   // Custom CSE 176e library for the remote
#include "radio.h"         // Radio library. Modified to include data structure sent via radio
#include <EEPROM.h>        // To save PID and trim calibration values to Non-volatile memory
#include <RotaryEncoder.h> // To use the knob and button

#define RADIO_CH 17 ///< Radio Channel
#define MAX_TRIM_ANGLE 30 ///< Maximum trim angle
#define TRIM_INCREMENT 10 ///< Increment by which trim is changed on remote

/*------------------
 * Section 1:
 * Declare global variables
 * -----------------*/

struct Data remoteToQuadData; ///< Structure for radio data. Struct "Data" initialized in radio.h

// Raw gimbal values. Organized as:
// [0] = A2 = pitch; [1] = A3 = roll; [2] = A1 = yaw; [3] = A0 = throt 
int gimbalRawValues[4];  ///< Values read by gimbal. Nominally 0 - 1024 but constrained by the potentiometer
int lowestGimbalValueReadSoFar[4]   = {  0,   0,   0,   0}; ///< lowest value possible by analog stick gimbal
int largestGimbalValueReadSoFar[4]  = {303, 318, 320, 306}; ///< highest value possible by analog stick gimbal
int saveLargestGimbalValuesSoFar[12]; ///< 255 + 255 + 255 = 765 is the highest save value with EEPROM for gimbal

// Values for rotary knob and its button
void knobPressed(bool); // Keeps track of the knob button
void knobsUpdate(); // Keeps track of the rotary knob value
bool knob_btn = 0; ///< Whether the knob button is pressed

// States of the remote
bool armToken = true;  ///< State variable for armed mode
bool isInTrimMode = false; ///< State variable for trim mode

// Variables for PID tuning and displaying values over LCD screen
int  cpa [3][6]; ///< Coefficients Properties Array for displaying and storing PID coefficients for pitch, yaw, roll. 3 rows, 6 columns
int  sign[9] = {1,1,1,  1,1,1,  1,1,1};  ///< Sign of PID tuning coefficients for storing via EEPROM
int  PIDpos = 0; ///< PID value being tuned currently
bool changeLeft = 1; ///< Keeps track of whether we are tuning whole integer or decimal PID value
int row = 0;  ///< LCD screen row to write to
int col = 0;  ///< LCD screen column to write to
int calculateColPos(int , bool  );
float trimAngle[2]; ///< Trim pitch and roll angles
String top = ""; ///< Initialize string to print on top row of LCD screen for PID tuning
int colPosStart[3]; ///< What column we are writing to LCD screen for PID tuning
bool knob_token = false; ///< trim or PID tuning variable
void updateDisplayPIDElementChange();

//Variables for converting integers to decimal. Integers may be saved on EEPROM, while float values are sent over radio.
void converToDecimalsBeforeSend();
float castIntPairToDecimal( int, int);

// Variables and function declarations for reading and writing to EEPROM.
void readPIDArrayFromEeprom(int);
void writePIDToEeprom();  // send current PID values to EEPROM
void writeGimbalsToEeprom(); // send calibrated gimbal values to EEPROM
int timesOverflowed = 0; ///< Keeps track of times an integer has overflowed for EEPROM writing

/*------------------
 * Section 2:
 * Set up remote
 * -----------------*/

/**
 * Setup the remote firmware by initializing radio, gimbal pins, LCD screen, 
 * knobs and buttons, and reading PID and trim values from EEPROM.
 */

void setup() {
    Serial.begin(9600);              // set up serial
    rfBegin(RADIO_CH);               // initialize radio on channel 17
    quad_remote_setup();             // initialize gimbal pins, lcd screen
    remoteToQuadData.arm = 0;        // to disable and enable throttle values to be reflected on quad.
    armToken = true;                 // every time the code resets, allow the arm symbol "A" to be displayed on remote
    isInTrimMode = false;            // start the remote in PID control tuning mode
    knob1_btn_cb = knobPressed;      // Initialize callback to check if knob being pressed
    knobs_update_cb = knobsUpdate;   // Initialize callback to check if knob value being changed
    knobsUpdate();                   // initialize knob
    initializeButtonPins();          // initialize buttons
    eepromRead();                    // read stored EEPROM calibration and PID values
    updateDisplayPIDElementChange(); // initialize LCD screen to PID tuning
    }

/*------------------
 * Section 3:
 * Loop
 * -----------------*/

/**
 * Loop endlessly. Read gimbal positions, determine if quadcopter should be armed, 
 * determine which LCD tuning state the remote should be in, determine whether the user is inputing commands, and
 * send data structure to quadcopter over radio.
 */

void loop() {
    readAndMapGimbals();            // Read raw gimbal values
    isInArmState(remoteToQuadData); // Are we arming the quadcopter right now?
    if (isInTrimMode){              // Trim state variable
      TrimTuningState();            // Display trim tuning on remote screen
    }
    else{
      PIDTuningState();             // Display PID tuning on remote screen.
    }
    converToDecimalsBeforeSend();   // Convert from integer values displayed on LCD to floats to send over radio
    rfSend(remoteToQuadData);       // send data over radio
    }

/*----------------------------
 * Section 4:
 * Function PIDTuningState
 * --------------------------*/

/**
* This function serves as a state machine. It is called when the remote is in the PID editing state
*/

void PIDTuningState(){
    if( digitalRead(BUTTON_DOWN_PIN)!=HIGH ){                         // Command to save PID values
        knob_btn = true;
        delay(1000);
        }
    if (armToken && remoteToQuadData.arm == 1){                       // if we want to arm, do it
        armToken = false;
        updateDisplayPIDElementChange();
        }
    else if(  (remoteToQuadData.arm == false)&&(knob_btn == true)  ){ //if not armed and user wants to calibrate. Also save PID values
        calibrate();                  // Calibrate Gimbals
        writePIDToEeprom();           // Save PID values
        knob_btn = false;             // change state variable as to not loop infinately
        }
    else if (  (remoteToQuadData.arm == true)&&(knob_btn == true)  ){ // if armed, just save PID values
        writePIDToEeprom();          // Save PID values
        knob_btn = false;            // change state variable as to not loop infinately
        }
    if (digitalRead(BUTTON_LEFT_PIN)!=HIGH){                          // Switch between Pitch/Yaw/Roll
        row = row + 1;                   // Pitch -> Roll -> Yaw
        if(row == 3){                    // Yaw -> Pitch 
            row = 0; 
            }
        knob_token = true;               // LCD state variable
        updateDisplayPIDElementChange(); // Update LCD Screen
        delay(1000);
        }
    if (digitalRead(BUTTON_CENTER_PIN)!=HIGH){                        // Integer <-> Decimal
        changeLeft = !changeLeft;                  // Integer or Decimal Editing State variable
        knob_token = true;                         // LCD State Variable
        col = calculateColPos(PIDpos, changeLeft); // Find what value we are editing
        updateDisplayPIDElementChange();           // Update LCD Screen
        delay(1000);
        }
    if (digitalRead(BUTTON_RIGHT_PIN)!=HIGH){                         // Proportional/Integral/Derivative
        PIDpos = PIDpos +1;                        // Proportional -> Integral -> Derivative
        if(PIDpos == 3){                           // Derivative -> Proportional
            PIDpos = 0;
            }
        knob_token = true;                         // LCD State Variable
        col = calculateColPos(PIDpos, changeLeft); // Find what value we are editing
        updateDisplayPIDElementChange();           // Update LCD Screen
        delay(1000);
        }
    if (digitalRead(BUTTON_UP_PIN)!=HIGH){                           // Current value + 100
        if(cpa[row][col] >= 0){                  // if positive, increase by 100
            cpa[row][col] = cpa[row][col] + 100;
            }
        else{                                    // if negative, decrease by 100
            cpa[row][col] = cpa[row][col] - 100;
            }
        knob_token = true;
        updateDisplayPIDNumChange();
        delay(1000);
        }
    if( digitalRead(BUTTON2_PIN)!=HIGH ){                          // Switch State to Trim Tuning
        isInTrimMode = true;               // Trim/PID editing State Variable
        updateDisplayTrimElementChange();  // Update LCD Screen
        delay(1000);
        }
    /*if (digitalRead(BUTTON_DOWN_PIN)!=HIGH){ // Reset all
        resetCoefficientValuesAndSigns();
        updateDisplayPIDElementChange();
        delay(1000);
        }*/
}

/*----------------------------
 * Section 5:
 * Function TrimTuningState
 * --------------------------*/

/**
* This function serves as a state machine. It is called when the remote is in the Trim editing state
*/

void TrimTuningState(){
    if (armToken && remoteToQuadData.arm == 1){ // if we want to arm, do it
        armToken = false;
        updateDisplayTrimElementChange(); // Display "A" on LCD screen to show user quad is armed
        }
    if( digitalRead(BUTTON2_PIN)!=HIGH ){ // Switch State to Trim Tuning
        isInTrimMode = false;             // Switch State Variable
        updateDisplayPIDElementChange();  // Update LCD Screen
        delay(1000);
        }
    if (digitalRead(BUTTON_UP_PIN)!=HIGH){ // decrease neutral pitch angle
        if ((!remoteToQuadData.pitchTrimSign) && (remoteToQuadData.pitchTrimMagnitude > TRIM_INCREMENT)){       // if positive and not going to zero
            remoteToQuadData.pitchTrimMagnitude = remoteToQuadData.pitchTrimMagnitude - TRIM_INCREMENT;
            }
        else if(!remoteToQuadData.pitchTrimSign && (remoteToQuadData.pitchTrimMagnitude == 0)){                 // if at zero but sign is positive but want to decrease
            remoteToQuadData.pitchTrimMagnitude = remoteToQuadData.pitchTrimMagnitude + TRIM_INCREMENT;
            remoteToQuadData.pitchTrimSign = 1;
            }
        else if ((!remoteToQuadData.pitchTrimSign) && (remoteToQuadData.pitchTrimMagnitude <= TRIM_INCREMENT)){ // if positive and going to zero
            remoteToQuadData.pitchTrimMagnitude = 0;
            remoteToQuadData.pitchTrimSign = 1;
            }
        else if (remoteToQuadData.pitchTrimSign && (remoteToQuadData.pitchTrimMagnitude < 255)){               // if negative and not maxed
            remoteToQuadData.pitchTrimMagnitude = remoteToQuadData.pitchTrimMagnitude + TRIM_INCREMENT;
            }
        updateDisplayTrimElementChange(); // Update LCD Screen
        delay(500);
        }
    if (digitalRead(BUTTON_DOWN_PIN)!=HIGH){ // increase neutral pitch angle
        if ((remoteToQuadData.pitchTrimSign) && (remoteToQuadData.pitchTrimMagnitude > TRIM_INCREMENT)){       // if negative and not going to zero
            remoteToQuadData.pitchTrimMagnitude = remoteToQuadData.pitchTrimMagnitude - TRIM_INCREMENT;
            }
        else if(remoteToQuadData.pitchTrimSign && (remoteToQuadData.pitchTrimMagnitude == 0)){                 // if at zero but sign is negative but want to decrease
            remoteToQuadData.pitchTrimMagnitude = remoteToQuadData.pitchTrimMagnitude + TRIM_INCREMENT;
            remoteToQuadData.pitchTrimSign = 0;
            }
        else if ((remoteToQuadData.pitchTrimSign) && (remoteToQuadData.pitchTrimMagnitude <= TRIM_INCREMENT)){ // if negative and going to zero
            remoteToQuadData.pitchTrimMagnitude = 0;
            remoteToQuadData.pitchTrimSign = 0;
            }
        else if (!remoteToQuadData.pitchTrimSign && (remoteToQuadData.pitchTrimMagnitude < 255)){              // if positive and not maxed
            remoteToQuadData.pitchTrimMagnitude = remoteToQuadData.pitchTrimMagnitude + TRIM_INCREMENT;
            }
        updateDisplayTrimElementChange(); // Update LCD Screen
        delay(500);
        }
    if (digitalRead(BUTTON_RIGHT_PIN)!=HIGH){ // increase neutral roll angle
        if ((remoteToQuadData.rollTrimSign) && (remoteToQuadData.rollTrimMagnitude > TRIM_INCREMENT)){          // if negative and not going to zero
            remoteToQuadData.rollTrimMagnitude = remoteToQuadData.rollTrimMagnitude - TRIM_INCREMENT;
            }
        else if(remoteToQuadData.rollTrimSign && (remoteToQuadData.rollTrimMagnitude == 0)){                    // if at zero but sign is negative but want to increase
          remoteToQuadData.rollTrimMagnitude = remoteToQuadData.rollTrimMagnitude + TRIM_INCREMENT;
            remoteToQuadData.rollTrimSign = 0;
            }
        else if ((remoteToQuadData.rollTrimSign) && (remoteToQuadData.rollTrimMagnitude <= TRIM_INCREMENT)){    // if negative and going to zero
            remoteToQuadData.rollTrimMagnitude = 0;
            remoteToQuadData.rollTrimSign = 0;
            }
        else if (!remoteToQuadData.rollTrimSign && (remoteToQuadData.rollTrimMagnitude < 255)){                 // if positive and not maxed
            remoteToQuadData.rollTrimMagnitude = remoteToQuadData.rollTrimMagnitude + TRIM_INCREMENT;
            }
        updateDisplayTrimElementChange(); // Update LCD Screen
        delay(500);
        }
    if (digitalRead(BUTTON_LEFT_PIN)!=HIGH){ // decrease neutral roll angle
        if ((!remoteToQuadData.rollTrimSign) && (remoteToQuadData.rollTrimMagnitude > TRIM_INCREMENT)){         // if positive and not going to zero
            remoteToQuadData.rollTrimMagnitude = remoteToQuadData.rollTrimMagnitude - TRIM_INCREMENT;
            }
        else if(!remoteToQuadData.rollTrimSign && (remoteToQuadData.rollTrimMagnitude == 0)){                   // if at zero but sign is positive but want to decrease
            remoteToQuadData.rollTrimMagnitude = remoteToQuadData.rollTrimMagnitude + TRIM_INCREMENT;
            remoteToQuadData.rollTrimSign = 1;
            }
        else if ((!remoteToQuadData.rollTrimSign) && (remoteToQuadData.rollTrimMagnitude <= TRIM_INCREMENT)){   // if positive and going to zero
            remoteToQuadData.rollTrimMagnitude = 0;
            remoteToQuadData.rollTrimSign = 1;
            }
        else if (remoteToQuadData.rollTrimSign && (remoteToQuadData.rollTrimMagnitude < 255)){                  // if negative and not maxed
            remoteToQuadData.rollTrimMagnitude = remoteToQuadData.rollTrimMagnitude + TRIM_INCREMENT;
            }
        updateDisplayTrimElementChange(); // Update LCD Screen
        delay(500);
        }
    if (digitalRead(BUTTON_CENTER_PIN)!=HIGH){     // save trim + PID values to EEPROM
        writePIDToEeprom();
        delay(1000);
        }
}

/*----------------------------
 * Section 6:
 * Function updateDisplayTrimElementChange
 * --------------------------*/

/**
* Updates the entire LCD screen to the tuning state.
* It should be called when an element changes on the LCD screen.
*/

void updateDisplayTrimElementChange(){
    calculateTuningValues(); // Calculate the tuning values displayed and sent to the quad (float) from the values saved to EEPROM (int)
    lcd.clear();
    lcd.print("TRIM:ROLL:");
    if(trimAngle[1] >= 0){       // print roll value if positive
        lcd.setCursor(11, 0);
        lcd.print(trimAngle[1]);
        }
    else{                        // print roll value if negative
        lcd.setCursor(10, 0);
        lcd.print(trimAngle[1]);
        }
    lcd.setCursor(0, 1);
    if(remoteToQuadData.arm == 1){ // Print whether quad is armed or not
        lcd.print("A   PITCH:");
        }
    else{
        lcd.print("    PITCH:");
        }
    if(trimAngle[0] >= 0){       // print pitch value if negative
        lcd.setCursor(11, 1);
        lcd.print(trimAngle[0]);
        }
    else{                        // print pitch value if negative
        lcd.setCursor(10, 1);
        lcd.print(trimAngle[0]);
        }
    }

/*----------------------------
 * Section 7:
 * Function calculateTuningValues
 * --------------------------*/

/**
* Calculates the tuning values displayed and sent to the quad (float) 
* from the values saved to EEPROM (int)
*/

void calculateTuningValues(){
    if(remoteToQuadData.pitchTrimSign){ // if pitch is negative
        trimAngle[0] = -float(remoteToQuadData.pitchTrimMagnitude) * (float(MAX_TRIM_ANGLE) / 255);
        }
    else{  // if piitch is positive
        trimAngle[0] = float(remoteToQuadData.pitchTrimMagnitude) * (float(MAX_TRIM_ANGLE) / 255);
        }
    if(remoteToQuadData.rollTrimSign){ // if roll is negative
        trimAngle[1] = -float(remoteToQuadData.rollTrimMagnitude) * (float(MAX_TRIM_ANGLE) / 255);
        }
    else{ // if roll is positive
        trimAngle[1] = float(remoteToQuadData.rollTrimMagnitude) * (float(MAX_TRIM_ANGLE) / 255);
        }
    }

/*----------------------------
 * Section 8:
 * Function readAndMapGimbals
 * --------------------------*/

/**
* Reads the analog gimbal positions and map to a value which may be sent over radio
*/

void readAndMapGimbals(){
    // [0] = A2 = pitch; [1] = A3 = roll; [2] = A1 = yaw; [3] = A0 = throt; 
    //A0 A2 A1 A3. throt at 0, pitch at 0. Also if throt low, roll to zero kills throt (and pitch)
    //A0 A2 A3 A1. throt at 0, pitch at 0. Also if throt low, yaw to zero kills throt (and pitch)
    gimbalRawValues[3]   = 1023 - analogRead(A0);// A0 ==  throt
    gimbalRawValues[0]   = 1023 - analogRead(A2);// A2 ==  pitch
    gimbalRawValues[1]   = 1023 - analogRead(A3);// A3 ==  roll
    gimbalRawValues[2]   = 1023 - analogRead(A1);// A1 ==  yaw  
    for(int i = 0; i < 4; i++){  // Ensure read value not outside of calibrated values
        if( gimbalRawValues[i] < lowestGimbalValueReadSoFar[i]){
            gimbalRawValues[i] = lowestGimbalValueReadSoFar[i];
            }
        if( gimbalRawValues[i] > largestGimbalValueReadSoFar[i] ){
            gimbalRawValues[i] = largestGimbalValueReadSoFar[i];
            }
        }
    // Map the raw gimbal values to unsigned 8 bit integer for saving to EEPROM
    remoteToQuadData.pitch  = map(gimbalRawValues[0], lowestGimbalValueReadSoFar[0], largestGimbalValueReadSoFar[0],   0, 255);
    remoteToQuadData.roll   = map(gimbalRawValues[1], lowestGimbalValueReadSoFar[1], largestGimbalValueReadSoFar[1],   0, 255);
    remoteToQuadData.yaw    = map(gimbalRawValues[2], lowestGimbalValueReadSoFar[2], largestGimbalValueReadSoFar[2],   0, 255); 
    remoteToQuadData.throt  = map(gimbalRawValues[3], lowestGimbalValueReadSoFar[3], largestGimbalValueReadSoFar[3],   0, 255);
    }

/*----------------------------
 * Section 9:
 * Function isInArmState
 * --------------------------*/

/**
* Arms the quadcopter if the gimbals are down and out
* @param  &d pointer to the the data structure containing throttle being sent via radio to the quadcopter
*/

void isInArmState(Data& d){
    if ((d.yaw <= 0) && (d.pitch >= 250) && (d.roll <= 0) && (d.throt <= 0 )){
        d.arm = 1; 
        }
    }

/*----------------------------
 * Section 10:
 * Function calibrate
 * --------------------------*/

/**
* Calibrates the gimbals and call function writeGimbalsToEeprom to save the new values.
* Scripted and hardcoded out of necessity.
*/

void calibrate() {
    lcd.clear();
    lcd.print("calibrating...  ");
    delay(2000);
    lcd.clear();
    long timecalstart = millis();
    lcd.print("wiggle  all da  gimbals         ");
    while (   (millis()-timecalstart)<10000)    { // user has 10 seconds to wiggle gimbals
        gimbalRawValues[3]   = 1023 - analogRead(A0);// A0 ==  throt
        gimbalRawValues[0]   = 1023 - analogRead(A2);// A2 ==  pitch
        gimbalRawValues[1]   = 1023 - analogRead(A3);// A3 ==  roll
        gimbalRawValues[2]   = 1023 - analogRead(A1);// A1 ==  yaw
        for(int i = 0; i < 4; i++){
            if( gimbalRawValues[i] < lowestGimbalValueReadSoFar[i]){
                lowestGimbalValueReadSoFar[i]  =  gimbalRawValues[i];
                }
            if( gimbalRawValues[i] > largestGimbalValueReadSoFar[i] ){
                largestGimbalValueReadSoFar[i] = gimbalRawValues[i];
                }
            }
        }
    lcd.clear();
    lcd.print("exiting         calibration mode");
    delay(2000);
    lcd.clear();
    writeGimbalsToEeprom();          // Save the newly calibrated gimbal values
    if (isInTrimMode){               // Determine which state to exit to, and update LCD screen accordingly
        updateDisplayTrimElementChange();
        }
    else{
        updateDisplayPIDElementChange();
        }
    }

/*----------------------------
 * Section 11:
 * Function calculateColPos
 * --------------------------*/

/**
* Determines which cpa array column value we wish to edit
* @param pidPos 0->2: Pitch, Roll, Yaw
* @param tuningWholeInteger Whether we are tuning an integer or decimal
*/

int calculateColPos(int pidPos, bool tuningWholeInteger ){
    if( tuningWholeInteger ){ // if we are tuning a whole integer
        col = pidPos*2 + 0;
        }
    else{
        col = pidPos*2 + 1;  // if we are tuning the decimals place
        }
        return col;
    }

/*------------------------------------
 * Section 12:
 * Function updateDisplayPIDElementChange
 * ---------------------------------*/

/**
* Updates the entire LCD screen to the PID state.
* It should be called when an element changes on the LCD screen.
* For example: arming the controller or changing which pitch, roll, or yaw PID value is being tuned.
*/

void updateDisplayPIDElementChange(){
    int left, right;
    if( col % 2 == 0){                                  // if tuning a whole integer
        left  = cpa[row][col];
        right = cpa[row][col+1];
        }
    else{                                               // if tuning a decimal
        left  = cpa[row][col-1];
        right = cpa[row][col]; 
        }
    lcd.clear();
    if(changeLeft && remoteToQuadData.arm == 1){        // if armed and editing ones integer
        top = "P.R.Y__A   P.I.D";
        }
    else if(!changeLeft && remoteToQuadData.arm == 1) { // if armed and editing decimal
        top = "P.R.Y  A___P.I.D";
        }
    else if(changeLeft && remoteToQuadData.arm == 0){   // if disarmed and editing ones integer
        top = "P.R.Y__    P.I.D";
        }
    else{                                               // if disarmed and editing decimal
        top = "P.R.Y   ___P.I.D";
        }
    lcd.print(top);
    // Set the right carrot (^) to point at the value we are editing 
    if(     row == 0 ){                    // Pitch
        colPosStart[0] = 0;
        lcd.setCursor(colPosStart[0], 1);
        }
    else if(row == 1){                     // Roll
        colPosStart[0] = 2;
        lcd.setCursor(colPosStart[0], 1);
        }
    else{                                  // Yaw
        colPosStart[0] = 4;
        lcd.setCursor(colPosStart[0], 1);
        }
    lcd.print("^");
    // Determine if negative sign should be printed
    if( left < 0 && right >=0){ // if only the integer is negative, the number is negative
        left = left*(-1); 
        lcd.setCursor(3, 1);
        lcd.print("-");
        }
    else if( left >= 0 && right <0 ){ // if only decimal is negative, the number is negative
        right = right*(-1);
        lcd.setCursor(3, 1);
        lcd.print("-");
        }
    else if( left < 0 && right   ){ // if both are negative, then the number is negative
        left =  left*(-1);
        right = right*(-1);
        lcd.setCursor(3, 1);
        lcd.print("-"); 
        }
    //Print the PID tuning number
    if(left < 10){ // if integer needs 1 space to print
        colPosStart[1] = 6;
        lcd.setCursor(colPosStart[1], 1);
        }
    else{ // if integer needs 2 spaces to print
        colPosStart[1] = 5;
        lcd.setCursor(colPosStart[1], 1);
        }
    lcd.print(left); // Print the integet
    lcd.print(".");
    if( right < 10 ){                     // Print 2 decimal leading zeros
        lcd.print("00");
        }
    else if( right >= 10 && right < 100 ){// Print 1 decimal leading zero
        lcd.print("0");
        }
    lcd.print(right); // Print the decimal
     // Set the right carrot (^) to point at the control value we are editing 
    if(     PIDpos == 0 ){              // Proportional
        colPosStart[2] = 11;
        lcd.setCursor(colPosStart[2], 1);
        }
    else if(PIDpos == 1){               // Integral
        colPosStart[2] = 13;
        lcd.setCursor(colPosStart[2], 1);
        }
    else{                               // Derivative
        colPosStart[2] = 15;
        lcd.setCursor(colPosStart[2], 1);
        }
    lcd.print("^");
    }

/*------------------------------------
 * Section 13:
 * Function updateDisplayPIDNumChange
 * ---------------------------------*/

/**
* Updates the numbers being displayed currently on the LCD screen.
* It should be called when a number changes.
*/

void updateDisplayPIDNumChange(){
    int left, right;
    if( col % 2 == 0){ // if tuning a whole integer
        left  = cpa[row][col];
        right = cpa[row][col+1];
        }
    else{             // if tuning a decimal place
        left  = cpa[row][col-1];
        right = cpa[row][col]; 
        }
    if(left >= 0 && right >= 0){ // if both positive
        lcd.clear();
        lcd.print(top);
        lcd.setCursor(colPosStart[0], 1);
        lcd.print("^");
        lcd.setCursor(colPosStart[1], 1);
        lcd.print(left);
        lcd.print(".");
        lcd.setCursor(colPosStart[1]+2, 1);
        if( right < 10 ){                     // Print 2 decimal leading zeros
            lcd.print("00");
            }
        else if( right >= 10 && right < 100){ // Print 1 decimal leading zero
            lcd.print("0");
            }
        lcd.print(right);
        lcd.setCursor(colPosStart[2], 1);
        lcd.print("^");
        }
    else{ // if one or more value is negative, print a negative sign
        if(left < 0 ){ //EEPROM will not save negative numbers. Sign still saved in float value.
          left = left*-1;
          }
        if(right < 0 ){ //EEPROM will not save negative numbers. Sign still saved in float value.
            right = right*-1;
            }
        lcd.clear();
        lcd.print(top);
        lcd.setCursor(colPosStart[0], 1);
        lcd.print("^");
        lcd.setCursor(3, 1);
        lcd.print("-");
        lcd.setCursor(colPosStart[1], 1);
        lcd.print(left);
        lcd.print(".");
        lcd.setCursor(colPosStart[1]+2, 1);
        if( right < 10 ){                     // Print 2 decimal leading zero
            lcd.print("00");
            }
        else if( right >= 10 && right < 100){ // Print 1 decimal leading zero
            lcd.print("0");
            }
        lcd.print(right);
        lcd.setCursor(colPosStart[2], 1);
        lcd.print("^");
        }
    }

/*------------------------------------
 * Section 14:
 * Function convertToDecimalBeforeSend
 * ---------------------------------*/

/**
* Takes the PID user input as displayed on the LCD (as unsigned integers) and saves it in the structure "remoteToQuadData" (as floats), 
* which will be sent over radio to the quadcopter. It also stores the sign of the PID values for EEPROM saving
*/

void converToDecimalsBeforeSend(){
    for (int i=0; i<3; i++){ // pitch, yaw, roll
        remoteToQuadData.coefficientsContainer[i].proportional = castIntPairToDecimal(cpa[i][0], cpa[i][1]); // save P val as float
        if(  remoteToQuadData.coefficientsContainer[i].proportional < 0){ // save sign
            sign[3*i] = 0;
            }
        else{
            sign[3*i] = 1;
            }
        remoteToQuadData.coefficientsContainer[i].integral     = castIntPairToDecimal(cpa[i][2], cpa[i][3]); // save I val as float
        if(  remoteToQuadData.coefficientsContainer[i].integral < 0){ // save sign
            sign[3*i+1] = 0;
            }
        else{
            sign[3*i+1] = 1;
            }
        remoteToQuadData.coefficientsContainer[i].derivative   = castIntPairToDecimal(cpa[i][4], cpa[i][5]); // save D val as float
        if(  remoteToQuadData.coefficientsContainer[i].derivative < 0){ // save sign
            sign[3*i+2] = 0;
            }
        else{
            sign[3*i+2] = 1;
            }
        }
    }

/*------------------------------------
 * Section 15:
 * Function castIntPairToDecimal
 * ---------------------------------*/

    /**
    * Takes the separated PID integers displayed on the LCD 
    * and combines them into one float value
    * @param left_int integer in the ones place on the LCD screen
    * @param right_int integer in the decimal place on the LCD screen
    * @return castedPIDValue float value which may be used in computation
    */

float castIntPairToDecimal( int left_int, int right_int){
    float castedDecimal = (float)right_int;
    float denominator = 1000.000; // allows after decimal to be precise up to the thousands place.
    float castedPIDValue;
    if( left_int > 0 ){ // if ones place positive, the number is positve
        castedPIDValue = ((float) left_int + (castedDecimal/denominator));  // ex. +01.100 or +1.230
        }
    else if(left_int < 0){ // if ones place negative, the number is negative
        castedPIDValue = ( (float) left_int - (castedDecimal/denominator)); // ex. -01.100 or -1.230
        }
    else{ // if ones place zero, the number takes sign from decimals place
        castedPIDValue =  castedDecimal/denominator;                        // ex.  00.011 or -0.123
        }
    return castedPIDValue;
    }

/*------------------------------------
 * Section 16:
 * Function knobPressed
 * ---------------------------------*/

/**
* Resets the current displayed PID or trim values
* @param  down the boolean which stores whether the knob is pressed
*/

void knobPressed(bool down) {
    if((down) && (!isInTrimMode)) {             // if editing PID values
        cpa[row][col] = knob1.setCurrentPos(0); // reset the quantity to zero
        updateDisplayPIDNumChange();            // reflect update on screen
        }
    else if (down){                        // if editing trim values
        remoteToQuadData.pitchTrimMagnitude = knob1.setCurrentPos(0); // reset the quantity to zero
        remoteToQuadData.rollTrimMagnitude = 0;
        remoteToQuadData.pitchTrimSign = 0; // positive
        remoteToQuadData.rollTrimSign = 0; // positive
        }
    }

/*------------------------------------
 * Section 17:
 * Function knobsUpdate
 * ---------------------------------*/

/**
* Updates the stored knob value so that it agrees with the stored PID value displayed on the LCD screen
*/

void knobsUpdate(){
    if(!isInTrimMode){
        if(knob_token){ // update to stored PID value if the screen is changed
            knob1.setCurrentPos( cpa[row][col]);
            knob_token = !knob_token; // get rid of the token so we may change the shown value
            }
        cpa[row][col] = knob1.getCurrentPos();  // update that value in the array as the knob is physically adjusted.
        updateDisplayPIDNumChange();            // reflect update on the screen.  Accounts for sign change as well.
        }
    }

/*------------------------------------
 * Section 18:
 * Function eepromRead
 * ---------------------------------*/

/**
* Read the values stored in EEPROM and store in a data structure
*/

void eepromRead(){
    int AddressNumForEEPROM = 0;
    for(int index = 0; index < 4; index++){ // 0->3: Lowest calibrated gimbal values
        lowestGimbalValueReadSoFar[index] = EEPROM.read(AddressNumForEEPROM);
        AddressNumForEEPROM = AddressNumForEEPROM + 1;
        }  
    for(int index = 0; index < 4; index++){ // 4->15: Highest calibrated gimbal values
        largestGimbalValueReadSoFar[index] =   EEPROM.read(AddressNumForEEPROM)
                                             + EEPROM.read(AddressNumForEEPROM+1)
                                             + EEPROM.read(AddressNumForEEPROM+2);
        AddressNumForEEPROM = AddressNumForEEPROM + 3;
        }  
    readPIDArrayFromEeprom(AddressNumForEEPROM);  // Read in all the correct values so the struct can load up correctly.
    for(int i =0; i < 9; i++){ // update negative sign values from 0 -> -1
        if( sign[i] == 0 ){
            sign[i] = -1;
            }
        }
    // Update Pitch PID floats from read EEPROM values.
    remoteToQuadData.coefficientsContainer[0].proportional = sign[0] * castIntPairToDecimal(cpa[0][0], cpa[0][1]);
    if( sign[0] == -1 ){cpa[0][0] = -1*cpa[0][0];}
    remoteToQuadData.coefficientsContainer[0].integral     = sign[1] * castIntPairToDecimal(cpa[0][2], cpa[0][3]);
    if( sign[1] == -1 ){cpa[0][2] = -1*cpa[0][2];}
    remoteToQuadData.coefficientsContainer[0].derivative   = sign[2] * castIntPairToDecimal(cpa[0][4], cpa[0][5]);
    if( sign[2] == -1 ){cpa[0][4] = -1*cpa[0][4];}
    // Update Roll PID floats from read EEPROM values.
    remoteToQuadData.coefficientsContainer[1].proportional = sign[3] * castIntPairToDecimal(cpa[1][0], cpa[1][1]);
    if( sign[3] == -1 ){cpa[1][0] = -1*cpa[1][0];}
    remoteToQuadData.coefficientsContainer[1].integral     = sign[4] * castIntPairToDecimal(cpa[1][2], cpa[1][3]);
    if( sign[4] == -1 ){cpa[1][2] = -1*cpa[1][2];}
    remoteToQuadData.coefficientsContainer[1].derivative   = sign[5] * castIntPairToDecimal(cpa[1][4], cpa[1][5]);
    if( sign[5] == -1 ){cpa[1][4] = -1*cpa[1][4];}
    // Update Yaw PID floats from read EEPROM values.
    remoteToQuadData.coefficientsContainer[2].proportional = sign[6] * castIntPairToDecimal(cpa[2][0], cpa[2][1]);
    if( sign[6] == -1 ){cpa[2][0] = -1*cpa[2][0];}
    remoteToQuadData.coefficientsContainer[2].integral     = sign[7] * castIntPairToDecimal(cpa[2][2], cpa[2][3]);
    if( sign[7] == -1 ){cpa[2][2] = -1*cpa[2][2];}
    remoteToQuadData.coefficientsContainer[2].derivative   = sign[8] * castIntPairToDecimal(cpa[2][4], cpa[2][5]);
    if( sign[8] == -1 ){cpa[2][4] = -1*cpa[2][4];}
    }

/*------------------------------------
 * Section 19:
 * Function readPIDArrayFromEeprom
 * ---------------------------------*/

/**
* Reads the PID values stored in EEPROM
* @param  currentEEPROMAdressNumber: necessary to read correct data
*/

void readPIDArrayFromEeprom(int currentEEPROMAddressNumber){
    for(int r = 0;  r < 3; r++){
        for(int c = 0; c < 6; c++){ // Address: 16 -> 33
            cpa[r][c] = EEPROM.read(currentEEPROMAddressNumber) + EEPROM.read(currentEEPROMAddressNumber + 1) + 
            EEPROM.read(currentEEPROMAddressNumber + 2) + EEPROM.read(currentEEPROMAddressNumber + 3);
            currentEEPROMAddressNumber = currentEEPROMAddressNumber + 4;
            }
        }  
    for(int i = 0; i < 9;i++){      // Address: 34 -> 42
        sign[i] = EEPROM.read(currentEEPROMAddressNumber);
        currentEEPROMAddressNumber = currentEEPROMAddressNumber + 1;
        }
    readTrimFromEeprom(currentEEPROMAddressNumber);
    }

/*------------------------------------
 * Section 20:
 * Function readTrimFromEeprom
 * ---------------------------------*/

/**
* Reads the trim values stored in EEPROM
* @param EepromAddress eeprom address to read correct data (should be at 43)
*/

void readTrimFromEeprom(int EepromAddress){
    remoteToQuadData.pitchTrimSign = EEPROM.read(EepromAddress);
    remoteToQuadData.pitchTrimMagnitude = EEPROM.read(EepromAddress+1);
    remoteToQuadData.rollTrimSign = EEPROM.read(EepromAddress+2);
    remoteToQuadData.rollTrimMagnitude = EEPROM.read(EepromAddress+3);
    }

/*------------------------------------
 * Section 21:
 * Function writeGimbalsToEeprom
 * ---------------------------------*/

/**
* Saves the calibrated gimbal values to the microcontroller's EEPROM
*/

void writeGimbalsToEeprom(){
    int AddressNumForEEPROM = 0; // first address where values are stored
    for(int index = 0; index < 4; index++){ // 0 -> 3
        EEPROM.write(AddressNumForEEPROM, lowestGimbalValueReadSoFar[index]);
        AddressNumForEEPROM = AddressNumForEEPROM + 1;
        }
    int x;
    for(int index = 0; index < 4; index++){ // 4 -> 15
        x = largestGimbalValueReadSoFar[index];
        if( x > 255 ){
            EEPROM.write(AddressNumForEEPROM, 255 );
            AddressNumForEEPROM = AddressNumForEEPROM+1;
            if( x > 510){
                EEPROM.write(AddressNumForEEPROM, 255);
                AddressNumForEEPROM = AddressNumForEEPROM + 1;
                EEPROM.write(AddressNumForEEPROM, x-510);
                AddressNumForEEPROM = AddressNumForEEPROM + 1;  
                }
            else{
                EEPROM.write(AddressNumForEEPROM, x - 255);
                AddressNumForEEPROM = AddressNumForEEPROM + 1;
                EEPROM.write(AddressNumForEEPROM, 0);
                AddressNumForEEPROM = AddressNumForEEPROM + 1;  
                }
            }
        else{
            EEPROM.write(AddressNumForEEPROM, x);
            AddressNumForEEPROM = AddressNumForEEPROM + 1;
            EEPROM.write(AddressNumForEEPROM, 0);
            AddressNumForEEPROM = AddressNumForEEPROM +1 ;
            EEPROM.write(AddressNumForEEPROM, 0);
            AddressNumForEEPROM = AddressNumForEEPROM + 1;
            }    
        }
    }

/*------------------------------------
 * Section 22:
 * Function writePIDToEeprom
 * ---------------------------------*/

/**
* Saves the PID and trim coefficients to the microcontroller's EEPROM
*/

void writePIDToEeprom(){
    int AddressNumForEEPROM = 16; // first address at which values may be stored
    float storeValue = 0.00;
    for(int r = 0;  r < 3; r++){ // Pitch, Yaw, Roll
        for(int c = 0; c < 6; c++){
            storeValue = cpa[r][c];
            timesOverflowed = 0;
            if( storeValue < 0 ){
                storeValue = (-1 * storeValue); // need to be positive number from 0-255.
                }
            while (storeValue > 255){
                timesOverflowed = timesOverflowed + 1;
                storeValue = storeValue - 255;
                }
            if (timesOverflowed > 2){
                EEPROM.write(AddressNumForEEPROM, 255);
                EEPROM.write(AddressNumForEEPROM + 1, 255);
                EEPROM.write(AddressNumForEEPROM + 2, 255);
                EEPROM.write(AddressNumForEEPROM + 3, storeValue);
                }
            else if (timesOverflowed > 1){
                EEPROM.write(AddressNumForEEPROM, 255);
                EEPROM.write(AddressNumForEEPROM + 1, 255);
                EEPROM.write(AddressNumForEEPROM + 2, 0);
                EEPROM.write(AddressNumForEEPROM + 3, storeValue);
                }
            else if (timesOverflowed > 0){
                EEPROM.write(AddressNumForEEPROM, 0);
                EEPROM.write(AddressNumForEEPROM + 1, 0);
                EEPROM.write(AddressNumForEEPROM + 2, 255);
                EEPROM.write(AddressNumForEEPROM + 3, storeValue);
            }
            else{
            EEPROM.write(AddressNumForEEPROM, 0);
            EEPROM.write(AddressNumForEEPROM + 1, 0);
            EEPROM.write(AddressNumForEEPROM + 2, 0);
            EEPROM.write(AddressNumForEEPROM + 3, storeValue);
            }
            AddressNumForEEPROM = AddressNumForEEPROM + 4;
            }
        } 
    int s = 0;
    for(int i = 0; i < 9;i++){
        s = sign[i];
        if(s == -1){
            s = 0;
            }
        EEPROM.write(AddressNumForEEPROM, s);
        AddressNumForEEPROM = AddressNumForEEPROM + 1;
        }
    EEPROM.write(AddressNumForEEPROM,remoteToQuadData.pitchTrimSign);
    EEPROM.write(AddressNumForEEPROM + 1, remoteToQuadData.pitchTrimMagnitude);
    EEPROM.write(AddressNumForEEPROM + 2, remoteToQuadData.rollTrimSign);
    EEPROM.write(AddressNumForEEPROM + 3, remoteToQuadData.rollTrimMagnitude);
    }

/*---------------------------------------
 * Section 23:
 * Function readCoefficientValuesAndSigns
 * ------------------------------------*/

/**
* Resets all PID coefficients.
*/

void resetCoefficientValuesAndSigns(){
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 6; j++){
            cpa[i][j] = 0; // reset all PID values
            }
        }
    for(int i =0; i < 9; i++){
        sign[i] = 1; // reset signs to positive
        }
    }

/*---------------------------------------
 * Section 24:
 * Function initializeButtonPins
 * ------------------------------------*/

/**
* Initializes all buttons on the remote
*/

void initializeButtonPins(){
    pinMode(BUTTON_UP_PIN, INPUT);
    pinMode(BUTTON_DOWN_PIN, INPUT);
    pinMode(BUTTON_LEFT_PIN, INPUT);
    pinMode(BUTTON_RIGHT_PIN, INPUT);
    pinMode(BUTTON_CENTER_PIN, INPUT);
    pinMode(BUTTON1_PIN, INPUT);
    pinMode(BUTTON2_PIN, INPUT);  
    }
