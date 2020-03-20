//TO DO: Add info for gate pins, etc.  Fix Xing signals.  Other fixes from RRXing

//Two sensors at ends of a block are checked to define a block and train direction
//Crossing signal and gate can be triggered for each block

//JMRI Sensors are set to indicate block occupancy, sensor detection, and direction
//Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
//Code on Github may be modified or withdraw at any time
//Copyright (c) 2019, 2020 Jerrold M. Grochow

//v4 Working version
//v5 Changed to bool for CMRI and related variables
//v6 Added Block Reset  2019-07-27
//v7 Added Crossing signals, removed red-green signals 2019-10-07
//v7b Added full reset if JMRI bit 47 toggled  2019-10-27
//v8 Added back red-green signals option 2019-11-21
//v9 Added optional crossing gate  2020-01-27

//JMRI: set up C/MRI node as SMINI to handle up to 3 blocks; set up as SUSIC to handle more blocks (limited by size of arrays in arduino code)
//JMRI: set up following Sensors and Lights to manage three blocks:
// [Note: sensors are named Entry and Exit and shown in diagrams as left and right for convenience]
// [Note: JMRI starts numbering C/MRI bits with 1, so C/MRI bit 0 in Arduino code is bit 1 in JMRI table]
//   Block occupancy*         (CMRI input bit 0,  8, 16)(JMRI sensor 1,  9, 17)
//   Entry sensor status      (CMRI input bit 1,  9, 17)(JMRI sensor 2, 10, 18)
//   Exit sensor stats        (CMRI input bit 2, 10, 18)(JMRI sensor 3, 11, 19)
//   Counterclockwise travel* (CMRI input bit 3, 11, 19)(JMRI sensor 4, 12, 20)
//   Clockwise travel*        (CMRI input bit 4, 12, 20)(JMRI sensor 5, 13, 21)
//   Block reset              (CMRI output bit 0, 8, 16)(JMRI light 1, 9, 17)
// * = determined by arduino code;  Entry and Exit sensor states are from the actual sensors
//[Note: Code assumes bit # increments by 8 for each subsequent block. Arrays are set up to allow for five blocks.
//[JMRI would have to be set up to allow at least 36 input bits - NOT YET TESTED BEYOND 3 BLOCKS.]
//C/MRI output bit 47 from JMRI tells arduino to go active
//   Setup a JMRI Light associated with CMRI output address 48
//C/MRI input bit 23 from arduino tells JMRI it is active
//   Setup a JMRI Sensor associated with CMRI input address 24

//Arduino code maintains a single crossing signal (alternating lights) for each block, not currently linked to JMRI


#include <CMRI.h>                //Simulate CMRI node
#include <Auto485.h>             //Provide for RS485 communication
#include <VarSpeedServo.h>       //Allow servo motor control of crossing gates

const int CMRI_ADDR = 5;         //CMRI Board #5
const int DE_PIN = 2;            //Arduino pin 2 for RS485 communication


//Constants:
const bool sensorActive = true;
const bool sensorInactive = false;

const unsigned long int sensorWait   = 1000;  //Wait between sensor reads
const unsigned long int idleWait     = 10000; //FUTURE USE
const unsigned long int shortWait    = 20;    //After sensor reads
const unsigned long int crossingWait = 400;   //Time for blink of crossing signal
const int              analogTrigger = 325;   //Analog sensor trigger value


//Physical setup  [CHANGE THIS INFORMATION FOR YOUR LAYOUT]
uint8_t   servoSpeed = 18;                   //For VarSpeedServo
int       gateDown   = 30;                   //Servo settings on my layout
int       gateUp     = 77;
const int numBlocks  = 3;                    //MAX = 3 for SMINI; 5 based on size of arrays in code (requires emulating larger CMRI board)
int       numSS      = numBlocks * 2;        //MAX = 10 based on size of arrays

//  Definitions for following variables:
//   sensorPin = arduino pin number for each sensor [Nano pins A1-A5 can be used as digital or analog pins;  A6-A7 are exclusively analog
//   sensorAnalog = true if analog sensor, false if digital sensor output
//   sensorLowActive = 1 if sensor active when LOW; 0= sensor active when HIGH
//    Digital IR Detector Module: for break-beam, set sensorLowActive to 0;  for reflector, set to 1
//    Analog IR Detector Module: for break-beam, set to 1; for reflector, set to 0
//    Analog IR Trans/Recvr: for break-beam, set to 1; for reflector, set to 0 (reverse if receiver output voltage reversed)
//    Digital Photoresistor Module: set to 0 (beam-break)
int  sensorPin []      = {   A5,    A2,    A2,    A4,    A4,    A1,     0,     0,     0,     0};
bool sensorAnalog []   = {false, false, false, false, false,  true, false, false, false, false};
int  sensorLowActive[] = {    1,     0,     0,     0,     0,     1,    -1,    -1,    -1,    -1};
//My layout sensorPin assignment; Exit sensor of Block n is also Entry sensor of Block n+1 (three adjacent blocks)
//Note: Program allows all sensors to be different and blocks to be non-continguous.
//  [A1, true,1]: Analog IR trans/recvr /Beam-break on arduino pin A1
//  [A2,false,0]: Digital IR Detector Module - Beam-break mode
//  [A4,false,0]: Digital Photoresistor Module
//  [A5,false,1]: Digital IR Detector Module - Reflector mode
//  [xx, true,1]: Analog IR/Beam-break transmitter/receiver
//  [xx, true,0]; Analog IR/Beam-Break transmitter/reverse-wired receiver

int lightPin[]       = {6, 7, 8, 9, 10, 11, 0, 0, 0, 0};
int gatePin[]        = {0, 0, 5, 0, 0};                   //Servo requires PWM pin
int lightType []     = {1, 2, 3, 0, 0};                   //0= red-green;  1= crossing;  2= crossing+gate
int lightLowActive[] = {0, 0, 1, 1,  0,  0, 0, 0, 0, 0};  //1=LOW to turn on; 0=HIGH to turn on
//  Crossing signals trackside (not linked to JMRI)

//*************** State and output variables
bool prevSensorState [] = {sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive}; //Initialize state of each sensor
bool curSensorState []  = {sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive};
int comboSensorState [] = { -1, -1, -1, -1, -1};          //Number representing combined current and previous states at a point in time
bool prevBlockOcc []    = {false, false, false, false, false};
bool blockOcc []        = {false, false, false, false, false};

enum direction {CW, CCW, INDET, UNK};                     //ClockWise, CounterClockWise, INDETerminate, UNKnown
direction blockDir []     = {UNK, UNK, UNK, UNK, UNK};    //Train direction determined in each block
direction prevBlockDir [] = {UNK, UNK, UNK, UNK, UNK};    //Previuosly determined train direction in each block

//************* Internal control variables
bool sensorChange []    = {false, false, false, false, false, false, false, false, false, false};
bool anySensorChange    = false;
int crossingSignalAlt[] = { -1, -1, -1, -1, -1};           //Switch to alternate crossing signals

//************ Time variables
unsigned long int sensorReadTime[]     = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   //Time that sensor last changed state
unsigned long int sensorDelayTime[]    = {0, 0, 0, 0, 0};  //Time to read sensor next for each block
unsigned long int crossingSignalTime[] = {0, 0, 0, 0, 0};  //Time to alternate crossing signal
unsigned long int crossingGateTime[]   = {0, 0, 0, 0, 0};  //Time for crossing to stay down (minimum)
unsigned long int blockIdleTime[]      = {0, 0, 0, 0, 0};  //How long since block last changed state

//************ Other state variables
bool JMRIPanelAvail = false;
unsigned long int curTime = 0;
bool powerUp = true;       //Need to knowfirst time sensors are activated for certain state change determination

int loopCount = 0;     //DEBUG

//================== Initialize libraries
VarSpeedServo servo0;              //Set up one servo for each crossing gate
Auto485 bus(DE_PIN);
CMRI cmri(CMRI_ADDR, 24, 48, bus); //SMINI = 24 inputs, 48 outputs [WILL NEED MORE INPUTS TO CONTROL 5 BLOCKS]
//NOTE: Set up JMRI to simulate SUSIC rather than SMINI to allow more inputs


//========================================================================================
void setup() {

  //*****************  SKETCH NAME  *****************************************************
  //                                                                                    *
  bus.begin(19200, SERIAL_8N2);          //MAKE SURE JMRI CONNECTION SET TO THIS SPEED  *
  bus.println("Setup complete: CMRI485_2Sen_Sig_v9 2020-01-27 2200");                 // *
  //                                                                                    *
  //*************************************************************************************

  //Define pins for this sketch
  pinMode(LED_BUILTIN, OUTPUT);

  int j;
  for (int i = 0; i < numSS; i++) {
    j = sensorPin[i];
    if (!sensorAnalog[i]) pinMode(j, INPUT_PULLUP);  //DO NOT INITIALIZE ANALOG PINS USED FOR ANALOG ONLY
  }
  for (int i = 0; i < numSS; i++)  {
    j = lightPin[i];
    pinMode(j, OUTPUT);   digitalWrite(j, HIGH ^ lightLowActive[i]); //Signal lights ON to test
    delay (2000);
    pinMode(j, OUTPUT);   digitalWrite(j, LOW ^ lightLowActive[i]); //Signal lights OFF
  }
  delay(shortWait);


  //Flash LED that board is ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1200);
  digitalWrite(LED_BUILTIN, LOW);

  curTime = millis();
  for (int i = 0; i < numBlocks; i++)   {         //Set timers to current time so no delay
    sensorDelayTime[i] = curTime - 1;             //Allows for < comparison in loop
    blockIdleTime[i] = curTime - 1 + idleWait;
  }

}


//=======================================================================================
void loop() {

  curTime = millis();

  //================ If JMRI panel not open, then return ===========

  //  loopCount = loopCount + 1;              //DEBUG
  //  bus.println ("Beg: " + String(curTime) + " " + String(loopCount));

  cmri.process();
  CheckJMRI();
  if (!JMRIPanelAvail) return;

  //========================= Cycle thru all listed blocks =======================
  //===== NOTE: all settings of sensor CMRI bits done in mainline;  determination of status and outputs done in subroutines ====

  anySensorChange = false;                       //Reset
  for (int i = 0; i < numBlocks; i++)   {
    int ii = i * 2;                              //Set up array indices for sensor pair for this block
    int ij = ii + 1;

    //==================== Skip to BLOCK RESET if Light set in JMRI ================================
    if (!cmri.get_bit(8 * i))    {

      //================== Get state of sensors  and set in JMRI ================================

      sensorChange [ii] = false;
      sensorChange [ij] = false;
      if (sensorDelayTime[i] < curTime) {

        //Entry sensor
        prevSensorState[ii] = curSensorState[ii];     //Save old sensor state
        sensorChange [ii] = SensorState(ii, i);       //Call subroutne
        //NOTE: sensorChange used in DEBUG only
        //      bus.println(String(millis()) + " SensorState: #"  + String(ii) + " " + String(curSensorState[ii]) + String(sensorChange[ii]));
        cmri.set_bit(1 + 8 * i, curSensorState[ii]);  //Tell JMRI sensor state

        //Exit sensor
        prevSensorState[ij] = curSensorState[ij];     //Save old sensor state
        sensorChange [ij] = SensorState(ij, i);       //Call subroutine
        //NOTE: sensorChange used in DEBUG only
        //      bus.println(String(millis()) + " SensorState: #"  + String(ij) + " " + String(curSensorState[ij]) + String(sensorChange[ij]));
        cmri.set_bit(2 + 8 * i, curSensorState[ij]);  //Tell JMRI sensor state

        //========================= Create combined sensor state number ======================================
        //Create single number from current and previous sensor states to simplify comparisons in later code, regardless of whether sensor just read or not
        comboSensorState[i] = (int) curSensorState[ii] * 8 + (int) curSensorState[ij] * 4 + (int) prevSensorState[ii] * 2 + (int) prevSensorState[ij];
        //Example: Entry sensor Active, exit sensor Inactive, previous entry sensor Active, previous exit sensor Inactive
        //         AIAI = 1010 binary = 10 decimal
        // Used in subroutines to determine block occ and train direction

        //======================== Get block occupancy, train direction status and set in JMRI ==============================
        //Redetermine block occ and train dir because combo state changes every time a sensor is read (even if sensor hasn't changed)

        prevBlockOcc [i] = blockOcc[i];               //Save current values
        prevBlockDir [i] = blockDir[i];

        blockOcc [i] = BlockOccState(i);              //Call subroutine to determine block occupancy
        blockDir [i] = BlockDir(i);                   //Call subroutine to determine train direction traversing block

        cmri.set_bit(0 + 8 * i, blockOcc[i]);         //Report block occupancy

        switch (blockDir[i]) {                        //Report direction of train travel
          case CCW:
            cmri.set_bit(3 + 8 * i, 1);               //Counterclockwise
            cmri.set_bit(4 + 8 * i, 0);
            break;
          case CW:
            cmri.set_bit(3 + 8 * i, 0);               //Clockwise
            cmri.set_bit(4 + 8 * i, 1);
            break;
          case INDET:
            cmri.set_bit(3 + 8 * i, 1);               //Indeterminate
            cmri.set_bit(4 + 8 * i, 1);
            break;
          default:
            cmri.set_bit(3 + 8 * i, 0);               //Unknown
            cmri.set_bit(4 + 8 * i, 0);
        }

      }
    }                  //End of processing every time a sensor is read

    //====================== Process if BLOCK RESET sent from JMRI ============================
    else {
      BlockReset (i, ii, ij);
    }

    //==================== Turn on signal lights, crossing signal, gate if block occupied ===========================
    switch (lightType[i])  {
      case 1:
        SignalLights(i, ii, ij);
        break;
      case 2:
        CrossingSignal(i, ii, ij);
        break;
      case 3:
        CrossingSignal(i, ii, ij);
        CrossingGate(i, ii, ij);
        break;
      default:
        break;
    }


    //DEBUG
    //    bus.println ("End: " + String(curTime) + " " + String(i) + " " + String(comboSensorState[i]) + " " + String(prevBlockOcc[i]) + String(blockOcc[i]) + " " + String(prevBlockDir[i]) + String(blockDir[i])
    //             + " PU SC1 SC2 SCT BI: " + String(powerUp) + " " + String(sensorChange[ii]) + " " + String(sensorChange[ij]) + " " +  String(sensorDelayTime[i]) + " " + String(blockIdleTime[i]));

    //======================== End of Block Processing ==================================================
  }

  //DEBUG
  //  bus.println();

  //Set powerUp to false after first sensor change after first run through
  if (anySensorChange) powerUp = false;

}
//==================== END OF LOOP =======================================================



//=**==**==**==**==**==**==**==**==**= SUBROUTINES =**==**==**==**==**==**==**==**==**==**==**==**=

//============= Get Sensor State =========================================================
bool SensorState(int k, int p) {           //k= sensor number; p= block number

  int analogSensorRead  = 0;
  bool stateChange = false;

  sensorReadTime[k] = curTime;

  //=== If digital sensor returning 0 or 1
  if (sensorAnalog[k] == 0) {
    curSensorState[k] = digitalRead(sensorPin[k])^sensorLowActive[k];           //Get sensor state
    delay(shortWait);
  }
  //=== Analog sensor
  else if (sensorAnalog[k] == 1) {
    analogSensorRead = analogRead(sensorPin [k]);
    delay(shortWait);
    analogSensorRead = (analogSensorRead + analogRead(sensorPin [k])) / 2;   //Average two reads
    curSensorState[k] = sensorInactive;
    if (analogSensorRead < analogTrigger) {
      if (sensorLowActive[k]) {
        curSensorState[k] = sensorActive;
      }
    }
    else if (analogSensorRead >= analogTrigger) {
      if (!sensorLowActive[k]) {
        curSensorState[k] = sensorActive;
      }
    }
  }
  //=== ERROR
  else {
    bus.println(String(millis()) + " SensorState ERROR: " + String(loopCount) + " curSensorState: " + String(curSensorState[k]) + " stateChange: " + String(stateChange));
  }

  if (curSensorState[k] != prevSensorState[k]) {                   //Change in state
    stateChange         = true;
    anySensorChange     = true;
    sensorDelayTime[p]  = curTime + sensorWait;
    blockIdleTime[p]    = curTime + idleWait;
  }

  //DEBUG
  //  bus.println(String(millis()) + " SensorState: " + String(k) + " " + String(loopCount) + " curSensorState: " + String(curSensorState[k]) + " stateChange: " + String(stateChange) + " analog read: " + String(analogSensorRead));

  return stateChange;
}

//================= Check Block Occupancy Status ===========================
bool BlockOccState(int p) {

  if (comboSensorState[p] > 3) {               //States > 3 means at least one sensor active
    digitalWrite(LED_BUILTIN, HIGH);
    return (true);
  }
  else if (comboSensorState[p] == 0 and prevBlockOcc[p])   {      //Specific cases when train between sensors
    digitalWrite (LED_BUILTIN, HIGH);
    return (true);
  }
  else if (comboSensorState[p] == 1 and prevBlockDir[p] == CCW) {
    digitalWrite (LED_BUILTIN, HIGH);
    return (true);
  }
  else if (comboSensorState[p] == 2 and prevBlockDir[p] == CW) {
    digitalWrite (LED_BUILTIN, HIGH);
    return (true);
  }
  else {                                        //Otherwise, block not occupied
    digitalWrite(LED_BUILTIN, LOW);
    return (false);
  }
}


//================= Check Direction ========================================
direction BlockDir(int p) {

  //If not at powerup, assume block empty
  if (!powerUp)  {
    if (comboSensorState[p] == 4)  {
      if (prevBlockOcc[p]) {
        return (CW);                                                //IAII going CW
      }
      else return (CCW);                                            //IAII entering block, going CCW
    }
    if (comboSensorState[p] == 8) {
      if (prevBlockOcc[p]) {
        return (CCW);                                                //AIII going CCW
      }
      else return (CW);                                              //AIII entering block, going CW
    }
  }
  //If at powerup, cannot determine train direction from first sensor reading because we don't know if train
  //sitting between two sensors in state 4 or 8
  else {
    if (comboSensorState[p] == 4 or comboSensorState[p] == 8 or comboSensorState[p] == 12) return (INDET);
  }

  //See state table at bottom of file
  if (comboSensorState[p] == 0)  {                                   //IIII
    if (prevBlockOcc[p])  {
      return (prevBlockDir[p]);
    }
    else return (UNK);                                               //If block empty, don't have a direction
  }
  else if (comboSensorState[p] == 5 or comboSensorState[p] == 10 or comboSensorState[p] == 15) {  //IAIA, AIAI, AAAA
    return (prevBlockDir[p]);
  }
  else if (comboSensorState[p] == 9 or comboSensorState[p] == 11 or comboSensorState[p] == 13) { //AIIA, AIAA, AAIA
    return (CCW);
  }
  else if (comboSensorState[p] == 6 or comboSensorState[p] == 7 or comboSensorState[p] == 14)  { //IAAI, IAAA, AAAI
    return (CW);
  }
  else if (comboSensorState[p] == 2 or comboSensorState[p] == 1)  { //IIAI or IIIA
    //Uses a trick here: current occupancy has already been determined from previous direction of travel
    if (blockOcc[p]) {
      return (prevBlockDir[p]);
    }
    else return (UNK);
  }
  else {
    return (UNK);                                                      //Should never get: IIAA, AAII (valid on powerup only)
  }
}


//================= Turn on Signals (Simple, for now) ========================================
void SignalLights(int k, int m, int n) {                //k = block, m= Green, n= Red

  //Simple on-off trackside lights for sensor activity;  no linkage of these lights to JMRI

  if (blockOcc[k]) {
    digitalWrite(lightPin[n], HIGH ^ lightLowActive[m]);  //Red
    digitalWrite(lightPin[m], LOW ^ lightLowActive[m]);
  }
  else  {
    digitalWrite(lightPin[m], HIGH ^ lightLowActive[n]);  //Green
    digitalWrite(lightPin[n], LOW ^ lightLowActive[n]);
  }
  delay(shortWait);                               //Wait a bit
}


//================= Turn on/alternate Crossing Signals ========================================
void CrossingSignal(int k, int m, int n) {         //k=block number, m= entry sensor, n= exit sensor

  //Simple flashing crossing signal when a block occupied;  no linkage of these lights to JMRI
  if (blockOcc[k])  {
    if (crossingSignalTime [k] < curTime)   {                //If time to alternate...
      if (crossingSignalAlt[k])  {
        crossingSignalAlt [k] = 0;
        digitalWrite(lightPin[m], HIGH ^ lightLowActive[m]); //Alternate on-off left-right
        digitalWrite(lightPin[n], LOW  ^ lightLowActive[n]);
        crossingSignalTime [k] = curTime + crossingWait;     //Set time to wait to alternate
      }
      else  {
        crossingSignalAlt [k] = 1;
        digitalWrite(lightPin[n], HIGH ^ lightLowActive[n]); //Alternate on-off right-left
        digitalWrite(lightPin[m], LOW  ^ lightLowActive[m]);
        crossingSignalTime [k] = curTime + crossingWait;     //Set time to wait to alternate
      }
    }
  }

  else   {                           //If block unoccupied, shut lights off
    if (crossingSignalAlt >= 0)  {   //Check to see if still turned on
      crossingSignalAlt [k] = -1;
      digitalWrite(lightPin[m], LOW ^ lightLowActive[m]);
      digitalWrite(lightPin[n], LOW ^ lightLowActive[n]);
      crossingSignalTime [k] = curTime;
    }
  }
}


//================= Lower/Raise Crossing Gate ========================================
void CrossingGate(int k, int m, int n) {         //k=block number, m= entry sensor, n= exit sensor

  if (blockOcc[k] != prevBlockOcc[k]) return;  //Don't worry about gate if no change in occupancy

  if (blockOcc[k])  {                        //Put gate down
    servo0.attach(gatePin[k]);  // attaches the servo to the servo object

    servo0.write(gateDown, servoSpeed, true);  // sets the servo position according to the scaled value
    delay(shortWait);                             // waits a bit before the next value is read and written
    servo0.detach();
  }
  else   {                           //If block unoccupied, put gate up
    servo0.attach(gatePin[k]);  // attaches the servo to the servo object

    servo0.write(gateUp, servoSpeed, true);  // sets the servo position according to the scaled value
    delay(shortWait);                             // waits a bit before the next value is read and written
    servo0.detach();
  }

  //Separate routine for manual operaiton
}

//================= Reset block info =====================================================
void BlockReset (int k, int m, int n)  {

  blockDir[k]         = UNK;
  prevBlockDir[k]     = UNK;
  blockOcc[k]         = false;
  prevBlockOcc[k]     = false;
  curSensorState[m]   = sensorInactive;
  curSensorState[n]   = sensorInactive;
  prevSensorState[m]  = sensorInactive;
  prevSensorState[n]  = sensorInactive;
  comboSensorState[k] = -1;
  sensorChange[m]     = false;
  sensorChange[n]     = false;
  anySensorChange     = false;

  //Reset JMRI bits
  cmri.set_bit(0 + 8 * k, 0);          //Block unoccupied
  cmri.set_bit(1 + 8 * k, 0);          //Sensors inactive
  cmri.set_bit(2 + 8 * k, 0);
  cmri.set_bit(3 + 8 * k, 0);          //Unknown direction
  cmri.set_bit(4 + 8 * k, 0);
}


//================= Check JMRI Panel availability =========================================
void CheckJMRI()  {

  //DEBUG
  JMRIPanelAvail = true;
  return;

  if (cmri.get_bit(47) == true) {       //See if JMRI bit turned on
    if (JMRIPanelAvail ==  false) {
      cmri.set_bit(23, 1);
      JMRIPanelAvail = true;
    }
  }
  else {                                //Otherwise, JMRI bit turned off
    if (JMRIPanelAvail == true)  {      //If previously turned on, reset to initial conditions
      powerUp = true;
    }
    JMRIPanelAvail = false;
    cmri.set_bit(23, 0);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(70);
    loopCount = loopCount + 1;
    //DEBUG
    //if (loopCount % 10 == 0) bus.println("WaitforPanel: " + String(loopCount) + "  " + String(prevTOCState[0]) + String(curTOCState[0]) + " " + String(prevPBState[0]) + String(curPBState[0]));
  }

}

//=============================================================================================


//Sensor State Table and other discussion available from author
