//Two sensors at ends of a block are checked to define a block and train direction
//Crossing signal and gate can be triggered for each block

//***SET UP FOR MULTIPLE BLOCKS WITH CROSSING SIGNAL***

//JMRI Sensors are set to indicate block occupancy, sensor detection, and direction
//Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
//Code on Github may be modified or withdraw at any time
//Copyright (c) 2019, 2020 Jerrold M. Grochow

//Based on CMRI485Sens_Sig_v8 2020-01-07
//v4 Working version
//v5 Changed to bool for CMRI and related variables
//v6 Added Block Reset  2019-07-27
//v7 Added Crossing signals, removed red-green signals 2019-10-07
//v7b Added full reset if JMRI bit 47 toggled  2019-10-27
//v8 Added optional crossing gate, added back red-green signals option 2019-11-21
//Forked as RRXing  2020-01-07
//v1
//v1a Bug fixes  2020-01-24
//Forked to RRXingSpeed 2020-01-31
//v1
//v2 Correct bugs in BlockSpeedMeasure, change bits used for speed, block reset, activation 2020-03-14

//JMRI: set up C/MRI node as SMINI to handle up to 3 blocks; set up as SUSIC to handle more blocks (limited by size of arrays in arduino code)
//JMRI: set up following Sensors and Lights to manage three blocks:
// [Note: sensors should be defined as Entry and Exit as would be seen if train moving clockwise]
// [Note: JMRI starts numbering C/MRI bits with 1, so C/MRI bit 0 in Arduino code is bit 1 in JMRI table]
// [Note: Code assumes bit # increments by 8 for each subsequent block. Arrays are set up to allow for five blocks.
//   Block occupancy*         (CMRI input bit 0,  8, 16, 24, 32)(JMRI sensor 1,  9, 17, 25, 33)
//   Entry sensor status**    (CMRI input bit 1,  9, 17, 25, 33)(JMRI sensor 2, 10, 18, 26, 34)
//   Exit sensor status**     (CMRI input bit 2, 10, 18, 26, 34)(JMRI sensor 3, 11, 19, 27, 35)
//   Counterclockwise travel* (CMRI input bit 3, 11, 19, 27, 35)(JMRI sensor 4, 12, 20, 28, 36)
//   Clockwise travel*        (CMRI input bit 4, 12, 20, 28, 36)(JMRI sensor 5, 13, 21, 29, 37)
//   Sound on/off crossing    (CMRI input bit 5, 13, 21, 29, 37)(JMRI sensor 6, 14, 22, 30, 38)
//   Speed***                 (CMRI input bits 40-47)           (JMRI sensor 41-48)
//   Block reset              (CMRI output bit 0, 1, 2, 3, 4)   (JMRI light 1, 2, 3, 4, 5)
// *  = determined by arduino code
//**  = "entry" and "exit" as would be seen moving clockwise; sensor states are from the actual sensors
//*** = determined by arduino code for a single block; requires JMRI script to display 8 bits as one number

//C/MRI output bit 23 from JMRI tells arduino to go active
//   Setup a JMRI Light associated with CMRI output address 24
//C/MRI input bit 23 from arduino tells JMRI it is active
//   Setup a JMRI Sensor associated with CMRI input address 24

//Arduino code maintains a single crossing signal (alternating lights) for each block, not currently linked to JMRI


#include <CMRI.h>                //Simulate CMRI node
#include <Auto485.h>             //Provide for RS485 communication
#include <VarSpeedServo.h>       //Allow servo motor control of crossing gates

const int CMRI_ADDR = 6;         //CMRI Board #6
const int DE_PIN    = 2;         //Arduino pin 2 for RS485 communication


//Constants:
const bool  sensorActive   = true;
const bool  sensorInactive = false;
const float SMPHfactor     = 56.8182 * 87; // 1000* 1/12 * 1/5280 * 3600 * [HO Scale]: Convert inches/msec to scale miles/hour


const unsigned long int sensorWait    =  1500;    //Wait between sensor reads
const unsigned long int idleWait      = 10000;    //FUTURE USE
const unsigned long int shortWait     =    20;    //After sensor reads
const unsigned long int crossingWait  =   400;    //Time for blink of crossing signal
const int               analogTrigger =   325;    //Analog sensor trigger value


//***************** Physical setup  [CHANGE THIS INFORMATION FOR YOUR LAYOUT]
uint8_t   servoSpeed = 18;                   //For VarSpeedServo
int       gateDown   = 30;                   //Servo settings on my layout
int       gateUp     = 72;
const int numBlocks  =  1;                   //MAX = 3 for SMINI; 5 based on size of arrays in code (requires emulating larger CMRI board)

//  Definitions for following variables:
//   sensorPin = arduino pin number for each sensor [Nano pins A1-A5 can be used as digital or analog pins;  A6-A7 are exclusively analog
//   sensorAnalog = true if analog sensor, false if digital sensor output
//   sensorLowActive = 1 if sensor active when LOW; 0= sensor active when HIGH
//    Digital IR Detector Module: for break-beam, set sensorLowActive to 0;  for reflector, set to 1
//    Analog IR Detector Module: for break-beam, set to 1; for reflector, set to 0
//    Analog IR Trans/Recvr: for break-beam, set to 1; for reflector, set to 0 (reverse if receiver output voltage reversed)
//    Digital Photoresistor Module: set to 0 (beam-break)
int  sensorPin []      = {    3,     6,     0,     0,     0,     0,     0,     0,     0,     0};
bool sensorAnalog []   = {false, false, false, false, false, false, false, false, false, false};
int  sensorLowActive[] = {    1,     1,     0,     0,     0,     1,    -1,    -1,    -1,    -1};
//Note: For contiguous blocks, Exit sensor of Block n can also Entry sensor of Block n+1 but must be entered twice
//Note: Program allows all sensors to be different and blocks to be non-continguous.
//  [-, true,1]: Analog IR trans/recvr /Beam-break on arduino pin A1
//  [-,false,0]: Digital IR Detector Module - Beam-break mode
//  [-,false,0]: Digital Photoresistor Module
//  [3 6,false,1]: Digital IR Detector Module - Reflector mode
//  [-, true,1]: Analog IR/Beam-break transmitter/receiver
//  [-, true,0]; Analog IR/Beam-Break transmitter/reverse-wired receiver

int   lightType[]      = { 3, 0, 0, 0, 0};                  //1= red-green;  2= crossing;  3= crossing+gate
int   lightPin[]       = {A3,A5, 0, 0, 0, 0, 0, 0, 0, 0};
int   lightLowActive[] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};   //1=LOW to turn on; 0=HIGH to turn on
int   gatePin[]        = { 9, 0, 0, 0, 0};                  //Servo requires PWM pin
int   gatePotPin[]     = {A1, 0, 0, 0, 0};                  //If gate also has manual operation
//***  Crossing signals and gate trackside are not linked to JMRI)
bool  speedEligible[]  = {   1,   0,   0,   0,   0};        //Block eligible to compute speed
float blockLength[]    = {21.5,-1.0, -1.0, -1.0, -1.0};     //Need block length to compute speed

//*************** State and output variables
bool  prevSensorState[]  = {sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive}; //Initialize state of each sensor
bool  curSensorState[]   = {sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive};
int   comboSensorState[] = { -1, -1, -1, -1, -1};          //Number representing combined current and previous states at a point in time
bool  prevBlockOcc[]     = {false, false, false, false, false};
bool  blockOcc[]         = {false, false, false, false, false};
int   curGateVal[]       = { -1, -1, -1, -1, -1};
float trainSpeed[]       = { -1.0, -1.0, -1.0, -1.0, -1.0};


enum direction {CW, CCW, INDET, UNK};                     //ClockWise, CounterClockWise, INDETerminate, UNKnown
direction curBlockDir []  = {UNK, UNK, UNK, UNK, UNK};    //Train direction determined in each block
direction prevBlockDir [] = {UNK, UNK, UNK, UNK, UNK};    //Previuosly determined train direction in each block
enum measStatus {EMPTY, CLK, CNTRCLK, COMP, CC, ERR};     //EMPTY block, measuring in CW/CCW direction, COMPleted measurement, Can't Compute
measStatus speedMeasStatus[] = {EMPTY, EMPTY, EMPTY, EMPTY, EMPTY};

//************* Internal control variables
bool sensorChange []      = {false, false, false, false, false, false, false, false, false, false};
bool blockSensorChange[]  = {false, false, false, false, false};
bool anySensorChange      = false;
int  crossingSignalAlt[]  = { -1, -1, -1, -1, -1};        //Switch to alternate crossing signals

//************ Time variables
unsigned long int sensorReadTime[]     = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   //Time that sensor last changed state
unsigned long int startSpeedTime[]     = {0, 0, 0, 0, 0};  //Start time for speed measurement
unsigned long int sensorDelayTime[]    = {0, 0, 0, 0, 0};  //Time to read sensor next for each block
unsigned long int crossingSignalTime[] = {0, 0, 0, 0, 0};  //Time to alternate crossing signal
unsigned long int crossingGateTime[]   = {0, 0, 0, 0, 0};  //Time for crossing to stay down (minimum)
unsigned long int blockIdleTime[]      = {0, 0, 0, 0, 0};  //How long since block last changed state

//************ Other variables
int numSS = numBlocks * 2;             //Number of sensors (duplicates still counted)
bool JMRIPanelAvail = false;
unsigned long int curTime = 0;
bool powerUp = true;                   //Need to knowfirst time sensors are activated for certain state change determination

int loopCount = 0;     //DEBUG

//================== Initialize libraries
VarSpeedServo servo0;                  //Set up one servo for each crossing gate
Auto485 bus(DE_PIN);
//CMRI cmri(CMRI_ADDR, 24, 48, bus);   //SMINI = 24 inputs, 48 outputs [WILL NEED MORE INPUTS TO CONTROL 5 BLOCKS]
CMRI cmri(CMRI_ADDR, 48, 24, bus);     //SUSIC with 48 inputs, 24 outputs (input = sent to JMRI; output = rcvd from JMRI)

//NOTE: Set up JMRI to simulate SUSIC rather than SMINI to allow more inputs


//========================================================================================
void setup() {

  //*****************  SKETCH NAME  *****************************************************
  //                                                                                    *
  bus.begin(19200, SERIAL_8N2);          //MAKE SURE JMRI CONNECTION SET TO THIS SPEED  *
  bus.println("Setup complete: CMRI485_RRXingSpeed-v2 2020-03-14 1500");        // *
  //                                                                                    *
  //*************************************************************************************

  //Activate special arduino pins for my layout, if any
  //For convenience on my layout, devices plugged into adjacent pins
  pinMode(sensorPin [0] + 1, OUTPUT);            digitalWrite(sensorPin [0] + 1, HIGH);
  pinMode(sensorPin [0] + 2, OUTPUT);            digitalWrite(sensorPin [0] + 2, LOW);
  pinMode(sensorPin [1] + 1, OUTPUT);            digitalWrite(sensorPin [1] + 1, HIGH);
  pinMode(sensorPin [1] + 2, OUTPUT);            digitalWrite(sensorPin [1] + 2, LOW);
  pinMode(lightPin  [0] + 1, OUTPUT);            digitalWrite(lightPin  [0] + 1, HIGH);  //Crossing signal
  //Activate arduino pins for this sketch
  pinMode(LED_BUILTIN, OUTPUT);

  for (int i = 0; i < numSS; i++) {
    int j = sensorPin[i];
    if (!sensorAnalog[i]) pinMode(j, INPUT_PULLUP);  //DO NOT INITIALIZE ANALOG PINS USED FOR ANALOG ONLY
    j = lightPin[i];
    pinMode(j, OUTPUT);   digitalWrite(j, HIGH ^ lightLowActive[i]); //Signal lights ON to test
    delay (500);
    pinMode(j, OUTPUT);   digitalWrite(j, LOW  ^ lightLowActive[i]); //Signal lights OFF
  }
  pinMode(gatePin[0], OUTPUT);                     //Gate must be PWM pin

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

  //loopCount = loopCount + 1;              //DEBUG
  // bus.println ("Beg: " + String(curTime) + " " + String(loopCount));

  cmri.process();
  CheckJMRI();
  if (!JMRIPanelAvail)  {
    //   curGateVal(0) = ManualGate(0);      //Allow manual gate operation
    return;
  }

  //========================= Cycle thru all listed blocks =======================
  //===== NOTE: all settings of sensor CMRI bits done in mainline;  determination of status and outputs done in subroutines ====

  anySensorChange = false;                       //Reset

  for (int i = 0; i < numBlocks; i++)   {
    int ii = i * 2;                              //Set up array indices for sensor pair for this block
    int ij = ii + 1;

    //==================== Skip to BLOCK RESET if Light set in JMRI ================================
    if (!cmri.get_bit(i))    {

      //================== Get state of sensors  and set in JMRI ================================

      blockSensorChange [i] = false;
      sensorChange [ii]     = false;
      sensorChange [ij]     = false;
      if (sensorDelayTime[i] < curTime) {

        //Entry sensor
        prevSensorState[ii] = curSensorState[ii];     //Save old sensor state
        sensorChange [ii] = SensorState(ii, i);       //Call SUBROUTINE
        // if (speedEligible[i])  bus.println(String(millis()) + " SensorState: #"  + String(ii) + " " + String(curSensorState[ii]) + String(sensorChange[ii]));
        cmri.set_bit(1 + 8 * i, curSensorState[ii]);  //Tell JMRI sensor state

        //Exit sensor
        prevSensorState[ij] = curSensorState[ij];     //Save old sensor state
        sensorChange [ij] = SensorState(ij, i);       //Call SUBROUTINE
        // if (speedEligible[i])  bus.println(String(millis()) + " SensorState: #"  + String(ij) + " " + String(curSensorState[ij]) + String(sensorChange[ij]));
        cmri.set_bit(2 + 8 * i, curSensorState[ij]);  //Tell JMRI sensor state

        //========================= Create combined sensor state number ======================================
        //Create single number from current and previous sensor states to simplify comparisons in later code, regardless of whether sensor just read or not
        comboSensorState[i] = (int) curSensorState[ii] * 8 + (int) curSensorState[ij] * 4 + (int) prevSensorState[ii] * 2 + (int) prevSensorState[ij];
        //Example: Entry sensor Active, exit sensor Inactive, previous entry sensor Active, previous exit sensor Inactive
        //         AIAI = 1010 binary = 10 decimal
        // Used in subroutines to determine block occ and train direction

        //======================== Get block occupancy, train direction status and set in JMRI ==============================
        //Redetermine block occ and train dir because combo state changes every time a sensor is read (even if sensor hasn't changed)

        prevBlockOcc [i] = blockOcc[i];               //Save current value
        blockOcc [i]     = BlockOccState(i);          //Call SUBROUTINE to determine block occupancy
        cmri.set_bit(0 + 8 * i, blockOcc[i]);         //Report block occupancy

        prevBlockDir [i] = curBlockDir[i];               //Save current value
        curBlockDir [i]     = BlockDir(i);               //Call SUBROUTINE to determine train direction traversing block
        switch (curBlockDir[i]) {                        //Report direction of train travel
          case CW:
            cmri.set_bit(3 + 8 * i, false);              //Clockwise
            cmri.set_bit(4 + 8 * i, true);
            break;
          case CCW:
            cmri.set_bit(3 + 8 * i, true);               //Counterclockwise
            cmri.set_bit(4 + 8 * i, false);
            break;
          case INDET:
            cmri.set_bit(3 + 8 * i, true);               //Indeterminate
            cmri.set_bit(4 + 8 * i, true);
            break;
          default:
            cmri.set_bit(3 + 8 * i, false);              //Unknown
            cmri.set_bit(4 + 8 * i, false);
        }

        if (speedEligible[i])        {                   //Only for specified blocks
          speedMeasStatus[i] = BlockSpeedMeasure(i, ii, ij); //Call SUBROUTINE to determine speed within block
          if (speedMeasStatus[i] == COMP)     {          //Translate speed into bits sent to JMRI
            uint8_t ts = round(trainSpeed[i] + 1.0);
            // bus.println ("L2:" + String(loopCount) + " " + String(curBlockDir[i]) + " " + String(startSpeedTime[i]) + " " + String(trainSpeed[i]) + " " + String(ts));
            cmri.set_byte(5, ts);                        //Sends speed as bit string to JMRI
          } //Speed transmit
        } //Speed measure eligible

      } //Not reset
    } //Not in sensor delay
    //End of processing every time a sensor is read

    //====================== Process if BLOCK RESET sent from JMRI ============================
    else {
      BlockReset (i, ii, ij);
    }

    //==================== Turn on/off signal lights, crossing signal, gate ===========================
    bool xSigOn = false;             //Is xing signal on?
    switch (lightType[i])  {
      case 1:
        SignalLights(i, ii, ij);
        break;
      case 2:
        xSigOn = CrossingSignal(i, ii, ij);
        break;
      case 3:
        xSigOn = CrossingSignal(i, ii, ij);
        curGateVal[i] = CrossingGate(i, ii, ij);
        break;
      default:          //Should never get here
        break;
    }
    if (xSigOn) cmri.set_bit(5 + 8 * i, true);     //Tell JMRI to play RRXing sound
    else        cmri.set_bit(5 + 8 * i, false);    //Tell JMRI not to play RRXing sound



    //DEBUG
    //    bus.println ("End: " + String(curTime) + " " + String(i) + " " + String(comboSensorState[i]) + " " + String(prevBlockOcc[i]) + String(blockOcc[i]) + " " + String(prevBlockDir[i]) + String(curBlockDir[i])
    //             + " PU SC1 SC2 SCT BI: " + String(powerUp) + " " + String(sensorChange[ii]) + " " + String(sensorChange[ij]) + " " +  String(sensorDelayTime[i]) + " " + String(blockIdleTime[i]));

    //======================== End of Block Processing ==================================================
  }

  //DEBUG
  //  bus.println();

  //Set powerUp to false after first sensor change after first run through
  if (anySensorChange) powerUp = false;

}
//==================== END OF LOOP =======================================================


//=**==**==**==**==**==**==**==**==**= SUBROUTINES =**==**==**==**==**==**==**==**==**==**==**==

//============= Get Sensor State =========================================================
bool SensorState(int s, int k) {           //s= sensor number; k= block number

  //Sets curSensorState, anySensorChange, sensorDelayTime, blockIdleTime

  int analogSensorRead  = 0;
  bool stateChange = false;

  sensorReadTime[s] = curTime;

  //=== If digital sensor returning 0 or 1
  if (!sensorAnalog[s]) {
    curSensorState[s] = digitalRead(sensorPin[s])^sensorLowActive[s];           //Get sensor state
    delay(shortWait);
  }
  //=== Analog sensor
  else if (sensorAnalog[s]) {
    analogSensorRead = analogRead(sensorPin [s]);
    delay(shortWait);
    analogSensorRead = (analogSensorRead + analogRead(sensorPin [s])) / 2;   //Average two reads
    curSensorState[s] = sensorInactive;
    if (analogSensorRead < analogTrigger) {
      if (sensorLowActive[s]) {
        curSensorState[s] = sensorActive;
      }
    }
    else            {
      if (!sensorLowActive[s]) {
        curSensorState[s] = sensorActive;
      }
    }
  }
  //=== ERROR
  else {
    // bus.println(String(millis()) + " SensorState ERROR: " + String(loopCount) + " curSensorState: " + String(curSensorState[s]) + " stateChange: " + String(stateChange));
  }

  if (curSensorState[s] != prevSensorState[s]) {                   //Change in state
    stateChange          = true;
    anySensorChange      = true;
    blockSensorChange[k] = true;
    sensorDelayTime[k]   = curTime + sensorWait;
    blockIdleTime[k]     = curTime + idleWait;
  }

  //DEBUG
  //  bus.println(String(millis()) + " SensorState: " + String(k) + " " + String(loopCount) + " curSensorState: " + String(curSensorState[s]) + " stateChange: " + String(stateChange) + " analog read: " + String(analogSensorRead));

  return stateChange;
}

//================= Check Block Occupancy Status ===========================
bool BlockOccState(int k) {                  //k=block

  if (comboSensorState[k] > 3) {               //States > 3 means at least one sensor active
    digitalWrite(LED_BUILTIN, HIGH);
    return (true);
  }
  else if (comboSensorState[k] == 0 and prevBlockOcc[k])   {      //Specific cases when train between sensors
    digitalWrite (LED_BUILTIN, HIGH);
    return (true);
  }
  else if (comboSensorState[k] == 1 and prevBlockDir[k] == CCW) {
    digitalWrite (LED_BUILTIN, HIGH);
    return (true);
  }
  else if (comboSensorState[k] == 2 and prevBlockDir[k] == CW) {
    digitalWrite (LED_BUILTIN, HIGH);
    return (true);
  }
  else {                                        //Otherwise, block not occupied
    digitalWrite(LED_BUILTIN, LOW);
    return (false);
  }
}


//================= Check Direction ========================================
direction BlockDir(int k) {                          //k=block

  //Not yet set up for change of direction within block
  //If not at powerup, assume block empty
  if (!powerUp)  {
    if (comboSensorState[k] == 4)  {
      if (prevBlockOcc[k]) {
        return (CW);                                                //IAII going CW
      }
      else return (CCW);                                            //IAII entering block, going CCW
    }
    if (comboSensorState[k] == 8) {
      if (prevBlockOcc[k]) {
        return (CCW);                                                //AIII going CCW
      }
      else return (CW);                                              //AIII entering block, going CW
    }
  }
  //If at powerup, cannot determine train direction from first sensor reading because we don't know if train
  //sitting between two sensors in state 4 or 8
  else {
    if (comboSensorState[k] == 4 or comboSensorState[k] == 8 or comboSensorState[k] == 12) return (INDET);
  }

  //See state table at bottom of file
  if (comboSensorState[k] == 0)  {                                   //IIII
    if (prevBlockOcc[k])  {
      return (prevBlockDir[k]);
    }
    else return (UNK);                                               //If block empty, don't have a direction
  }
  else if (comboSensorState[k] == 5 or comboSensorState[k] == 10 or comboSensorState[k] == 15) {  //IAIA, AIAI, AAAA
    return (prevBlockDir[k]);
  }
  else if (comboSensorState[k] == 9 or comboSensorState[k] == 11 or comboSensorState[k] == 13) { //AIIA, AIAA, AAIA
    return (CCW);
  }
  else if (comboSensorState[k] == 6 or comboSensorState[k] == 7 or comboSensorState[k] == 14)  { //IAAI, IAAA, AAAI
    return (CW);
  }
  else if (comboSensorState[k] == 2 or comboSensorState[k] == 1)  { //IIAI or IIIA
    //Uses a trick here: current occupancy has already been determined from previous direction of travel
    if (blockOcc[k]) {
      return (prevBlockDir[k]);
    }
    else return (UNK);
  }
  else {
    return (UNK);                                                      //Should never get: IIAA, AAII (valid on powerup only)
  }
}


//================= Speed Display ========================================================
measStatus BlockSpeedMeasure (int k, int m, int n)  {                    //k=block, m=entry sensor, n=exit sensor

  //Returns  speed measurement progress
  // bus.println ("S0a:" + String(loopCount) + " BO: " + String(blockOcc[k]) + " SM: " + String(speedMeasStatus[k]) + " " + String(curSensorState[m]) + " " + String(curSensorState[n]));
  // bus.println ("S0b:" + String(loopCount) + " BD: " + String(curBlockDir[k]) + " " + String(startSpeedTime[k]) + " " + String(trainSpeed[k]));

  if (blockSensorChange[k]) {                       //Only bother with speed measure if sensor change
    // bus.println ("S0c:" + String(loopCount) + " BD: " + String(curBlockDir[k]) + " " + String(startSpeedTime[k]) + " " + String(trainSpeed[k]));
    if (blockOcc[k])                   {
      switch (speedMeasStatus[k])      {
        case EMPTY:   //Only compute speed if block previously empty
          // bus.println ("S1=EMPTY");
          if (curSensorState[m] != prevSensorState[m]) {
            startSpeedTime[k] = millis();
            return (CLK);
          }
          if (curSensorState[n] != prevSensorState[n]) {
            startSpeedTime[k] = millis();
            return (CNTRCLK);
          }
          return (ERR);
        case CLK:
          if (curSensorState[n])            {                  //xAxx
            //Tripped opposing sensor, compute speed
            trainSpeed[k] = blockLength[k] / (millis() - startSpeedTime[k]) * SMPHfactor;
            startSpeedTime[k] = 0;                                //Reset
            // bus.println ("S2-CLK");
            return (COMP);
          }
          else return (speedMeasStatus[k]);                    //No change in state
        case CNTRCLK:
          if (curSensorState[m])            {                  //Axxx
            //Tripped opposing sensor, compute speed
            trainSpeed[k] = blockLength[k] / (millis() - startSpeedTime[k]) * SMPHfactor;
            startSpeedTime[k] = 0;                                //Reset
            // bus.println ("S3-CNTRCLK");
            return (COMP);
          }
          else return (speedMeasStatus[k]);                    //No change in state
        case COMP:
          // bus.println ("S4-COMP");
          return (EMPTY);
        case ERR:
          // bus.println ("S5-ERR");
          return (ERR);
        default:
          return (ERR);
      } //speedMeasStatus

      return (ERR);                                  //No change in state
    }  //Block occupied

    else                                   {                     //Block unoccupied
      trainSpeed[k] = -1.0;
      // bus.println ("S6-UNOCC");
      return (EMPTY);
    } //Block unoccpied

    // bus.println ("S7-ERR");
    return (ERR);                           //Should never get here

  } // Block occupancy

  return (speedMeasStatus[k]);                //No change
} //BlockSpeedMeasure


//================= Turn on Signals (Simple, for now) ========================================
void SignalLights(int k, int m, int n) {                //k = block, m= Green, n= Red

  //Simple on-off trackside lights for sensor activity;  no linkage of these lights to JMRI

  if (blockOcc[k]) {
    digitalWrite(lightPin[n], HIGH ^ lightLowActive[n]);  //Red
    digitalWrite(lightPin[m], LOW ^ lightLowActive[m]);
  }
  else  {
    digitalWrite(lightPin[m], HIGH ^ lightLowActive[m]);  //Green
    digitalWrite(lightPin[n], LOW ^ lightLowActive[n]);
  }
  delay(shortWait);                               //Wait a bit
}


//================= Turn on/alternate Crossing Signals ========================================
bool CrossingSignal(int k, int m, int n) {         //k=block number, m= first light, n= second light

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
    return (true);                       //Signal on
  }

  else   {                               //If block unoccupied, shut lights off
    if (crossingSignalAlt >= 0)  {       //Check to see if still turned on
      crossingSignalAlt [k] = -1;
      digitalWrite(lightPin[m], LOW ^ lightLowActive[m]);
      digitalWrite(lightPin[n], LOW ^ lightLowActive[n]);
      crossingSignalTime [k] = curTime;
    }
    return (false);                      //Signal not on
  }
}


//================= Lower/Raise Crossing Gate ========================================
int CrossingGate(int k, int m, int n) {         //k=block number, m= entry sensor, n= exit sensor

  if (blockOcc[k] == prevBlockOcc[k])  return (curGateVal[k]);      //Only look at gate if occupancy has changed

  if (blockOcc[k])  {                             //See if block currently occupied
    //Put gate down since block not previously occupied
    servo0.attach(gatePin[k]);                      // attaches the servo to the servo object
    delay(shortWait);
    servo0.write(gateDown, servoSpeed, true);  // sets the servo position, waits for complete
    delay(shortWait);                             // waits a bit
    servo0.detach();
    return (gateDown);
  }
  else   {                                        //If block unoccupied, ...
    //Put gate up if unoccupied
    servo0.attach(gatePin[k]);                       // attaches the servo to the servo object
    delay(shortWait);
    servo0.write(gateUp, servoSpeed, true);  // sets the servo position, waits for complete
    delay(shortWait);                             // waits a bit
    servo0.detach();
    return (gateUp);
  }

}


//====================== Manual gate operation via potentiometer ==========================================
int ManualGate (int k)        {                    //k=block *** NOT YET WORKING 2020-02-05 ***

  int gVal = analogRead(gatePotPin[k]);            // reads the value of the potentiometer (value between 0 and 1023)
  gVal = map(gVal, 0, 1023, gateDown, gateUp);  // scale it to use it with the servo (value from 0 and 180)
  if (gVal <= (max(gateUp, gateDown) - 5) || gVal >= (min(gateUp, gateDown) + 5))   {
    //Need to move gate
    servo0.attach(gatePin[k]);                        // attaches the servo to the servo object
    delay(shortWait);
    servo0.write(gVal, servoSpeed, true);          // sets the servo position according to the scaled value
    delay(shortWait);                                 // waits a bit before the next value is read and written
    servo0.detach();
    delay(shortWait);
    return (gVal);
  }
  return (curGateVal[k]);                         //No change in gate value
}


//================= Reset block info =====================================================
void BlockReset (int k, int m, int n)  {        //k=block m=entry sensor n=exit sensor

  curBlockDir[k]       = UNK;
  prevBlockDir[k]      = UNK;
  blockOcc[k]          = false;
  prevBlockOcc[k]      = false;
  curSensorState[m]    = sensorInactive;
  curSensorState[n]    = sensorInactive;
  prevSensorState[m]   = sensorInactive;
  prevSensorState[n]   = sensorInactive;
  comboSensorState[k]  = -1;
  sensorChange[m]      = false;
  sensorChange[n]      = false;
  blockSensorChange[k] = false;
  anySensorChange      = false;
  speedMeasStatus[k]   = EMPTY;


  //Reset JMRI bits
  cmri.set_bit(0 + 8 * k, false);          //Block unoccupied
  cmri.set_bit(1 + 8 * k, false);          //Sensors inactive
  cmri.set_bit(2 + 8 * k, false);
  cmri.set_bit(3 + 8 * k, false);          //Unknown direction
  cmri.set_bit(4 + 8 * k, false);
  cmri.set_bit(5 + 8 * k, false);          //No crossing sound

  if (speedEligible[k]) cmri.set_byte(3, 0); //Reset speed measure bits
}


//================= Check JMRI Panel availability =========================================
void CheckJMRI()  {

  //DEBUG
  //  JMRIPanelAvail = true;
  //  return;

  if (cmri.get_bit(23) == true) {       //See if JMRI bit turned on
    if (JMRIPanelAvail ==  false) {
      cmri.set_bit(23, true);
      JMRIPanelAvail = true;
    }
  }
  else {                                //Otherwise, JMRI bit turned off
    if (JMRIPanelAvail == true)  {      //If previously turned on, reset to initial conditions
      powerUp = true;
    }
    JMRIPanelAvail = false;
    cmri.set_bit(23, false);
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
