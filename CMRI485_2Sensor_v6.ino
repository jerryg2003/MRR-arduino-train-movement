//Two sensors at ends of a block are checked to define a block and train direction
//JMRI Sensors are set to indicate block occupancy, sensor detection, and direction
//Code provided "AS IS" with no warranties! PLEASE DO NOT DISTRIBUTE FURTHER WITHOUT PERMISSION OF AUTHOR
//Code on Github may be modified or withdraw at any time
//Copyright (c) 2019 Jerrold M. Grochow
//Last update: Jerry Grochow  07-27-2019 (comments updated 10-07-2019)
//v4 Working version
//v5 Changed to bool for CMRI and related variables
//v6 Added Block Reset

//JMRI: set up following Sensors and Lights to manage three blocks:
// Note: named Entry-Exit and shown in diagrams as left-right for convenience
// Nore: JMRI starts numbering C/MRI bits with 1 so C/MRI bit 0 in Arduino code is bit 1 in JMRI table
//   Block occupancy(CMRI output bit 0, 8, 16)(JMRI sensor 1, 9, 17)
//   Entry sensor(CMRI output bit 1, 9, 17)(JMRI sensor 2, 10, 18)
//   Exit sensor(CMRI output bit 2, 10, 18)(JMRI sensor 3, 11, 19)
//   Counterclockwise travel (CMRI output bit 3, 11, 19)(JMRI sensor 4, 12, 20)
//   Clockwise travel (CMRI output bit 4, 12, 20)(JMRI sensor 5, 13, 21)
//   Block reset (CMRI input bit 0, 8, 16) (JMRI light 1, 9, 17)
//[Code assumes bit # increments by 8 for each subsequent block. Arrays are set up for five blocks, but would require expanded CMRI node to handle more bits - NOT YET TESTED.]
//C/MRI output bit 47 from JMRI tells arduino to go active
//   Setup JMRI Light associated with CMRI address 48
//C/MRI input bit 23 from arduino tells JMRI it is active
//   Setup JMRI Sensor associated with CMRI address 24

//Arduino code maintains two simple trackside signals for each block, not currently linked to JMRI


#include <CMRI.h>                //Simulate CMRI node
#include <Auto485.h>             //Allow RS485 communication

const int CMRI_ADDR = 5;         //CMRI Board #5
const int DE_PIN = 2;


//Constants:
const bool sensorActive = true;
const bool sensorInactive = false;

const unsigned long int sensorWait = 1000;
const unsigned long int idleWait = 10000; //For block reset in oddball cases
const int shortWait = 20;
const int analogTrigger = 325;            //Analog sensor trigger value

//Physical setup  [CHANGE THIS INFORMATION FOR YOUR LAYOUT]
const int numBlocks = 3;                  //MAX = 3 for SMINI; 5 based on size of arrays in code (requires emulating larger CMRI board)
int numSS = numBlocks * 2;                //MAX = 10 based on size of arrays
//My layout sensorPin setup; has exit sensor of Block n as Entry sensor of Block n+1 (three adjacent blocks); can have all different.
//  Blk 0 Entry  = Digital IR/reflector module on arduino pin A1
//  Blk 0 Exit   = Digital IR/Beam-break module A4
//  Blk 1 Entry  = Digital IR/Beam-break module A4
//  Blk 2 Exit   = Analog IR/Beam-break transmitter/receiver A5
//  Blk 3 Entry  = Analog IR/Beam-break transmitter/receiver A5
//  Blk 3 Exit   = Analog IR/Beam-Break transmitter/receiver A7
int sensorPin []      = {A1, A4, A4, A5, A5, A7, 0, 0, 0, 0};
bool sensorAnalog []  = {false, false, false, true, true, true, false, false, false, false};
int sensorLowActive[] = {1, 0, 0, 1, 1, 1, -1, -1, -1, -1};
//  1= sensor active when LOW; 0= sensor active when HIGH
//  Digital IR Detector Module: for break-beam, set sensorLowActive to 0;  for reflector, set to 1
//  Analog IR Detector Module or Trans/Recr: for break-beam, set to 1; for reflector, set to 0
//  Digital Photoresistor Module: set to 0 (beam-break)
int signalPin []      = {4, 6, 5, 7, 9, 10, 0, 0, 0, 0};
//  Simple on-off signals trackside (not linked to JMRI)

//State and output variables
bool prevSensorState [] = {sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive}; //Initialize state of each sensor
bool curSensorState []  = {sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive, sensorInactive};
int comboSensorState [] = { -1, -1, -1, -1, -1};    //Number representing combined current and previous states at a point in time
bool prevBlockOcc []    = {false, false, false, false, false};
bool blockOcc []        = {false, false, false, false, false};

enum direction {CW, CCW, INDET, UNK};               //ClockWise, CounterClockWise, INDETerminate, UNKnown
direction blockDir []     = {UNK, UNK, UNK, UNK, UNK};  //Train direction determined in each block
direction prevBlockDir [] = {UNK, UNK, UNK, UNK, UNK}; //Previuosly determined train direction in each block

//Internal control variables
bool sensorChange [] = {false, false, false, false, false, false, false, false, false, false};
bool anySensorChange = false;
unsigned long int sensorReadTime []  = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   //Time that sensor last changed state
unsigned long int sensorDelayTime [] = {0, 0, 0, 0, 0};    //Time to read sensor next for each block
unsigned long int blockIdleTime []   = {0, 0, 0, 0, 0};    //How long since block last changed state

bool JMRIPanelAvail = false;
unsigned long int curTime = 0;
bool powerUp = true;       //Need to knowfirst time sensors are activated for certain state change determination

int loopCount = 0;     //DEBUG

//Initialize libraries
Auto485 bus(DE_PIN);
CMRI cmri(CMRI_ADDR, 24, 48, bus); //SMINI = 24 inputs, 48 outputs [WILL NEED MORE TO CONTROL 5 BLOCKS]


//========================================================================================
void setup() {

  //Define pins for this sketch
  pinMode(LED_BUILTIN, OUTPUT);

  int j;
  for (int i = 0; i < numSS; i++) {
    j = sensorPin[i];
    if (sensorAnalog [j] == 0) pinMode(j, INPUT_PULLUP);  //DO NOT INITIALIZE ANALOG PINS USED FOR ANALOG ONLY
  }
  for (int i = 0; i < numSS; i++)  {
    j = signalPin[i];
    pinMode(j, OUTPUT);   digitalWrite(j, HIGH);          //Signal lights ON to test
  }
  delay(shortWait);

  //*****************  SKETCH NAME  *****************************************************
  //                                                                                    *
  bus.begin(19200, SERIAL_8N2);          //MAKE SURE JMRI CONNECTION SET TO THIS SPEED  *
  bus.println("Setup complete: CMRI485_2Sensor_v6 2019-07-27 2300");                 // *
  //                                                                                    *
  //*************************************************************************************

  //Flash LED that board is ready
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1200);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(800);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
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

    if (!cmri.get_bit(8 * i))    {                    //See if BLOCK RESET light set in JMRI

      //============================= Get state of sensors  and set in JMRI ==============================

      sensorChange [ii] = false;
      sensorChange [ij] = false;
      if (sensorDelayTime[i] < curTime) {

        //Entry sensor
        prevSensorState[ii] = curSensorState[ii];     //Save old sensor state
        sensorChange [ii] = SensorState(ii, i);       //Call subroutne
        //      bus.println(String(millis()) + " SensorState: #"  + String(ii) + " " + String(curSensorState[ii]) + String(sensorChange[ii]));
        cmri.set_bit(1 + 8 * i, curSensorState[ii]);  //Tell JMRI sensor state

        //Exit sensor
        prevSensorState[ij] = curSensorState[ij];     //Save old sensor state
        sensorChange [ij] = SensorState(ij, i);       //Call subroutine
        //      bus.println(String(millis()) + " SensorState: #"  + String(ij) + " " + String(curSensorState[ij]) + String(sensorChange[ij]));
        cmri.set_bit(2 + 8 * i, curSensorState[ij]);  //Tell JMRI sensor state

        //========================= Create combined sensor state number ======================================
        //Create single number from current and previous sensor states to simplify comparisons in later code, regardless of whether sensor just read or not
        comboSensorState[i] = (int) curSensorState[ii] * 8 + (int) curSensorState[ij] * 4 + (int) prevSensorState[ii] * 2 + (int) prevSensorState[ij];
        //Example: Entry sensor Active, exit sensor Inactive, previous entry sensor Active, previous exit sensor Inactive
        //         AIAI = 1010 binary = 10 decimal
        // Used in subroutines to determine block occ and train direction
        
        //======================== Get block occupancy, train direction status and set in JMRI ==============================
        //Redetermine block occ and train dir because combo state has changed every time a sensor is read

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
      blockDir[i]          = UNK;
      blockOcc[i]          = false;
      curSensorState[ii]   = 0;
      curSensorState[ij]   = 0;
      comboSensorState[i]  = 0;
      anySensorChange      = true;
      cmri.set_bit(0 + 8 * i, 0);                //Block unoccupied
      cmri.set_bit(1 + 8 * i, 0);                //Sensors inactive
      cmri.set_bit(2 + 8 * i, 0);
      cmri.set_bit(3 + 8 * i, 0);                //Unknown direction
      cmri.set_bit(4 + 8 * i, 0);
    }

    //==================== Turn on signal lights ============================================
    SignalLights(ii, ij);


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
void SignalLights(int m, int n) {         //m= entry sensor, n= exit sensor

  //NEEDS WORK: simple on-off trackside lights for sensor activity;  no linkage of these lights to JMRI

  if (curSensorState[m]) {
    digitalWrite(signalPin[m], HIGH);
  }
  else {
    digitalWrite(signalPin[m], LOW);
  }

  if (curSensorState[n]) {
    digitalWrite(signalPin[n], HIGH);
  }
  else {
    digitalWrite(signalPin[n], LOW);
  }
  delay(shortWait);                               //Wait a bit
}


//================= Check JMRI Panel availability ==========================
void CheckJMRI()  {

  //DEBUG
    JMRIPanelAvail = true;
    return;

  if (cmri.get_bit(47) == 1) {
    if (JMRIPanelAvail ==  false) {
      cmri.set_bit(23, 1);
      JMRIPanelAvail = true;
    }
  }
  else {
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
