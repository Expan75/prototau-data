/* ------ HIGH & LOW VOLTAGE ALARMS DISABLED ------ */

#include "tempdata.h"
        /*Debugging*/
#define DEBUG false   //verbose debug, more human-readable
#define PID_DEBUG false //for fan tuning
#define PRESS_DEBUG false   //usual debug mode, outputs CSV format
#define ALARM_DEBUG false //gives detail on alarm states
#define PURGE_DEBUG false //displays amp-sec accumulator, purge timings
#define DEBUG_INTERVAL 1000 //milliseconds

#define CELLS 18
        /*Intervals and Durations*/
#define FSM_INTERVAL 10          //milliseconds
#define STANDBY_DELAY_TIME 500  //milliseconds
#define FAN_SPOOLUP_TIME 10     //milliseconds
#define STARTUP_PURGE_TIME 300   //milliseconds
#define SHUTDOWN_DELAY_TIME 1000 //milliseconds
#define PURGE_INTERVAL 2300      //amp-seconds
#define PURGE_DURATION 0.2       //seconds
#define RESET_TIME 3             //seconds

        /*HASS-50S calibration*/
#define CURRENT_ZERO_POINT 508     //analogRead value given at 0A
#define CURRENT_SENSITIVITY 0.625  // (V/A)/50 (this is the format given in data sheet)

        /*Fan PID Tuning*/
#define FAN_P 2
#define FAN_I 2
#define FAN_D 0

#define FAN_MIN_DUTY 100  //0-100
#define FAN_MAX_DUTY 100 //0-100
#define FAN_SPOOLUP_DUTY 100

        /*Output Pins*/
#define FAN_PIN 2
#define SUPPLY_VALVE_PIN 3
#define PURGE_VALVE_PIN 4
#define FC_RELAY_PIN 5

        /*Input Pins*/
#define CURRENT_READ_PIN A2
#define TEMP_READ_PIN A0
#define RUN_FLAG_PIN 8
#define PRESSURE_READ_PIN A1
#define VOLTAGE_READ_PIN A3

#define PRESS_AVG_READINGS 32 //No. of samples for moving avg. filter on pressure data

unsigned long timerStartTime;
unsigned long fanStartTime;
unsigned long purgeStartTime; 

enum FCStates { //States for the fuel cell FSM
  STANDBY = 0,
  STARTUP = 1,
  RUN = 2,
  SHUTDOWN = 3,
  ALARM = 4
};

char *FCStateStrings[] = {"STANDBY", "STARTUP", "RUN", "SHUTDOWN", "ALARM"}; //String versions for debugging

enum startupStates {  //States for startup sub-FSM
  FAN_SPOOLUP = 0,
  STARTUP_PURGE = 1,
  END = 2
};

char *startupStrings[] = {"FAN_SPOOLUP", "STARTUP_PURGE", "END"};

enum alarmStates {  //Possible alarms
  NONE = 0,
  HIGH_FC_TEMP = 1,
  HIGH_FC_TEMP_DELAYED = 2,
  LOW_FC_TEMP = 3,
  LOW_FC_TEMP_DELAYED = 4,
  HIGH_FC_CURRENT = 5,
  LOW_FC_CURRENT = 6,
  HIGH_FC_VOLTAGE = 7,
  LOW_FC_VOLTAGE = 8,
  HIGH_FC_PRES = 9,
  HIGH_FC_PRES_DELAYED = 10,
  LOW_FC_PRES = 11,
  LOW_FC_PRES_DELAYED = 12
};

char *alarmStrings[] = {"NONE", "HIGH_FC_TEMP", "HIGH_FC_TEMP_DELAYED", "LOW_FC_TEMP", "LOW_FC_TEMP_DELAYED", "HIGH_FC_CURRENT", "LOW_FC_CURRENT",
  "HIGH_FC_VOLTAGE", "LOW_FC_VOLTAGE", "HIGH_FC_PRES", "HIGH_FC_PRES_DELAYED", "LOW_FC_PRES", "LOW_FC_PRES_DELAYED"};

FCStates FCState; //Fuel cell state
startupStates startupState; //Startup state
alarmStates alarmState; //Alarm state
bool FCAlarm; //Has alarm been triggered?
bool FCOn;  //Has a run signal been recieved?
bool debugThisCycle = false; //Send debug data this FSM cycle? (for some functions)


void setup() { //Run at beginning
  /*Init. variables*/
  FCAlarm = false; 
  FCOn = false;
  FCState = STANDBY;
  /*Init. pins*/
  pinMode(FAN_PIN, OUTPUT);
  pinMode(SUPPLY_VALVE_PIN, OUTPUT);
  pinMode(PURGE_VALVE_PIN, OUTPUT);
  pinMode(FC_RELAY_PIN, OUTPUT);
  pinMode(RUN_FLAG_PIN, INPUT);
  for(int j=31; j<=53; j++){
    pinMode(j,OUTPUT);
    digitalWrite(j,LOW);
  }
  /*Init. Serial for debug*/
  if(DEBUG || PID_DEBUG || PRESS_DEBUG || ALARM_DEBUG || PURGE_DEBUG){
    Serial.begin(115200);
  }
  delay(100);
  /*Print headers for logging to CSV*/
  if(PRESS_DEBUG){
      Serial.print("Millis");
      Serial.print(", ");
      Serial.print("Current");
      Serial.print(", ");
      Serial.print("Temp.");
      Serial.print(", ");
      Serial.print("Voltage");
      Serial.print(", ");
      Serial.print("Press.");
      Serial.print(", ");
      Serial.print("State");
      Serial.print(", ");
      Serial.println("Alarm");
    }
}

void loop() {
  checkAlarms(); //Check if alarm is triggered
  FCOn = digitalRead(RUN_FLAG_PIN); //Poll run flag input
  static unsigned long lastRunMillis = 0; //Init. last FSM cycle timestamp
  unsigned long currentMillis = millis(); //Get current time
  if((currentMillis-lastRunMillis) >= FSM_INTERVAL){ //If the FSM is due to be updated
    fuelCellFSM(); //Update the FSM
    lastRunMillis = currentMillis; //Update last FSM cycle timestamp
    /*Debug outputs for logging*/
    if(PRESS_DEBUG){
      Serial.print(millis());
      Serial.print(", ");
      Serial.print(calculateCurrent(analogRead(CURRENT_READ_PIN)));
      Serial.print(", ");
      Serial.print(calculateTemp(analogRead(TEMP_READ_PIN)));
      Serial.print(", ");
      Serial.print(calculateVoltage(analogRead(VOLTAGE_READ_PIN)));
      Serial.print(", ");
      Serial.print(pressMovingAvg(calculatePressure(analogRead(PRESSURE_READ_PIN)),false));
      Serial.print(", ");
      Serial.print(FCStateStrings[FCState]);
      Serial.print(", ");
      Serial.println(alarmStrings[alarmState]);
    }
  }
  /*Alternative debug, more human-readable*/
  if(DEBUG){
      static unsigned long lastDebugMessage = 0;
      if((currentMillis-lastDebugMessage) >= DEBUG_INTERVAL){
        Serial.print("------- DEBUG AT ");
        Serial.print(currentMillis);
        Serial.println("ms -------");
        Serial.print("Alarm flag:    ");
        Serial.println(FCAlarm);
        Serial.print("Alarm state:   ");
        Serial.println(alarmStrings[alarmState]);
        Serial.print("Run flag:      ");
        Serial.println(FCOn);
        Serial.print("FC State:      ");
        Serial.println(FCStateStrings[FCState]);
        if(FCState==STARTUP){
          Serial.print("Startup State: ");
          Serial.println(startupStrings[startupState]);
        }
        Serial.print("Analogue Current Value: ");
        Serial.println(analogRead(CURRENT_READ_PIN));
        Serial.print("Stack Current:          ");
        Serial.print(calculateCurrent(analogRead(CURRENT_READ_PIN)));
        Serial.println("A");
        Serial.print("Analogue Temp. Value:   ");
        Serial.println(analogRead(TEMP_READ_PIN));
        Serial.print("Calculated Temperature: ");
        Serial.print(calculateTemp(analogRead(TEMP_READ_PIN)));
        Serial.println("C");
        Serial.print("Analogue Voltage Value: ");
        Serial.println(analogRead(VOLTAGE_READ_PIN));
        Serial.print("Calculated Voltage:     ");
        Serial.print(calculateVoltage(analogRead(VOLTAGE_READ_PIN)));
        Serial.println("V");
        Serial.print("Analogue Press. Value:  ");
        Serial.println(analogRead(PRESSURE_READ_PIN));
        Serial.print("Calculated Pressure:    ");
        Serial.print(calculatePressure(analogRead(PRESSURE_READ_PIN)));
        Serial.println(" bar (g)");
        lastDebugMessage=currentMillis;
        debugThisCycle = true; //Other functions that check this variable will print a debug output this cycle
      }
      else{
        debugThisCycle = false; // ^as above^, will not print a debug output
      }
  }
  
}

bool purgeControl(){ //Purge control function
  bool purgeValve = digitalRead(PURGE_VALVE_PIN); //Get current state of purge valve
  bool FCRelay = !digitalRead(FC_RELAY_PIN); //Get inverse of relay state
  float FCCurrent = calculateCurrent(analogRead(CURRENT_READ_PIN)); //get present current
  float currentTime = millis(); //get current time
  currentTime /= 1000; //convert to seconds (as float)
  unsigned long debugTime = millis(); //(used for debug output)
  
  bool openPurgeValve;

  static float lastCallTime = 0; //Init last function call time
  static float ampSecSincePurge = 0; //Init amp-sec accumulator
  static float purgeTimeLeft = 0; //If purging, how long left to purge for

  /*If this function hasn't run for a while, reset the static variables*/
  if(currentTime - lastCallTime >= RESET_TIME){
    ampSecSincePurge = 0;
    purgeTimeLeft = 0;
    if(DEBUG)
      Serial.println("Timed out - resetting purge variables");
  }
  /*If currently purging, reset amp-sec accumulator, otherwise update it*/
  if(purgeValve)
    ampSecSincePurge = 0;
  else
    ampSecSincePurge += FCCurrent * (currentTime - lastCallTime); 
  /*If the accumulator reaches the threshold, start purging*/
  if(ampSecSincePurge > PURGE_INTERVAL){
    openPurgeValve = true;
    if(DEBUG)
      Serial.println("Purging Fuel Cell");
    purgeTimeLeft = PURGE_DURATION; //start purgeTimeLeft countdown
  }
  else{
    if(purgeTimeLeft > 0){ //If the timer hasn't reached zero
      openPurgeValve = true;
      purgeTimeLeft -= (currentTime - lastCallTime); //decrement remaining purge time by last cycle time
    }
    else{
      openPurgeValve = false; //Close purge valve
    }
  }

  if(debugThisCycle){
    Serial.print("Amp-sec accumulator:    ");
    Serial.println(ampSecSincePurge);
    if(purgeValve)
      Serial.println("Currently purging");
  }
  if(PURGE_DEBUG){
    Serial.print(currentTime, 4);
    Serial.print(", ");
    Serial.print(openPurgeValve);
    Serial.print(", ");
    Serial.print(ampSecSincePurge, 4);
    Serial.print(", ");
    Serial.print(purgeTimeLeft, 4);
    Serial.print(", ");
    Serial.println(calculateVoltage(analogRead(VOLTAGE_READ_PIN)));
  }
  lastCallTime = currentTime; //Update last call time
  return(openPurgeValve); //return appropriate purge valve state
}

unsigned char fanControl(){ //Fan PID function
  float FCTemp = calculateTemp(analogRead(TEMP_READ_PIN));
  float FCCurrent = calculateCurrent(analogRead(CURRENT_READ_PIN));
  float currentTime = millis();
  currentTime /= 1000;
  float pTerm = 0;
  float iTerm = 0;
  float dTerm = 0;
  float updatedFanCmd = 0;

  static float lastCallTime = 0;
  static float lastITerm = 0;
  static float lastTempDiff = 0;
  static float previousFanCmd = 0;
  
  float optTemp = lookupTemp(FCCurrent);

  float tempDiff = FCTemp - optTemp;

  if (currentTime - lastCallTime >= RESET_TIME){
    lastITerm = 0;
    lastTempDiff = tempDiff;
  }

  pTerm = FAN_P * tempDiff;
  iTerm = (FAN_I * tempDiff * (currentTime - lastCallTime)) + lastITerm;
  
  if(iTerm > FAN_MAX_DUTY)
    iTerm = FAN_MAX_DUTY;
  if(iTerm < 0)
    iTerm = 0;     //can adjust limits of I term in tuning

  dTerm = FAN_D * (tempDiff - lastTempDiff)/(currentTime - lastCallTime);

  updatedFanCmd = pTerm + iTerm + dTerm;

  if(updatedFanCmd > FAN_MAX_DUTY)
    updatedFanCmd = FAN_MAX_DUTY;
    
  if(updatedFanCmd < FAN_MIN_DUTY)
    updatedFanCmd = FAN_MIN_DUTY;
  
  previousFanCmd = updatedFanCmd;
  lastCallTime = currentTime;
  lastTempDiff = tempDiff;
  lastITerm = iTerm;
  if(PID_DEBUG){
    Serial.print(FCTemp);
    Serial.print(", ");
    Serial.print(optTemp);
    Serial.print(", ");
    Serial.print(pTerm);
    Serial.print(", ");
    Serial.print(iTerm);    
    Serial.print(", ");
    //Serial.print(dTerm);
    //Serial.print(", ");
    Serial.print(updatedFanCmd);
    Serial.print(", ");
    Serial.println((currentTime - lastCallTime));
  }
  return(updatedFanCmd);
}

float calculateCurrent(int analogReading){
  float result = 0;
  analogReading -= CURRENT_ZERO_POINT; 
  result = 5 * analogReading; //5V range of analogue input
  result /= 1024; //1024 resolution
  result *= 50; //50A nominal (from datasheet)
  result /= CURRENT_SENSITIVITY; //(sens. relative to nominal)

  for(int i=0;i<10;i++){
    int mask = 1;
    digitalWrite((i+44),(analogReading & (mask << i)));
  }
  
  /*if(debugThisCycle){
    Serial.print("Read Current: ");
    Serial.print(result);
    Serial.println("A");
  }*/
  
  return(result);
}

float pressMovingAvg(float newReading, bool addNewVal){
  static float readings[PRESS_AVG_READINGS];
  static bool firstRun = true;
  static float total = 0;
  static int index = 0;
  float average = 0;

  if(firstRun){
    for(int i=0; i < PRESS_AVG_READINGS; i++){
      readings[i] = 0;
    }
    firstRun=false;
  }
  if(addNewVal){
    total -= readings[index];
    readings[index] = newReading;
    total += readings[index];
    index++;
  }
  
  if(index >= PRESS_AVG_READINGS){
    index = 0;
  }

  average = total / PRESS_AVG_READINGS;
  return(average);
  
}

float calculateTemp(int analogReading){
  float temperature = TEMP_CALC_START;
  int k = 0;
  uint16_t testVal = pgm_read_word_near(analogVal + k);
  while((testVal > analogReading) && ((k+1) < TEMP_CALC_VALS)){
    k++;
    testVal = pgm_read_word_near(analogVal + k);
  } //should get the next 5 degree step up from current temperature
    //if greater resolution is needed, interpolation can be implemented
  temperature = TEMP_CALC_START + k * TEMP_CALC_STEP;
  return(temperature);
}

float calculateVoltage(int analogReading){
  float voltage = analogReading;
  voltage *= 5;
  voltage /= 1024;//convert analogread to voltage
  voltage *= 5.3; // voltage divider ratio
  return(voltage);
}

float calculatePressure(int analogReading){
  float out = analogReading;
  out -= 60;      // Zero point
  out /= 32;  // Use to calibrate scale 
  return(out);
}

float lookupTemp(float FCCurrent){
  float I[] = {0, 7.3, 14.5, 29, 51.7, 65.3, 77, 87.1}; // currents given in table
  float m[] = {0.548, 0.556, 0.483, 0.529, 0.589, 0.513, 0.495}; // slopes between given points
  float c[] = {26.000, 25.944, 27.000, 25.670, 22.588, 27.513, 28.881}; // y-intercepts for those slopes
  int k = 0;
  
  while((k<=6)&&(I[k+1] < FCCurrent))
    k++;

  float out = (m[k] * FCCurrent) + c[k]; //linear interpolation
  return(out);
}

float lookupTempMin(float FCCurrent){
  float I[] = {0, 7.3, 14.5, 29, 51.7, 65.3, 77, 87.1}; // currents given in table
  float m[] = {0.548, 0.556, 0.483, 0.529, 0.589, 0.513, 1.782}; // slopes between given points
  float c[] = {6.000, 5.944, 7.000, 5.670, 2.588, 7.513, -90.228}; // y-intercepts for those slopes
  int k = 0;
  
  while((k<=6)&&(I[k+1] < FCCurrent))
    k++;

  float out = (m[k] * FCCurrent) + c[k]; //linear interpolation
  return(out);
}

float lookupTempMax(float FCCurrent){
  float I[] = {0, 7.3, 14.5, 29, 51.7, 65.3, 77, 87.1}; // currents given in table
  float m[] = {0.411, 0.278, 0.345, 0.352, 0.368, 0.000, 0.000}; // slopes between given points
  float c[] = {52.000, 52.972, 52.000, 51.780, 50.992, 75.000, 75.000}; // y-intercepts for those slopes
  int k = 0;
  
  while((k<=6)&&(I[k+1] < FCCurrent))
    k++;

  float out = (m[k] * FCCurrent) + c[k]; //linear interpolation
  return(out);
}


void fuelCellFSM(){
  switch (FCState) {
    case STANDBY:
      FCStandbyDuring();
      break;
    case STARTUP:
      FCStartupDuring();
      break;
    case RUN:
      FCRunDuring();
      break;
    case SHUTDOWN:
      FCShutdownDuring();
      break;
    case ALARM:
      FCAlarmDuring();
      break;
  }
  return;
}

void enterFCState(FCStates state){
  if(DEBUG){
    //Serial.print("Entering state: ");
  }
  switch (state) {
    case STANDBY:
      if(DEBUG){
        //Serial.println("STANDBY");
      }
      FCStandbyEntry();
      FCState = STANDBY;
      break;
    case STARTUP:
      if(DEBUG){
        //Serial.println("STARTUP");
      }
      FCStartupEntry();
      FCState = STARTUP;
      break;
    case RUN:
      if(DEBUG){
        //Serial.println("RUN");
      }
      FCRunEntry();
      FCState = RUN;
      break;
    case SHUTDOWN:
      if(DEBUG){
        //Serial.println("SHUTDOWN");
      }
      FCShutdownEntry();
      FCState = SHUTDOWN;
      break;
    case ALARM:
      if(DEBUG){
        //Serial.println("ALARM");
      }
      FCAlarmEntry();
      FCState = ALARM;
      break;
  }
  return;
}

void FCStandbyEntry(){
  digitalWrite(FAN_PIN, LOW); //0%
  digitalWrite(SUPPLY_VALVE_PIN, LOW); //Closed
  digitalWrite(PURGE_VALVE_PIN, LOW); //Closed
  digitalWrite(FC_RELAY_PIN, LOW); //Open
  timerStartTime = millis();
  return;
}

void FCStandbyDuring(){
  if(((millis()-timerStartTime)>=STANDBY_DELAY_TIME) && FCOn){
    enterFCState(STARTUP);
  }
  return;
}

void FCStartupEntry(){
  startupState = FAN_SPOOLUP;
  FCStartupFanSpoolupEntry();
  return;
}

void FCStartupDuring(){
  switch (startupState) {
    case FAN_SPOOLUP:
      FCStartupFanSpoolupDuring();
      break;
    case STARTUP_PURGE:
      FCStartupPurgeDuring();
      break;
  }
  return;
}

void FCStartupFanSpoolupEntry(){
  if(DEBUG){
    Serial.println("Spooling up fan.");
  }
  analogWrite(FAN_PIN, map(FAN_SPOOLUP_DUTY,0,100,0,255)); //Start Fan
  digitalWrite(SUPPLY_VALVE_PIN, LOW); //Closed
  digitalWrite(PURGE_VALVE_PIN, LOW); //Closed
  digitalWrite(FC_RELAY_PIN, LOW); //Open
  fanStartTime = millis();
  return;
}

void FCStartupFanSpoolupDuring(){
  if((millis()-fanStartTime)>=FAN_SPOOLUP_TIME){
    startupState = STARTUP_PURGE;
    FCStartupPurgeEntry();
  }
  return;
}

void FCStartupPurgeEntry(){
  if(DEBUG){
    Serial.println("Performing startup purge.");
  }
  analogWrite(FAN_PIN, FAN_MIN_DUTY); //Start Fan
  digitalWrite(SUPPLY_VALVE_PIN, HIGH); //Open
  digitalWrite(PURGE_VALVE_PIN, HIGH); //Open
  digitalWrite(FC_RELAY_PIN, LOW); //Open
  /*TODO: apply load during this phase to prolong cell life*/
  purgeStartTime = millis();
  return;
}

void FCStartupPurgeDuring(){
  if((millis()-purgeStartTime)>=STARTUP_PURGE_TIME){
    startupState = END;
    FCStartupEndEntry();
  }
  return;
}

void FCStartupEndEntry(){
  analogWrite(FAN_PIN, FAN_MIN_DUTY); //Fan at minimum
  digitalWrite(SUPPLY_VALVE_PIN, HIGH); //Open
  digitalWrite(PURGE_VALVE_PIN, HIGH); //Open
  digitalWrite(FC_RELAY_PIN, HIGH); //Closed
  enterFCState(RUN);
  return;
}

void FCRunEntry(){
  return;
}

void FCRunDuring(){
  if(!FCOn){
    enterFCState(SHUTDOWN);
  }
  else{
    unsigned char fanOut = fanControl();
    fanOut = map(fanOut, 0, 100, 0, 255);
    if(DEBUG && debugThisCycle){
      Serial.print("Fan PWM:                ");
      Serial.println(fanOut);
    }
    analogWrite(FAN_PIN, fanControl()); //Use PID function
    digitalWrite(SUPPLY_VALVE_PIN, HIGH); //Open
    digitalWrite(PURGE_VALVE_PIN, purgeControl()); //Use purge control func.
    digitalWrite(FC_RELAY_PIN, HIGH); //Closed
  }
  return;
}

void FCShutdownEntry(){
  digitalWrite(FAN_PIN, LOW);//Fan off
  digitalWrite(SUPPLY_VALVE_PIN, LOW);//Closed
  digitalWrite(PURGE_VALVE_PIN, LOW);//Closed
  digitalWrite(FC_RELAY_PIN, LOW); //Open
  timerStartTime = millis();
  return;
}

void FCShutdownDuring(){
  if(((millis()-timerStartTime)>=SHUTDOWN_DELAY_TIME)){
    enterFCState(STANDBY);
  }
  return;
}

void FCAlarmEntry(){
  digitalWrite(FAN_PIN, LOW);//Fan off
  digitalWrite(SUPPLY_VALVE_PIN, LOW);//Closed
  digitalWrite(PURGE_VALVE_PIN, LOW);//Closed
  digitalWrite(FC_RELAY_PIN, LOW); //Open
  return;
}

void FCAlarmDuring(){
  //do nothing, wait for reset
  return;
}

void checkAlarms(){
  FCAlarm = false;
  static unsigned long HFCP = 0, HFCPD = 0, LFCP = 0, LFCPD = 0, HFCV = 0, LFCV = 0;
  static unsigned long HFCT = 0, HFCTD = 0, LFCT = 0, LFCTD = 0, HFCC = 0, LFCC = 0; //timers
  static unsigned long lastCalledMillis = 0;
  unsigned long currentMillis = millis();
  unsigned long interval = currentMillis - lastCalledMillis;
  /*Always-enabled alarms:*/
  float FCTemp = calculateTemp(analogRead(TEMP_READ_PIN));
  float FCCurrent = calculateCurrent(analogRead(CURRENT_READ_PIN));
  if(ALARM_DEBUG){
    /*Serial.print("checkAlarms() measures current at: ");
    Serial.print(FCCurrent);
    Serial.print("A. HFCC is: ");
    Serial.print(HFCC);
    Serial.print("ms. Int'vl is: ");
    Serial.print(interval);
    Serial.print("ms. ");*/
  }
  float FCVoltage = calculateVoltage(analogRead(VOLTAGE_READ_PIN));
  float FCPressure = pressMovingAvg(calculatePressure(analogRead(PRESSURE_READ_PIN)),true);
  
  if(FCTemp >= 75){
    HFCT += interval;
    if(HFCT >= 5000);{
      if(ALARM_DEBUG){
          Serial.println("HFCT");
        }
      FCAlarm = true;
      alarmState = HIGH_FC_TEMP;
    }
  }
  else{
    HFCT = 0;
  }
  
  if(FCTemp <= -10){
    LFCT += interval;
    if(LFCT >= 5000);{if(ALARM_DEBUG){
          Serial.println("LFCT");
        }
      FCAlarm = true;
      alarmState = LOW_FC_TEMP;
    }
  }
  else{
    LFCT = 0;
  }
  
  if(FCCurrent > 78){
    
    if(ALARM_DEBUG){
      Serial.print("Current measured >78A. ");
      Serial.print("Before incrementing, HFCC = ");
      Serial.println(HFCC);
    }
    HFCC += interval;
    if(ALARM_DEBUG){
      Serial.print("After incrementing, HFCC = ");
      Serial.println(HFCC);
    }
    if(ALARM_DEBUG){
      Serial.print("Current = ");
      Serial.print(FCCurrent);
      Serial.print("A, HFCC = ");
      Serial.println(HFCC);
    }
    if(HFCC >= 2000);{
      if(ALARM_DEBUG){
          Serial.println("HFCC > 2000. Alarm state triggered.");
        }
      FCAlarm = true;
      alarmState = HIGH_FC_CURRENT;
    }
  }
  else{
    HFCC = 0;
  }
  
  if(FCCurrent <= -3){
    LFCC += interval;
    if(LFCC >= 2000){
      if(ALARM_DEBUG){
          Serial.println("LFCC");
        }
      FCAlarm = true;
      alarmState = LOW_FC_CURRENT;
    }
  }
  else{
    LFCC = 0;
  }
  
  if(FCPressure >= 0.69){
    HFCP += interval;
    if(HFCP >= 1000){
      if(ALARM_DEBUG){
          Serial.println("HFCP");
        }
      FCAlarm = true;
      alarmState = HIGH_FC_PRES;
    }
  }
  else{
    HFCP = 0;
  }

  if(FCPressure >= 0.56){
    HFCPD += interval;
    if(HFCPD >= 5000){
      if(ALARM_DEBUG){
          Serial.println("HFCPD");
        }
      FCAlarm = true;
      alarmState = HIGH_FC_PRES_DELAYED;
    }
  }
  else{
    HFCPD = 0;
  }

  if(FCVoltage >= (1.1*CELLS)){
    HFCV += interval;
    if(HFCV >= 5000){
      if(ALARM_DEBUG){
          Serial.println("HFCV");
        }
      //FCAlarm = true;
      //alarmState = HIGH_FC_VOLTAGE;
    }
  }
  else{
    HFCV = 0;
  }

  if(((FCState==STARTUP) && (startupState==STARTUP_PURGE)) || (FCState == RUN)){
    if(FCPressure <= 0.07){
      LFCP += interval;
      if(LFCP >= 1000){
        FCAlarm = true;
        alarmState = LOW_FC_PRES;
        if(ALARM_DEBUG){
          Serial.println("LFCP");
        }
      }
    }
    else{
      LFCP = 0;
    }
  }

  if(FCState==RUN){
    if(FCVoltage <= 0.3 * CELLS){
      LFCV += interval;
      if(LFCV >=  1000 ){
        if(ALARM_DEBUG){
          Serial.println("LFCV");
        }
        //FCAlarm = true;
        //alarmState = LOW_FC_VOLTAGE;
      }
    }
    else{
      LFCV = 0;
    }

    if(FCPressure <= 0.16){
      if(ALARM_DEBUG){
        Serial.print("Press measured <0.16barg. ");
        Serial.print("Before incrementing, LFCPD = ");
        Serial.println(LFCPD);
      }
      LFCPD += interval;
      if(ALARM_DEBUG){
        Serial.print("After incrementing, LFCPD = ");
        Serial.println(LFCPD);
        Serial.print("Press = ");
        Serial.print(FCPressure);
        Serial.print("barg, LFCPD = ");
        Serial.println(LFCPD);
      }
      if(LFCPD >= 5000){
      
        if(ALARM_DEBUG){
          Serial.println("LFCPD");
        }
        FCAlarm = true;
        alarmState = LOW_FC_PRES_DELAYED;
      }
    }
    else{
      LFCPD = 0;
    }
  }
  
  if((FCState==STARTUP) || (FCState==RUN)){
    if(FCTemp >= lookupTempMax(FCCurrent)){
      HFCTD += interval;
      if(HFCTD >= 60000){
        if(ALARM_DEBUG){
          Serial.println("HFCTD");
        }
        FCAlarm = true;
        alarmState = HIGH_FC_TEMP_DELAYED;
      }
    }
    else{
      HFCTD = 0;
    }
    if(FCTemp <= lookupTempMin(FCCurrent)){
      LFCTD += interval;
      if(LFCTD >= 60000){
        if(ALARM_DEBUG){
          Serial.println("LFCTD");
        }
        FCAlarm = true;
        alarmState = LOW_FC_TEMP_DELAYED;
      }
    }
    else{
      LFCTD = 0;
    }
  }
  
  if(FCAlarm){
    if(ALARM_DEBUG){
          Serial.println("ENTERING ALARM");
          Serial.println(alarmState);
    }
    enterFCState(ALARM);
  }
  digitalWrite((31+alarmState),HIGH);
  lastCalledMillis = currentMillis;
  return;
}

