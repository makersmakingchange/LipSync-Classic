/*
//                                                                                                  
//  +++         .+++:    /++++++++/:.     .:/+++++/: .+++/`     .+++/  ++++.      ++++.     `-/++++++/:
//  oooo         .ooo:    +ooo:--:+ooo/   :ooo/:::/+/  -ooo+`   .ooo+`  ooooo:     .o-o`   `/ooo+//://+:
//  oooo         .ooo:    +ooo`    :ooo-  oooo`     `   .ooo+` .ooo+`   oooooo/`   .o-o`  .oooo-`       
//  oooo         .ooo:    +ooo`    -ooo-  -ooo+:.`       .ooo+.ooo/`    ooo:/oo+.  .o-o`  +ooo.         
//  oooo         .ooo:    +ooo.`..:ooo+`   `:+oooo+:`     `+ooooo/      ooo: :ooo- .o-o`  oooo          
//  oooo         .ooo:    +ooooooooo+:`       `-:oooo-     `+ooo/       ooo/  .+oo/.o-o`  +ooo.         
//  oooo         .ooo:    +ooo-...``             `oooo      /ooo.       ooo/   `/oo-o-o`  .oooo-        
//  oooo::::::.  .ooo:    +ooo`           :o//:::+ooo:      /ooo.       ooo/     .o-o-o`   ./oooo/:::/+/
//  +ooooooooo:  .ooo:    /ooo`           -/++ooo+/:.       :ooo.       ooo:      `.o.+      `-/+oooo+/-
//
//An open-source mouth operated sip and puff joystick that enables people with limited hand function to emulate a mouse on their computer and/or smartphone.
*/

//Developed by : MakersMakingChange
//Firmware : LipSync_Firmware
//VERSION: 2.71.1 (01 Apr 2021) 
//Copyright Neil Squire Society 2021.


#include <EEPROM.h>
#include <Mouse.h>
#include <math.h>

//***CUSTOMIZABLE VARIABLES***//
#define SPEED_COUNTER 5                           //Default cursor speed level
#define PRESSURE_THRESHOLD 10                     //Pressure sip and puff threshold
#define PRESSURE_THRESHOLD_MIN 5                  //Minimum Pressure sip and puff threshold
#define PRESSURE_THRESHOLD_MAX 50                 //Maximum Pressure sip and puff threshold
#define ROTATION_ANGLE 0                          //CCW Rotation angle between Screen "up" to LipSync "up" {0,90,180,270}  

#define PUFF_COUNT_THRESHOLD_MED 150              //Threshold between short and medium puff input in cycle counts 
#define PUFF_COUNT_THRESHOLD_LONG 750             //Threshold between medium and long puff in cycle counts 
#define SIP_COUNT_THRESHOLD_MED 150               //Threshold between short and medium puff input in cycle counts 
#define SIP_COUNT_THRESHOLD_LONG 750              //Threshold between medium and long puff in cycle counts

#define DEBUG_MODE false                          //Enable debug information to serial output (Default: false)
#define RAW_MODE false                             //Enable raw FSR readings to serial output (Default: false)
                                                  //Output: "RAW:1:xCursor,yCursor,Action:xUp,xDown,yUp,yDown"

// INPUTS: (1: Short puff, 2: Short sip, 3: Long puff, 4: Long sip, 5: Very long puff, 6: Very long sip)
// OUTPUTS: (0: Left Click, 1: Right Click, 2: Drag, 3: Scroll, 4: Middle Click, 5: Initialization, 6: Calibration )

// INPUT - ACTION Mapping     
#define INPUT_1 0                                 //A1.Short Puff: Left Click  
#define INPUT_2 1                                 //A2.Short Sip: Right Click  
#define INPUT_3 2                                 //A3.Long Puff: Drag
#define INPUT_4 3                                 //A4.Long Sip: Scroll
#define INPUT_5 5                                 //A5.Very Long Puff: Cursor Home Initialization
#define INPUT_6 4                                 //A6.Very Long Sip: Cursor Middle Click
      

//***DON'T CHANGE THESE VARIABLES***//
#define CURSOR_DEFAULT_SPEED 30                   //Maximum default USB cursor speed                  
#define CURSOR_DELTA_SPEED 5                      //Delta value that is used to calculate USB cursor speed levels
#define CURSOR_RADIUS 30.0                        //Joystick deadband
#define CURSOR_DEFAULT_COMP_FACTOR 1.0            //Default comp factor
#define CHANGE_DEFAULT_TOLERANCE 0.44             //The tolerance in % for changes between current reading and previous reading ( %100 is max FSRs reading )

//***PIN ASSIGNMENTS***//
#define BUTTON_UP_PIN 8                           // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define BUTTON_DOWN_PIN 7                         // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1_PIN 4                               // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2_PIN 5                               // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL_PIN A3                      // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4_PIN A4                               // Bluetooth PIO4_PIN Command Pin - digital output pin A4

#define PRESSURE_PIN A5                           // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH_PIN A0                         // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW_PIN A1                          // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH_PIN A2                         // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW_PIN A10                         // Y Direction Low (Cartesian negative y : down) - analog input pin A10

// LipSync EEPROM Memory Usage - DO NOT CHANGE
#define EEPROM_modelNumber 0          //int:0,1; 255 on fresh Arduino
#define EEPROM_speedCounter 2         //int:2,3; 
#define EEPROM_defaultIsSet 4         //int:4,5; 
#define EEPROM_yHighComp 6            //float:6,7,8,9; 
#define EEPROM_yLowComp 10            //float:10,11,12,13; 
#define EEPROM_xHighComp 14           //float:14,15,16,17; 
#define EEPROM_xLowComp 18            //float:18,19,20,21; 
#define EEPROM_xHighMax 22            //int:22,23; 
#define EEPROM_xLowMax 24             //int:24,25; 
#define EEPROM_yHighMax 26            //int:26,27; 
#define EEPROM_yLowMax 28             //int:28,29; 
#define EEPROM_rotationAngle 30       //int:30,31; 
#define EEPROM_pressureThreshold 32   //int:32,33; 
#define EEPROM_debugIntValue 34       //int:34,35; 
#define EEPROM_rawIntValue 36         //int:36,37;
#define EEPROM_deadzoneValue 38       //int:38,39;
#define EEPROM_buttonMode 40          //int:40,41;
#define EEPROM_buttonMapping0 42      //int:42,43; 
#define EEPROM_buttonMapping1 44      //int:44,45; 
#define EEPROM_buttonMapping2 46      //int:46,47; 
#define EEPROM_buttonMapping3 48      //int:48,49; 
#define EEPROM_buttonMapping4 50      //int:50,51; 
#define EEPROM_buttonMapping5 52      //int:52,53; 
#define EEPROM_configNumber 54        //int:54,55; 3 when Bluetooth configured 

//***SERIAL SETTINGS VARIABLE***//

#define API_ENABLED true                      //Enable API Serial interface = true , Disable API serial interface = false       

const int defaultButtonMapping[6] = {INPUT_1, INPUT_2, INPUT_3, INPUT_4, INPUT_5, INPUT_6};     //Default sip and puff buttons action on the factory reset

//***VARIABLE DECLARATION***//

//***Map Sip & Puff actions to cursor buttons for mode 1***//
int actionButton[6]; 

//Number of available sip and puff actions 
int actionButtonSize = sizeof(actionButton)/sizeof(int);

int xHigh, yHigh, xLow, yLow;                                                //Current FSR reading variables
int xHighPrev, yHighPrev, xLowPrev, yLowPrev;                                //Previous FSR reading variables                       
int xHighNeutral, xLowNeutral, yHighNeutral, yLowNeutral;                    //Individual neutral starting positions for each FSR

int xHighMax, xLowMax, yHighMax, yLowMax;         //Max FSR values which are set to the values from EEPROM

float xHighYHighRadius, xHighYLowRadius, xLowYLowRadius, xLowYHighRadius;
float xHighYHigh, xHighYLow, xLowYLow, xLowYHigh;

int xHighChangeTolerance, yHighChangeTolerance, xLowChangeTolerance, yLowChangeTolerance;       //The tolerance of changes in FSRs readings 

int cursorDeltaBox;                               //The delta value for the boundary range in all 4 directions about the x,y center
int cursorDelta;                                  //The amount cursor moves in some single or combined direction

unsigned int puffCount, sipCount;                 //The puff and long sip incremental counter variables

int pollCounter = 0;                              //Cursor poll counter

int xCursor = 0;                                  //Mouse cursor component
int yCursor = 0;                                  //Mouse cursor component

int cursorDelay;
float cursorFactor;
int cursorMaxSpeed;

float yHighComp = 1.0;
float yLowComp = 1.0;
float xHighComp = 1.0;
float xLowComp = 1.0;

float yHighDebug, yLowDebug, xHighDebug, xLowDebug;
int yHighMaxDebug, yLowMaxDebug, xHighMaxDebug, xLowMaxDebug;

//Cursor Speed Level structure 
typedef struct {
  int _delay;
  float _factor;
  int _maxSpeed;
} _cursor;

//Define characteristics of each speed level ( Example: 5,-1.0,10)
_cursor setting1 = {5, -1.1, CURSOR_DEFAULT_SPEED - (5 * CURSOR_DELTA_SPEED)};
_cursor setting2 = {5, -1.1, CURSOR_DEFAULT_SPEED - (4 * CURSOR_DELTA_SPEED)};
_cursor setting3 = {5, -1.1, CURSOR_DEFAULT_SPEED - (3 * CURSOR_DELTA_SPEED)};
_cursor setting4 = {5, -1.1, CURSOR_DEFAULT_SPEED - (2 * CURSOR_DELTA_SPEED)};
_cursor setting5 = {5, -1.1, CURSOR_DEFAULT_SPEED - (CURSOR_DELTA_SPEED)};
_cursor setting6 = {5, -1.1, CURSOR_DEFAULT_SPEED};
_cursor setting7 = {5, -1.1, CURSOR_DEFAULT_SPEED + (CURSOR_DELTA_SPEED)};
_cursor setting8 = {5, -1.1, CURSOR_DEFAULT_SPEED + (2 * CURSOR_DELTA_SPEED)};
_cursor setting9 = {5, -1.1, CURSOR_DEFAULT_SPEED + (3 * CURSOR_DELTA_SPEED)};
_cursor setting10 = {5, -1.1, CURSOR_DEFAULT_SPEED + (4 * CURSOR_DELTA_SPEED)};
_cursor setting11 = {5, -1.1, CURSOR_DEFAULT_SPEED + (5 * CURSOR_DELTA_SPEED)};

_cursor cursorParams[11] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9, setting10, setting11};

// Cursor Rotation Angle Variables
int rotationAngle=0;                                      //Declare rotation angle variables
float rotationAngle11, rotationAngle12, rotationAngle21, rotationAngle22;

int cursorSpeedCounter; 

float sipThreshold;                                     //Declare sip and puff variables 
float puffThreshold;
float cursorPressure;

int modelNumber;                                        //Declare LipSync model number variable 

bool debugModeEnabled;                                  //Declare raw and debug enable variable
bool rawModeEnabled;
bool settingsEnabled = false;                           //Serial input settings command mode enabled or disabled 


//-----------------------------------------------------------------------------------//

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {
  
  Serial.begin(115200);                           //Setting baud rate for serial communication which is used for diagnostic data returned from Bluetooth and microcontroller
  
  initializePins();                               //Initialize Arduino input and output pins

  Mouse.begin();                                  //Initialize the HID mouse functions
  delay(1000);
  
  getModelNumber(false);                          //Get LipSync model number; Perform factory reset on initial upload.
  delay(10);
  
  setCursorInitialization(1,false);               //Set the Home joystick and generate movement threshold boundaries
  delay(10);
  
  getCursorCalibration(false);                    //Get FSR Max calibration values 
  delay(10);
  
  getChangeTolerance(CHANGE_DEFAULT_TOLERANCE,false); // Get change tolerance using max FSR readings and default tolerance percentage 
  delay(10);
  
  getPressureThreshold(false);                    //Get the pressure sensor threshold boundaries
  delay(10);
  
  debugModeEnabled = getDebugMode(false);         //Get the debug mode state
  delay(10);
  
  rawModeEnabled = getRawMode(false);             //Get the raw mode state
  delay(50); 
  
  getCompFactor();                                //Get the default values that are stored in EEPROM
  delay(10);
  
  cursorSpeedCounter = getCursorSpeed(false);     //Read the saved cursor speed parameter from EEPROM
  cursorDelay = cursorParams[cursorSpeedCounter]._delay;
  cursorFactor = cursorParams[cursorSpeedCounter]._factor;
  cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;
  delay(10);
  
  getButtonMapping(false); 
  delay(10);
   
  rotationAngle = getRotationAngle(false);        //Read the saved rotation angle from EEPROM
  updateRotationAngle();
  delay(10);

  ledBlink(4, 250, 3);                            //End initialization visual feedback

  forceCursorDisplay();                           //Display cursor on screen by moving it
  
}

//-----------------------------------------------------------------------------------//

//***START OF MAIN LOOP***//

void loop() {
  
  settingsEnabled=serialSettings(settingsEnabled);       //Check to see if setting option is enabled in Lipsync

  cursorHandler();                                //Read the joystick values and output mouse cursor movements.

  //Perform sip and puff actions raw mode is disabled 
  if(!rawModeEnabled) {
    sipAndPuffHandler();                           //Pressure sensor sip and puff functions
  }                                                       
  delay(5);
  pushButtonHandler(BUTTON_UP_PIN,BUTTON_DOWN_PIN); 
}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------//


//*** CURSOR HANDLER FUNCTION***//

void cursorHandler(void) {

  // Reset cursor values
  xCursor = 0;
  yCursor = 0;

  // Measure force sensitive resitors
  xHigh = analogRead(X_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
  xLow  = analogRead(X_DIR_LOW_PIN);              //Read analog values of FSR's : A1
  yHigh = analogRead(Y_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
  yLow  = analogRead(Y_DIR_LOW_PIN);              //Read analog values of FSR's : A10

  //Check the FSR changes from previous reading and set the skip flag to true if the changes are in the change tolerance range
  bool skipChange = abs(xHigh - xHighPrev) < xHighChangeTolerance 
                 && abs(xLow  - xLowPrev)  < xLowChangeTolerance 
                 && abs(yHigh - yHighPrev) < yHighChangeTolerance 
                 && abs(yLow  - yLowPrev)  < yLowChangeTolerance;
  
  // Set FSR values for next skip check
  xHighPrev = xHigh;
  xLowPrev = xLow;
  yHighPrev = yHigh;
  yLowPrev = yLow;

  // Calculate the magnitude of the movement for each direction / quadrant
  xHighYHigh = sqrt(sq(((xHigh - xHighNeutral) > 0) ? (float)(xHigh - xHighNeutral) : 0.0) + sq(((yHigh - yHighNeutral) > 0) ? (float)(yHigh - yHighNeutral) : 0.0));     //The sq() function raises thr input to power of 2 and is returning the same data type int->int
  xHighYLow = sqrt(sq(((xHigh - xHighNeutral) > 0) ? (float)(xHigh - xHighNeutral) : 0.0) + sq(((yLow - yLowNeutral) > 0) ? (float)(yLow - yLowNeutral) : 0.0));    //The sqrt() function raises input to power 1/2, returning a float type
  xLowYHigh = sqrt(sq(((xLow - xLowNeutral) > 0) ? (float)(xLow - xLowNeutral) : 0.0) + sq(((yHigh - yHighNeutral) > 0) ? (float)(yHigh - yHighNeutral) : 0.0));          //These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xLowYLow = sqrt(sq(((xLow - xLowNeutral) > 0) ? (float)(xLow - xLowNeutral) : 0.0) + sq(((yLow - yLowNeutral) > 0) ? (float)(yLow - yLowNeutral) : 0.0));         //a larger digital value with a positive application force, a large negative difference

  //Check to see if the joystick has moved outside the deadband
  if ((xHighYHigh > xHighYHighRadius) || (xHighYLow > xHighYLowRadius) || (xLowYLow > xLowYLowRadius) || (xLowYHigh > xLowYHighRadius)) {
    
    pollCounter++;      //Add to the poll counter
    delay(20); 
    
    //Perform cursor movement actions if joystick has been in active zone for 3 or more poll counts
    if(!skipChange && pollCounter >= 3) {
        if ((xHighYHigh >= xHighYLow) && (xHighYHigh >= xLowYHigh) && (xHighYHigh >= xLowYLow)) {     //Quadrant 1
          xCursor = xCursorHigh(xHigh);
          yCursor = yCursorHigh(yHigh);
          
          (rawModeEnabled)? sendRawData(xCursor,yCursor,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow) : moveCursor(xCursor, yCursor, 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xHighYLow > xHighYHigh) && (xHighYLow > xLowYLow) && (xHighYLow > xLowYHigh)) {   //Quadrant 4
          xCursor = xCursorHigh(xHigh);
          yCursor = yCursorLow(yLow);
          
          (rawModeEnabled)? sendRawData(xCursor,yCursor,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow) : moveCursor(xCursor, yCursor, 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xLowYLow >= xHighYHigh) && (xLowYLow >= xHighYLow) && (xLowYLow >= xLowYHigh)) {  //Quadrant 3
          xCursor = xCursorLow(xLow);
          yCursor = yCursorLow(yLow);
          
          (rawModeEnabled)? sendRawData(xCursor,yCursor,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow) : moveCursor(xCursor, yCursor, 0);
          delay(cursorDelay);
          pollCounter = 0;
        } else if ((xLowYHigh > xHighYHigh) && (xLowYHigh >= xHighYLow) && (xLowYHigh >= xLowYLow)) { //Quadrant 2
          xCursor = xCursorLow(xLow);
          yCursor = yCursorHigh(yHigh);
          
          (rawModeEnabled)? sendRawData(xCursor,yCursor,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow) : moveCursor(xCursor, yCursor, 0);
          delay(cursorDelay);
          pollCounter = 0;
        }
    }   
  } else if(rawModeEnabled) {
    sendRawData(0,0,sipAndPuffRawHandler(),xHigh,xLow,yHigh,yLow);
    delay(cursorDelay);
    }

  //Debug information 
  
  if(debugModeEnabled) {
    Serial.print("LOG:3:");
    Serial.print(xHigh);
    Serial.print(",");
    Serial.print(xLow);
    Serial.print(",");
    Serial.print(yHigh);
    Serial.print(",");
    Serial.println(yLow); 
    delay(150);
  }
  
}

//***INITIALIZE PINS FUNCTION ***//
void initializePins(void) {
  pinMode(LED_1_PIN, OUTPUT);                     //Set the LED pin 1 as output(GREEN LED)
  pinMode(LED_2_PIN, OUTPUT);                     //Set the LED pin 2 as output(RED LED)
  pinMode(TRANS_CONTROL_PIN, OUTPUT);             //Set the transistor pin as output
  pinMode(PIO4_PIN, OUTPUT);                      //Set the bluetooth command mode pin as output

  pinMode(PRESSURE_PIN, INPUT);                   //Set the pressure sensor pin input
  pinMode(X_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Right FSR )
  pinMode(X_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Left FSR )
  pinMode(Y_DIR_HIGH_PIN, INPUT);                 //Define Force sensor pinsas input ( Up FSR )
  pinMode(Y_DIR_LOW_PIN, INPUT);                  //Define Force sensor pinsas input ( Down FSR )


  pinMode(BUTTON_UP_PIN, INPUT_PULLUP);           //Set increase cursor speed button pin as input
  pinMode(BUTTON_DOWN_PIN, INPUT_PULLUP);         //Set decrease cursor speed button pin as input

  pinMode(2, INPUT_PULLUP);                       //Set unused pins as inputs with pullups
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);
}


//***GET MODEL NUMBER FUNCTION***//

void getModelNumber(bool responseEnabled) {
  EEPROM.get(EEPROM_modelNumber, modelNumber);
  if (modelNumber != 1) {                          //If the previous firmware was different model then factory reset the settings 
    factoryReset(false);
    delay(10);
    
    modelNumber = 1;                               //And store the model number in EEPROM 
    EEPROM.put(EEPROM_modelNumber, modelNumber);
    delay(10);
  }  
  if(responseEnabled){
    Serial.println("SUCCESS,0:MN,0:1");
  }
}

//***GET VERSION FUNCTION***//

void getVersionNumber(void) {
  Serial.println("SUCCES,0:VN,0:V2.71");
}

//***GET CURSOR SPEED FUNCTION***//

int getCursorSpeed(bool responseEnabled) {
  int speedCounter = SPEED_COUNTER;
  if(API_ENABLED) {
    EEPROM.get(EEPROM_speedCounter, speedCounter);
    delay(5);
    if(speedCounter<0 || speedCounter >10){
      speedCounter = SPEED_COUNTER;
      EEPROM.put(EEPROM_speedCounter, speedCounter);
      delay(5);
    }
  } 
  
  if(responseEnabled){
    Serial.print("SUCCESS,0:SS,0:");
    Serial.println(speedCounter); 
    delay(5);     
  } 
  return speedCounter;
}

//***SET CURSOR SPEED FUNCTION***//

int setCursorSpeed(int inputSpeedCounter, bool responseEnabled) {

  bool isValidSpeed = true;
  int speedCounter = inputSpeedCounter;
  if(speedCounter>=0 && speedCounter <=10){
    EEPROM.put(EEPROM_speedCounter, speedCounter);
    delay(10);
    if(!API_ENABLED){ speedCounter = SPEED_COUNTER; }
    isValidSpeed = true;
  } else {
    EEPROM.get(EEPROM_speedCounter, speedCounter);
    delay(10); 
    isValidSpeed = false;
  }
  delay(5); 
  
 if(responseEnabled) {
    (isValidSpeed) ? Serial.print("SUCCESS,0:SS,1:"):Serial.print("FAIL,2:SS,1:");
    Serial.println(inputSpeedCounter); 
    delay(5);
  }  
  return speedCounter;
}

//***INCREASE CURSOR SPEED LEVEL FUNCTION***//

int increaseCursorSpeed(int speedCounter,bool responseEnabled) {
  speedCounter++;

  if (speedCounter == 11) {
    ledBlink(6, 50, 3);
    speedCounter = 10;
  } else {
    ledBlink(speedCounter+1, 100, 1);
    
    cursorDelay = cursorParams[speedCounter]._delay;
    cursorFactor = cursorParams[speedCounter]._factor;
    cursorMaxSpeed = cursorParams[speedCounter]._maxSpeed;

    EEPROM.put(EEPROM_speedCounter, speedCounter);
    delay(25);
  }
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:"); 
  Serial.print("SS,2:");
  Serial.println(speedCounter); 
  delay(5);
  return speedCounter;
}

//***DECREASE CURSOR SPEED LEVEL FUNCTION***//

int decreaseCursorSpeed(int speedCounter,bool responseEnabled) {
  speedCounter--;
  if (speedCounter == -1) {
    ledBlink(6, 50, 3);
    speedCounter = 0;
  } else if (speedCounter == 0) {
    ledBlink(1, 350, 1);
    cursorDelay = cursorParams[speedCounter]._delay;
    cursorFactor = cursorParams[speedCounter]._factor;
    cursorMaxSpeed = cursorParams[speedCounter]._maxSpeed;

    EEPROM.put(EEPROM_speedCounter, speedCounter);
    delay(25);
 
  } else {
    ledBlink(speedCounter+1, 100, 1);

    cursorDelay = cursorParams[speedCounter]._delay;
    cursorFactor = cursorParams[speedCounter]._factor;
    cursorMaxSpeed = cursorParams[speedCounter]._maxSpeed;

    EEPROM.put(EEPROM_speedCounter, speedCounter);
    delay(25);
  }
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:"); 
  Serial.print("SS,2:");
  Serial.println(speedCounter);  
  delay(5);
  return speedCounter;
}

//***GET PRESSURE THRESHOLD FUNCTION***//
void getPressureThreshold(bool responseEnabled) {
  float pressureNominal = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  int pressureThreshold = PRESSURE_THRESHOLD;
  if(API_ENABLED) {
    EEPROM.get(EEPROM_pressureThreshold, pressureThreshold);
    delay(5);
    if(pressureThreshold<=PRESSURE_THRESHOLD_MIN || pressureThreshold>PRESSURE_THRESHOLD_MAX) {
      EEPROM.put(EEPROM_pressureThreshold, PRESSURE_THRESHOLD);
      delay(5);
      pressureThreshold = PRESSURE_THRESHOLD;
    }    
  } 
  
  sipThreshold = pressureNominal + ((pressureThreshold * 5.0)/100.0);    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  puffThreshold = pressureNominal - ((pressureThreshold * 5.0)/100.0);   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  if(responseEnabled) {
    Serial.print("SUCCESS,0:PT,0:");
    Serial.print(pressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal);
    delay(5);
  }
}

//***SET PRESSURE THRESHOLD FUNCTION***//

void setPressureThreshold(int inputPressureThreshold, bool responseEnabled) {
  bool isValidThreshold = true;
  int pressureThreshold = inputPressureThreshold;
  float pressureNominal = (((float)analogRead(PRESSURE_PIN)) / 1024.0) * 5.0; // Read neutral pressure transducer analog value [0.0V - 5.0V]

  if (pressureThreshold>=5 && pressureThreshold<=50) {
    EEPROM.put(EEPROM_pressureThreshold, pressureThreshold); // Update value to memory from serial input
    delay(10); 
    if(!API_ENABLED){ pressureThreshold = PRESSURE_THRESHOLD; }
    // Update threshold variables
    sipThreshold = pressureNominal + ((pressureThreshold * 5.0)/100.0);    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
    puffThreshold = pressureNominal - ((pressureThreshold * 5.0)/100.0);   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
    isValidThreshold = true;
  } else {
    EEPROM.get(EEPROM_pressureThreshold, pressureThreshold);
    delay(10); 
    isValidThreshold = false;
  }
  delay(5); 
  
 if(responseEnabled) {
    (isValidThreshold) ? Serial.print("SUCCESS,0:PT,1:"):Serial.print("FAIL,2:PT,1:");
    Serial.print(inputPressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal); 
    delay(5);
  }  
}

//***GET DEBUG MODE STATE FUNCTION***//

bool getDebugMode(bool responseEnabled) {
  bool debugState=DEBUG_MODE;
  int debugIntValue;
  if(API_ENABLED) {
    EEPROM.get(EEPROM_debugIntValue, debugIntValue);
    delay(5);
    if(debugIntValue!=0 && debugIntValue!=1) {
      EEPROM.put(EEPROM_debugIntValue, DEBUG_MODE);
      delay(5);
      debugState=DEBUG_MODE;
      }   
  } else {
    debugState=DEBUG_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    Serial.print("SUCCESS,0:DM,0:");
    Serial.println(debugState); 
    delay(5);
    if(debugState){
      sendDebugData();
    }
   }
  return debugState;
}

//***SET DEBUG MODE STATE FUNCTION***//

bool setDebugMode(bool debugState,bool responseEnabled) {

  (debugState) ? EEPROM.put(EEPROM_debugIntValue, 1) : EEPROM.put(EEPROM_debugIntValue, 0);
  delay(10);
  if(!API_ENABLED) { debugState=DEBUG_MODE; }    
  delay(5);
  
  if(responseEnabled) {
    Serial.print("SUCCESS,0:DM,1:");
    Serial.println(debugState); 
    delay(5);
    if(debugState){
      sendDebugData();
    }
   }
  return debugState;
}

//***SEND DEBUG DATA FUNCTION***//

void sendDebugData() {
  delay(100);
  Serial.print("LOG:1:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  delay(100);
  Serial.print("LOG:2:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(100);
}

//***SEND RAW DATA FUNCTION***//
// Output format: "RAW:1:xCursor,yCursor,Action:xUp,xDown,yUp,yDown"

void sendRawData(int x, int y, int action, int xUp, int xDown,int yUp,int yDown) {

  Serial.print("RAW:1:"); 
  Serial.print(x); 
  Serial.print(","); 
  Serial.print(y); 
  Serial.print(",");
  Serial.print(action); 
  Serial.print(":"); 
  Serial.print(xUp); 
  Serial.print(","); 
  Serial.print(xDown); 
  Serial.print(",");
  Serial.print(yUp); 
  Serial.print(",");
  Serial.println(yDown); 
  
}

//***GET RAW MODE STATE FUNCTION***//

bool getRawMode(bool responseEnabled) {
  bool rawState=RAW_MODE;
  int rawIntValue;
  if(API_ENABLED) {
    EEPROM.get(EEPROM_rawIntValue, rawIntValue);
    delay(5);
    if(rawIntValue!=0 && rawIntValue!=1) { 
      EEPROM.put(EEPROM_rawIntValue, RAW_MODE);
      delay(5);
      rawState=RAW_MODE;
      }   
  } else {
    rawState=RAW_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    Serial.print("SUCCESS,0:RM,0:");
    Serial.println(rawState); 
    delay(5);
   }
  return rawState;
}

//***SET RAW MODE STATE FUNCTION***//

bool setRawMode(bool rawState,bool responseEnabled) {
  
  (rawState) ? EEPROM.put(EEPROM_rawIntValue, 1) : EEPROM.put(EEPROM_rawIntValue, 0);
  delay(5);    
  if(!API_ENABLED) { rawState=RAW_MODE; }
  delay(5);    
    
  if(responseEnabled) {
    Serial.print("SUCCESS,0:RM,1:");
    Serial.println(rawState); 
    delay(5);
   }
  return rawState;
}

//***GET COMP FACTOR VALUES FUNCTION***///

void getCompFactor(void) {

  int compFactorIsSet;
  float defaultCompFactor = CURSOR_DEFAULT_COMP_FACTOR;

  EEPROM.get(EEPROM_defaultIsSet, compFactorIsSet);
  delay(10);

  if (compFactorIsSet == 1) {
    //Get the Comp values from Memory 
    EEPROM.get(EEPROM_yHighComp, yHighComp);
    delay(10);
    EEPROM.get(EEPROM_yLowComp, yLowComp);
    delay(10);
    EEPROM.get(EEPROM_xHighComp, xHighComp);
    delay(10);
    EEPROM.get(EEPROM_xLowComp, xLowComp);
    delay(10);
  } else {
    //Set the Comp values for first time
    EEPROM.put(EEPROM_yHighComp, defaultCompFactor);
    delay(10);

    EEPROM.put(EEPROM_yLowComp, defaultCompFactor);
    delay(10);

    EEPROM.put(EEPROM_xHighComp, defaultCompFactor);
    delay(10);

    EEPROM.put(EEPROM_xLowComp, defaultCompFactor);
    delay(10);

    compFactorIsSet = 1;
    EEPROM.put(EEPROM_defaultIsSet, compFactorIsSet);
    delay(10);
  }


}

//***SET COMP FACTOR VALUES FUNCTION***///

void setCompFactor(void) {
  int xMax = (xHighMax > xLowMax) ? xHighMax : xLowMax;
  int yMax = (yHighMax > yLowMax) ? yHighMax : yLowMax;
  float finalMax = (xMax > yMax) ? (float)xMax : (float)yMax;

  yHighComp = (finalMax - yHighNeutral) / (yHighMax - yHighNeutral);
  yLowComp = (finalMax - yLowNeutral) / (yLowMax - yLowNeutral);
  xHighComp = (finalMax - xHighNeutral) / (xHighMax - xHighNeutral);
  xLowComp = (finalMax - xLowNeutral) / (xLowMax - xLowNeutral);

  EEPROM.put(EEPROM_yHighComp, yHighComp);
  delay(10);
  EEPROM.put(EEPROM_yLowComp, yLowComp);
  delay(10);
  EEPROM.put(EEPROM_xHighComp, xHighComp);
  delay(10);
  EEPROM.put(EEPROM_xLowComp, xLowComp);
  delay(10);
}

//***GET CURSOR INITIALIZATION FUNCTION***//

void getCursorInitialization() {
  Serial.print("SUCCESS,0:IN,0:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  delay(10);  
}

//***SET CURSOR INITIALIZATION FUNCTION***//

void setCursorInitialization(int mode, bool responseEnabled) {

  ledOn(1); //Turn on Green LED

  xHigh = analogRead(X_DIR_HIGH_PIN);               //Set the initial neutral x-high value of joystick
  delay(10);

  xLow = analogRead(X_DIR_LOW_PIN);                 //Set the initial neutral x-low value of joystick
  delay(10);

  yHigh = analogRead(Y_DIR_HIGH_PIN);               //Set the initial neutral y-high value of joystick
  delay(10);

  yLow = analogRead(Y_DIR_LOW_PIN);                 //Set the initial Initial neutral y-low value of joystick
  delay(10);

  //Set the neutral values 
  xHighPrev = xHighNeutral = xHigh;
  xLowPrev = xLowNeutral = xLow;
  yHighPrev = yHighNeutral = yHigh;
  yLowPrev = yLowNeutral = yLow;

  //Get comp factors from memory if mode set to 1 
  if(mode==1){
    getCompFactor();
  } //Recalculate and set comp factors to memory if mode set to 2
  else if(mode==2) {
    setCompFactor();
  }

  //Get the Comp values from Memory 
  EEPROM.get(EEPROM_yHighComp, yHighComp);
  delay(10);
  EEPROM.get(EEPROM_yLowComp, yLowComp);
  delay(10);
  EEPROM.get(EEPROM_xHighComp, xHighComp);
  delay(10);
  EEPROM.get(EEPROM_xLowComp, xLowComp);
  delay(10);

  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.print("IN,1:"); 
  Serial.print(xHighNeutral); 
  Serial.print(","); 
  Serial.print(xLowNeutral); 
  Serial.print(",");
  Serial.print(yHighNeutral); 
  Serial.print(",");
  Serial.println(yLowNeutral); 
  
  ledClear();
}

//*** GET CURSOR CALIBRATION FUNCTION***//

void getCursorCalibration(bool responseEnable) {
  
  //Get the max values from Memory 
  EEPROM.get(EEPROM_xHighMax, xHighMax);
  delay(10);
  EEPROM.get(EEPROM_xLowMax, xLowMax);
  delay(10);
  EEPROM.get(EEPROM_yHighMax, yHighMax);
  delay(10);
  EEPROM.get(EEPROM_yLowMax, yLowMax);
  delay(10);

  xHighYHighRadius = CURSOR_RADIUS;
  xHighYLowRadius = CURSOR_RADIUS;
  xLowYLowRadius = CURSOR_RADIUS;
  xLowYHighRadius = CURSOR_RADIUS;

  if(responseEnable){
    Serial.print("SUCCESS,0:CA,0:"); 
    Serial.print(xHighMax); 
    Serial.print(","); 
    Serial.print(xLowMax); 
    Serial.print(",");
    Serial.print(yHighMax); 
    Serial.print(",");
    Serial.println(xHighMax); 
  }
  delay(10);
}

//*** SET CURSOR CALIBRATION FUNCTION***//

void setCursorCalibration(bool responseEnabled) {

  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:0");                                                   //Start the joystick calibration sequence 
  ledBlink(4, 300, 3);

  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:1"); 
  ledBlink(6, 500, 1);
  yHighMax = analogRead(Y_DIR_HIGH_PIN);
  ledBlink(1, 1000, 2);

  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:2"); 
  ledBlink(6, 500, 1);
  xHighMax = analogRead(X_DIR_HIGH_PIN);
  ledBlink(1, 1000, 2);

  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:3"); 
  ledBlink(6, 500, 1);
  yLowMax = analogRead(Y_DIR_LOW_PIN);
  ledBlink(1, 1000, 2);

  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:4"); 
  ledBlink(6, 500, 1);
  xLowMax = analogRead(X_DIR_LOW_PIN);
  ledBlink(1, 1000, 2);

  setCompFactor();

  EEPROM.put(EEPROM_xHighMax, xHighMax);
  delay(10);
  EEPROM.put(EEPROM_xLowMax, xLowMax);
  delay(10);
  EEPROM.put(EEPROM_yHighMax, yHighMax);
  delay(10);
  EEPROM.put(EEPROM_yLowMax, yLowMax);
  delay(10);

  ledBlink(5, 250, 3);

   (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.print("CA,1:5:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(xHighMax); 
  delay(10);
}

//*** GET CHANGE TOLERANCE VALUE CALIBRATION FUNCTION***//

void getChangeTolerance(float changePercent, bool responseEnabled) {
  xHighChangeTolerance=(int)(xHighMax * (changePercent/100.0));
  xLowChangeTolerance=(int)(xLowMax * (changePercent/100.0));
  yHighChangeTolerance=(int)(yHighMax * (changePercent/100.0));
  yLowChangeTolerance=(int)(yLowMax * (changePercent/100.0));
  if(responseEnabled){
    Serial.print("SUCCESS,0:CT,0:"); 
    Serial.print(changePercent); 
    Serial.print(","); 
    Serial.print(xHighChangeTolerance); 
    Serial.print(","); 
    Serial.print(xLowChangeTolerance); 
    Serial.print(","); 
    Serial.print(yHighChangeTolerance); 
    Serial.print(",");
    Serial.println(yLowChangeTolerance); 
  }
  delay(10);
}

//***GET BUTTON MAPPING FUNCTION***//

void getButtonMapping(bool responseEnabled) {
  bool isValidMapping = true;
  memcpy(actionButton, defaultButtonMapping, actionButtonSize);     //Copy the default sip and puff button action mapping
  
  if (API_ENABLED) {
    for (int i = 0; i < actionButtonSize; i++) {                    //Check if it's a valid mapping
      int buttonMapping;
      EEPROM.get(EEPROM_buttonMapping0+i*2, buttonMapping);
      delay(10);
      if(buttonMapping<0 || buttonMapping >7) {
        isValidMapping = false;
        break;
      } else {
        actionButton[i]=buttonMapping;
        delay(5);
      }
    }
    if(!isValidMapping){
      for(int i = 0; i < actionButtonSize; i++){                       //Save the default mapping into EEPROM if it's not a valid mapping
        EEPROM.put(EEPROM_buttonMapping0+i*2, defaultButtonMapping[i]);
        delay(10);
        actionButton[i]=defaultButtonMapping[i];
        delay(5);
      }
    }   
  }
  if(responseEnabled) {
    Serial.print("SUCCESS,0:MP,0:");
    Serial.print(actionButton[0]); 
    Serial.print(actionButton[1]); 
    Serial.print(actionButton[2]); 
    Serial.print(actionButton[3]); 
    Serial.print(actionButton[4]); 
    Serial.println(actionButton[5]); 
    delay(5);
   }
}

//***SET BUTTON MAPPING FUNCTION***//

void setButtonMapping(int buttonMapping[],bool responseEnabled) {
  
  bool isValidMapping = true;
  
   for(int i = 0; i < actionButtonSize; i++){           //Check if it's a valid mapping
    if(buttonMapping[i]<0 || buttonMapping[i] >7) {     // Up to 8 input actions but 6 available 
      isValidMapping = false;
      break;
    }
   }
   
   if(isValidMapping){                                  //Save the mapping into EEPROM if it's a valid mapping
    for(int i = 0; i < actionButtonSize; i++){
      EEPROM.put(EEPROM_buttonMapping0+i*2, buttonMapping[i]);
      delay(10);
      actionButton[i]=buttonMapping[i];
      delay(5);
    }     
    if(!API_ENABLED) { memcpy(actionButton, defaultButtonMapping, actionButtonSize);  }
   } 
   delay(5);

  if(responseEnabled) {
    (isValidMapping) ? Serial.print("SUCCESS,0:MP,1:") : Serial.print("FAIL,2:MP,1:");
    Serial.print(buttonMapping[0]); 
    Serial.print(buttonMapping[1]); 
    Serial.print(buttonMapping[2]); 
    Serial.print(buttonMapping[3]); 
    Serial.print(buttonMapping[4]); 
    Serial.println(buttonMapping[5]); 
    delay(5);
   }
}

//***GET ROTATION ANGLE FUNCTION***///

int getRotationAngle(bool responseEnabled) {
  int tempRotationAngle = ROTATION_ANGLE;
   
   if(API_ENABLED) {
     //Get the rotation angle from memory 
      EEPROM.get(EEPROM_rotationAngle, tempRotationAngle);
      delay(10);
   } else {
      tempRotationAngle = ROTATION_ANGLE;
   }

  if(responseEnabled) {
    Serial.print("SUCCESS,0:RA,0:");
    Serial.println(tempRotationAngle); 
    delay(5);
   }

   return tempRotationAngle;
}

//***SET ROTATION ANGLE FUNCTION***///  

void setRotationAngle(int inputRotationAngle, bool responseEnabled) {

  bool isValidRotationAngle = true;
  
  if(inputRotationAngle >= 0 && inputRotationAngle <=360) {
    rotationAngle = inputRotationAngle; //update value to global variable
    EEPROM.put(EEPROM_rotationAngle, rotationAngle); // Update value to memory from serial input
    delay(10);
    if(!API_ENABLED) {rotationAngle = ROTATION_ANGLE; }    //Use default rotation angle if bad serial input
    isValidRotationAngle = true;
  } else {
    isValidRotationAngle = false;
  }
  delay(5);
  
  if(responseEnabled) {
    (isValidRotationAngle) ? Serial.print("SUCCESS,0:RA,1:"):Serial.print("FAIL,2:RA,1:");
    Serial.println(inputRotationAngle);
    delay(5);
  }

  updateRotationAngle(); // Update rotation transform

}

//***UPDATE ROTATION ANGLES FUNCTION***///

void updateRotationAngle(void){
  
  // Update rotation angle variables
  float rotationAngleRad = rotationAngle * M_PI / 180.0; //convert rotation angle from degrees to radians

  //calculate transform matrix elements.
  rotationAngle11 = cos(rotationAngleRad);
  rotationAngle12 = sin(rotationAngleRad);
  rotationAngle21 = -rotationAngle12; // -sin(rotation_angle_rad)
  rotationAngle22 = rotationAngle11; //cos(rotation_angle_rad)
  
}



//***FACTORY RESET FUNCTION***//

void factoryReset(bool responseEnabled) { 
           
    setCursorSpeed(SPEED_COUNTER,false);                                  // set default cursor speed counter
    delay(10);
    
    setPressureThreshold(PRESSURE_THRESHOLD, false);                      //set default pressure threshold
    delay(10);
    
    setRotationAngle(ROTATION_ANGLE, false);                              //set default rotation angle
    delay(10);
    
    setDebugMode(DEBUG_MODE,false);                                       //set default debug mode
    delay(10);  
    
    //EEPROM.put(EEPROM_RawIntValue, RAW_MODE);
    setRawMode(RAW_MODE,false);                                           //set default button mapping
    delay(10);  
    
    setButtonMapping(defaultButtonMapping,false);                         //set default action mapping
    delay(10);

    //Set the default values
    cursorSpeedCounter = SPEED_COUNTER; 
    cursorDelay =     cursorParams[cursorSpeedCounter]._delay;
    cursorFactor =    cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed =  cursorParams[cursorSpeedCounter]._maxSpeed;
    
    debugModeEnabled = DEBUG_MODE;  
    rawModeEnabled = RAW_MODE;

    getCompFactor();                                          
    delay(10);
      

  if(responseEnabled) {
    Serial.println("SUCCESS,0:FR,0:0");
    delay(5);
   }
   ledBlink(2, 250, 1);
}

//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String inString = "";  
    bool settingsFlag = enabled;                   //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
     if (Serial.available()>0)  
     {  
       inString = Serial.readString();            //Check if serial has received or read input string and word "SETTINGS" is in input string.
       if (settingsFlag==false && inString=="SETTINGS") {
        Serial.println("SUCCESS,0:SETTINGS");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && inString=="EXIT") {
        Serial.println("SUCCESS,0:EXIT");
       settingsFlag=false;                         //Set the return flag to false so settings actions can be exited
       }
       else if (settingsFlag==true && isValidCommandFormat(inString)){ //Check if command's format is correct and it's in settings mode
        writeSettings(inString);                  //Sub function to process valid strings
        settingsFlag=false;   
       }
       else {
        Serial.print("FAIL,0:");
        Serial.println(inString);
        settingsFlag=false;      
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***VALIDATE INPUT COMMAND FORMAT FUNCTION***//

bool isValidCommandFormat (String inputCommand) {
  bool isValidFormart;
  if ((inputCommand.length()==(6) || inputCommand.length()==(7) || inputCommand.length()==(8) || inputCommand.length()==(11)) && inputCommand.charAt(2)==',' && inputCommand.charAt(4)==':'){ 
    isValidFormart = true;
   }
   else {
     isValidFormart = false;
   }
  return isValidFormart;
}

//***VALIDATE INPUT COMMAND PARAMETER FUNCTION***//

bool isValidCommandParamter(String paramterString) {
  bool isValidParamter;
  if (isStrNumber(paramterString)){ 
    isValidParamter = true;
   }
   else {
     isValidParamter = false;
   }
  return isValidParamter;
}

//***PERFORM SETTINGS FUNCTION TO CHANGE SETTINGS USING SOFTWARE***//

void writeSettings(String inputChangeString) {
  
    String changeString = inputChangeString;
    changeString.replace(",","");                 //Remove commas 
    changeString.replace(":","");                 //Remove :
    
    String paramterString = changeString.substring(3);
    char changeChar[changeString.length()+1];
    changeString.toCharArray(changeChar, changeString.length()+1);

    //Check if command's parameter is valid
    if(!isValidCommandParamter(paramterString)) {
      Serial.print("FAIL,2:");
      Serial.println(inputChangeString);
      delay(5);   
    }
    //Get Model number : "MN,0:0"
    else if(changeChar[0]=='M' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getModelNumber(true);
      delay(5);
    } 
    //Get version number : "VN,0:0"
    else if(changeChar[0]=='V' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getVersionNumber();
      delay(5);
    }   
    //Get cursor speed value if received "SS,0:0", decrease the cursor if received "SS,1:1" and increase the cursor speed if received "SS,1:2" , and set the cursor speed value if received "SS,2:{speed 0 to 10}"
    else if(changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      cursorSpeedCounter = getCursorSpeed(true);
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='1' && ( changeString.length()==4 || changeString.length()==5)) {
      cursorSpeedCounter = setCursorSpeed(paramterString.toInt(),true);
      delay(5);
    } 
    else if(changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='2' && changeChar[3]=='1' && changeString.length()==4) {
      cursorSpeedCounter = decreaseCursorSpeed(cursorSpeedCounter,true);
      delay(5);
    } else if (changeChar[0]=='S' && changeChar[1]=='S' && changeChar[2]=='2' && changeChar[3]=='2' && changeString.length()==4) {
      cursorSpeedCounter = increaseCursorSpeed(cursorSpeedCounter,true);
      delay(5);
    }
     //Get pressure threshold values if received "PT,0:0" and set pressure threshold values if received "PT,1:{threshold 1% to 50%}"
      else if(changeChar[0]=='P' && changeChar[1]=='T' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getPressureThreshold(true);
      delay(5);
    } else if (changeChar[0]=='P' && changeChar[1]=='T' && changeChar[2]=='1' && ( changeString.length()==4 || changeString.length()==5)) {
      setPressureThreshold(paramterString.toInt(),true);
      delay(5);
    } 
     //Get rotation angle values if received "RA,0:0" and set rotation angle values if received "RA,1:{0,90,180,270}"
      else if(changeChar[0]=='R' && changeChar[1]=='A' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getRotationAngle(true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='A' && changeChar[2]=='1' && ( changeString.length()==4 || changeString.length()==5 || changeString.length()==6)) {
      setRotationAngle(paramterString.toInt(),true);
      delay(5);
    } 
     //Get debug mode value if received "DM,0:0" , set debug mode value to 0 if received "DM,1:0" and set debug mode value to 1 if received "DM,1:1"
     else if(changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      debugModeEnabled = getDebugMode(true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='0' && changeString.length()==4) {
      debugModeEnabled = setDebugMode(0,true);
      delay(5);
    } else if (changeChar[0]=='D' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      debugModeEnabled = setDebugMode(1,true);
      delay(5);
    } 
    //Get raw mode value if received "RM,0:0" , set raw mode value to 0 if received "RM,1:0" and set raw mode value to 1 if received "RM,1:1"
     else if(changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      rawModeEnabled = getRawMode(true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='0' && changeString.length()==4) {
      rawModeEnabled = setRawMode(0,true);
      delay(5);
    } else if (changeChar[0]=='R' && changeChar[1]=='M' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      rawModeEnabled = setRawMode(1,true);
      delay(5);
    } 
     //Get cursor initialization values if received "IN,0:0" and perform cursor initialization if received "IN,1:1"
     else if(changeChar[0]=='I' && changeChar[1]=='N' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getCursorInitialization();
      delay(5);
    } else if (changeChar[0]=='I' && changeChar[1]=='N' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      setCursorInitialization(2,true);
      delay(5);
    } 
     //Get cursor calibration values if received "CA,0:0" and perform cursor calibration if received "CA,1:1"
      else if(changeChar[0]=='C' && changeChar[1]=='A' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getCursorCalibration(true);
      delay(5);
    } else if (changeChar[0]=='C' && changeChar[1]=='A' && changeChar[2]=='1' && changeChar[3]=='1' && changeString.length()==4) {
      setCursorCalibration(true);
      delay(5);
    } 
     //Get change tolerance values if received "CT,0:0" 
      else if(changeChar[0]=='C' && changeChar[1]=='T' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getChangeTolerance(CHANGE_DEFAULT_TOLERANCE,true);
      delay(5);
    }
    //Get Button mapping : "MP,0:0" , Set Button mapping : "MP,1:012345"
    else if (changeChar[0]=='M' && changeChar[1]=='P' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      getButtonMapping(true);
      delay(5);
    } else if(changeChar[0]=='M' && changeChar[1]=='P' && changeChar[2]=='1' && changeString.length()==9) {
      int tempButtonMapping[actionButtonSize];
      for(int i = 0; i< actionButtonSize; i++){
         tempButtonMapping[i]=changeChar[3+i] - '0';          //Convert char arrat to int array
      }
      setButtonMapping(tempButtonMapping,true);
      delay(5);
    }
     //Perform factory reset if received "FR,0:0"
     else if(changeChar[0]=='F' && changeChar[1]=='R' && changeChar[2]=='0' && changeChar[3]=='0' && changeString.length()==4) {
      factoryReset(true);
      delay(5);
    } else {
      Serial.print("FAIL,1:");            //Invalid command
      Serial.println(inputChangeString);
      delay(5);        
      }
}

//***PUSH BUTTON SPEED HANDLER FUNCTION***//

void pushButtonHandler(int switchPin1, int switchPin2) {
    //Cursor speed control push button functions below
  if (digitalRead(switchPin1) == LOW) {
    delay(200);
    clearButtonAction();
    delay(50);
    if (digitalRead(switchPin2) == LOW) {
      setCursorCalibration(false);                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      cursorSpeedCounter = increaseCursorSpeed(cursorSpeedCounter,false);                      //Call increase cursor speed function if push button up is pressed 
    }
  }

  if (digitalRead(switchPin2) == LOW) {
    delay(200);
    clearButtonAction();
    delay(50);
    if (digitalRead(switchPin1) == LOW) {
      setCursorCalibration(false);                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      cursorSpeedCounter = decreaseCursorSpeed(cursorSpeedCounter,false);                      //Call increase cursor speed function if push button up is pressed 
    }
  }
}

//***SIP AND PUFF ACTION HANDLER FUNCTION***//

void sipAndPuffHandler() {
  //Read pressure sensor for sip and puff functions
  cursorPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   //Read the pressure transducer analog value and convert it a voltage between [0.0V - 5.0V]

  //Check if the pressure is under puff pressure threshold 
  if (cursorPressure < puffThreshold) {             
    //Puff detected
    while (cursorPressure < puffThreshold) { // Continue measuring pressure until puff stops
      cursorPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      puffCount++;                                //Count how long the pressure value has been under puff pressure threshold
      if (puffCount == PUFF_COUNT_THRESHOLD_MED) { // When first count threshold is reached, turn on light
        ledOn(2); //Turn on RED LED  
      } else if (puffCount == PUFF_COUNT_THRESHOLD_LONG) {
        ledClear(); //Turn off RED LED
      }
      delay(5);
    }

    //USB puff actions 
      if (puffCount < PUFF_COUNT_THRESHOLD_MED) {
        performButtonAction(actionButton[0]);
      } else if (puffCount >= PUFF_COUNT_THRESHOLD_MED && puffCount < PUFF_COUNT_THRESHOLD_LONG) {
        performButtonAction(actionButton[2]);
      } else if (puffCount >= PUFF_COUNT_THRESHOLD_LONG) {
        performButtonAction(actionButton[4]);
      }
    puffCount = 0;                                //Reset puff counter
  }

  //Check if the pressure is above sip pressure threshold 
  if (cursorPressure > sipThreshold) {
    // Sip detected
    while (cursorPressure > sipThreshold) { // Continue measuring pressure until sip stops
      cursorPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;
      sipCount++;                                 //Count how long the pressure value has been above sip pressure threshold
      if (sipCount == SIP_COUNT_THRESHOLD_MED) { // When first count threshold is reached, turn on light
        ledOn(1); //Turn on green led
      } else if(sipCount == SIP_COUNT_THRESHOLD_LONG){
        ledClear(); // Turn off LEDs.
      }
      delay(5);
    }

    //USB Sip actions 
      if (sipCount < SIP_COUNT_THRESHOLD_MED) {
        performButtonAction(actionButton[1]);
      } else if (sipCount >= SIP_COUNT_THRESHOLD_MED && sipCount < SIP_COUNT_THRESHOLD_LONG) {
        performButtonAction(actionButton[3]);
      } else if(sipCount >= SIP_COUNT_THRESHOLD_LONG){
        //Perform seconday function if sip counter value is more than 750 ( 5 second Long Sip )
        performButtonAction(actionButton[5]);
      }
    sipCount = 0;                                 //Reset sip counter
  }
}

int sipAndPuffRawHandler() {
  int currentAction = 0;
  cursorPressure = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;   
  
  //Measure the pressure value and compare the result with puff pressure Thresholds 
  if (cursorPressure < puffThreshold) {
        delay(5);
        currentAction = 1;
  }
  //Measure the pressure value and compare the result with sip pressure Thresholds 
  if (cursorPressure > sipThreshold) {
        delay(5);
        currentAction = 2;
  }
  return currentAction;
}

void clearButtonAction(){
  ledClear();
  if (Mouse.isPressed(MOUSE_LEFT)) {
    Mouse.release(MOUSE_LEFT);
  } else if (Mouse.isPressed(MOUSE_RIGHT)) {
    Mouse.release(MOUSE_RIGHT);
  } 
  delay(5);
}

void performButtonAction(int actionButtonNumber) {
    switch (actionButtonNumber) {
      case 0: {
        //Left Click: Perform mouse left click action if puff counter value is under 150 ( 1 Second Short Puff )
        ledClear();
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.click(MOUSE_LEFT);
          delay(5);
        }
        break;
      }
      case 1: {
        //Right Click: Perform mouse right click action if sip counter value is under 150 ( 1 Second Short Sip )
        ledClear();
        Mouse.click(MOUSE_RIGHT);
        delay(5);
        break;
      }
      case 2: {
        //Drag: Perform mouse left press action ( Drag Action ) if puff counter value is under 750 and more than 150 ( 3 Second Long Puff )
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
          ledClear();
        } else {
          ledOn(2); //Turn on RED LED
          Mouse.press(MOUSE_LEFT);
          delay(5);
        }
        break;
      }
      case 3: {
        //Scroll: Perform mouse scroll action if sip counter value is under 750 and more than 150 ( 3 Second Long Sip )
        ledOn(1); // Turn on Green LED
        cursorScroll();
        delay(5);
        break;
      }
      case 4: {
        //Perform cursor middle click
        cursorMiddleClick();
        delay(5);
        break;
      }
      case 5: {
        //Cursor Initialization: Perform cursor manual home initialization to reset default value of FSR's if puff counter value is more than 750 ( 5 second Long Puff )
        clearButtonAction();
        ledClear();
        ledBlink(4, 350, 3); 
        setCursorInitialization(2,false);
        delay(5);
        break;
      }
      case 6: {
        //Cursor Calibration: Perform cursor Calibration to reset default value of FSR's if puff counter value is more than 750 ( 5 second Long Puff )
        clearButtonAction();
        ledClear();
        setCursorCalibration(false);
        delay(5);
        break;
      }
    }
}

//***LED ON FUNCTION***//

void ledOn(int ledNumber) {
  switch (ledNumber) {
    case 1: { //Turn GREEN LED on
        digitalWrite(LED_1_PIN, HIGH);
        delay(5);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
    case 2: { // Turn RED LED on
        digitalWrite(LED_2_PIN, HIGH);
        delay(5);
        digitalWrite(LED_1_PIN, LOW);
        break;
      }
  }
}

//***LED CLEAR FUNCTION***//

void ledClear(void) {
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
}

//***LED BLINK FUNCTION***//

void ledBlink(int numBlinks, int delayBlinks, int ledNumber) {
  if (numBlinks < 0) numBlinks *= -1;

  switch (ledNumber) {
    case 1: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < numBlinks; i++) {
          digitalWrite(LED_1_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_1_PIN, LOW);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, HIGH);
          delay(delayBlinks);
          digitalWrite(LED_2_PIN, LOW);
          delay(delayBlinks);
        }
        break;
      }
    case 6: {
        digitalWrite(LED_1_PIN, LOW);
        digitalWrite(LED_2_PIN, LOW);
        break;
      }
  }
}

//***FORCE DISPLAY OF CURSOR***//

void forceCursorDisplay(void) {
  Mouse.move(1, 0, 0);
  delay(25);
  Mouse.move(-1, 0, 0);
  delay(25);
}


//***SECONDARY ACTION FUNCTION SELECTION***//

void secondaryAction(void) {
  while (1) {
    xHigh = analogRead(X_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
    xLow = analogRead(X_DIR_LOW_PIN);               //Read analog values of FSR's : A1
    yHigh = analogRead(Y_DIR_HIGH_PIN);             //Read analog values of FSR's : A2
    yLow = analogRead(Y_DIR_LOW_PIN);               //Read analog values of FSR's : A10

    digitalWrite(LED_2_PIN, HIGH);                  //Turn red LED on

    //todo implement angle code

    if (xHigh > (xHighNeutral + 50)) {
      cursorMiddleClick();
      break;
    } else if (xLow > (xLowNeutral + 50)) {
      cursorMiddleClick();
      break;
    } else if (yHigh > (yHighNeutral + 50)) {
      cursorSwipe();
      break;
    } else if (yLow > (yLowNeutral + 50)) {
      cursorSwipe();
      break;
    }
  }
  digitalWrite(LED_2_PIN, LOW);
}

//***CURSOR MOVEMENT FUNCTION ***//
void moveCursor(int xCursor, int yCursor, int wheel){
  
  // Apply rotation transform to inputs
  int uCursor = rotationAngle11*xCursor + rotationAngle12*yCursor; 
  int vCursor = rotationAngle21*xCursor + rotationAngle22*yCursor;
  
  Mouse.move(uCursor, vCursor, wheel);                //output transformed mouse movement

}

//***SWIPE FUNCTION***//

void cursorSwipe(void) {
  
  for (int i = 0; i < 3; i++) Mouse.move(0, 126, 0);
  Mouse.press(MOUSE_LEFT);
  delay(125);

  for (int j = 0; j < 3; j++) Mouse.move(0, -126, 0);
  Mouse.release(MOUSE_LEFT);
  delay(125);
}

//***CURSOR MIDDLE CLICK FUNCTION***//

void cursorMiddleClick(void) {
  Mouse.click(MOUSE_MIDDLE);
  delay(125);
}

//***CURSOR SCROLL FUNCTION***//

void cursorScroll(void) {
  if(debugModeEnabled) {
    Serial.println("cursorScroll Mode Started");
  }
  
  while (1) { //continue in scroll mode until released by a sip or a puff input
    
    int xCursor = 0;
    int yCursor = 0;
   
    // read joystick movements
    xHigh = analogRead(X_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
    xLow = analogRead(X_DIR_LOW_PIN);               //Read analog values of FSR's : A1
    yHigh = analogRead(Y_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
    yLow = analogRead(Y_DIR_LOW_PIN);               //Read analog values of FSR's : A10

    
    // Read sip and puff input
    float scrollRelease = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;

    // Read FSR Inputs
    xHighYHigh = sqrt(sq(((xHigh - xHighNeutral) > 0) ? (float)(xHigh - xHighNeutral) : 0.0) + sq(((yHigh - yHighNeutral) > 0) ? (float)(yHigh - yHighNeutral) : 0.0));     //The sq() function raises thr input to power of 2 and is returning the same data type int->int
    xHighYLow = sqrt(sq(((xHigh - xHighNeutral) > 0) ? (float)(xHigh - xHighNeutral) : 0.0) + sq(((yLow - yLowNeutral) > 0) ? (float)(yLow - yLowNeutral) : 0.0));    //The sqrt() function raises input to power 1/2, returning a float type
    xLowYHigh = sqrt(sq(((xLow - xLowNeutral) > 0) ? (float)(xLow - xLowNeutral) : 0.0) + sq(((yHigh - yHighNeutral) > 0) ? (float)(yHigh - yHighNeutral) : 0.0));          //These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
    xLowYLow = sqrt(sq(((xLow - xLowNeutral) > 0) ? (float)(xLow - xLowNeutral) : 0.0) + sq(((yLow - yLowNeutral) > 0) ? (float)(yLow - yLowNeutral) : 0.0));         //a larger digital value with a positive application force, a large negative difference

    //Check to see if the joystick has moved
    if ((xHighYHigh > xHighYHighRadius) || (xHighYLow > xHighYLowRadius) || (xLowYLow > xLowYLowRadius) || (xLowYHigh > xLowYHighRadius)) {
      
      //Joystick moved - determine which quadrant     
      if ((xHighYHigh >= xHighYLow) && (xHighYHigh >= xLowYHigh) && (xHighYHigh >= xLowYLow)) {     //Quadrant 1
            xCursor = xCursorHigh(xHigh);
            yCursor = yCursorHigh(yHigh);
                        
          } else if ((xHighYLow > xHighYHigh) && (xHighYLow > xLowYLow) && (xHighYLow > xLowYHigh)) {   //Quadrant 4
            xCursor = xCursorHigh(xHigh);
            yCursor = yCursorLow(yLow);            
            
          } else if ((xLowYLow >= xHighYHigh) && (xLowYLow >= xHighYLow) && (xLowYLow >= xLowYHigh)) {  //Quadrant 3
            xCursor = xCursorLow(xLow);
            yCursor = yCursorLow(yLow);
           
            
          } else if ((xLowYHigh > xHighYHigh) && (xLowYHigh >= xHighYLow) && (xLowYHigh >= xLowYLow)) { //Quadrant 2
            xCursor = xCursorLow(xLow);
            yCursor = yCursorHigh(yHigh);
            
          }

        // Apply rotation transform to inputs
        int uCursor = rotationAngle11*xCursor + rotationAngle12*yCursor; 
        int vCursor = rotationAngle21*xCursor + rotationAngle22*yCursor;

        Mouse.move(0, 0, -1* vCursor); // Apply vertical direction to scroll 
        delay(cursorDelay * 35);  // 5 x 35 = 175 ms
        
    
    }
    else if ((scrollRelease > sipThreshold) || (scrollRelease < puffThreshold)) { // if sip or puff, stop scroll mode
      break;
    }
        
    delay(cursorDelay);
  }
    if(debugModeEnabled) {
    Serial.println("cursorScroll Mode Ended");
  }
}

//***CHECK IF STRING IS A NUMBER FUNCTION***//

boolean isStrNumber(String str){
  
  for(byte i=0;i<str.length();i++)
  {
    if(!isDigit(str.charAt(i))) return false;
  }
  return true;
}

//***Y HIGH CURSOR MOVEMENT MODIFIER FUNCTION***//

int yCursorHigh(int j) {

  if (j > yHighNeutral) {
    //Calculate Y up factor ( 1.25 multiplied by Y high comp multiplied by ratio of Y value to Y High Maximum value )
    float yHighNeutral_factor = 1.25 * (yHighComp * (((float)(j - yHighNeutral)) / (yHighMax - yHighNeutral)));

    //Use the calculated Y up factor to none linearize the maximum speeds
    int k = (int)(round(-1.0 * pow(cursorMaxSpeed, yHighNeutral_factor)) - 1.0);

    //Select maximum speed
    int maxSpeed = round(-1.0 * pow(cursorMaxSpeed, 1.25*yHighComp)) - 1.0;

    //Map the value to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(-1.0 * pow(cursorMaxSpeed, 1.25*yHighComp)) - 1.0), 0, maxSpeed); 

    //Set a constrain
    k = constrain(k,-1 * cursorMaxSpeed, 0);

    return k;
  } else {
    return 0;
  }
}

//***Y LOW CURSOR MOVEMENT MODIFIER FUNCTION***//

int yCursorLow(int j) {

  if (j > yLowNeutral) {
    //Calculate Y down factor ( 1.25 multiplied by Y low comp multiplied by ratio of Y value to Y Low Maximum value )
    float yLowNeutral_factor = 1.25 * (yLowComp * (((float)(j - yLowNeutral)) / (yLowMax - yLowNeutral)));

    //Use the calculated Y down factor to none linearize the maximum speeds
    int k = (int)(round(1.0 * pow(cursorMaxSpeed, yLowNeutral_factor)) - 1.0);

    //Select maximum speed
    int maxSpeed = round(1.0 * pow(cursorMaxSpeed, 1.25*yLowComp)) - 1.0;

    //Map the values to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(1.0 * pow(cursorMaxSpeed, 1.25*yLowComp)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain   
    k = constrain(k,0,cursorMaxSpeed);
    
    return k;
  } else {
    return 0;
  }
}

//***X HIGH CURSOR MOVEMENT MODIFIER FUNCTION***//

int xCursorHigh(int j) {

  if (j > xHighNeutral) {
    //Calculate X right factor ( 1.25 multiplied by X high comp multiplied by ratio of X value to X High Maximum value )
    float xHighNeutral_factor = 1.25 * (xHighComp * (((float)(j - xHighNeutral)) / (xHighMax - xHighNeutral)));

    //Use the calculated X down factor to none linearize the maximum speeds
    int k = (int)(round(1.0 * pow(cursorMaxSpeed, xHighNeutral_factor)) - 1.0);

    //Select maximum speed
    int maxSpeed = round(1.0 * pow(cursorMaxSpeed, 1.25*xHighComp)) - 1.0;

    //Map the values to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(1.0 * pow(cursorMaxSpeed, 1.25*xHighComp)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain
    k = constrain(k,0,cursorMaxSpeed);
    
    return k;
  } else {
    return 0;
  }
}

//***X LOW CURSOR MOVEMENT MODIFIER FUNCTION***//

int xCursorLow(int j) {

  if (j > xLowNeutral) {
    //Calculate X left factor ( 1.25 multiplied by X low comp multiplied by ratio of X value to X low Maximum value )
    float xLowNeutral_factor = 1.25 * (xLowComp * (((float)(j - xLowNeutral)) / (xLowMax - xLowNeutral)));

    //Use the calculated X down factor to none linearize the maximum speeds
    int k = (int)(round(-1.0 * pow(cursorMaxSpeed, xLowNeutral_factor)) - 1.0);

    //Select maximum speed
    int maxSpeed = round(-1.0 * pow(cursorMaxSpeed, 1.25*xLowComp)) - 1.0;

    //Map the values to a value between 0 and the selected maximum speed
    k = map(k, 0, (round(-1.0 * pow(cursorMaxSpeed, 1.25*xLowComp)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain 
    k = constrain(k,-1 * cursorMaxSpeed, 0);
     
    return k;
  } else {
    return 0;
  }
}
