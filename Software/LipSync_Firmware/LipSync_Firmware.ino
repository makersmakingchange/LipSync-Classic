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
//VERSION: 3.0-beta (21 May Apr 2021) 
//Copyright Neil Squire Society 2021.


#include <EEPROM.h>
#include <Mouse.h>
#include <math.h>

                                                 
//***OUTPUT ACTIONS***// - DO NOT CHANGE 
// These are the different actions the LipSync can perform based on different sip and puff inputs.
#define OUTPUT_NOTHING 0                              // No action
#define OUTPUT_LEFT_CLICK 1                           // Generates a short left click
#define OUTPUT_RIGHT_CLICK 2                          // Generates a short right click
#define OUTPUT_DRAG 3                                 // Initiates drag mode, holding down left click until cancelled
#define OUTPUT_SCROLL 4                               // Initiates scroll mode. Vertical motion generates mouse scroll wheel movement.
#define OUTPUT_MIDDLE_CLICK 5                         // Generates a short middle click
#define OUTPUT_CURSOR_HOME_RESET 6                    // Initiates the cursor home reset routine to reset center position. 
#define OUTPUT_CURSOR_CALIBRATION 7                   // Initiates the cursor calibration to calibrate joystick range and reset center position.

//***OUTPUT MAPPING***// - CUSTOMIZABLE
//These values can be changed to remap different output actions to different input actions
#define ACTION_SHORT_PUFF   OUTPUT_LEFT_CLICK        //Default: Left Click
#define ACTION_SHORT_SIP    OUTPUT_RIGHT_CLICK       //Default: Right Click
#define ACTION_LONG_PUFF    OUTPUT_DRAG              //Default: Drag
#define ACTION_LONG_SIP     OUTPUT_SCROLL            //Default: Scroll
#define ACTION_VLONG_PUFF   OUTPUT_CURSOR_HOME_RESET //Default: Cursor Home Reset
#define ACTION_VLONG_SIP    OUTPUT_NOTHING           //Default: No action

//Example - Reverse sip and puff so puff does right click and sip does left click.
//To use these settings, uncomment these lines and comment out the corresponding lines above.
//#define ACTION_SHORT_PUFF   OUTPUT_RIGHT_CLICK     
//#define ACTION_SHORT_SIP    OUTPUT_LEFT_CLICK        


//***CUSTOMIZABLE VARIABLES***//
#define ROTATION_ANGLE 0                          //CCW Rotation angle between Screen "up" to LipSync "up" {0,90,180,270}  igin/v3.0

#define PUFF_COUNT_THRESHOLD_MED 150              //Threshold between short and medium puff input in cycle counts
#define PUFF_COUNT_THRESHOLD_LONG 750             //Threshold between medium and long puff in cycle counts
#define SIP_COUNT_THRESHOLD_MED 150               //Threshold between short and medium puff input in cycle counts
#define SIP_COUNT_THRESHOLD_LONG 750              //Threshold between medium and long puff in cycle counts

#define CURSOR_RADIUS 30                          //Joystick deadband
#define CURSOR_DEFAULT_SPEED 30                   //Default USB cursor speed     
#define SPEED_COUNTER 5                           //Default cursor speed level

//***DON'T CHANGE THESE VARIABLES***//            

#define PRESSURE_THRESHOLD 10                     //Pressure sip and puff threshold
#define PRESSURE_THRESHOLD_MIN 5                  //Minimum Pressure sip and puff threshold
#define PRESSURE_THRESHOLD_MAX 50                 //Maximum Pressure sip and puff threshold
#define ROTATION_ANGLE 0                          //CCW Rotation angle between Screen "up" to LipSync "up" {0,90,180,270}  
#define DEBUG_MODE false                          //Enable debug information to serial output (Default: false)
#define RAW_MODE false                             //Enable raw FSR readings to serial output (Default: false)
                                                  //Output: "RAW:1:xCursor,yCursor,Action:xUp,xDown,yUp,yDown" 
//***DON'T CHANGE THESE VARIABLES***//
#define CURSOR_DEFAULT_SPEED 30                   //Maximum default USB cursor speed                  
#define CURSOR_DELTA_SPEED 5                      //Delta value that is used to calculate USB cursor speed levels
#define CURSOR_DEADBAND 30                        //Joystick deadband
#define CURSOR_DEFAULT_COMP_FACTOR 1.0            //Default comp factor
#define CHANGE_DEFAULT_TOLERANCE 0.44             //The tolerance in % for changes between current reading and previous reading ( %100 is max FSRs reading )
#define INPUT_ACTION_COUNT 6                      //Number of available sip and puff input types  
#define CURSOR_LIFT_THRESOLD 100                  //Opposite FSR value nearing liftoff during purposeful movement (ADC steps)

const int BUTTON_MAPPING[INPUT_ACTION_COUNT] = 
  {ACTION_SHORT_PUFF, ACTION_SHORT_SIP, ACTION_LONG_PUFF, 
   ACTION_LONG_SIP, ACTION_VLONG_PUFF, ACTION_SHORT_PUFF};     
const int DEFAULT_BUTTON_MAPPING[INPUT_ACTION_COUNT] = {1, 2, 3, 4, 6, 0};     //MMC default sip and puff buttons actions

//***DON'T CHANGE THESE VARIABLES***//
#define XHIGH_DIRECTION 1                         //Mouthpiece right movements correspond to positive (i.e. right) mouse movement
#define XLOW_DIRECTION -1                         //Mouthpiece left movements correspond to negative (i.e. left) mouse movement
#define YHIGH_DIRECTION -1                        //Mouthpiece up movements correspond to negative (i.e. up) mouse movement
#define YLOW_DIRECTION 1                          //Mouthpiece down movements correspond to positive (i.e. down) mouse movement
      

//***PIN ASSIGNMENTS***// - DO NOT CHANGE
#define LED_1_PIN 4                               // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2_PIN 5                               // LipSync LED Color2 : RED - digital outputpin 4
#define BUTTON_DOWN_PIN 7                         // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define BUTTON_UP_PIN 8                           // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define TRANS_CONTROL_PIN A3                      // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4_PIN A4                               // Bluetooth PIO4_PIN Command Pin - digital output pin A4
#define PRESSURE_PIN A5                           // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH_PIN A0                         // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW_PIN A1                          // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH_PIN A2                         // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW_PIN A10                         // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***LIPSYNC EEPROM MEMORY***// - DO NOT CHANGE
#define EEPROM_modelNumber 0                      //int:0,1; 255 on fresh Arduino
#define EEPROM_speedCounter 2                     //int:2,3; 
#define EEPROM_defaultIsSet 4                     //int:4,5; 
#define EEPROM_yHighComp 6                        //float:6,7,8,9; 
#define EEPROM_yLowComp 10                        //float:10,11,12,13; 
#define EEPROM_xHighComp 14                       //float:14,15,16,17; 
#define EEPROM_xLowComp 18                        //float:18,19,20,21; 
#define EEPROM_xHighMax 22                        //int:22,23; 
#define EEPROM_xLowMax 24                         //int:24,25; 
#define EEPROM_yHighMax 26                        //int:26,27; 
#define EEPROM_yLowMax 28                         //int:28,29; 
#define EEPROM_rotationAngle 30                   //int:30,31; 
#define EEPROM_pressureThreshold 32               //int:32,33; 
#define EEPROM_debugModeEnabled 34                //int:34,35; 
#define EEPROM_rawModeEnabled 36                  //int:36,37;
#define EEPROM_deadzoneValue 38                   //int:38,39;
#define EEPROM_buttonMode 40                      //int:40,41;
#define EEPROM_buttonMapping1 42                  //int:42,43; 
#define EEPROM_buttonMapping2 44                  //int:44,45; 
#define EEPROM_buttonMapping3 46                  //int:46,47; 
#define EEPROM_buttonMapping4 48                  //int:48,49; 
#define EEPROM_buttonMapping5 50                  //int:50,51; 
#define EEPROM_buttonMapping6 52                  //int:52,53; 
#define EEPROM_configNumber 54                    //int:54,55; 3 when Bluetooth configured 

typedef void (*FunctionPointer)(bool,int);

typedef struct {
  String _command;
  String _parameter;
  FunctionPointer _function;
} _functionList;

_functionList getModelNumberFunction =          {"MN,0","0",&getModelNumber};
_functionList getVersionNumberFunction =        {"VN,0","0",&getVersionNumber};
_functionList getCursorSpeedFunction =          {"SS,0","0",&getCursorSpeed};
_functionList setCursorSpeedFunction =          {"SS,1","",&setCursorSpeed};
_functionList decreaseCursorSpeedFunction =     {"SS,2","1",&decreaseCursorSpeed};
_functionList increaseCursorSpeedFunction =     {"SS,2","2",&increaseCursorSpeed};

_functionList getPressureThresholdFunction =    {"PT,0","0",&getPressureThreshold};
_functionList setPressureThresholdFunction =    {"PT,1","",&setPressureThreshold};
_functionList getRotationAngleFunction =        {"RA,0","0",&getRotationAngle};
_functionList setRotationAngleFunction =        {"RA,1","",&setRotationAngle};
_functionList getDebugModeFunction =            {"DM,0","0",&getDebugMode};
_functionList setDebugModeFunction =            {"DM,1","",&setDebugMode};

_functionList getRawModeFunction =              {"RM,0","0",&getRawMode};
_functionList setRawModeFunction =              {"RM,1","",&setRawMode};
_functionList getCursorInitializationFunction = {"IN,0","0",&getCursorInitialization};
_functionList setCursorInitializationFunction = {"IN,1","1",&setCursorInitialization};
_functionList getCursorCalibrationFunction =    {"CA,0","0",&getCursorCalibration};
_functionList setCursorCalibrationFunction =    {"CA,1","1",&setCursorCalibration};

_functionList getChangeToleranceFunction =      {"CT,0","0",&getChangeTolerance};
_functionList getButtonMappingFunction =        {"MP,0","0",&getButtonMapping};
_functionList setButtonMappingFunction =        {"MP,1","r",&setButtonMapping}; //"r" denotes an array parameter 
_functionList factoryResetFunction =            {"FR,1","1",&factoryReset};

/*
_functionList apiFunction[22] = {getModelNumberFunction, 
getVersionNumberFunction,
getCursorSpeedFunction,
setCursorSpeedFunction,
decreaseCursorSpeedFunction,
increaseCursorSpeedFunction,
getPressureThresholdFunction,
setPressureThresholdFunction,
getRotationAngleFunction,
setRotationAngleFunction,
getDebugModeFunction,
setDebugModeFunction,
getRawModeFunction,
setRawModeFunction,
getCursorInitializationFunction,
setCursorInitializationFunction,
getCursorCalibrationFunction,
setCursorCalibrationFunction,
getChangeToleranceFunction,
getButtonMappingFunction,
setButtonMappingFunction,
factoryResetFunction
};
 */
_functionList apiFunction[18] = {getModelNumberFunction, 
getVersionNumberFunction,
getCursorSpeedFunction,
setCursorSpeedFunction,
decreaseCursorSpeedFunction,
increaseCursorSpeedFunction,
getPressureThresholdFunction,
setPressureThresholdFunction,
getRotationAngleFunction,
setRotationAngleFunction,
getDebugModeFunction,
setDebugModeFunction,
getRawModeFunction,
setRawModeFunction,
getCursorInitializationFunction,
setCursorInitializationFunction,
getButtonMappingFunction,
setButtonMappingFunction
};

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
  

//***SERIAL SETTINGS VARIABLE***//
#define API_ENABLED true                      //Enable API Serial interface = true , Disable API serial interface = false       

//***VARIABLE DECLARATION***//

//***Map Sip & Puff actions to cursor buttons for mode 1***//
int actionButton[INPUT_ACTION_COUNT]; 

int xHigh, yHigh, xLow, yLow;                                                //Current FSR reading variables
int xHighPrev, yHighPrev, xLowPrev, yLowPrev;                                //Previous FSR reading variables                       
int xHighNeutral, xLowNeutral, yHighNeutral, yLowNeutral;                    //Individual neutral starting positions for each FSR

int xHighMax, xLowMax, yHighMax, yLowMax;         //Max FSR values which are set to the values from EEPROM

float xHighYHigh, xHighYLow, xLowYLow, xLowYHigh;  //Squared Distance from joytick center in each quadrant
float xHighYHighRadius, xHighYLowRadius, xLowYLowRadius, xLowYHighRadius; // Squared deadband distance from center

int xHighChangeTolerance, yHighChangeTolerance, xLowChangeTolerance, yLowChangeTolerance;       //The tolerance of changes in FSRs readings 

unsigned int puffCount, sipCount;                 //The puff and long sip incremental counter variables
int pollCounter = 0;                              //Cursor poll counter

int cursorSpeedCounter;                           // Variable to track current cursor speed level
int cursorDelay;                                  // Current cursor delay
float cursorFactor;                               // Current cursor factor
int cursorMaxSpeed;                               // Current cursor max speed (at full joystick deflection)

float yHighComp = 1.0;
float yLowComp = 1.0;
float xHighComp = 1.0;
float xLowComp = 1.0;

float yHighDebug, yLowDebug, xHighDebug, xLowDebug;
int yHighMaxDebug, yLowMaxDebug, xHighMaxDebug, xLowMaxDebug;

int rotationAngle = ROTATION_ANGLE;                       //Rotation angle variables
float rotationAngle11, rotationAngle12, rotationAngle21, rotationAngle22;

float sipThreshold;                                       //Sip pressure threshold in volts
float puffThreshold;                                      //Puff pressure threshold in volts

float cursorPressure;                                     //Variable to hold pressure readings

int modelNumber;                                        //Declare LipSync model number variable 

bool debugModeEnabled;                                    //Declare raw and debug enable variable
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
  
  setCursorInitialization(false,1);               //Set the Home joystick and generate movement threshold boundaries
  delay(10);
  
  getCursorCalibration(false);                    //Get FSR Max calibration values 
  delay(10);
  
  getChangeTolerance(false); // Get change tolerance using max FSR readings and default tolerance percentage 
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

  bool outputMouse = false;
  
  // Reset cursor values
  int xCursor = 0;
  int yCursor = 0;

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
  xHighYHigh = sq(((xHigh - xHighNeutral) > 0) ? (xHigh - xHighNeutral) : 0) + sq(((yHigh - yHighNeutral) > 0) ? (yHigh - yHighNeutral) : 0);     //The sq() function raises thr input to power of 2 and is returning the same data type int->int
  xHighYLow  = sq(((xHigh - xHighNeutral) > 0) ? (xHigh - xHighNeutral) : 0) + sq(((yLow  - yLowNeutral)  > 0) ? (yLow  - yLowNeutral)  : 0);    //
  xLowYHigh  = sq(((xLow  - xLowNeutral)  > 0) ? (xLow  - xLowNeutral)  : 0) + sq(((yHigh - yHighNeutral) > 0) ? (yHigh - yHighNeutral) : 0);    //These are the squared vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xLowYLow   = sq(((xLow  - xLowNeutral)  > 0) ? (xLow  - xLowNeutral)  : 0) + sq(((yLow  - yLowNeutral)  > 0) ? (yLow  - yLowNeutral)  : 0);    //a larger digital value with a positive application force, a large negative difference

  //Check to see if the joystick has moved outside the deadband
  if ((xHighYHigh > xHighYHighRadius) || (xHighYLow > xHighYLowRadius) || (xLowYLow > xLowYLowRadius) || (xLowYHigh > xLowYHighRadius)) {

    //Secondary check to see if joystick has moved by looking for low FSR values (e.g. joystick unloaded->high resistance-> low voltage)
    if ( (xHigh < CURSOR_LIFT_THRESOLD) || 
         (xLow  < CURSOR_LIFT_THRESOLD) || 
         (yHigh < CURSOR_LIFT_THRESOLD) || 
         (yLow  < CURSOR_LIFT_THRESOLD)){
          skipChange = false; // Don't skip if joystick if moved and held
         }
    
    pollCounter++;      //Add to the poll counter
    delay(20); 
    
    //Perform cursor movement actions if joystick has been in active zone for 3 or more poll counts
    if(!skipChange && pollCounter >= 3) {
       //Quadrant 1 (Upper left)
      if ((xHighYHigh >= xHighYLow) && (xHighYHigh >= xLowYHigh) && (xHighYHigh >= xLowYLow)) {    
        xCursor = XHIGH_DIRECTION*cursorModifier(xHigh, xHighNeutral, xHighMax, xHighComp);
        yCursor = YHIGH_DIRECTION*cursorModifier(yHigh, yHighNeutral, yHighMax, yHighComp);
        outputMouse = true;   
      } 
      //Quadrant 4 (Lower Left)
      else if ((xHighYLow > xHighYHigh) && (xHighYLow > xLowYLow) && (xHighYLow > xLowYHigh)) {   
        xCursor = XHIGH_DIRECTION*cursorModifier(xHigh, xHighNeutral, xHighMax, xHighComp);
        yCursor = YLOW_DIRECTION*cursorModifier(yLow, yLowNeutral, yLowMax, yLowComp);
        outputMouse = true;          
      }
      //Quadrant 3 (Lower Right)
      else if ((xLowYLow >= xHighYHigh) && (xLowYLow >= xHighYLow) && (xLowYLow >= xLowYHigh)) {  
        xCursor = XLOW_DIRECTION*cursorModifier(xLow, xLowNeutral, xLowMax, xLowComp);
        yCursor = YLOW_DIRECTION*cursorModifier(yLow, yLowNeutral, yLowMax, yLowComp);
        outputMouse = true;
      } 
      //Quadrant 2 (Upper Right)
      else if ((xLowYHigh > xHighYHigh) && (xLowYHigh >= xHighYLow) && (xLowYHigh >= xLowYLow)) { 
        xCursor = XLOW_DIRECTION*cursorModifier(xLow, xLowNeutral, xLowMax, xLowComp);
        yCursor = YHIGH_DIRECTION*cursorModifier(yHigh, yHighNeutral, yHighMax, yHighComp);
        outputMouse = true;         
      }
        
    } //end check skipchange and poll counter   
  } //end check deadband 
  
  if (outputMouse){
    moveCursor(xCursor, yCursor, 0);
    delay(cursorDelay);
    pollCounter = 0;
  }
  else if(rawModeEnabled) {
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
    //printCommandResponse(true,0,"MN,0:1");
    Serial.println("SUCCESS,0:MN,0:1");
  }
}

//***GET VERSION FUNCTION***//

void getVersionNumber(void) {
  //printCommandResponse(true,0,"VN,0:V2.71");
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
    //printCommandResponse(true,0,"SS,0:"+speedCounter);
    
    Serial.print("SUCCESS,0:SS,0:");
    Serial.println(speedCounter); 
    
    delay(5);     
  } 
  return speedCounter;
}

//***SET CURSOR SPEED FUNCTION***//

void setCursorSpeed(bool responseEnabled, int inputSpeedCounter) {

  bool isValidSpeed = true;
  if(inputSpeedCounter>=0 && inputSpeedCounter <=10){
    cursorSpeedCounter = inputSpeedCounter;
    EEPROM.put(EEPROM_speedCounter, cursorSpeedCounter);
    delay(10);
    if(!API_ENABLED){ cursorSpeedCounter = SPEED_COUNTER; }
    isValidSpeed = true;
  } else {
    EEPROM.get(EEPROM_speedCounter, cursorSpeedCounter);
    delay(10); 
    isValidSpeed = false;
  }
  delay(5); 
  
 if(responseEnabled) {
    //(isValidSpeed) ? printCommandResponse(true,0,"SS,1:"+cursorSpeedCounter) : printCommandResponse(false,2,"SS,2:"+inputSpeedCounter);
    if(isValidSpeed) {
      Serial.print("SUCCESS,0:SS,1:");
      Serial.println(cursorSpeedCounter);
    } else {
      Serial.print("FAIL,2:SS,1:");
      Serial.println(inputSpeedCounter);      
    }
    delay(5);
  } 
}

//***INCREASE CURSOR SPEED LEVEL FUNCTION***//

void increaseCursorSpeed(bool responseEnabled) {
  cursorSpeedCounter++;

  if (cursorSpeedCounter == 11) {
    ledBlink(6, 50, 3);
    cursorSpeedCounter = 10;
  } else {
    ledBlink(cursorSpeedCounter+1, 100, 1);
    cursorDelay = cursorParams[cursorSpeedCounter]._delay;
    cursorFactor = cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;

    EEPROM.put(EEPROM_speedCounter, cursorSpeedCounter);
    delay(25);
  }
  //(responseEnabled) ? printCommandResponse(true,0,"SS,2:"+cursorSpeedCounter) : printManualResponse("SS,2:"+cursorSpeedCounter);
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:"); 
  Serial.print("SS,2:");
  Serial.println(cursorSpeedCounter); 
  
  delay(5);
}

//***DECREASE CURSOR SPEED LEVEL FUNCTION***//

void decreaseCursorSpeed(bool responseEnabled) {
  cursorSpeedCounter--;
  if (cursorSpeedCounter == -1) {
    ledBlink(6, 50, 3);
    cursorSpeedCounter = 0;
  } else if (cursorSpeedCounter == 0) {
    ledBlink(1, 350, 1);
    cursorDelay = cursorParams[cursorSpeedCounter]._delay;
    cursorFactor = cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;

    EEPROM.put(EEPROM_speedCounter, cursorSpeedCounter);
    delay(25);
 
  } else {
    ledBlink(cursorSpeedCounter+1, 100, 1);

    cursorDelay = cursorParams[cursorSpeedCounter]._delay;
    cursorFactor = cursorParams[cursorSpeedCounter]._factor;
    cursorMaxSpeed = cursorParams[cursorSpeedCounter]._maxSpeed;

    EEPROM.put(EEPROM_speedCounter, cursorSpeedCounter);
    delay(25);
  }
  //(responseEnabled) ? printCommandResponse(true,0,"SS,2:"+cursorSpeedCounter) : printManualResponse("SS,2:"+cursorSpeedCounter);
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:"); 
  Serial.print("SS,2:");
  Serial.println(cursorSpeedCounter);  
  
  delay(5);
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
    //printCommandResponse(true,0,"PT,0:"+(String)pressureThreshold+":"+(String)pressureNominal);
    
    Serial.print("SUCCESS,0:PT,0:");
    Serial.print(pressureThreshold);
    Serial.print(":");
    Serial.println(pressureNominal);
    
    delay(5);
  }
}

//***SET PRESSURE THRESHOLD FUNCTION***//

void setPressureThreshold(bool responseEnabled,int inputPressureThreshold) {
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
    //(isValidThreshold) ? printCommandResponse(true,0,"PT,1:"+(String)pressureThreshold+":"+(String)pressureNominal): printCommandResponse(false,2,"PT,1:"+(String)inputPressureThreshold+":"+(String)pressureNominal);
    
    (isValidThreshold) ? Serial.print("SUCCESS,0:PT,1:"+pressureThreshold):Serial.print("FAIL,2:PT,1:"+inputPressureThreshold);
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
    EEPROM.get(EEPROM_debugModeEnabled, debugIntValue);
    delay(5);
    if(debugIntValue!=0 && debugIntValue!=1) {
      EEPROM.put(EEPROM_debugModeEnabled, DEBUG_MODE);
      delay(5);
      debugState=DEBUG_MODE;
      }   
  } else {
    debugState=DEBUG_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    //printCommandResponse(true,0,"DM,0:"+debugState);
    
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

void setDebugMode(bool responseEnabled,bool inpuDebugState) {

  bool isValidDebugState= true;
  if (inpuDebugState==0 || inpuDebugState==1) {
    debugModeEnabled = inpuDebugState;
    EEPROM.put(EEPROM_debugModeEnabled, debugModeEnabled);
    delay(10);
    if(!API_ENABLED) { debugModeEnabled = DEBUG_MODE; }    
    isValidDebugState = true;
  } else {
    isValidDebugState = false;
  }
  delay(5);
  
  if(responseEnabled) {
    //printCommandResponse(true,0,"DM,1:"+debugModeEnabled);
    if(isValidDebugState){
      Serial.print("SUCCESS,0:DM,1:");
      Serial.println(debugModeEnabled);       
    } else {
      Serial.print("FAIL,2:DM,1:");
      Serial.println(inpuDebugState);       
    }
    delay(5);
    
    if(debugModeEnabled){ sendDebugData();}
    delay(5);
   }
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
    EEPROM.get(EEPROM_rawModeEnabled, rawIntValue);
    delay(5);
    if(rawIntValue!=0 && rawIntValue!=1) { 
      delay(5);
      rawState=RAW_MODE;
      }   
  } else {
    rawState=RAW_MODE;
    delay(5);   
  }

  if(responseEnabled) {
    //printCommandResponse(true,0,"RM,0:"+rawState);
    
    Serial.print("SUCCESS,0:RM,0:");
    Serial.println(rawState); 
    
    delay(5);
   }
  return rawState;
}

//***SET RAW MODE STATE FUNCTION***//

void setRawMode(bool responseEnabled,bool inputRawState) {

  bool isValidRawState = true;
  if (inputRawState==0 || inputRawState==1) {
    rawModeEnabled=inputRawState;
    EEPROM.put(EEPROM_rawModeEnabled, inputRawState);
    delay(5);    
    if(!API_ENABLED) { rawModeEnabled=RAW_MODE; }
    isValidRawState = true;
  } else {
    isValidRawState = false;
  }
  delay(5);
    
  if(responseEnabled) {
    //printCommandResponse(true,0,"RM,1:"+rawModeEnabled);
    if(isValidRawState){
      Serial.print("SUCCESS,0:RM,1:");
      Serial.println(rawModeEnabled);    
    } else {
      Serial.print("FAIL,2:RM,1:");
      Serial.println(inputRawState);       
    }
    delay(5);   
   }
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

void getCursorInitialization(bool responseEnabled) {
  //printCommandResponse(true,0,"IN,0:"+(String)xHighNeutral+","+(String)xLowNeutral+","+(String)yHighNeutral+","+(String)yLowNeutral);
  
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

void setCursorInitialization(bool responseEnabled,int mode) {

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

  //(responseEnabled) ? printCommandResponse(true,0,"IN,1:"+(String)xHighNeutral+","+(String)xLowNeutral+","+(String)yHighNeutral+","+(String)yLowNeutral) : printManualResponse("IN,1:"+(String)xHighNeutral+","+(String)xLowNeutral+","+(String)yHighNeutral+","+(String)yLowNeutral);
  
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

  xHighYHighRadius = CURSOR_DEADBAND*CURSOR_DEADBAND;
  xHighYLowRadius = CURSOR_DEADBAND*CURSOR_DEADBAND;
  xLowYLowRadius = CURSOR_DEADBAND*CURSOR_DEADBAND;
  xLowYHighRadius = CURSOR_DEADBAND*CURSOR_DEADBAND;

  if(responseEnable){
    //printCommandResponse(true,0,"CA,0:"+(String)xHighMax+","+(String)xLowMax+","+(String)yHighMax+","+(String)yLowMax);
    
    Serial.print("SUCCESS,0:CA,0:"); 
    Serial.print(xHighMax); 
    Serial.print(","); 
    Serial.print(xLowMax); 
    Serial.print(",");
    Serial.print(yHighMax); 
    Serial.print(",");
    Serial.println(yLowMax); 
    
  }
  delay(10);
}

//*** SET CURSOR CALIBRATION FUNCTION***//

void setCursorCalibration(bool responseEnabled) {
  //(responseEnabled) ? printCommandResponse(true,0,"CA,1:0"): printManualResponse("CA,1:0");
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:0");                                                   //Start the joystick calibration sequence 
  
  ledBlink(4, 300, 3);
  //(responseEnabled) ? printCommandResponse(true,0,"CA,1:1"): printManualResponse("CA,1:1");
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:1"); 
  
  ledBlink(6, 500, 1);
  yHighMax = analogRead(Y_DIR_HIGH_PIN);
  ledBlink(1, 1000, 2);

  //(responseEnabled) ? printCommandResponse(true,0,"CA,1:2"): printManualResponse("CA,1:2");
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:2"); 
  
  ledBlink(6, 500, 1);
  xHighMax = analogRead(X_DIR_HIGH_PIN);
  ledBlink(1, 1000, 2);

  //(responseEnabled) ? printCommandResponse(true,0,"CA,1:3"): printManualResponse("CA,1:3");
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.println("CA,1:3"); 
  
  ledBlink(6, 500, 1);
  yLowMax = analogRead(Y_DIR_LOW_PIN);
  ledBlink(1, 1000, 2);
  
  //(responseEnabled) ? printCommandResponse(true,0,"CA,1:4"): printManualResponse("CA,1:4");
  
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
  //(responseEnabled) ? printCommandResponse(true,0,"CA,1:5:"+(String)xHighMax+","+(String)xLowMax+","+(String)yHighMax+","+(String)yLowMax): printManualResponse("CA,1:5:"+(String)xHighMax+","+(String)xLowMax+","+(String)yHighMax+","+(String)yLowMax);
  
  (responseEnabled) ? Serial.print("SUCCESS,0:") : Serial.print("MANUAL,0:");
  Serial.print("CA,1:5:"); 
  Serial.print(xHighMax); 
  Serial.print(","); 
  Serial.print(xLowMax); 
  Serial.print(",");
  Serial.print(yHighMax); 
  Serial.print(",");
  Serial.println(yLowMax); 
  
  delay(10);
}

//*** GET CHANGE TOLERANCE VALUE CALIBRATION FUNCTION***//

void getChangeTolerance(bool responseEnabled) {
  float changePercent = CHANGE_DEFAULT_TOLERANCE;
  xHighChangeTolerance=(int)(xHighMax * (changePercent/100.0));
  xLowChangeTolerance=(int)(xLowMax * (changePercent/100.0));
  yHighChangeTolerance=(int)(yHighMax * (changePercent/100.0));
  yLowChangeTolerance=(int)(yLowMax * (changePercent/100.0));
  if(responseEnabled){
    //printCommandResponse(true,0,"CT,0:"+(String)changePercent+","+xHighChangeTolerance+","+xLowChangeTolerance+","+yHighChangeTolerance+","+yLowChangeTolerance);
    
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
//Retrieve button mapping
void getButtonMapping(bool responseEnabled) {
  bool isValidMapping = true;
  //memcpy(actionButton, BUTTON_MAPPING, INPUT_ACTION_COUNT);     //Copy the default sip and puff button action mapping
  
  if (API_ENABLED) {
    for (int i = 0; i < INPUT_ACTION_COUNT; i++) {                    //Check if it's a valid mapping
      int buttonMapping;
      EEPROM.get(EEPROM_buttonMapping1+i*2, buttonMapping);
      delay(10);
      if(buttonMapping < 0 || buttonMapping > 7) {
        isValidMapping = false;
        break;
      } else {
        actionButton[i]=buttonMapping;
        delay(5);
      }
    }
    if(!isValidMapping){
      for(int i = 0; i < INPUT_ACTION_COUNT; i++){                       //Save the default mapping into EEPROM if it's not a valid mapping
        EEPROM.put(EEPROM_buttonMapping1+i*2, BUTTON_MAPPING[i]);
        delay(10);
        actionButton[i]=BUTTON_MAPPING[i];
        delay(5);
      }
    }   
  }
  if(responseEnabled) {
    //printCommandResponse(true,0,"MP,0:"+actionButton[0]+actionButton[1]+actionButton[2]+actionButton[3]+actionButton[4]+actionButton[5]);
    
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

void setButtonMapping(bool responseEnabled,int inputButtonMapping[]) {
  
  bool isValidMapping = true;
  
  
   for(int i = 0; i < INPUT_ACTION_COUNT; i++){           // Check each action for validity
    if(inputButtonMapping[i] < 0 || inputButtonMapping[i] > 7) {     // Up to 8 input actions but 6 available 
      isValidMapping = false;
      break;
    }
   }
   
   if(isValidMapping){  //Valid mapping                                 
    for(int i = 0; i < INPUT_ACTION_COUNT; i++){
      EEPROM.put(EEPROM_buttonMapping1+i*2, inputButtonMapping[i]); //Save the mapping into EEPROM if it's a valid mapping
      delay(10);
      actionButton[i]=inputButtonMapping[i];
      delay(5);
    }     
    if(!API_ENABLED) { memcpy(actionButton, BUTTON_MAPPING, INPUT_ACTION_COUNT);  } 
   } 
   delay(5);

  if(responseEnabled) {
    if(isValidMapping) {
      //printCommandResponse(true,0,"MP,1:"+actionButton[0]+actionButton[1]+actionButton[2]+actionButton[3]+actionButton[4]+actionButton[5]);
      
      Serial.print("SUCCESS,0:MP,1:");
      Serial.print(actionButton[0]); 
      Serial.print(actionButton[1]); 
      Serial.print(actionButton[2]); 
      Serial.print(actionButton[3]); 
      Serial.print(actionButton[4]); 
      Serial.println(actionButton[5]); 
      
    } else {
      //printCommandResponse(false,2,"MP,1:"+inputButtonMapping[0]+inputButtonMapping[1]+inputButtonMapping[2]+inputButtonMapping[3]+inputButtonMapping[4]+inputButtonMapping[5]);
      
      Serial.print("FAIL,2:MP,1:");
      Serial.print(inputButtonMapping[0]); 
      Serial.print(inputButtonMapping[1]); 
      Serial.print(inputButtonMapping[2]); 
      Serial.print(inputButtonMapping[3]); 
      Serial.print(inputButtonMapping[4]); 
      Serial.println(inputButtonMapping[5]); 
      
    }
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
    //printCommandResponse(true,0,"RA,0:");
    Serial.print("SUCCESS,0:RA,0:");
    Serial.println(tempRotationAngle); 
    delay(5);
   }

   return tempRotationAngle;
}

//***SET ROTATION ANGLE FUNCTION***///  

void setRotationAngle(bool responseEnabled,int inputRotationAngle) {

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
    //(isValidRotationAngle) ? printCommandResponse(true,0,"RA,1:"+rotationAngle): printCommandResponse(true,2,"RA,1:"+inputRotationAngle);    
    (isValidRotationAngle) ? Serial.println("SUCCESS,0:RA,1:"+rotationAngle):Serial.println("FAIL,2:RA,1:"+inputRotationAngle);
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
           
    setCursorSpeed(false,SPEED_COUNTER);                                  // set default cursor speed counter
    delay(10);
    
    setPressureThreshold(false, PRESSURE_THRESHOLD);                      //set default pressure threshold
    delay(10);
    
    setRotationAngle(false, ROTATION_ANGLE);                              //set default rotation angle
    delay(10);
    
    setDebugMode(false,DEBUG_MODE);                                       //set default debug mode
    delay(10);  
    
    setRawMode(false,RAW_MODE);                                           //set default button mapping
    delay(10);  
    
    setButtonMapping(false,BUTTON_MAPPING);                               //set default action mapping
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
    //printCommandResponse(true,0,"FR,0:0");
    Serial.println("SUCCESS,0:FR,0:0");
    delay(5);
   }
   ledBlink(2, 250, 1);
}

//***SERIAL SETTINGS FUNCTION TO CHANGE SPEED AND COMMUNICATION MODE USING SOFTWARE***//

bool serialSettings(bool enabled) {

    String commandString = "";  
    bool settingsFlag = enabled;                            //Set the input parameter to the flag returned. This will help to detect that the settings actions should be performed.
     if (Serial.available()>0)  
     {  
       commandString = Serial.readString();            //Check if serial has received or read input string and word "SETTINGS" is in input string.
       if (settingsFlag==false && commandString=="SETTINGS") {
        //printCommandResponse(true,0,commandString);
        Serial.println("SUCCESS,0:SETTINGS");
       settingsFlag=true;                         //Set the return flag to true so settings actions can be performed in the next call to the function
       }
       else if (settingsFlag==true && commandString=="EXIT") {
        //printCommandResponse(true,0,commandString);
        Serial.println("SUCCESS,0:EXIT");
       settingsFlag=false;                         //Set the return flag to false so settings actions can be exited
       }
       else if (settingsFlag==true){
       //else if (settingsFlag==true && isValidCommandFormat(commandString)){ //Check if command's format is correct and it's in settings mode
        performCommand(commandString);                  //Sub function to process valid strings
        settingsFlag=false;   
       }
       else {
        //printCommandResponse(false,0,commandString);
        Serial.print("FAIL,0:");
        Serial.println(commandString);
        settingsFlag=false;      
       }
       Serial.flush();  
     }  
    return settingsFlag;
}

//***VALIDATE INPUT COMMAND FORMAT FUNCTION***//

bool isValidCommandFormat (String inputCommandString) {
  bool isValidFormart;
  if ((inputCommandString.length()==(6) || inputCommandString.length()==(7) || inputCommandString.length()==(8) || inputCommandString.length()==(11)) && inputCommandString.charAt(2)==',' && inputCommandString.charAt(4)==':'){ 
    isValidFormart = true;
   }
   else {
     isValidFormart = false;
   }
  return isValidFormart;
}

//***VALIDATE INPUT COMMAND PARAMETER FUNCTION***//

bool isValidCommandParamter(String inputParamterString) {
  bool isValidParamter;
  if (isStrNumber(inputParamterString)){ 
    isValidParamter = true;
   }
   else {
     isValidParamter = false;
   }
  return isValidParamter;
}

//***CHECK IF STRING IS A NUMBER FUNCTION***//

boolean isStrNumber(String str){
  
  for(byte i=0;i<str.length();i++)
  {
    if(!isDigit(str.charAt(i))) return false;
  }
  return true;
}

//***SERIAL PRINT OUT COMMAND RESPONSE FUNCTION***//
/*
void printCommandResponse(bool response, int responseNumber, String responseString) {
  (response) ? Serial.print("SUCCESS,") : Serial.print("FAIL,");
  Serial.print(responseNumber);
  Serial.print(":");
  Serial.println(responseString);
}
*/


//***SERIAL PRINT OUT MANUAL RESPONSE FUNCTION***//
/*
void printManualResponse(String responseString) {
  Serial.print("MANUAL,0:");
  Serial.println(responseString);
}
*/

//***PERFORM COMMAND FUNCTION TO CHANGE SETTINGS USING SOFTWARE***//
// This function takes processes an input string from the serial and calls the 
// corresponding API function, or outputs an error.
void performCommand(String inputString) {
  int inputCommandIndex = inputString.indexOf(':');
  
  //Extract command string from input string
  String inputCommandString = inputString.substring(0, inputCommandIndex);
  
  //Extract parameter string from input string
  String inputParameterString = inputString.substring(inputCommandIndex+1);
  
  // Determine total number of API commands
  int totalCommandNumber=sizeof(apiFunction)/sizeof(apiFunction[0]);

  //Iterate through each API command
  for(int apiIndex = 0; apiIndex < totalCommandNumber; apiIndex++){
    
    // Test if input command string matches API command and input parameter string matches API parameter string
    if(inputCommandString == apiFunction[apiIndex]._command 
    && (inputParameterString == apiFunction[apiIndex]._parameter 
    || apiFunction[apiIndex]._parameter=="" || apiFunction[apiIndex]._parameter=="r")){
      
      // Matching Command String found
      if(isValidCommandParamter(inputParameterString)) {   //Invalid parameter

        //Handle parameters that are an array as a special case.
        if(apiFunction[apiIndex]._parameter=="r"){   //"r" denotes an array parameter 
          
          int inputParameterArray[inputParameterString.length() + 1];
          for(int arrayIndex=0; arrayIndex<inputParameterString.length(); arrayIndex++)
          {
            inputParameterArray[arrayIndex]=inputParameterString.charAt(arrayIndex)-'0';
          }
          apiFunction[apiIndex]._function(true, inputParameterArray);
          delay(5);     
        }
        else {
          apiFunction[apiIndex]._function(true, inputParameterString.toInt());
          delay(5);
        }
      } else {
      Serial.print("FAIL,2:");
      Serial.println(inputString);
      delay(5);
      }
      break;
    }
    else if(apiIndex== (totalCommandNumber-1)) { // command doesn’t exist
    Serial.print("FAIL,1:");
    Serial.println(inputString);
    delay(5);
    break;
    }
  } //end iterate through API functions

}

/*
void performCommand(String inputCommandString) {
  
    String commandString = inputCommandString;
    commandString.replace(",","");                 //Remove commas 
    commandString.replace(":","");                 //Remove :
    
    String paramterString = commandString.substring(3);
    char commandChar[commandString.length()+1];
    commandString.toCharArray(commandChar, commandString.length()+1);

    //Get Model number : "MN,0:0"
    if(commandChar[0]=='M' && commandChar[1]=='N' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? getModelNumber(true) : printCommandResponse(false,2,inputCommandString);
      getModelNumber(true);
      delay(5);
    } 
    //Get version number : "VN,0:0"
    else if(commandChar[0]=='V' && commandChar[1]=='N' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? getVersionNumber() : printCommandResponse(false,2,inputCommandString);
      getVersionNumber();
      delay(5);
    }   
    //Get cursor speed value if received "SS,0:0", decrease the cursor if received "SS,1:1" and increase the cursor speed if received "SS,1:2" , and set the cursor speed value if received "SS,2:{speed 0 to 10}"
    else if(commandChar[0]=='S' && commandChar[1]=='S' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? (void)getCursorSpeed(true) : printCommandResponse(false,2,inputCommandString);
      getCursorSpeed(true);
      delay(5);
    } else if (commandChar[0]=='S' && commandChar[1]=='S' && commandChar[2]=='1' && ( commandString.length()==4 || commandString.length()==5)) {
      //(isValidCommandParamter(paramterString)) ? setCursorSpeed(true,paramterString.toInt()) : printCommandResponse(false,2,inputCommandString);
      setCursorSpeed(true,paramterString.toInt());
      delay(5);
    } 
    else if(commandChar[0]=='S' && commandChar[1]=='S' && commandChar[2]=='2' && commandChar[3]=='1' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? decreaseCursorSpeed(true) : printCommandResponse(false,2,inputCommandString);
      decreaseCursorSpeed(true);
      delay(5);
    } else if (commandChar[0]=='S' && commandChar[1]=='S' && commandChar[2]=='2' && commandChar[3]=='2' && commandString.length()==4) {
       //(isValidCommandParamter(paramterString)) ? increaseCursorSpeed(true) : printCommandResponse(false,2,inputCommandString);
      increaseCursorSpeed(true);
      delay(5);
    }
     //Get pressure threshold values if received "PT,0:0" and set pressure threshold values if received "PT,1:{threshold 1% to 50%}"
      else if(commandChar[0]=='P' && commandChar[1]=='T' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? getPressureThreshold(true) : printCommandResponse(false,2,inputCommandString);
      getPressureThreshold(true);
      delay(5);
    } else if (commandChar[0]=='P' && commandChar[1]=='T' && commandChar[2]=='1' && ( commandString.length()==4 || commandString.length()==5)) {
      //(isValidCommandParamter(paramterString)) ? setPressureThreshold(true,paramterString.toInt()) : printCommandResponse(false,2,inputCommandString);
      setPressureThreshold(true,paramterString.toInt());
      delay(5);
    } 
     //Get rotation angle values if received "RA,0:0" and set rotation angle values if received "RA,1:{0,90,180,270}"
      else if(commandChar[0]=='R' && commandChar[1]=='A' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? (void)getRotationAngle(true) : printCommandResponse(false,2,inputCommandString);
      getRotationAngle(true);
      delay(5);
    } else if (commandChar[0]=='R' && commandChar[1]=='A' && commandChar[2]=='1' && ( commandString.length()==4 || commandString.length()==5 || commandString.length()==6)) {
      //(isValidCommandParamter(paramterString)) ? setRotationAngle(true,paramterString.toInt()) : printCommandResponse(false,2,inputCommandString);
      setRotationAngle(true,paramterString.toInt());
      delay(5);
    } 
     //Get debug mode value if received "DM,0:0" , set debug mode value to 0 if received "DM,1:0" and set debug mode value to 1 if received "DM,1:1"
     else if(commandChar[0]=='D' && commandChar[1]=='M' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? (void)getDebugMode(true) : printCommandResponse(false,2,inputCommandString);
      getDebugMode(true);
      delay(5);
    } else if (commandChar[0]=='D' && commandChar[1]=='M' && commandChar[2]=='1' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? setDebugMode(true,0) : printCommandResponse(false,2,inputCommandString);
      setDebugMode(true,0);
      delay(5);
    } else if (commandChar[0]=='D' && commandChar[1]=='M' && commandChar[2]=='1' && commandChar[3]=='1' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? setDebugMode(true,1) : printCommandResponse(false,2,inputCommandString);
      setDebugMode(true,1);
      delay(5);
    } 
    //Get raw mode value if received "RM,0:0" , set raw mode value to 0 if received "RM,1:0" and set raw mode value to 1 if received "RM,1:1"
     else if(commandChar[0]=='R' && commandChar[1]=='M' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? (void)getRawMode(true) : printCommandResponse(false,2,inputCommandString);
      getRawMode(true);
      delay(5);
    } else if (commandChar[0]=='R' && commandChar[1]=='M' && commandChar[2]=='1' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? setRawMode(true,0) : printCommandResponse(false,2,inputCommandString);
      setRawMode(true,0);
      delay(5);
    } else if (commandChar[0]=='R' && commandChar[1]=='M' && commandChar[2]=='1' && commandChar[3]=='1' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? setRawMode(true,1) : printCommandResponse(false,2,inputCommandString);
      setRawMode(true,1);
      delay(5);
    } 
     //Get cursor initialization values if received "IN,0:0" and perform cursor initialization if received "IN,1:1"
     else if(commandChar[0]=='I' && commandChar[1]=='N' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
     //(isValidCommandParamter(paramterString)) ? getCursorInitialization(true) : printCommandResponse(false,2,inputCommandString);
      getCursorInitialization(true);
      delay(5);
    } else if (commandChar[0]=='I' && commandChar[1]=='N' && commandChar[2]=='1' && commandChar[3]=='1' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? setCursorInitialization(true,2) : printCommandResponse(false,2,inputCommandString);
      setCursorInitialization(true,2);
      delay(5);
    } 
     //Get cursor calibration values if received "CA,0:0" and perform cursor calibration if received "CA,1:1"
      else if(commandChar[0]=='C' && commandChar[1]=='A' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? getCursorCalibration(true) : printCommandResponse(false,2,inputCommandString);
      getCursorCalibration(true);
      delay(5);
    } else if (commandChar[0]=='C' && commandChar[1]=='A' && commandChar[2]=='1' && commandChar[3]=='1' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? setCursorCalibration(true) : printCommandResponse(false,2,inputCommandString);
      setCursorCalibration(true);
      delay(5);
    } 
     //Get change tolerance values if received "CT,0:0" 
      else if(commandChar[0]=='C' && commandChar[1]=='T' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? getChangeTolerance(true) : printCommandResponse(false,2,inputCommandString);
      getChangeTolerance(true);
      delay(5);
    }
    //Get Button mapping : "MP,0:0" , Set Button mapping : "MP,1:012345"
    else if (commandChar[0]=='M' && commandChar[1]=='P' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? getButtonMapping(true) : printCommandResponse(false,2,inputCommandString);
      getButtonMapping(true);
      delay(5);
    } else if(commandChar[0]=='M' && commandChar[1]=='P' && commandChar[2]=='1' && commandString.length()==9) {
      int tempButtonMapping[INPUT_ACTION_COUNT];
      for(int i = 0; i< INPUT_ACTION_COUNT; i++){
         tempButtonMapping[i]=commandChar[3+i] - '0';          //Convert char arrat to int array
      }
      //(isValidCommandParamter(paramterString)) ? setButtonMapping(true,tempButtonMapping) : printCommandResponse(false,2,inputCommandString);
      setButtonMapping(true,tempButtonMapping);
      delay(5);
    }
     //Perform factory reset if received "FR,0:0"
     else if(commandChar[0]=='F' && commandChar[1]=='R' && commandChar[2]=='0' && commandChar[3]=='0' && commandString.length()==4) {
      //(isValidCommandParamter(paramterString)) ? factoryReset(true) : printCommandResponse(false,2,inputCommandString);
      factoryReset(true);
      delay(5);
    } else {
      //printCommandResponse(false,1,inputCommandString);
      Serial.print("FAIL,1:");            //Invalid command
      Serial.println(commandString);
      delay(5);        
    }
    
}
*/
//***PUSH BUTTON SPEED HANDLER FUNCTION***//

void pushButtonHandler(int switchUpPin, int switchDownPin) {
    //Cursor speed control push button functions below
  if (digitalRead(switchUpPin) == LOW) {
    delay(200);
    clearButtonAction();
    delay(50);
    if (digitalRead(switchDownPin) == LOW) {
      setCursorCalibration(false);                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      increaseCursorSpeed(false);                      //Call increase cursor speed function if push button up is pressed 
    }
  }

  if (digitalRead(switchDownPin) == LOW) {
    delay(200);
    clearButtonAction();
    delay(50);
    if (digitalRead(switchUpPin) == LOW) {
      setCursorCalibration(false);                      //Call joystick calibration if both push button up and down are pressed 
    } else {
      decreaseCursorSpeed(false);                      //Call increase cursor speed function if push button up is pressed 
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

// Returns a 0 if nothing detected, 1 if a puff is detected and a 2 if sip is deteced

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

//***PERFORM BUTTON ACTION FUNCTION**//
// Perform mapped output actions (e.g. left click) based on input action (e.g. short puff)
void performButtonAction(int outputAction) {
    switch (outputAction) {
      case OUTPUT_NOTHING: {
        //do nothing
        break;
      }
      case OUTPUT_LEFT_CLICK: {
        //Left Click: Perform mouse left click action
        //Default: puff counter value is under PUFF_COUNT_THRESHOLD_MED ( 1 Second Short Puff )
        ledClear();
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.click(MOUSE_LEFT);
          delay(5);
        }
        break;
      }
      case OUTPUT_RIGHT_CLICK: {
        //Right Click: Perform mouse right click action
        //Default: if sip counter value is under SIP_COUNT_THRESHOLD_MED ( 1 Second Short Sip )
        ledClear();
        Mouse.click(MOUSE_RIGHT);
        delay(5);
        break;
      }
      case OUTPUT_DRAG: {
        //Drag: Perform mouse left press action ( Drag Action ) 
        //Default: if puff counter value is under 750 and more than PUFF_COUNT_THRESHOLD_MED ( 3 Second Long Puff )
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
      case OUTPUT_SCROLL: {
        //Scroll: Perform mouse scroll action
        //Default: if sip counter value is under 750 and more than SIP_COUNT_THRESHOLD_MED ( 3 Second Long Sip )
        ledOn(1); // Turn on Green LED
        cursorScroll();
        delay(5);
        break;
      }
      case OUTPUT_MIDDLE_CLICK: {
        //Perform cursor middle click
        cursorMiddleClick();
        delay(5);
        break;
      }
      case OUTPUT_CURSOR_HOME_RESET: {
        //Cursor Initialization: Perform cursor manual home initialization to reset default value of FSR's
        //Default: if puff counter value is more than 750 ( 5 second Long Puff )
        clearButtonAction();
        ledClear();
        ledBlink(4, 350, 3); 
        setCursorInitialization(false,2);
        delay(5);
        break;
      }
      case OUTPUT_CURSOR_CALIBRATION: {
        //Cursor Calibration: Perform cursor Calibration to reset default value of FSR's
        //Default: if puff counter value is more than 750 ( 5 second Long Puff )
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
//Turns off both LEDs
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
    xLow  = analogRead(X_DIR_LOW_PIN);               //Read analog values of FSR's : A1
    yHigh = analogRead(Y_DIR_HIGH_PIN);             //Read analog values of FSR's : A0
    yLow  = analogRead(Y_DIR_LOW_PIN);               //Read analog values of FSR's : A10
    
    // Read sip and puff input
    float scrollRelease = (((float)analogRead(PRESSURE_PIN)) / 1023.0) * 5.0;

    // Read FSR Inputs
    xHighYHigh = sq(((xHigh - xHighNeutral) > 0) ? (xHigh - xHighNeutral) : 0) + sq(((yHigh - yHighNeutral) > 0) ? (yHigh - yHighNeutral) : 0);     //The sq() function raises thr input to power of 2 and is returning the same data type int->int
    xHighYLow  = sq(((xHigh - xHighNeutral) > 0) ? (xHigh - xHighNeutral) : 0) + sq(((yLow  - yLowNeutral)  > 0) ? (yLow  - yLowNeutral)  : 0);    //
    xLowYHigh  = sq(((xLow  - xLowNeutral)  > 0) ? (xLow  - xLowNeutral)  : 0) + sq(((yHigh - yHighNeutral) > 0) ? (yHigh - yHighNeutral) : 0);    //These are the squared vector magnitudes of each quadrant 1-4. Since the FSRs all register
    xLowYLow   = sq(((xLow  - xLowNeutral)  > 0) ? (xLow  - xLowNeutral)  : 0) + sq(((yLow  - yLowNeutral)  > 0) ? (yLow  - yLowNeutral)  : 0);    //a larger digital value with a positive application force, a large negative difference
      //Check to see if the joystick has moved
    if ((xHighYHigh > xHighYHighRadius) || (xHighYLow > xHighYLowRadius) || (xLowYLow > xLowYLowRadius) || (xLowYHigh > xLowYHighRadius)) {
      
      //Joystick moved - determine which quadrant     
      if ((xHighYHigh >= xHighYLow) && (xHighYHigh >= xLowYHigh) && (xHighYHigh >= xLowYLow)) {     //Quadrant 1
            xCursor = XHIGH_DIRECTION*cursorModifier(xHigh, xHighNeutral, xHighMax, xHighComp);
            yCursor = YHIGH_DIRECTION*cursorModifier(yHigh, yHighNeutral, yHighMax, yHighComp);
                        
          } else if ((xHighYLow > xHighYHigh) && (xHighYLow > xLowYLow) && (xHighYLow > xLowYHigh)) {   //Quadrant 4
            xCursor = XHIGH_DIRECTION*cursorModifier(xHigh, xHighNeutral, xHighMax, xHighComp);
            yCursor = YLOW_DIRECTION*cursorModifier(yLow, yLowNeutral, yLowMax, yLowComp);            
            
          } else if ((xLowYLow >= xHighYHigh) && (xLowYLow >= xHighYLow) && (xLowYLow >= xLowYHigh)) {  //Quadrant 3
            xCursor = XLOW_DIRECTION*cursorModifier(xLow, xLowNeutral, xLowMax, xLowComp);
            yCursor = YLOW_DIRECTION*cursorModifier(yLow, yLowNeutral, yLowMax, yLowComp);
           
            
          } else if ((xLowYHigh > xHighYHigh) && (xLowYHigh >= xHighYLow) && (xLowYHigh >= xLowYLow)) { //Quadrant 2
            xCursor = XLOW_DIRECTION*cursorModifier(xLow, xLowNeutral, xLowMax, xLowComp);
            yCursor = YHIGH_DIRECTION*cursorModifier(yHigh, yHighNeutral, yHighMax, yHighComp);
            
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

//***FSR CURSOR MOVEMENT MODIFIER FUNCTION***//
// Converts FSR voltage readings into mouse cursor movements
int cursorModifier(int rawValue, int neutralValue, int maxValue, float compValue) {
  int cursorOutput = 0;
  
  if (rawValue > neutralValue) { //FSR pressed 
    //Calculate X left factor ( 1.25 multiplied by X low comp multiplied by ratio of X value to X low Maximum value )
    float neutralFactor = 1.25 * (compValue * (((float)(rawValue - neutralValue)) / (maxValue - neutralValue)));

    //Use the calculated X down factor to none linearize the maximum speeds
    cursorOutput = (int)(round(1.0 * pow(cursorMaxSpeed, neutralFactor)) - 1.0);

    //Select maximum speed
    int maxSpeed = round(1.0 * pow(cursorMaxSpeed, 1.25*compValue)) - 1.0;

    //Map the values to a value between 0 and the selected maximum speed
    cursorOutput = map(cursorOutput, 0, (round(1.0 * pow(cursorMaxSpeed, 1.25*compValue)) - 1.0), 0, maxSpeed); 
    
    //Set a constrain 
    cursorOutput = constrain(cursorOutput,0, cursorMaxSpeed);   
  } //end FSR pressed
  
  return cursorOutput;
}
