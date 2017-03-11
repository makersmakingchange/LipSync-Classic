/*
  ----------------------------------------------------------
  =    //        //            ///               ///       =
  =    ////      //         ///  ///         ///    ///    =
  =    // //     //         ///              ///           =
  =    //  //    //          ///              ///          =
  =    //   //   //            ///              ///        =
  =    //    //  //              ///              ///      =
  =    //     // //               ///              ///     =
  =    //      ////        ///   ///         ///   ///     =
  =    //       ///           ///               ///        =
  ----------------------------------------------------------
*/

//AUTHOR: Ivan Gourlay 22 June 2016
//VERSION: 2.5 (2 Mar 2017)
/*
   REVISION HISTORY:
   27 Jun 2016
   08 Jul 2016
   11 Jul 2016
   15 Jul 2016
   18 Jul 2016
   29 Jul 2016
   02 Aug 2016
   03 Aug 2016
   16 Aug 2016
   18 Aug 2016
   25 Aug 2016
   26 Aug 2016
   27 Aug 2016
   29 Aug 2016
   30 Aug 2016
   04 Sept 2016
   13 Sept 2016
   14 Sept 2016
   19 Sept 2016
   26 Sept 2016
   27 Sept 2016
   29 Sept 2016
   06 Oct 2016
   12 Oct 2016
   17 Oct 2016
   25 Oct 2016
   04 Nov 2016
   07 Nov 2016
   ---
   30 Nov 2016
   02 Dec 2016
   12 Dec 2016
   13 Dec 2016
   14 Dec 2016
   15 Dec 2016
   20 Dec 2016
   23 Dec 2016
   24 Dec 2016
   30 Dec 2016
   03 Jan 2017
   10 Jan 2017
   19 Jan 2017
   22 Jan 2017
   26 Jan 2017

   18 Feb 2017
   19 Feb 2017
   20 Feb 2017
   21 Feb 2017
   23 Feb 2017
   24 Feb 2017
   25 Feb 2017
   27 Feb 2017
   28 Feb 2017

   01 Mar 2017
   02 Mar 2017
*/

#include <EEPROM.h>
#include <Mouse.h>
#include <math.h>

//***PIN ASSIGNMENTS***//

#define MODE_SELECT 12                            // LipSync Mode Select - USB mode (comm_mode = 0; jumper on) or Bluetooth mode (comm_mode = 1; jumper off) - digital input pin 12 (internally pulled-up)
#define PUSH_BUTTON_UP 8                          // Cursor Control Button 1: UP - digital input pin 8 (internally pulled-up)
#define PUSH_BUTTON_DOWN 7                        // Cursor Control Button 2: DOWN - digital input pin 7 (internally pulled-up)
#define LED_1 4                                   // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2 5                                   // LipSync LED Color2 : RED - digital outputpin 4

#define TRANS_CONTROL A3                          // Bluetooth Transistor Control Pin - digital output pin A3
#define PIO4 A4                                   // Bluetooth PIO4 Command Pin - digital output pin A4

#define PRESSURE_CURSOR A5                        // Sip & Puff Pressure Transducer Pin - analog input pin A5
#define X_DIR_HIGH A0                             // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW A1                              // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH A2                             // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW A10                             // Y Direction Low (Cartesian negative y : down) - analog input pin A10

//***VARIABLE DECLARATION***//

int xh, yh, xl, yl;                               // xh: x-high, yh: y-high, xl: x-low, yl: y-low
int x_right, x_left, y_up, y_down;                // individual neutral starting positions for each FSR

int xh_max = 800;                                 // may just declare these variables but not initialize them because
int xl_max = 800;                                 // these values will be pulled from the EEPROM
int yh_max = 800;
int yl_max = 800;

int x_box_right, x_box_left, y_box_up, y_box_down;
float constant_radius = 30.0;                     // constant radius is initialized to 30.0 but may be changed in joystick initialization
float xh_yh_radius, xh_yl_radius, xl_yl_radius, xl_yh_radius;
float xh_yh, xh_yl, xl_yl, xl_yh;
int move_cursor_dir = 0;
int box_delta;                                    // the delta value for the boundary range in all 4 directions about the x,y center
int cursor_delta;                                 // amount cursor moves in some single or combined direction
int speed_counter = 4;                            // cursor speed counter
int cursor_click_status = 0;                      // value indicator for click status, ie. tap, back and drag
int comm_mode = 0;                                // 0 == USB Communications or 1 == Bluetooth Communications
int config_done;                                  // Binary check of completed Bluetooth configuration
unsigned long puff_count, sip_count;              // long puff and long sip incremental counter

int poll_counter = 0;                             // cursor poll counter
int init_counter = 0;                             // serial port initialization counter
int calibration_counter = 0;                      // automatic calibration counter [self-correcting]

int default_cursor_speed = 30;
int delta_cursor_speed = 5;

int cursor_delay;
float cursor_factor;
int cursor_max_speed;

float yh_comp = 1.0;
float yl_comp = 1.0;
float xh_comp = 1.0;
float xl_comp = 1.0;

float yh_check, yl_check, xh_check, xl_check;
int xhm_check, xlm_check, yhm_check, ylm_check;
float sip_threshold, puff_threshold, cursor_click, cursor_back;

typedef struct {
  int _delay;
  float _factor;
  int _max_speed;
} _cursor;

_cursor setting1 = {5, -1.1, default_cursor_speed - (4 * delta_cursor_speed)}; // 5,-1.0,10
_cursor setting2 = {5, -1.1, default_cursor_speed - (3 * delta_cursor_speed)}; // 5,-1.2,10
_cursor setting3 = {5, -1.1, default_cursor_speed - (2 * delta_cursor_speed)};
_cursor setting4 = {5, -1.1, default_cursor_speed - (delta_cursor_speed)};
_cursor setting5 = {5, -1.1, default_cursor_speed};
_cursor setting6 = {5, -1.1, default_cursor_speed + (delta_cursor_speed)};
_cursor setting7 = {5, -1.1, default_cursor_speed + (2 * delta_cursor_speed)};
_cursor setting8 = {5, -1.1, default_cursor_speed + (3 * delta_cursor_speed)};
_cursor setting9 = {5, -1.1, default_cursor_speed + (4 * delta_cursor_speed)};

_cursor cursor_params[9] = {setting1, setting2, setting3, setting4, setting5, setting6, setting7, setting8, setting9};

//float cursor_factor = {-1.0, -1.2, -1.4, -1.6, -1.8, -2.0};

int single = 0;
int puff1, puff2;

//-----------------------------------------------------------------------------------------------------------------------------------

//***MICROCONTROLLER AND PERIPHERAL MODULES CONFIGURATION***//

void setup() {

  Serial.begin(115200);                           // setting baud rate for serial coms for diagnostic data return from Bluetooth and microcontroller ***MAY REMOVE LATER***
  Serial1.begin(115200);                          // setting baud rate for Bluetooth module

  pinMode(LED_1, OUTPUT);                         // visual feedback #1
  pinMode(LED_2, OUTPUT);                         // visual feedback #2
  pinMode(TRANS_CONTROL, OUTPUT);                 // transistor pin output
  pinMode(PIO4, OUTPUT);                          // command mode pin output

  pinMode(PRESSURE_CURSOR, INPUT);                // pressure sensor pin input
  pinMode(X_DIR_HIGH, INPUT);                     // redefine the pins when all has been finalized
  pinMode(X_DIR_LOW, INPUT);                      // ditto above
  pinMode(Y_DIR_HIGH, INPUT);                     // ditto above
  pinMode(Y_DIR_LOW, INPUT);                      // ditto above

  pinMode(MODE_SELECT, INPUT_PULLUP);             // LOW: USB (default with jumper in) HIGH: Bluetooth (jumper removed)
  pinMode(PUSH_BUTTON_UP, INPUT_PULLUP);          // increase cursor speed button
  pinMode(PUSH_BUTTON_DOWN, INPUT_PULLUP);        // decrease cursor speed button

  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(13, INPUT_PULLUP);

  Serial_Initialization();
  Set_Default();                                  // should only occur once per initialization of a new microcontroller
  delay(10);
  Pressure_Sensor_Initialization();               // register nominal air pressure and generate activation threshold boundaries
  delay(10);
  Joystick_Initialization();                      // home joystick and generate movement threshold boundaries
  delay(10);
  Communication_Mode_Status();                    // identify the selected communication mode
  delay(10);
  Mouse_Configure();                              // conditionally activate the HID mouse functions
  delay(10);
  BT_Configure();                                 // conditionally configure the Bluetooth module [WHAT IF A NEW BT MODULE IS INSTALLED?]
  delay(10);
  cursor_speed_value();                           // reads saved cursor speed parameter from EEPROM
  delay(10);

  int exec_time = millis();
  Serial.print("Configuration time: ");
  Serial.println(exec_time);

  blink(4, 100, 3);                               // end initialization visual feedback

  cursor_delay = cursor_params[speed_counter]._delay;
  cursor_factor = cursor_params[speed_counter]._factor;
  cursor_max_speed = cursor_params[speed_counter]._max_speed;

  // functions below are for diagnostic feedback only

  Serial.print("config_done: ");
  Serial.println(EEPROM.get(0, puff1));
  delay(5);
  Serial.print("speed_counter: ");
  Serial.println(EEPROM.get(2, puff2));
  delay(5);
  Serial.print("cursor_delay: ");
  Serial.println(cursor_params[puff2]._delay);
  delay(5);
  Serial.print("cursor_factor: ");
  Serial.println(cursor_params[puff2]._factor);
  delay(5);
  Serial.print("cursor_max_speed: ");
  Serial.println(cursor_params[puff2]._max_speed);
  delay(5);
  Serial.print("yh_comp factor: ");
  Serial.println(EEPROM.get(6, yh_check));
  delay(5);
  Serial.print("yl_comp factor: ");
  Serial.println(EEPROM.get(10, yl_check));
  delay(5);
  Serial.print("xh_comp factor: ");
  Serial.println(EEPROM.get(14, xh_check));
  delay(5);
  Serial.print("xl_comp factor: ");
  Serial.println(EEPROM.get(18, xl_check));
  delay(5);
  Serial.print("xh_max: ");
  Serial.println(EEPROM.get(22, xhm_check));
  delay(5);
  Serial.print("xl_max: ");
  Serial.println(EEPROM.get(24, xlm_check));
  delay(5);
  Serial.print("yh_max: ");
  Serial.println(EEPROM.get(26, yhm_check));
  delay(5);
  Serial.print("yl_max: ");
  Serial.println(EEPROM.get(28, ylm_check));
  delay(5);

}

//-----------------------------------------------------------------------------------------------------------------------------------

//***START OF INFINITE LOOP***//

void loop() {


  if (single == 0) {
    Serial.println(" ");
    Serial.println(" --- ");
    Serial.println("This is the 01 March - FSR Experimental Exponential Cursor Algorithm WIP");
    Serial.println(" ");
    Serial.println("Enhanced functions:");
    Serial.println("Tap and drag");
    Serial.println("Scrolling");
    Serial.println("Joystick calibration");
    Serial.println("Hands-free home positioning reset");
    Serial.println(" --- ");

    forceDisplayCursor();

    //mouseCommand(0,30,0,0);
    //mouseCommand(0,127,0,0);

    /*
      int ral3 = 5;
      EEPROM.put(4, ral3);        // testing Bluetooth config ***CAN BE REMOVED
      delay(100);
    */

    single++;

    Serial.print("X high: ");
    Serial.println(analogRead(X_DIR_HIGH));
    delay(10);
    Serial.print("X low: ");
    Serial.println(analogRead(X_DIR_LOW));
    delay(10);
    Serial.print("Y high: ");
    Serial.println(analogRead(Y_DIR_HIGH));
    delay(10);
    Serial.print("Y low: ");
    Serial.println(analogRead(Y_DIR_LOW));
    delay(10);
  }

  //cursor movement control functions below

  //xh_yh, xl_yh, xl_yl, xh_yl = 0.0;               // reset radial computations to 0.0

  xh = analogRead(X_DIR_HIGH);                    // A0 :: NOT CORRECT MAPPINGS
  xl = analogRead(X_DIR_LOW);                     // A1
  yh = analogRead(Y_DIR_HIGH);                    // A2
  yl = analogRead(Y_DIR_LOW);                     // A10

  /*
    if ((xh - x_right) > (xl - x_left)) {

      if ((yh - y_up) > (yl - y_down)) {

        xh_yh = sqrt(sq(xh - x_right) + sq(yh - y_up));

      } else { //if ((yh - y_up) < (yl - y_down)) {

        xh_yl = sqrt(sq(xh - x_right) + sq(yl - y_down));
      }
    } else { //if ((xh - x_right) < (xl - x_left)) {

      if ((yh - y_up) > (yl - y_down)) {

        xl_yh = sqrt(sq(xl - x_left) + sq(yh - y_up));

      } else { //if ((yh - y_up) < (yl - y_down)) {

        xl_yl = sqrt(sq(xl - x_left) + sq(yl - y_down));

      }
    }
  */
  ///*
  xh_yh = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));     // sq() function raises input to power of 2, returning the same data type int->int ...
  xh_yl = sqrt(sq(((xh - x_right) > 0) ? (float)(xh - x_right) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));   // the sqrt() function raises input to power 1/2, returning a float type
  xl_yh = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yh - y_up) > 0) ? (float)(yh - y_up) : 0.0));      // These are the vector magnitudes of each quadrant 1-4. Since the FSRs all register
  xl_yl = sqrt(sq(((xl - x_left) > 0) ? (float)(xl - x_left) : 0.0) + sq(((yl - y_down) > 0) ? (float)(yl - y_down) : 0.0));    // a larger digital value with a positive application force, a large negative difference
  //*/                                                    // can also be calculated as a large positive number

  if (isnan(xh_yh)) xh_yh = 0.0;
  if (isnan(xh_yl)) xh_yl = 0.0;
  if (isnan(xl_yh)) xl_yh = 0.0;
  if (isnan(xl_yl)) xl_yl = 0.0;
  /*
    Serial.print("xh - x_right: ");
    Serial.print(xh - x_right);
    Serial.print(" :: ");
    Serial.print("xl - x_left: ");
    Serial.print(xl - x_left);
    Serial.print(" :: ");
    Serial.print("yh - y_up: ");
    Serial.print(yh - y_up);
    Serial.print(" :: ");
    Serial.print("yl - y_down: ");
    Serial.print(yl - y_down);

    Serial.print(" :: ");
    Serial.print(xh_yh);
    Serial.print(" :: ");
    Serial.print(xh_yl);
    Serial.print(" :: ");
    Serial.print(xl_yh);
    Serial.print(" :: ");
    Serial.println(xl_yl);
    //*/

  if (xh_yh > xh_yh_radius || xh_yl > xh_yl_radius || xl_yl > xl_yl_radius || xl_yh > xl_yh_radius) {

    poll_counter++;

    delay(15);    // originally 15 ms

    if (poll_counter >= 3) {

      if (comm_mode == 0) {

        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          Mouse.move(x_cursor_high(xh), y_cursor_high(yh), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          Mouse.move(x_cursor_high(xh), y_cursor_low(yl), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
          Mouse.move(x_cursor_low(xl), y_cursor_low(yl), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2");
          Mouse.move(x_cursor_low(xl), y_cursor_high(yh), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        }
      } else {

        if ((xh_yh >= xh_yl) && (xh_yh >= xl_yh) && (xh_yh >= xl_yl)) {
          //Serial.println("quad1");
          mouseCommand(cursor_click_status, x_cursor_high(xh), y_cursor_high(yh), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        } else if ((xh_yl > xh_yh) && (xh_yl > xl_yl) && (xh_yl > xl_yh)) {
          //Serial.println("quad4");
          mouseCommand(cursor_click_status, x_cursor_high(xh), y_cursor_low(yl), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        } else if ((xl_yl >= xh_yh) && (xl_yl >= xh_yl) && (xl_yl >= xl_yh)) {
          //Serial.println("quad3");
          mouseCommand(cursor_click_status, x_cursor_low(xl), y_cursor_low(yl), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        } else if ((xl_yh > xh_yh) && (xl_yh >= xh_yl) && (xl_yh >= xl_yl)) {
          //Serial.println("quad2");
          mouseCommand(cursor_click_status, x_cursor_low(xl), y_cursor_high(yh), 0);
          delay(cursor_delay);
          poll_counter = 0;
          calibration_counter = 0;
        }
      }
    }
  }


  //cursor speed control push button functions below

  if (digitalRead(PUSH_BUTTON_UP) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
      Joystick_Calibration();
    } else {
      increase_cursor_speed();      // increase cursor speed with push button up
    }
    calibration_counter = 0;
  }

  if (digitalRead(PUSH_BUTTON_DOWN) == LOW) {
    delay(250);
    if (digitalRead(PUSH_BUTTON_UP) == LOW) {
      Joystick_Calibration();
    } else {
      decrease_cursor_speed();      // decrease cursor speed with push button down
      //delay(10);                   // software debounce
    }
    calibration_counter = 0;
  }

  //pressure sensor sip and puff functions below

  cursor_click = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;

  if (cursor_click < puff_threshold) {
    while (cursor_click < puff_threshold) {
      cursor_click = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      puff_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);
    }
    Serial.println(puff_count);             //***REMOVE

    if (comm_mode == 0) {
      if (puff_count < 150) {
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.click(MOUSE_LEFT);
          delay(25);
        }
      } else if (puff_count > 150 && puff_count < 750) {
        if (Mouse.isPressed(MOUSE_LEFT)) {
          Mouse.release(MOUSE_LEFT);
        } else {
          Mouse.press(MOUSE_LEFT);
          delay(25);
        }
      } else if (puff_count > 750) {
        blink(4, 350, 3);   // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      } else {
        /*
            --> RE-EVALUATE THIS FUNCTION
           Mouse.press(MOUSE_LEFT)
           delay(750);
        */
      }
    } else {
      if (puff_count < 150) {
        cursor_click_status = 1; //change this stuff to hex
        mouseCommand(cursor_click_status, 0, 0, 0);
        mouseClear();
        cursor_click_status = 0;
        delay(25);
      } else if (puff_count > 150 && puff_count < 750) {
        if (cursor_click_status == 0) {
          cursor_click_status = 1;
        } else if (cursor_click_status == 1) {
          cursor_click_status = 0;
        }
      } else if (puff_count > 750) {
        blink(4, 350, 3);     // visual prompt for user to release joystick for automatic calibration of home position
        Manual_Joystick_Home_Calibration();
      } else {

        /*
            --> RE-EVALUATE THIS FUNCTION
           cursor_click_status = 1;
           mouseCommand(cursor_click_status, 0, 0, 0);
           delay(750);
        */
      }
    }

    puff_count = 0;
    calibration_counter = 0;
  }

  if (cursor_click > sip_threshold) {
    while (cursor_click > sip_threshold) {
      cursor_click = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;
      sip_count++;         // NEED TO FIGURE OUT ROUGHLY HOW LONG ONE CYCLE OF THIS WHILE LOOP IS -> COUNT THRESHOLD
      delay(5);
    }
    Serial.println(sip_count);             //***REMOVE

    if (comm_mode == 0) {
      if (sip_count < 150) {
        Mouse.click(MOUSE_RIGHT);
        delay(25);
      } else if (sip_count > 150 && sip_count < 750) {
        mouseScroll();
        delay(25);
      } else {

        Mouse.click(MOUSE_MIDDLE);
        delay(25);

        /*
            --> RE-EVALUATE THIS FUNCTION
           Mouse.press(MOUSE_RIGHT)
           delay(750);
        */
      }
    } else {
      if (sip_count < 150) {
        cursor_click_status = 2;
        mouseCommand(cursor_click_status, 0, 0, 0);
        mouseClear();
        cursor_click_status = 0;
        delay(25);
      } else if (sip_count > 150) {
        mouseScroll();
        delay(25);
      } else {
        cursor_click_status = 3;
        mouseCommand(cursor_click_status, 0, 0, 0);
        mouseClear();
        cursor_click_status = 0;
        delay(25);

        /*
            --> RE-EVALUATE THIS FUNCTION
           cursor_click_status = 2;
           mouseCommand(cursor_click_status, 0, 0, 0);
           delay(750);
        */
      }
    }

    sip_count = 0;
    calibration_counter = 0;
  }

  //calibration_counter++; //comment back in to experiment with auto_calibration

  if (calibration_counter > 1000) {
    //Auto_Joystick_Home_Calibration();

    // ^^ THIS IS AN UN-TESTED IMPLEMENTATION OF THIS FUNCTION WHICH MAY HELP WITH GRADUALLY ACCUMULATED HOME POSITION SHIFT
  }


}

//***END OF INFINITE LOOP***//

//-----------------------------------------------------------------------------------------------------------------------------------

//***LED BLINK FUNCTIONS***//

void blink(int num_Blinks, int delay_Blinks, int LED_number ) {
  if (num_Blinks < 0) num_Blinks *= -1;

  switch (LED_number) {
    case 1: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 2: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
    case 3: {
        for (int i = 0; i < num_Blinks; i++) {
          digitalWrite(LED_1, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_1, LOW);
          delay(delay_Blinks);
          digitalWrite(LED_2, HIGH);
          delay(delay_Blinks);
          digitalWrite(LED_2, LOW);
          delay(delay_Blinks);
        }
        break;
      }
  }
}

//***CURSOR SPEED FUNCTIONS***//

void cursor_speed_value(void) {
  int var;
  EEPROM.get(2, var);
  delay(5);
  speed_counter = var;
}

void increase_cursor_speed(void) {
  speed_counter++;

  if (speed_counter == 9) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 8;
  } else {
    blink(speed_counter, 100, 1);

    cursor_delay = cursor_params[speed_counter]._delay;
    cursor_factor = cursor_params[speed_counter]._factor;
    cursor_max_speed = cursor_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("+");
  }
}

void decrease_cursor_speed(void) {
  speed_counter--;

  if (speed_counter == -1) {
    blink(6, 50, 3);     // twelve very fast blinks
    speed_counter = 0;
  } else if (speed_counter == 0) {
    blink(1, 350, 1);

    cursor_delay = cursor_params[speed_counter]._delay;
    cursor_factor = cursor_params[speed_counter]._factor;
    cursor_max_speed = cursor_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  } else {
    blink(speed_counter, 100, 1);

    cursor_delay = cursor_params[speed_counter]._delay;
    cursor_factor = cursor_params[speed_counter]._factor;
    cursor_max_speed = cursor_params[speed_counter]._max_speed;

    EEPROM.put(2, speed_counter);
    delay(25);
    Serial.println("-");
  }
}

//***CURSOR MOVEMENT FUNCTIONS***//

int y_cursor_high(int j) {

  if (j > y_up) {

    //int k = (int)(round(-1.0 * cursor_max_speed * (1.0 - exp(cursor_factor * (yh_comp * (((float)(j - y_up)) / y_up))))));

    float y_up_factor = 1.25 * (yh_comp * (((float)(j - y_up)) / (yh_max - y_up)));

    int k = (int)(round(-1.0 * pow(cursor_max_speed, y_up_factor)) - 1.0);

    if (k <= (-1 * cursor_max_speed) ) { // originally (k < 0 || k > 0)

      k = -1 * cursor_max_speed;

      /*
      Serial.print("yh:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else if ( (k < 0) && (k > (-1 * cursor_max_speed))) {

      /*
      Serial.print("yh:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else {
      k = 0;

      /*
      Serial.print("yh:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    }
  } else {
    return 0;
  }
}

int y_cursor_low(int j) {

  if (j > y_down) {

    //int k = (int)(round(1.0 * cursor_max_speed * (1.0 - exp(cursor_factor * (yl_comp * (((float)(j - y_down)) / y_down))))));

    float y_down_factor = 1.25 * (yl_comp * (((float)(j - y_down)) / (yl_max - y_down)));

    int k = (int)(round(1.0 * pow(cursor_max_speed, y_down_factor)) - 1.0);

    if (k >= cursor_max_speed) {

      k = cursor_max_speed;

      /*
      Serial.print("yl:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else if ((k > 0) && (k < cursor_max_speed)) {

      /*
      Serial.print("yl:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else {

      k = 0;

      /*
      Serial.print("yl:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    }
  } else {
    return 0;
  }
}

int x_cursor_high(int j) {

  if (j > x_right) {

    //int k = (int)(round(1.0 * cursor_max_speed * (1.0 - exp(cursor_factor * (xh_comp * (((float)(j - x_right)) / x_right))))));

    float x_right_factor = 1.25 * (xh_comp * (((float)(j - x_right)) / (xh_max - x_right)));

    int k = (int)(round(1.0 * pow(cursor_max_speed, x_right_factor)) - 1.0);

    if (k >= cursor_max_speed) {

      k = cursor_max_speed;

      /*
      Serial.print("xh:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else if ((k > 0) && (k < cursor_max_speed)) {

      /*
      Serial.print("xh:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else {
      k = 0;

      /*
      Serial.print("xh:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    }
  } else {
    return 0;
  }
}

int x_cursor_low(int j) {

  if (j > x_left) {

    //int k = (int)(round(-1.0 * cursor_max_speed * (1.0 - exp(cursor_factor * (xl_comp * (((float)(j - x_left)) / x_left))))));

    float x_left_factor = 1.25 * (xl_comp * (((float)(j - x_left)) / (xl_max - x_left)));

    int k = (int)(round(-1.0 * pow(cursor_max_speed, x_left_factor)) - 1.0);

    if ( k <= (-1 * cursor_max_speed) ) {
      k = -1 * cursor_max_speed;

      /*
      Serial.print("xl:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else if ( (k < 0) && (k > -1 * cursor_max_speed)) {

      /*
      Serial.print("xl:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    } else {
      k = 0;

      /*
      Serial.print("xl:");
      Serial.print(j);
      Serial.print(", ");
      Serial.print("k:");
      Serial.println(k);
      //*/

      return k;
    }
  } else {
    return 0;
  }
}

//***BLUETOOTH HID MOUSE FUNCTIONS***//

void mouseCommand(int buttons, int x, int y, int scroll) {

  byte BTcursor[7];

  BTcursor[0] = 0xFD;
  BTcursor[1] = 0x5;
  BTcursor[2] = 0x2;
  BTcursor[3] = lowByte(buttons); //check this out
  BTcursor[4] = lowByte(x);       //check this out
  BTcursor[5] = lowByte(y);       //check this out
  BTcursor[6] = lowByte(scroll);  //check this out

  Serial1.write(BTcursor, 7);
  Serial1.flush();

  delay(40);    // reduced poll_coutner delay by 30ms so increase delay(10) to delay (40)
}

void mouseClear(void) {

  byte BTcursor[7];

  BTcursor[0] = 0xFD;
  BTcursor[1] = 0x5;
  BTcursor[2] = 0x2;
  BTcursor[3] = 0x00;
  BTcursor[4] = 0x00;
  BTcursor[5] = 0x00;
  BTcursor[6] = 0x00;

  Serial1.write(BTcursor, 7);
  Serial1.flush();
  delay(40);
}
//police department is closed for the holiday
//***MOUSE SCROLLING FUNCTION***//

void mouseScroll(void) {
  while (1) {

    int scroll_up = analogRead(Y_DIR_HIGH);                      // A2
    int scroll_down = analogRead(Y_DIR_LOW);                     // A10

    float scroll_release = (((float)analogRead(PRESSURE_CURSOR)) / 1023.0) * 5.0;

    if (comm_mode == 0) {

      if (scroll_up > y_up + 30) {
        Mouse.move(0, 0, -1 * y_cursor_high(scroll_up));
        delay(cursor_delay * 70);   // started with this factor change as necessary
      } else if (scroll_down > y_down + 30) {
        Mouse.move(0, 0, -1 * y_cursor_low(scroll_down));
        delay(cursor_delay * 70);   // started with this factor change as necessary
      } else if (scroll_release > sip_threshold || scroll_release < puff_threshold) {
        break;
      }
    } else {

      if (scroll_up > y_up + 30) {
        mouseCommand(0, 0, 0, -1 * y_cursor_high(scroll_up));
        delay(cursor_delay * 70);
      } else if (scroll_down < y_down + 30) {
        mouseCommand(0, 0, 0, -1 * y_cursor_low(scroll_down));
        delay(cursor_delay * 70);
      } else if (scroll_release > sip_threshold || scroll_release < puff_threshold) {
        break;
      }

    }
  }
  delay(250);
}

//***FORCE DISPLAY OF CURSOR***//

void forceDisplayCursor(void) {
  if (comm_mode == 0) {
    Mouse.move(1, 0, 0);
    delay(25);
    Mouse.move(-1, 0, 0);
    delay(25);
  } else {
    // BT_Connected_Status();
    /*
      mouseCommand(0, 1, 0, 0);
      delay(25);
      mouseCommand(0, -1, 0, 0);
      delay(25);
    */
  }
}

//***COMMUNICATION MODE STATUS***//

void Communication_Mode_Status(void) {
  if (digitalRead(MODE_SELECT) == LOW) {
    comm_mode = 0;                                // 0 == USB communication
    Serial.println("comm_mode = 0");
    BT_Low_Power_Mode();
    delay(10);
    //blink(6, 125, 1);
  } else if (digitalRead(MODE_SELECT) == HIGH) {
    comm_mode = 1;                                // 1 == Bluetooth communication
    delay(10);
    Serial.println("comm_mode = 1");
    //blink(6, 125, 2);
  }
}

//***JOYSTICK INITIALIZATION FUNCTION***//

void Joystick_Initialization(void) {
  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  delay(10);
  Serial.println(xh);                     // Recommend keeping in for diagnostic purposes

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  delay(10);
  Serial.println(xl);                     // Recommend keeping in for diagnostic purposes

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  delay(10);
  Serial.println(yh);                     // Recommend keeping in for diagnostic purposes

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  delay(10);
  Serial.println(yl);                     // Recommend keeping in for diagnostic purposes

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  EEPROM.get(6, yh_comp);
  delay(10);
  EEPROM.get(10, yl_comp);
  delay(10);
  EEPROM.get(14, xh_comp);
  delay(10);
  EEPROM.get(18, xl_comp);
  delay(10);
  EEPROM.get(22, xh_max);
  delay(10);
  EEPROM.get(24, xl_max);
  delay(10);
  EEPROM.get(26, yh_max);
  delay(10);
  EEPROM.get(28, yl_max);
  delay(10);

  //40.0 works well for a constant radius

  constant_radius = 30.0;

  xh_yh_radius = constant_radius;
  xh_yl_radius = constant_radius;
  xl_yl_radius = constant_radius;
  xl_yh_radius = constant_radius;

/*
  xh_max = 800;
  xl_max = 800;
  yh_max = 800;
  yl_max = 800;
*/

}

//***PRESSURE SENSOR INITIALIZATION FUNCTION***//

void Pressure_Sensor_Initialization(void) {
  float nominal_cursor_value = (((float)analogRead(PRESSURE_CURSOR)) / 1024.0) * 5.0; // Initial neutral pressure transducer analog value [0.0V - 5.0V]
  delay(10);

  sip_threshold = nominal_cursor_value + 0.5;    //Create sip pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  //delay(20);

  puff_threshold = nominal_cursor_value - 0.5;   //Create puff pressure threshold value ***Larger values tend to minimize frequency of inadvertent activation
  //delay(20);
}

//***ARDUINO/GENUINO HID MOUSE INITIALIZATION FUNCTION***//

void Mouse_Configure(void) {
  if (comm_mode == 0) {                       // USB mode is comm_mode == 0, this is when the jumper on J13 is installed
    Mouse.begin();                            // Initialize the HID mouse functions from Mouse.h header file
    delay(25);                                // Allow extra time for initialization to take effect ***May be removed later
  }
}

//----------------------RN-42 BLUETOOTH MODULE INITIALIZATION SECTION----------------------//

//***BLUETOOTH CONFIGURATION STATUS FUNCTION***//

void BT_Config_Status(void) {
  int BT_EEPROM = 3;                               // Local integer variable initialized and defined for use with EEPROM GET function
  EEPROM.get(0, BT_EEPROM);                        // Assign value of EEPROM memory at index zero (0) to int variable BT_EEPROM
  delay(10);
  Serial.println(BT_EEPROM);                       // Only for diagnostics, may be removed later
  config_done = BT_EEPROM;                         // Assign value of local variable BT_EEPROM to global variable config_done
  delay(10);
}

//***BLUETOOTH CONFIGURATION FUNCTION***//

void BT_Configure(void) {
  if (comm_mode == 1) {
    BT_Config_Status();                    // check if Bluetooth has previously been configured
    delay(10);
    if (config_done == 0) {                           // if Bluetooth has not been configured then execute configuration sequence
      BT_Command_Mode();                               // enter Bluetooth command mode
      BT_Config_Sequence();                           // send configuarion data to Bluetooth module
      BT_configAOK();                                 // returns diagnostic responses from Bluetooth
      delay(10);
    } else {
      Serial.println("Bluetooth configuration has previously been completed.");
      delay(10);
    }
  }
}

void BT_Command_Mode(void) {                 //***CHANGE THE TRANSISTOR CONTROLS ONCE THE PNP IS SWITCHED IN FOR THE NPN***
  //digitalWrite(TRANS_CONTROL, HIGH);         // transistor base pin HIGH to ensure Bluetooth module is off
  //digitalWrite(PIO4, HIGH);                 // command pin high
  delay(10);

  //digitalWrite(TRANS_CONTROL, LOW);        // transistor base pin LOW to power on Bluetooth module
  delay(10);

  for (int i = 0; i < 3; i++) {             // cycle PIO4 pin high-low 3 times with 1 sec delay between each level transition
    //digitalWrite(PIO4, HIGH);
    delay(75);
    //digitalWrite(PIO4, LOW);
    delay(75);
  }

  //digitalWrite(PIO4, LOW);                  // drive PIO4 pin low as per command mode instructions
  delay(10);
  Serial1.print("$$$");                     // enter Bluetooth command mode :: "$$$" CANNOT be Serial.println("$$$") ONLY Serial.print("$$$")
  delay(10);                              // time delay to visual inspect the red LED is flashing at 10Hz which indicates the Bluetooth module is in Command Mode
  Serial.println("Bluetooth Command Mode Activated");
}

void BT_Config_Sequence(void) {                    // change all Serial instructions to Serial1
  Serial1.println("ST,255");                 // turn off the 60 sec timer for command mode
  delay(15);
  Serial1.println("SA,1");                   // ***NEW ADDITION - Authentication Values 1: default security
  delay(15);
  Serial1.println("SX,1");                   // ***NEW ADDITION - Bonding 1: enabled - accepts connections to paired devices
  delay(15);
  Serial1.println("SN,LipSyncBT_Autoload1"); // change name of BT module
  delay(15);
  Serial1.println("SM,6");                   // ***NEW ADDITION - Pairing "SM,6": pairing auto-connect mode
  delay(15);
  Serial1.println("SH,0220");                // configure device as HID mouse
  delay(15);
  Serial1.println("S~,6");                   // activate HID profile
  delay(15);

  //Serial1.println("SW,<hex value>");       // sniff mode conserves power by polling the radio for comms ***POWER CONSERVATION
  //delay(100);
  //Serial1.println("SY,<hex value>");       // set transmit power settings ***POWER CONSERVATION

  Serial1.println("SQ,16");                  // configure for latency NOT throughput -> turn off: "SQ,0"
  delay(15);
  Serial1.println("S?,1");                   // 1:ENABLE role switch -> slave device attempts role switch -> indicates better performance for high speed data
  delay(15);
  Serial1.println("R,1");                    // reboot BT module
  delay(15);

  int val0 = 1;
  int val1 = 3;
  //delay(250);
  EEPROM.put(0, val0);                        // EEPROM address 0 gets configuration completed value (== 1)
  delay(10);
  EEPROM.put(2, val1);                        // EEPROM address 1 gets default cursor speed counter value (== 20) ***SHOULD ONLY OCCUR ONCE UNLESS THE LIPSYNC IS FACTORY RESET??
  delay(10);
  int val3;
  EEPROM.get(0, val3);  // diagnostics
  delay(10);            // diagnostics
  Serial.println(val3); // diagnostics
}

void BT_Low_Power_Mode(void) {
  BT_Command_Mode();                          // enter BT command mode
  Serial1.println('Z');                       // enter deep sleep mode (<2mA) when not connected
  delay(10);
  //digitalWrite(TRANS_CONTROL, LOW);
  Serial.println("Bluetooth Deep Sleep Mode Activated");
  delay(10);
  //blink(2, 1000, 3);                          // visual feedback
}

void BT_Connected_Status(void) {
  while (1) {
    Serial1.println("GK");
    delay(100);
    if (Serial1.available() > 0) {
      if ((char)Serial1.read() == '1') {
        Serial.println("BT is now connected!");
        delay(10);
        mouseCommand(0, 1, 0, 0);
        delay(25);
        mouseCommand(0, -1, 0, 0);
        delay(25);
        break;
      }
    }
  }
}

void BT_configAOK(void) {                    // diagnostic feedback from Bluetooth configuration
  while (Serial1.available() > 0) {
    Serial.print((char)Serial1.read());
  }
  Serial.println("");
  Serial.println("Configuration complete.");
}

//***JOYSTICK SPEED CALIBRATION***//

void Joystick_Calibration(void) {

  Serial.println("Prepare for joystick calibration!");
  Serial.println(" ");
  blink(4, 300, 3);

  Serial.println("Move mouthpiece to the furthest vertical up position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yh_max = analogRead(Y_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(yh_max);

  Serial.println("Move mouthpiece to the furthest horizontal right position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xh_max = analogRead(X_DIR_HIGH);
  blink(1, 1000, 2);
  Serial.println(xh_max);

  Serial.println("Move mouthpiece to the furthest vertical down position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  yl_max = analogRead(Y_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(yl_max);

  Serial.println("Move mouthpiece to the furthest horizontal left position and hold it there until the LED turns SOLID RED, then release the mouthpiece.");
  blink(6, 500, 1);
  xl_max = analogRead(X_DIR_LOW);
  blink(1, 1000, 2);
  Serial.println(xl_max);

  int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  Serial.print("delta_max_total: ");
  Serial.println(delta_max_total);

  float avg_delta_max = ((float)(delta_max_total)) / 4;

  Serial.print("avg_delta_max: ");
  Serial.println(avg_delta_max);

  yh_comp = avg_delta_max / (yh_max - y_up);
  yl_comp = avg_delta_max / (yl_max - y_down);
  xh_comp = avg_delta_max / (xh_max - x_right);
  xl_comp = avg_delta_max / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);
  EEPROM.put(22, xh_max);
  delay(10);
  EEPROM.put(24, xl_max);
  delay(10);
  EEPROM.put(26, yh_max);
  delay(10);
  EEPROM.put(28, yl_max);
  delay(10);

  blink(5, 250, 3);

  Serial.println(" ");
  Serial.println("Joystick speed calibration procedure is complete.");
}

//***AUTOMATIC JOYSTICK POSITION CALIBRATION***//
void Auto_Joystick_Home_Calibration(void) {

  xh = analogRead(X_DIR_HIGH);
  xl = analogRead(X_DIR_LOW);
  yh = analogRead(Y_DIR_HIGH);
  yl = analogRead(Y_DIR_LOW);

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  blink(1, 50, 1);

  calibration_counter = 0;
}

//***MANUAL JOYSTICK POSITION CALIBRATION***///
void Manual_Joystick_Home_Calibration(void) {

  xh = analogRead(X_DIR_HIGH);            // Initial neutral x-high value of joystick
  delay(10);
  Serial.println(xh);                     // Recommend keeping in for diagnostic purposes

  xl = analogRead(X_DIR_LOW);             // Initial neutral x-low value of joystick
  delay(10);
  Serial.println(xl);                     // Recommend keeping in for diagnostic purposes

  yh = analogRead(Y_DIR_HIGH);            // Initial neutral y-high value of joystick
  delay(10);
  Serial.println(yh);                     // Recommend keeping in for diagnostic purposes

  yl = analogRead(Y_DIR_LOW);             // Initial neutral y-low value of joystick
  delay(10);
  Serial.println(yl);                     // Recommend keeping in for diagnostic purposes

  x_right = xh;
  x_left = xl;
  y_up = yh;
  y_down = yl;

  int delta_max_total = (yh_max - y_up) + (yl_max - y_down) + (xh_max - x_right) + (xl_max - x_left);

  //Serial.print("delta_max_total: ");
  //Serial.println(delta_max_total);

  float avg_delta_max = ((float)(delta_max_total)) / 4;

  //Serial.print("avg_delta_max: ");
  //Serial.println(avg_delta_max);

  yh_comp = avg_delta_max / (yh_max - y_up);
  yl_comp = avg_delta_max / (yl_max - y_down);
  xh_comp = avg_delta_max / (xh_max - x_right);
  xl_comp = avg_delta_max / (xl_max - x_left);

  EEPROM.put(6, yh_comp);
  delay(10);
  EEPROM.put(10, yl_comp);
  delay(10);
  EEPROM.put(14, xh_comp);
  delay(10);
  EEPROM.put(18, xl_comp);
  delay(10);

  calibration_counter = 0;

  Serial.println("Home position calibration complete.");

}

//***SPECIAL INITIALIZATION OPERATIONS***//

void Serial_Initialization(void) {
  while (!Serial || !Serial1) {

    if (init_counter < 80) {
      delay(25);
      init_counter++;
    } else {
      break;
    }
  }
  Serial.println("Serial is good!");
}

void Set_Default(void) {

  int default_config_setup;
  int default_cursor_setting;
  int set_default;
  float default_comp_factor = 1.0;

  EEPROM.get(4, set_default);
  delay(10);

  if (set_default != 1) {

    default_config_setup = 0;
    EEPROM.put(0, default_config_setup);
    delay(10);

    default_cursor_setting = 4;
    EEPROM.put(2, default_cursor_setting);
    delay(10);

    EEPROM.put(6, default_comp_factor);
    delay(10);

    EEPROM.put(10, default_comp_factor);
    delay(10);

    EEPROM.put(14, default_comp_factor);
    delay(10);

    EEPROM.put(18, default_comp_factor);
    delay(10);

    set_default = 1;
    EEPROM.put(4, set_default);
    delay(10);

  }
}
