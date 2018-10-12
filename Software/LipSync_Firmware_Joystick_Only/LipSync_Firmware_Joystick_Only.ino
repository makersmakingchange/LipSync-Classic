#include "Joystick.h"
#include <math.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 4, 0,
  true, true, false, 
  false, false, false,
  false, false, 
  false, false, false);

//***PIN ASSIGNMENTS***//
#define LED_1 4                                   // LipSync LED Color1 : GREEN - digital output pin 5
#define LED_2 5                                   // LipSync LED Color2 : RED - digital outputpin 4
#define X_DIR_HIGH A0                             // X Direction High (Cartesian positive x : right) - analog input pin A0
#define X_DIR_LOW A1                              // X Direction Low (Cartesian negative x : left) - digital output pin A1
#define Y_DIR_HIGH A2                             // Y Direction High (Cartesian positive y : up) - analog input pin A2
#define Y_DIR_LOW A10                             // Y Direction Low (Cartesian negative y : down) - analog input pin A10

void setup() {  
  pinMode(LED_1, OUTPUT);                         // visual feedback #1
  pinMode(LED_2, OUTPUT);                         // visual feedback #2

  pinMode(X_DIR_HIGH, INPUT);                     // redefine the pins when all has been finalized
  pinMode(X_DIR_LOW, INPUT);                      // ditto above
  pinMode(Y_DIR_HIGH, INPUT);                     // ditto above
  pinMode(Y_DIR_LOW, INPUT);                      // ditto above

  // nothing but a cool LEDs indicator
  for (int i = 0; i < 10; i++){
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, LOW);
    delay(100);
    digitalWrite(LED_1, LOW);
    digitalWrite(LED_2, HIGH);
    delay(100);
  }

  digitalWrite(LED_1, LOW);
  digitalWrite(LED_2, LOW);
  
  Joystick.begin();
}

void loop() {
  int x_h = analogRead(X_DIR_HIGH);
  int x_l = analogRead(X_DIR_LOW);
  int y_h = analogRead(Y_DIR_LOW);
  int y_l = analogRead(Y_DIR_HIGH);

  x_h = map(x_h, 0, 1023, 0, 16);
  x_l = map(x_l, 0, 1023, 0, 16);
  y_h = map(y_h, 0, 1023, 0, 16);
  y_l = map(y_l, 0, 1023, 0, 16);

  int xx_tmp = x_h - x_l;
  int yy_tmp = y_h - y_l;
  int xx = (xx_tmp >= 0)? sq(xx_tmp):-sq(xx_tmp);
  int yy = (yy_tmp >= 0)? sq(yy_tmp):-sq(yy_tmp);
  xx += (xx_tmp >= 0)? int(sqrt(yy_tmp)):-int(sqrt(-yy_tmp));
  yy += (yy_tmp >= 0)? int(sqrt(xx_tmp)):-int(sqrt(-xx_tmp));

  xx = constrain(xx, -128, 128);
  yy = constrain(yy, -128, 128);

  xx = map(xx, -128, 128, 0, 1023);
  yy = map(yy, -128, 128, 0, 1023);

  Joystick.setXAxis(xx);
  Joystick.setYAxis(yy);
}
