/*
 LipSync_Common.h - Library for common LipSync items between versions
 Copyright Neil Squire, 2021.
 Released under a CC-BY-SA License. 
  
 */

#ifndef LipSync_Common_h
#define LipSync_Common_h
 
#include "Arduino.h"

// LipSync EEPROM Memory Usage

#define EEPROM_modelNumber 0          //int:0,1; 
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
#define EEPROM_RawIntValue 36         //int:36,37; 
#define EEPROM_buttonMapping0 42      //int:42,43; 
#define EEPROM_buttonMapping1 44      //int:44,45; 
#define EEPROM_buttonMapping2 46      //int:46,47; 
#define EEPROM_buttonMapping3 48      //int:48,49; 
#define EEPROM_buttonMapping4 50      //int:50,51; 
#define EEPROM_buttonMapping5 52      //int:52,53; 
#define EEPROM_configNumber 54        //int:54,55; 3 when Bluetooth configured 

#endif
