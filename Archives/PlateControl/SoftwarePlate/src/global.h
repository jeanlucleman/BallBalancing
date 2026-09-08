#ifndef GLOBAL_H
#define GLOBAL_H

//#define ESP_8266
#define ESP_32
#include "settings.h"
#include "functions.h"


#ifdef ESP_8266
    #include <LittleFS.h>
  #else
    #include <SPIFFS.h>
  #endif
#define DEBUG                           // Comment this line to disable debug
#ifdef DEBUG
  #define debug(x) Serial.print(x);
  #define debugln(x) Serial.println(x);
  #define debugf2(x,y) Serial.printf(x,y);
  #define debugf3(x,y,z) Serial.printf(x,y,z);
  #define debugf4(x,y,z,w) Serial.printf(x,y,z,w);
  #define debugf5(a,b,c,d,e) Serial.printf(a,b,c,d,e)
  #define debugf6(a,b,c,d,e,f) Serial.printf(a,b,c,d,e,f)
  #define debugInit(baud) Serial.begin(baud)
  #define debugDated(x) printDated(x)
  #define debuglnDated(x) printlnDated(x)
  
#else
  #define debug(x) // do nothing
  #define debugln(x)
  #define debugf2(x,y)
  #define debugf3(x,y,z)
  #define debugf4(x,y,z,w) 
  #define debugf6(a,b,c,d,e)
  #define debugf6(a,b,c,d,e,f)
  #define debugInit(x)
  #define debugDated(x) 
  #define debuglnDated(x) 
#endif

bool initialisations();
extern Settings commonData;
extern Settings hardwareSettings;
// extern int pinXp;
// extern int pinXm;
// extern int pinYp;
// extern int pinYm;
extern int pinServoX;
extern int pinServoY;
extern int mdlDutyX;
extern int mdlDutyY;
extern int minDutyX;
extern int minDutyY;
extern int maxDutyX;
extern int maxDutyY;
extern int xMin;
extern int xMax;
extern int yMin;
extern int yMax;

#endif