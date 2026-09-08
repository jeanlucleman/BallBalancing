//MyFunctions.cpp
// -------------------INCLUDES ------------------------
  
  #include <Arduino.h>
  #include "global.h"
  
  #include <ESPAsyncWebServer.h>
  // #include <time.h>
  // #include "functions.h"

  #ifdef ESP_8266
    #include "FS.h"
  #elif defined ESP_32
    #include <SPIFFS.h>
  #endif
  Settings commonData("commonSettings");
  Settings hardwareSettings("hardwareSettings");
  void readHardwareSettings();
bool initialisations()
  {
    try
      {
        // Serial.begin(74880);
        debugln("\n");
        debugln("Début des initialisations...");
        delay(500);
      }
    catch(...)
      {
        return false;
      } 
    #ifdef ESP_8266
      debugln("ESP board = ESP8266");
    #elif defined ESP_32
      debugln("ESP board = ESP32");
    #endif
    commonData.ini();  // L'initialisation de cet objet n'est pas fait lors de l'instantiation. La raison est que celle-ci se déroule 
    // dès la déclaration du type de l'objet (Settins myData;) et qu'à ce stade toutes les initialisations de l'ESP ne sont pas terminées
    // ce qui crée un plantage.
    hardwareSettings.ini();
    debugln("Listing des settings:");
    commonData.list();
    hardwareSettings.list();
    readHardwareSettings();


    return true;
  }
  void readHardwareSettings()
    {
      // pinXp=hardwareSettings.getValue("pinXp").toInt();
      // pinXm=hardwareSettings.getValue("pinXm").toInt();
      // pinYp=hardwareSettings.getValue("pinYp").toInt();
      // pinYm=hardwareSettings.getValue("pinYm").toInt();
      pinServoX=hardwareSettings.getValue("pinServoX").toInt();
      pinServoY=hardwareSettings.getValue("pinServoY").toInt();
      mdlDutyX=hardwareSettings.getValue("mdlDutyX").toInt(); 
      mdlDutyY=hardwareSettings.getValue("mdlDutyY").toInt(); 
      minDutyX=hardwareSettings.getValue("minDutyX").toInt(); 
      minDutyY=hardwareSettings.getValue("minDutyY").toInt(); 
      maxDutyX=hardwareSettings.getValue("maxDutyX").toInt(); 
      maxDutyY=hardwareSettings.getValue("maxDutyY").toInt(); 
      xMin=hardwareSettings.getValue("xMin").toInt();
      xMax=hardwareSettings.getValue("xMax").toInt();
      yMin=hardwareSettings.getValue("yMin").toInt();
      yMax=hardwareSettings.getValue("yMax").toInt();
    }

