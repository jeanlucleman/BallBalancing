#include <Arduino.h>

#include "global.h"
#include "functions.h"
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "clock.h"
#include <ESPAsyncWebServer.h>
#include <version.h> 
#include <Adafruit_MPU6050.h>
#define TEMPLATE_PLACEHOLDER '$' // Allows to not have problème with the character % in style, making confusion with % in place holder. 
// Not understood as I continue t use % and not $...
  WiFiUDP UdpClock;
  NTPClient timeClient(UdpClock, "pool.ntp.org",3600,3600); // works only if STA mode OK. Time offset 1 hour, update each hour. See https://randomnerdtutorials.com/esp8266-nodemcu-date-time-ntp-client-server-arduino/
  Clock* myClock=Clock::getInstance(); // Creates a clock. It gives the current date/time by date/time at t0 + millis elapsed since t0.
  bool staModeOK=false;  // Report success of STA connexion
  //Authentication Variables that will be used by the UDP clients (doorSensores)
  String AP_SSID, AP_PSW; // WIFI Name of this Access Point giving access to the ESP8266 though AP mode (ex. 192.168.4.1)
  //Authentication Variables that will be used by the http clients (users)
  String STA_SSID, STA_PSW; // WIFI Name of the router allowing access to the ESP8266 through the home router (ex. 192.168.2.98)
  //WiFi settings for the AP mode
  IPAddress APlocal_IP(192, 168, 4, 1);   //Access Point (AP) Local IP Address
  IPAddress APgateway(192, 168, 4, 1);     //Access Point (AP) Gateway Address
  IPAddress APsubnet(255, 255, 255, 0);    //Access Point (AP) Subnet Address
  AsyncWebServer * server;                // Server creation (it can be access from AP of STA mode)
  bool setupClock();                      // Initialize a NTP client to connect on a NTP server to get time and date
  bool setClockFromOTP();                 // Get time and date from a NTP client and update the clock (see Clock class)
  bool startStaMode();
  void setupWifi();      // Setup and start Wifi in AP and STA mode, then starts the server
  void startServer();
  bool rebootRequested=false;             // When the server receives /reboot, this boolean is set to true and is read in the loop
  String pageName;                        // Nom de la page html à afficher (appelé dans tmplAdmin.html, ex. info => info.html qui est dans le répertoire data, )
  // bool startStab=true;


// TOUCHPAD
// Variables for the touchpad
// int pinXp; // Input pin for touch pad XP - can be a digital pin, used only to write LOW or HIGH
// int pinXm; // Input pin for touch pad XM - Must be an analog pin able to read and write
// int pinYp; // Input pin for touch pad YP - Must be an analog pin able to read and write
// int pinYm; // Input pin for touch pad YM - can be a digital pin, used only to write LOW or HIGH
// Note: input analog pins must not be on ADC2 which is used by WiFi

// TaskHandle_t taskReadTouchpad = NULL; // Task to read touchpad
// Values are given with connector on the left side, in landscape orientation
// and in the range 0 (0V), 4095 (3.3V)
// Analog read of touch pad resistance. Défault value to be calibrated in a dedicated web page
int xMin=0; // X value left side, 
int xMax=150; // X value right side
int yMin=0; // Y value bottom side
int yMax=150; // Y value top side

float xOrigin=75;
float yOrigin=75;

// Return the touchpad X value on 10 bits (0-4095)
// int readTouchX(void); 
// Return the touchpad Y value on 10 bits (0-4095)
// int readTouchY(void); 
// Return true if ball on the touchpad
// bool checkContact(void); 
struct BallPosition
{
    float x;
    float y;

    uint32_t sequence;
    uint32_t timestamp;

    bool valid;
};
BallPosition latestBall;
BallPosition ballNow;
bool getBallPosition(BallPosition& ball);
BallPosition ballTarget;
portMUX_TYPE ballMux = portMUX_INITIALIZER_UNLOCKED;
// STEPPER MOTOR CONTROL
// Connections to A4988
const int xDirPin = 23;  // Direction
const int xStepPin = 18; // Step
const int yDirPin = 27;  // Direction
const int yStepPin = 19; // Step
const int ms1Pin = 2;    // ms pin are used to define the step. Hith 3 pin HIGH step = 1/16 of full step
const int ms2Pin = 4;
const int ms3Pin = 15;

// A verifier:
#define CAM_RX 32
#define CAM_TX 33


int       pinServoX;
int       pinServoY;
int       mdlDutyX;
int       mdlDutyY;
int       minDutyX;
int       minDutyY;
int       maxDutyX;
int       maxDutyY;

// Write X and Y values on the 2 servos
void setServo(int xValue, int yValue); 
void setServo(int xValue, int yValue)
  {
    // Serial.printf("xValue: %d   yValue: %d\n",xValue,yValue);
    ledcWrite(0, xValue); // Write the value to the PWM channel 0 (X axis)
    ledcWrite(1, yValue); // Write the value to the PWM channel 1 (Y axis)
  }

// ACCELEROMETER
Adafruit_MPU6050 mpu;
TaskHandle_t taskReadMPU6050; // Task to process data change
sensors_event_t a, g, temp; // Data from accelerometer



// bool contact=false;
// int xBallNow;
// int yBallNow;
// int xBallPrev;
// int yBallPrev;
BallPosition ballPrev;
// int xSpeed;
float xBallSpeed;
float yBallSpeed;
// int ySpeed;

unsigned long lastCurrentMicros;
unsigned long currentMicros;
unsigned long dt;
// bool contactLost=false;
// int nCycle=0;

void processCamMessage(char* message);
float avgXSpeed=0;

void camUartTask(void *parameter)
{
    static char buffer[64];
    uint8_t index = 0;

    for (;;)
    {
        while (Serial2.available())
        {
            char c = Serial2.read();

            if (c == '\n')
            {
                buffer[index] = '\0';

                processCamMessage(buffer);

                index = 0;
            }
            else if (c != '\r')
            {
                if (index < sizeof(buffer) - 1)
                {
                    buffer[index++] = c;
                }
                else
                {
                    // Invalid/too long message
                    index = 0;
                }
            }
        }

        // Give the CPU to other tasks
        vTaskDelay(1);
    }
}

void setup() 
  {
    Serial.begin(115200);            //Set Serial Port to 115200 baud
    debugln("SETUP");
    Serial2.begin(115200,SERIAL_8N1,CAM_RX,CAM_TX); // Serial port for camera
    Serial.println("Serial2 started");
    xTaskCreatePinnedToCore(camUartTask,"CamUART",4096,nullptr,2,nullptr,0);
    if (!initialisations())
      {
        debugln(F("Initialization failed!"));
        return;
      }
    // Serial.printf("Pin Xp: %d\n",pinXp);
    // Serial.println(pinXp);
    // Serial.println(pinXm);
    // Serial.println(pinYp);
    // Serial.println(pinYm);


    Serial.println(xMin);
    Serial.println(xMax);
    Serial.println(yMin);
    Serial.println(yMax);

    setupWifi();                  
    setupClock(); 
    startServer(); 
    getBallPosition(ballPrev);
    // ballPrev.x=readTouchX();
    // ballPrev.y=readTouchY();  
    debuglnDated("Setup completed!");
    debugln("--------------------------------");
  }
int prevX=mdlDutyX;
int prevY=mdlDutyY;
int n=1;
int sumDx=0;
int sumDy=0;
// int stepX=90;
// int stepY=90;
// int xStopL=1500;
// int xStopR=3500;
// int yStopT=2500;
// int yStopB=850;




// int xBallTarget=xOrigin;
// int yBallTarget=yOrigin;
void loop() {
  
  ballTarget.x=xOrigin;
  ballTarget.y=yOrigin;


  if(getBallPosition(ballNow))
    {
      Serial.printf("xBallTarget: %d   yBallTarget: %d\n",ballTarget.x,ballTarget.y);
      int dx=ballNow.x-ballTarget.x;
      int dy=ballNow.y-ballTarget.y;
      sumDx+=dx;
      sumDy+=dy;
      // int xDuty=mdlDutyX + (float)dx/200.0 + xBallSpeed*abs(xBallSpeed)/0.2 + (float)sumDx/10000.0;     
      int xDuty=mdlDutyX + (float)dx/200.0 + xBallSpeed*2.5 + (float)sumDx/10000.0;     
      int yDuty=mdlDutyY + (float)dy/200.0 + yBallSpeed*2.3 + (float)sumDy/10000.0;
      // int yDuty=mdlDutyY + dy/200 + ySpeed*abs(ySpeed)/105 + sumDy/10000;

      xDuty=(xDuty+(n-1)*prevX)/n;

      yDuty=(yDuty+(n-1)*prevY)/n;
      prevX=xDuty;
      prevY=yDuty;

      if(xDuty>maxDutyX){xDuty=maxDutyX;}
      if(yDuty>maxDutyY){yDuty=maxDutyY;}
      if(xDuty<minDutyX){xDuty=minDutyX;}
      if(yDuty<minDutyY){yDuty=minDutyY;}
      // Serial/printf("dx= %0000d  dy= %0000d   Vx=%000d   Vy= %000d  SumDx= %d  SumDy= %d  xDuty=%d   yDuty=%d\n", dx,dy,xSpeed,ySpeed,sumDx,sumDy,xDuty,yDuty);  
      setServo(xDuty,yDuty);
    }


 

  delay(50);
}

String getFileContent(String fileName)
  {
    File file;
    #ifdef ESP_8266
      file = LittleFS.open("/" + fileName, "r");
    #elif defined ESP_32
      file = SPIFFS.open(fileName);
    #endif
    if(!file)
      {
        return "There was an error opening the file " +fileName;
      }
    String fileContent;
    //String lineRead;
    //while (file.available()) 
    //  {
    printlnDated("Reading file content from " + fileName);
    fileContent=file.readString();
    debuglnDated("File content: \n\r" + fileContent);
        //lineRead = file.readStringUntil('\n'); // one line should be in the form key,value,description
        //debugln("" + lineRead);
        //lineRead.trim(); // to remove cr or other control character beside the string
        //fileContent = fileContent + lineRead;
    //  }
    file.close();
    debuglnDated("Closing file '" + fileName + "'");
    return fileContent;

  }
String getFileList()
  {
    debuglnDated("Entrée dans getFileList");
    String listFiles;
    #ifdef ESP_8266
    if (!LittleFS.begin())
    #elif defined ESP_32
    if (!SPIFFS.begin(false))
    #endif
    {
      debugln("An Error has occurred while mounting file system");
      return "";
    }
    String sep="$$";
    #ifdef ESP_8266
    Dir root = LittleFS.openDir("/");

    while (root.next()) 
      {
        listFiles += String(root.fileName()) + sep;
        if(root.fileSize()) 
          {
            File f = root.openFile("r");
            debugDated("Taille: ");
            debugln(f.size());
          }
      }


    #elif defined ESP_32
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file)
    {
      listFiles += String(file.name()) + sep;
      file = root.openNextFile();
    }

    #endif
    debuglnDated("Return from getFileList");
    return listFiles;
  }
String processorTmplAdmin(const String &var)
  { 
    if (var == "DESCRIPTION")
      {
        //return commonData.getDescription("espName");
        return "Ball balancer";
      }
    if (var== "PAGENAME")
      {
        // Serial.println(pageName);
        return pageName;
      }
    return "";
  }
String processorSummary(const String &var)
  { 
    return "";
  }
String processorServoCalibration(const String &var)
  { 
    Serial.println("processorServoCalibration");
    Serial.println(var);
    if (var == "MINDUTYX")
      {
        Serial.println("MINDUTYX");
        return hardwareSettings.getValue("minDutyX");
      }
    if (var == "MAXDUTYX")
      {
        return hardwareSettings.getValue("maxDutyX");
      }
    if (var == "MDLDUTYX")
      {
        return hardwareSettings.getValue("mdlDutyX");
      }
    if (var == "MINDUTYY")
      {
        return hardwareSettings.getValue("minDutyY");
      }
    if (var == "MAXDUTYY")
      {
        return hardwareSettings.getValue("maxDutyY");
      }
    if (var == "MDLDUTYY")
      {
        return hardwareSettings.getValue("mdlDutyY");
      }
      
    return "";
  }
String processorConfWifi(const String &var)
  { // le & signifie que var est passé par référence
    if (var == "STAIP")
      {
        return commonData.getValue("staIp");
      }
    if (var == "MAINSSID")
      {
        return commonData.getValue("STA_SSID");
      }
    if (var == "MAINPSW")
      {
        return commonData.getValue("STA_PSW");
      }
    if (var == "STAGATEWAY")
      {
        return commonData.getValue("staGateway");
      }  
    if (var == "STASUBNET")
      {
        return commonData.getValue("staSubnet");
      }
    if (var == "STADNS")
      {
        return commonData.getValue("staDNS");
      }
    return "";
  }      
String processorConfHardware(const String &var)
  { // le & signifie que var est passé par référence
    if (var == "PINXP")
      {
        return hardwareSettings.getValue("pinXp");
      }
    if (var == "PINXM")
      {
        return hardwareSettings.getValue("pinXm");
      }
    if (var == "PINYP")
      {
        return hardwareSettings.getValue("pinYp");
      }
    if (var == "PINYM")
      {
        return hardwareSettings.getValue("pinYm");
      }

    if (var == "PINSERVOX")
      {
        return hardwareSettings.getValue("pinServoX");
      }
    if (var == "PINSERVOY")
      {
        return hardwareSettings.getValue("pinServoY");
      }

    return "";
  }      
String processorFiles(const String &var)
      { // le & signifie que var est passé par référence
        return "";
      }      
String processorInfo(const String &var)
    { // le & signifie que var est passé par référence
      // if (var == "ESPNAME")
      //   {
      //     return "xxxx";//macDeviceData.getValue("espName") + " (type: " + MACDEVICETYPE + ")" ;
      //     //return espName;
      //   }
      if (var=="FIRMWAREVERSION")
        {
          return VERSION;
        }
      if (var == "CURRENTIP")
        {
          return WiFi.localIP().toString();
        }
      if (var == "CURRENTSSID")
        {
          return WiFi.SSID();
        }
      // if (var == "CURRENTSTRENGTH")
      //   {
      //     return "TBD";
      //   }
      if (var == "CURRENTGATEWAY")
        {
          return WiFi.gatewayIP().toString();
        }
      if (var == "CURRENTSUBNET")
        {
          return WiFi.subnetMask().toString();
        }
      if (var == "CURRENTDNS")
        {
          return WiFi.dnsIP().toString();
        }
      if (var == "CURRENTAPIP")
        {
          return WiFi.softAPIP().toString();
        }
      if (var == "CURRENTAPMAC")
        {
          return WiFi.softAPmacAddress();
        }
      if (var == "CURRENTSTAMAC")
        {
          return WiFi.macAddress();
        }
      if (var=="LOCALTIME")
        {
          return myClock->getNow_YYYY_MM_DD_HH_MM_SS();
        }
      if (var=="UPTIME")
        {
          unsigned long uptime=millis()/1000;
          int days = uptime/86400;
          int hours = (uptime-86400*days)/3600;
          int minutes = (uptime -86400*days-3600*hours)/60;
          int secondes = uptime -86400*days-3600*hours - 60*minutes;
          return String(days)+"j " + String(hours)+ "h " + String(minutes) + "min "+ String(secondes) + "s";
        }
      if (var == "STARTTIME")
        {
          //unsigned long startMillis =13000;
          return myClock->getDateTime_YYYY_MM_DD_HH_MM_SS(0); // The clock counter start at millis = 0
        }
      return "";
    }      
void startServer()
  {
    #ifdef ESP_32
      #define FILESYSTEM SPIFFS
    #elif defined ESP_8266
      #define FILESYSTEM LittleFS
    #endif
    server = new AsyncWebServer(80);
    server->serveStatic("/w3.css", SPIFFS, "/css/w3.css").setCacheControl("max-age=2592000"); 
    server->serveStatic("/jquery-1.8.3.min.js", SPIFFS,"/js/jquery-1.8.3.min.js" ).setCacheControl("max-age=2592000");
    // server->serveStatic("/copyright.js", LittleFS,"/js/copyright.js" ).setCacheControl("max-age=2592000");
    server->serveStatic("/favicon.ico",SPIFFS,"/images/favicon.png").setCacheControl("max-age=2592000");
    server->serveStatic("/delete.png", SPIFFS, "/images/delete.png").setCacheControl("max-age=2592000");
    server->on("/confWifi", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        pageName="confWifi";
        request->send(FILESYSTEM, "/tmplAdmin.html", String(), false, processorTmplAdmin);
      });
    server->on("/confWifi.html", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        request->send(FILESYSTEM, "/confWifi.html", String(), false, processorConfWifi);
      });
    server->on("/confHardware", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        pageName="confHardware";
        request->send(FILESYSTEM, "/tmplAdmin.html", String(), false, processorTmplAdmin);
      });
    server->on("/confHardware.html", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        request->send(FILESYSTEM, "/confHardware.html", String(), false, processorConfHardware);
      });
    server->on("/servoCalibration", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        pageName="servoCalibration";
        request->send(FILESYSTEM, "/tmplAdmin.html", String(), false, processorTmplAdmin);
      });
    server->on("/servoCalibration.html", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        Serial.println("Loading servoCalibration.html");
        request->send(FILESYSTEM, "/servoCalibration.html", String(), false, processorServoCalibration);
      });      
    server->on("/info.html", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
      request->send(FILESYSTEM, "/info.html", String(), false, processorInfo);
      });  
    server->on("/files", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        pageName="files";
        request->send(FILESYSTEM, "/tmplAdmin.html", String(), false, processorTmplAdmin);
      });
    server->on("/files.html", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
      request->send(FILESYSTEM, "/files.html", String(), false, processorFiles);
      });
    server->on("/getFileList", HTTP_GET, [](AsyncWebServerRequest *request)
      {
        debuglnDated(F("The server received a request 'getFileList'"));
        String fileList=getFileList();
        request->send(200, "text/plain", fileList);
        debuglnDated(F("The server returned file list"));
      });
    // Réception d'une commande de type /getFileContent?fileName=MDS_01.txt
    server->on("/getFileContent", HTTP_GET, [](AsyncWebServerRequest *request)
      {
        debugln(F("The server received a request 'getFileContent'"));
        int paramsNr = request->params();
        debug("Avec ");
        debug(paramsNr);
        debugln(" paramêtre(s)");
        String fileName;
        String fileContent="Rien...";
        for(int i=0;i<paramsNr;i++)
          {
            const AsyncWebParameter* p = request->getParam(i);
            if(p->name()=="fileName")
              {
                fileName=p->value();
                debugln("Fichier demandé: " + fileName);
                fileContent = getFileContent(fileName); 
              }
          }

          request->send(200, "text/plain", fileContent);
          debugln("Contenu envoyé!");
      });
    server->on("/saveFile", HTTP_POST,[](AsyncWebServerRequest *request) {
      debugln("Request saveFile received");
      //char fpn[80];
      String fpn;
      // char *ptr = fpn;
      char fileContent[2000];
      if (request->hasParam("filePathName", true)) 
        {
          // * ptr++='/';
          fpn="/" + request->getParam("filePathName", true)->value();
          // strcpy(ptr,request->getParam("filePathName", true)->value().c_str());
          debugln(fpn);
        }
      if (request->hasParam("fileText", true)) 
        {
          strcpy(fileContent,request->getParam("fileText", true)->value().c_str());
          request->send(200, "text/plain", "Text saved...");
        }
      File fileToSave;
      if (fileToSave)
          {
            fileToSave.close();
          }
      debugln("Opening fileToSave in write mode");
      #ifdef ESP_8266
        fileToSave = LittleFS.open(fpn, "w"); 
      #else
        fileToSave = SPIFFS.open(fpn, "w"); 
      #endif
      fileToSave.printf("%s",fileContent);
      fileToSave.flush();
      fileToSave.close();
    });
    server->on("/setServo", HTTP_POST, [](AsyncWebServerRequest *request) 
      {
        String xValue;
        String yValue;
        Serial.println("Set servo");
        if (request->hasParam("xValue", true))
          {
            xValue = request->getParam("xValue", true)->value();
            Serial.printf("Régler la servo x sur %s\n",xValue);
          }
        if (request->hasParam("yValue", true))
          {
            yValue = request->getParam("yValue", true)->value();
            Serial.printf("Régler la servo y sur %s\n",yValue);
          }
        setServo(xValue.toInt(),yValue.toInt());

      });
    server->on("/updateConfWifi", HTTP_POST, [](AsyncWebServerRequest *request) 
      {
        printlnDated("Updating credentials...");
        // String espName;
        String mainSSID;
        String mainPSW;
        // String fallbackSSID;
        // String fallbackPSW;
        String staGateway;
        String staSubnet;
        String staDNS;
        String staIp;
        // String gmtOffset;
        // String dst;
        bool commonDataChange = false;
        // bool macDeviceDataChange = false;
        if (request->hasParam("mainSSID", true))
          {
            mainSSID = request->getParam("mainSSID", true)->value();
            if(commonData.setValue("STA_SSID", mainSSID, "Main SSID")){commonDataChange=true;};
          }
        if (request->hasParam("mainPSW", true))
          {
            mainPSW = request->getParam("mainPSW", true)->value();
            if(commonData.setValue("STA_PSW", mainPSW, "Psw main SSID")){commonDataChange=true;};
          }
        if (request->hasParam("staIp", true))
          {
            staIp = request->getParam("staIp", true)->value();
            if(commonData.setValue("staIp", staIp, "Adresse Ip souhaitée")){commonDataChange=true;};
          }
          
        if (request->hasParam("staSubnet", true))
          {
            staSubnet = request->getParam("staSubnet", true)->value();
            if(commonData.setValue("staSubnet", staSubnet, "Subnet en mode STA")){commonDataChange=true;};
          }
        if (request->hasParam("staGateway", true))
          {
            staGateway = request->getParam("staGateway", true)->value();
            if(commonData.setValue("staGateway", staGateway, "Gateway en mode STA")){commonDataChange=true;};
          }
        if (request->hasParam("staDNS", true))
          {
            staDNS = request->getParam("staDNS", true)->value();
            if(commonData.setValue("staDNS", staDNS, "DNS en mode STA")){commonDataChange=true;};
          }
        if (commonDataChange){commonData.save();}
        // if (macDeviceDataChange){macDeviceData.save();}
      });
    server->on("/updateConfHardware", HTTP_POST, [](AsyncWebServerRequest *request) 
      {
        printlnDated("Updating config Hardware...");
        // String espName;
        String pinXp;
        String pinXm;
        String pinYp;
        String pinYm;
        String pinServoX;
        String pinServoY;
          bool hardwareDataChange = false;
        // bool macDeviceDataChange = false;
        if (request->hasParam("pinXp", true))
          {
            pinXp = request->getParam("pinXp", true)->value();
            if(hardwareSettings.setValue("pinXp", pinXp, "pin Xp")){hardwareDataChange=true;};
          }
        if (request->hasParam("pinXm", true))
          {
            pinXp = request->getParam("pinXm", true)->value();
            if(hardwareSettings.setValue("pinXm", pinXp, "pin Xm")){hardwareDataChange=true;};
          }
        if (request->hasParam("pinYp", true))
          {
            pinYp = request->getParam("pinYp", true)->value();
            if(hardwareSettings.setValue("pinYp", pinYp, "pin Yp")){hardwareDataChange=true;};
          }
        if (request->hasParam("pinYm", true))
          {
            pinYm = request->getParam("pinYm", true)->value();
            if(hardwareSettings.setValue("pinYm", pinYm, "pin Ym")){hardwareDataChange=true;};
          }
        if (request->hasParam("pinServoX", true))
          {
            pinServoX = request->getParam("pinServoX", true)->value();
            if(hardwareSettings.setValue("pinServoX", pinServoX, "Pin servo X")){hardwareDataChange=true;};
          }

        if (request->hasParam("pinServoY", true))
          {
            pinServoY = request->getParam("pinServoY", true)->value();
            if(hardwareSettings.setValue("pinServoY", pinServoY, "Pin servo Y")){hardwareDataChange=true;};
          }

        if (hardwareDataChange){hardwareSettings.save();}
        // if (macDeviceDataChange){macDeviceData.save();}
      });          
    server->on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        request->send(200, "text/plain", "ESP will restart. Return to home page!");
        rebootRequested=true;// ESP.restart();
      });
    server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        pageName="summary";
        debugln("Chargement page Synthèse...");
        request->send(FILESYSTEM, "/tmplAdmin.html", String(), false, processorTmplAdmin);
      });
    server->on("/summary.html", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        request->send(FILESYSTEM, "/summary.html", String(), false, processorSummary);
      });
    server->on("/info", HTTP_GET, [](AsyncWebServerRequest *request) 
      {
        pageName="info";
        debugln("Chargement page Info...");
        request->send(FILESYSTEM, "/tmplAdmin.html", String(), false, processorTmplAdmin);
      });
    
    // Request from ESP Hooter

    server->on("/checkConnexion", HTTP_GET, [](AsyncWebServerRequest *request)
      {
        request->send(200, "text/plain", "OK");
      });
    server->begin();
  }
void printDated(String text)
  {
    debug(myClock->getNow_YYYY_MM_DD_HH_MM_SS() + " " +  text);
  }
/**
 * @brief Prints on the serial console a text with a time prefix.Carriage return at end of line
 * 
 * @param text texte to be displayed
 */
void printlnDated(String text)
  {
    // debugln(text);
    debugf3("%s %s\n\r",myClock->getNow_YYYY_MM_DD_HH_MM_SS().c_str() ,text.c_str());
  }
bool setupClock()
  {
    if(staModeOK)
      {
        debugln("staMode is OK, starting otp time...");
        timeClient.begin();  // Initialize a NTPClient to get time
        timeClient.setTimeOffset(commonData.getValue("GMT_offset").toInt()*3600);// To be read from settingsto  adjust for your timezone
        return setClockFromOTP();
      }
    return false;
  }
bool setClockFromOTP()
    {
      debugln("Trying to set myClock");
      if(staModeOK)
        {
          timeClient.update();
          time_t epochTime = timeClient.getEpochTime();
          myClock->setClockFromUC(epochTime);
          String response= String(epochTime);
          // myCom->writeString(Action::enm_time,response,true); // false); // Time is sent to phone detector when ESP_Hub starts
          debuglnDated(F("Clock has been updated from OTP server"));
          return true;
        }
      else 
        {
          debuglnDated(F("Clock update not done because STA is off"));
          return false;
        }
    }
void getCredentials() // Read wifi credential from settings file
  {
    Serial.println("getCredentials");
    AP_SSID  = commonData.getValue("AP_SSID");
    AP_PSW   = commonData.getValue("AP_PSW");
    STA_SSID = commonData.getValue("STA_SSID");
    STA_PSW  = commonData.getValue("STA_PSW");
  }
void setupWifi()      // Setup and start Wifi in AP and STA mode, then starts the server
  {
    getCredentials();//AP_SSID,AP_PSW, STA_SSID, STA_PSW);  // Gets credentials for AP and STA configuration
    WiFi.mode(WIFI_AP_STA);                             //Set WiFi mode to AP and STA 
    debugln("WIFI Mode set to: AP Station");
    // Configuring AP
    WiFi.softAPConfig(APlocal_IP, APgateway, APsubnet);  
    WiFi.softAP(AP_SSID.c_str(), AP_PSW.c_str(),1,0);              //WiFi.softAP(ssid, password, channel, hidden, max_connection)                         
    debugln("WIFI Named " + AP_SSID + " started (Access point mode)"); //Send info to monitor if debug = true
    delay(50);                                             //Wait a bit
    IPAddress IP = WiFi.softAPIP();                        //Get server IP
    debug("AccessPoint IP: ");
    debugln(IP);
    // Configuring STA
    staModeOK=startStaMode();

  }
bool startStaMode()   // Start wifi in STA mode
  {
    IPAddress local_IP ; 
    IPAddress gateway;
    IPAddress subnet;
    IPAddress dns; // Cette ligne est nécessaire si on appelle des sites extérieurs (comme pool.ntp.org).
    bool staModeSuccess;
    local_IP.fromString(commonData.getValue("staIp"));
    gateway.fromString(commonData.getValue("staGateway")); //192, 168, 2, 1);
    subnet.fromString(commonData.getValue("staSubnet")); //(255, 255, 0, 0);
    dns.fromString(commonData.getValue("staDNS")); //(192, 168, 2, 1); // Cette ligne est nécessaire si on appelle des sites extérieurs (comme pool.ntp.ord).
    debugln(commonData.getValue("staIp"));
    debugln(commonData.getValue("staGateway")); //192, 168, 2, 1);
    debugln(commonData.getValue("staSubnet")); //(255, 255, 0, 0);
    debugln(commonData.getValue("staDNS")); //
    debugln(STA_SSID);
    debugln(STA_PSW);
    
    
    // IPAddress prim_dns(192,168,1,1); //https://github.com/me-no-dev/ESPAsyncWebServer/issues/928
    // IPAddress sec_dns(192,168,1,1);
    if (!WiFi.config(local_IP, gateway, subnet,dns)) //, prim_dns, sec_dns)) // Configures static IP address
      {
        debugln("STA Failed to configure");
      }
    WiFi.begin(STA_SSID.c_str(), STA_PSW.c_str());
    delay(100);
    debugln(F("Tentative de connexion en mode STA..."));
    unsigned long startMillis = millis();
    staModeSuccess=true;
    while (WiFi.status() != WL_CONNECTED)
      {
        // yield();
        if (millis() > startMillis + 25000)
          {
            debugln(F("\nWifi connexion in STA mode timeout reached!"));
            staModeSuccess=false;
            WiFi.mode(WIFI_AP);
            debugln("Passage en mode AP");
            break;
          }
        delay(1000);
      }
    if(staModeSuccess)
      {
        debugln("\nAdresse IP sur le réseau : " + WiFi.localIP().toString());
        debugln("Adresse DNS              : " + WiFi.dnsIP().toString());
        debugln("Gateway                  : " + WiFi.gatewayIP().toString());
        debug("Connexion OK en mode STA sur ");
        debugln(WiFi.localIP());
      }
    // Starting server (if STA mode fails, is accessible in AP mode on 192.168.4.1)
    debug("Mac address: ")
    debugln(WiFi.macAddress());
    
    return staModeSuccess;
  }
// bool controlBall(BallPosition ball)
//   {
   

//         Serial.printf("X= %d    Y=%d dt=%d ",ball.x,ball.y,dt); 
//         // xSpeed=(xBallNow-xBallPrev);
//         xBallSpeed=(xBallNow-xBallPrev)/(float)nCycle/dt*1000;
//         if(avgXSpeed=0){avgXSpeed=xBallSpeed;}
//         if(avgXSpeed>0)
//           {
//             if(xBallSpeed>3.0*avgXSpeed){xBallSpeed=avgXSpeed;}
//             if(xBallSpeed<-3.0*avgXSpeed){xBallSpeed=avgXSpeed;}                
//           }

//         avgXSpeed=(19.0*avgXSpeed + abs(xBallSpeed))/20.0;
//         yBallSpeed=(yBallNow-yBallPrev)/(float)nCycle/dt*1000;
//         char bufferX[10];
//         char bufferY[10];
//         char bufferAvgX[10];
//         dtostrf(xBallSpeed, 8, 3, bufferX);
//         dtostrf(yBallSpeed, 8, 3, bufferY);
//         dtostrf(avgXSpeed, 8, 3, bufferAvgX);
//         Serial.printf("DX %d  nCycle %d  xBallSpeed = %s   avgSpeedX= %s yBallSpeed = %s  ",(xBallNow-xBallPrev),nCycle,bufferX, bufferAvgX, bufferY);

//         ballPrev=ball;

//         lastCurrentMicros=currentMicros;
//     }
void stopControl()
  {
    Serial.println("Camera has stopped sending valid ball positions. Stopping control.");
    // Implement any additional logic needed to stop the control of the ball
  }
bool getBallPosition(BallPosition& ball)
  {
    currentMicros=micros();
    dt=currentMicros-lastCurrentMicros;   
    portENTER_CRITICAL(&ballMux);
    ball= latestBall;
    portEXIT_CRITICAL(&ballMux);
    if (ball.valid)
    {
        uint32_t age = millis() - ball.timestamp;
        if (age < 200)
        {
            // This is the latest valid position
            // Use ballNow.x and ballNow.y for control
            return true;
         }
        else
        {
            // Camera has stopped sending
            stopControl();
            return false; // Or handle as needed
        }
    }
    else
    {
        // No valid position received yet
        return false;
    }
  }
// int readTouchY(void) {
//   pinMode(pinYp, INPUT);
//   pinMode(pinYm, INPUT);
//   pinMode(pinXp, OUTPUT);
//   digitalWrite(pinXp, HIGH);
//   pinMode(pinXm, OUTPUT);
//   digitalWrite(pinXm, LOW);
//   // delay(20);
//   int y1;
//   int y2;
//   do
//     {
//       y1 = analogRead(pinYp);
//       delay(4);
//       y2 = analogRead(pinYp);
//     } while (abs(y1-y2)>10);

//   return (y1+y2)/2-yOrigin;
// }
// int readTouchX(void) {
//   pinMode(pinXp, INPUT);
//   pinMode(pinXm, INPUT);
//   // digitalWrite(pinXp, LOW);
//   // digitalWrite(pinXm, LOW);
//   pinMode(pinYp, OUTPUT);
//   digitalWrite(pinYp, HIGH);
//   pinMode(pinYm, OUTPUT);
//   digitalWrite(pinYm, LOW);
//   // delay(20);
//   int x1;
//   int x2;
//   do
//     {
//       x1 = analogRead(pinXm);
//       delay(4);
//       x2 = analogRead(pinXm);
//     } while (abs(x2-x1)>10);
//   return 4095-(x1+x2)/2-xOrigin;
// }
// bool checkContact(void) {
//   // Set X+ to ground
//   pinMode(pinXp, OUTPUT);
//   digitalWrite(pinXp, LOW);
//   // Set Y- to VCC
//   pinMode(pinYm, OUTPUT);
//   digitalWrite(pinYm, HIGH);  
//   // Hi-Z X- and Y+
//   digitalWrite(pinXm, LOW);
//   pinMode(pinXm, INPUT);
//   digitalWrite(pinYp, LOW);
//   pinMode(pinYp, INPUT);
//   // delay(20);
//   int z1 = analogRead(pinXm); // No contact = 0
//   int z2 = analogRead(pinYp); // No contact = 4095
//   return (4095-(z2-z1) > 10); // Without contact -> 0
// }
void processCamMessage(char* message)
{
    if (strncmp(message, "BALL,", 5) == 0)
    {
        uint32_t sequence;
        float x;
        float y;

        if (sscanf(
                message + 5,
                "%lu,%f,%f",
                &sequence,
                &x,
                &y
            ) == 3)
        {
          portENTER_CRITICAL(&ballMux);

          latestBall.x = x;
          latestBall.y = y;
          latestBall.sequence = sequence;
          latestBall.timestamp = millis();
          latestBall.valid = true;

          portEXIT_CRITICAL(&ballMux);

          return;
        }
    }

    if (strncmp(message, "MSG,", 4) == 0)
    {
        Serial.println(message + 4);
        return;
    }

    Serial.printf(
        "CAM: %s\n",
        message
    );
}
// void processCamMessages()
// {
//     static char buffer[64];
//     static uint8_t index = 0;

//     while (Serial2.available())
//     {
//         char c = Serial2.read();

//         if (c == '\n')
//         {
//             buffer[index] = '\0';

//             processCamMessage(buffer);

//             index = 0;
//         }
//         else if (c != '\r')
//         {
//             if (index < sizeof(buffer) - 1)
//             {
//                 buffer[index++] = c;
//             }
//             else
//             {
//                 index = 0;
//             }
//         }
//     }
// }
