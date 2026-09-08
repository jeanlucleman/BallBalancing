  #include <Arduino.h>
  #include "clock.h"
  #include "global.h"
  // #include "functions.h"
float millisCorrectionFactor=1;
Clock* Clock::pInstance;

Clock* Clock::getInstance(){
  Serial.begin(115200);
  debugln();
	if (pInstance == NULL)
    {
    //unsigned long periodRefreshClockMillis = defaultPeriodRefreshClockMillis;
      debugln();
      debugln("Création d'une nouvelle instance de clock");
      debugln();
		  pInstance = new Clock();
    }
  else
    {
      debugln("Instance de clock déjà existante");
    }
	return pInstance;
}
// Clock* myClock1=Clock::getInstance(); // Creates a clock. It gives the current date/time by date/time at t0 + millis elapsed since t0. t0 is the date/time given by a call to Central Unit at t0

// Clock* Clock::getInstance(int gmtOffset, int daylightOffset){
// 	if (pInstance == 0)
// 		{
//       //String ntpServer="pool.ntp.org";
//       unsigned long periodRefreshClockMillis = defaultPeriodRefreshClockMillis;
//       pInstance = new Clock( gmtOffset,  daylightOffset,  periodRefreshClockMillis);
//     }
// 	return pInstance;
// }

// Clock* Clock::getInstance(int gmtOffset, int daylightOffset, unsigned long periodRefreshClockMillis){
// 	if (pInstance == 0)
// 		{pInstance = new Clock( gmtOffset,  daylightOffset,  periodRefreshClockMillis);}
// 	return pInstance;
// }


Clock::Clock(){
  // Serial.begin(115200);
  // delay(1000);
  debugln(F("Initialisation dans Clock:Clock"));
}
// void Clock::setDst(int dst){
//   _dayLightOffset=dst * 3600;
// }

void Clock::setGmtOffset(int gmtOffset){
  _gmtOffset=gmtOffset;//  * 3600;
}


void Clock::setClockFromUC(unsigned long now)
  {
    unsigned long prevLastTimeUpdate=lastTimeUpdate;
    // Serial.print(F("  [clock.cpp] lastTimeUpdate1: "));
    // Serial.println(lastTimeUpdate);
    unsigned long prevMillisLastTimeUpdate = millisLastTimeUpdate;
    lastTimeUpdate=now; // Now as date time given by the UC. Given in second from Unix EPOCH (1/1/1970)

    // Serial.print(F("  [clock.cpp] lastTimeUpdate2: "));
    // Serial.println(lastTimeUpdate);
    millisLastTimeUpdate = millis(); // Now as millis. Given in milliseconds elapsed since device start.
    millisCorrectionFactor= float((lastTimeUpdate-prevLastTimeUpdate)*1000)/float(millisLastTimeUpdate-prevMillisLastTimeUpdate);
    if (millisCorrectionFactor>1){millisCorrectionFactor=1;}// For the first start
    // Serial.println((lastTimeUpdate-prevLastTimeUpdate));
    // Serial.println(millisLastTimeUpdate-prevMillisLastTimeUpdate);
    // Serial.print("millisCorrectionFactor: ");
    // Serial.println (millisCorrectionFactor);

  }

// void Clock::refreshClock()
//   {
//     time(&lastTimeUpdate);
//     millisLastTimeUpdate = millis();
//   }

time_t Clock::localDateTime()
  {
    return lastTimeUpdate + millis()/1000 -millisLastTimeUpdate/1000 + _gmtOffset ;
  }


/*** 
 * This function will return the local time for a given millis (number of milliseconds elapsed since the device started) 
 ***/
struct tm * Clock::getLocalTime(unsigned long timeMillis)
  {
    timeMillis=timeMillis/millisCorrectionFactor;
    time_t  dateTime=lastTimeUpdate + timeMillis/1000 -millisLastTimeUpdate/1000 + _gmtOffset ;// +_dayLightOffset; // la division doit être faite sur un nombre >0 donc la soustraction est faite APRES la division
    return localtime(&dateTime);
  }

/***
 * This function returns the formatted datetime now. 
 */

String Clock::getNow_YYYY_MM_DD_HH_MM_SS() {
  char buffer [80];
  struct tm * timeInfo = getLocalTime(millis());
  strftime (buffer,80,"%Y/%m/%d %H:%M:%S", timeInfo);
  return buffer;
}

String Clock::getDateTime_YYYY_MM_DD_HH_MM_SS(unsigned long timeMillis) {
  char buffer [80];
  struct tm * timeInfo = getLocalTime(timeMillis);
  strftime (buffer,80,"%Y/%m/%d %H:%M:%S", timeInfo);
  return buffer;
}


String Clock::getNow_YYYY_MM_DD(){
  char buffer [80];
  struct tm * timeInfo = getLocalTime(millis());
  strftime (buffer,80,"%Y-%m-%d", timeInfo);
  return buffer;
}
String Clock::getNow_HH_MM_SS(){
  char buffer [80];
  struct tm * timeInfo = getLocalTime(millis());
  strftime (buffer,80,"%H:%M:%S", timeInfo);
  return buffer;
  }

String Clock::getNow_HH_MM_SS_MMM(){
  char buffer [80];
  struct tm * timeInfo = getLocalTime(millis());
  strftime (buffer,80,"%H:%M:%S", timeInfo);
  String mmm = "." + String((millis()-millisLastTimeUpdate)%1000);
  return buffer + mmm;
  }
