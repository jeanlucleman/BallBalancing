#include "functions.h"

//#ifdef esp8266
//#include <time.h>
//#endif
// Clock* myClock1=Clock::getInstance(); // Creates a clock. It gives the current date/time by date/time at t0 + millis elapsed since t0. t0 is the date/time given by a call to Central Unit at t0

vector<String> split(String my_str, String separator) 
  {
    vector<String> result;
    unsigned int start=0;
    for (unsigned int i=0; i<my_str.length();i++)
      {
      if(my_str.substring(i,i+1)==separator)
        {
          result.push_back(my_str.substring(start,i));
          start=i+1;
        }
      }
    if(start<my_str.length())
      {
        result.push_back(my_str.substring(start,my_str.length()));
      }
    return result;
  }

long interpol(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int StringSplit(String sInput, char cDelim, String param[], int iMaxParams)
{
    int iParamCount = 0;
    int iPosDelim=0, iPosStart = 0;

    do {
        // Searching the delimiter using indexOf()
        //Serial.print("iPosDelim : ");Serial.println(iPosDelim);
        iPosDelim = sInput.indexOf(cDelim,iPosStart);
        //Serial.print("iPosDelim : ");Serial.println(iPosDelim);
        if (iPosDelim >= (iPosStart+1)) {
            // Adding a new parameter using substring() 
            param[iParamCount] = sInput.substring(iPosStart,iPosDelim);
            //Serial.println("param[iParamCount] : " + param[iParamCount]);
            iParamCount++;
            // Checking the number of parameters
            if (iParamCount >= iMaxParams) {
                return (iParamCount);
            }
            iPosStart = iPosDelim + 1;
        }
    } while (iPosDelim >= 0);
    if (iParamCount < iMaxParams) {
        // Adding the last parameter as the end of the line
        param[iParamCount] = sInput.substring(iPosStart);
        iParamCount++;
    }

    return (iParamCount);
}

unsigned int HEX2INT(String value)
  {
     return strtoul(value.c_str(),0,16);
  }

String INT2HEX(unsigned int value)
  {
    char str[10];      
    sprintf(str,"%06X",value);
    return str;
  }
String time_t2HHMM(time_t thisTime)
  {
    int HH, MM;
    HH=(thisTime/3600)%24;
    MM=(thisTime%3600)/60;
    char buffer[6];
    sprintf(buffer,"%02d:%02d\0",HH,MM);
    return (buffer);
  }

  time_t HHMMSS2Time_t(String HHMMSS)
  {
    int hour, min, sec;
    if (sscanf(HHMMSS.c_str(), "%d:%d:%d", &hour, &min, &sec) != 3) return -1;
    return hour * 3600 + min * 60 + sec;
  }
  time_t HHMM2Time_t(String HHMMSS)
  {
    int hour, min;
    if (sscanf(HHMMSS.c_str(), "%d:%d", &hour, &min) != 2) return -1;
    return hour * 3600 + min * 60 ;
  }
