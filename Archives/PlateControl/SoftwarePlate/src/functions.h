#ifndef FUNCTIONS_H
#define FUNCTIONS_H

#include <Arduino.h>
#include <vector>
// #include "clock.h"
using namespace std;
//void setName(String name);
//String getName();
// This returns a dynamic array containing the tests between comma 
vector<String> split(String my_str, String separator);
// String getLocalTime_HH_MM_SS();
// String getLocalDateTime();
// String getLocalDate_YYYY_MM_DD();


int StringSplit(String sInput, char cDelim, String param[], int iMaxParams); // https://arduino.stackexchange.com/questions/1013/how-do-i-split-an-incoming-string
void printDated(String text);
void printlnDated(String text);
unsigned int HEX2INT(String value);
String INT2HEX(unsigned int value);
String time_t2HHMM(time_t thisTime);
time_t HHMMSS2Time_t(String HHMMSS);
time_t HHMM2Time_t(String HHMMSS);
#endif 