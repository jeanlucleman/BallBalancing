#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>


// vector<String> split(String my_str, String separator); 

// This class allows to create, edit, use settings in a structure (key, value, description). 
// These settings are save in a file through SPIFFS. The settings can be created and modified by the funtions
// setValue(). The value and description can be accessed by the property getValue() and getDecsiption().
// When a settings is created or modified, it is automarically saved in the settings.txt file.
class Stepper {       
  private:

    // Flag to check if the Settings object has been properly initialized. The full initialization cannot be done in the class constructor.
    // The reason is that the instantiation is made before the main setup is started and this creates a program crash...
    bool _ini;


    int _dirPin;
    int _stepPin;
    int _ms1Pin;
    int _ms2Pin;
    int _ms3Pin;

  public:          
    // Function to complete initialisation of the Settings object
    void ini();
    // Constructor of Settings class
    Stepper(int dirPin, int stepPin,int ms1Pin,int ms2Pin,int ms3Pin, String name); 
    int sens;
    String name;
    void setSens(byte _sens);
    int stepPerRev;
    int stepCounter; // The number of steps already done from the horizontal position
    void calibration();
    void movOneStep();
    void movTo1(int stepTarget);
     uint32_t t1;
    // unsigned long t2;

  };





#endif 