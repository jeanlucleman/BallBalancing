#include <Arduino.h>
#include "stepper.h"
  

Stepper::Stepper(int dirPin, int stepPin,int ms1Pin,int ms2Pin,int ms3Pin, String _name) 
  { 
    _ini=false; // this private boolean will be set to true when ini() function is done
    _dirPin=dirPin;
    _stepPin=stepPin;
    _ms1Pin=ms1Pin;
    _ms2Pin=ms2Pin;
    _ms3Pin=ms3Pin;
    name=_name;
    pinMode(stepPin,OUTPUT); 
    pinMode(dirPin,OUTPUT);
    pinMode(ms1Pin,OUTPUT); digitalWrite(ms1Pin,HIGH); 
    pinMode(ms2Pin,OUTPUT); digitalWrite(ms2Pin,HIGH);
    pinMode(ms3Pin,OUTPUT); digitalWrite(ms3Pin,HIGH);
  }



void Stepper::ini(){

    _ini=true;
  }

void Stepper::setSens(byte sens)
  {
    digitalWrite(_dirPin,sens); 
    _sens=sens;
  }
void Stepper::calibration()
  {




  }
void Stepper::movTo(int stepTarget,int speed)
  {
    int stepsToDo=stepTarget-_stepCounter;
    stepsToDo>0?setSens(LOW):setSens(HIGH);
    for (size_t i = 0; i < abs(stepsToDo); i++)
      {
        movOneStep(speed);
        // delayMicroseconds(pause);
      }     
    // Serial.printf("Sortie de movBySteps dans l'objet stepper %s\n",name);
}
void Stepper::movBySteps(int steps, int speed)
  {
        steps>0?setSens(LOW):setSens(HIGH);
        for (size_t i = 0; i < abs(steps); i++)
          {
            movOneStep(speed);
            // delayMicroseconds(pause);
          }     
        // Serial.printf("Sortie de movBySteps dans l'objet stepper %s\n",name);   
      
    // t1=micros();
    // Serial.printf("t1=%d\n",t1);
    // delayMicroseconds(50);
  }
void Stepper::resetCounter()
  {
    _stepCounter=0;
  }
int Stepper::sens()
  {
    return _sens;
  }
int Stepper::stepCounter()
  {
    return _stepCounter;
  }
void Stepper::movOneStep(int speed)
  {
    if(abs(_stepCounter + (_sens==LOW?1:-1))>=stepMax)
      {
        _sens==LOW?Serial.print("+"):Serial.print("-");
        return;
      }
    if(speed<25){speed=25;}
    digitalWrite(_stepPin,HIGH); 
    delayMicroseconds(speed);
    digitalWrite(_stepPin,LOW); 
    delayMicroseconds(speed);
    _stepCounter+=(_sens==LOW?1:-1);
  }

// void Stepper::movTo1(int stepToDo)
//   {
//     // Serial.printf("%s: Prev. t1=%d  micros()=%d ",name,t1,micros());
//     uint32_t dt = micros() -t1;

//     // Serial.printf("dt=%d   stepToDo=%d ",dt,stepToDo);
//     if(abs(stepToDo)>0)
//       {
//         int pause=dt/abs(stepToDo)/3;
//         // Serial.printf("Pause: %d\n",pause); 
//         if(pause>300){pause=300;}   
//         if(pause<25){pause=25;}
//         stepToDo>0?setSens(LOW):setSens(HIGH);
//         // Serial.printf("%s : aller au step n° %d\n", name,stepToDo);
//         for (size_t i = 0; i < abs(stepToDo); i++)
//           {
//             movOneStep(speed);
//             delayMicroseconds(pause);
//           }     
//         // Serial.printf("Sortie de movTo dans l'objet stepper %s\n",name);   
//       }
//     t1=micros();
//     // Serial.printf("t1=%d\n",t1);
//     // delayMicroseconds(50);
//   }
