// https://github.com/bluino/esp32_wifi_balancing_robot/tree/master#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include "stepper.h" // stepper class
#include <Wire.h>
Adafruit_MPU6050 mpu;
TaskHandle_t taskReadMPU6050; // Task to process data change
TaskHandle_t taskMovToX; // Task handle for the movToX task
TaskHandle_t taskMovToY; // Task handle for the movToY task
// Connections to A4988
const int xDirPin = 23;  // Direction
const int xStepPin = 18; // Step
const int yDirPin = 27;  // Direction
const int yStepPin = 19; // Step
const int ms1Pin = 2;    // ms pin are used to define the step. Hith 3 pin HIGH step = 1/16 of full step
const int ms2Pin = 4;
const int ms3Pin = 15;
#define SDA_PIN 21
#define SCL_PIN 22

Stepper stepperX(xDirPin, xStepPin,ms1Pin,ms2Pin,ms3Pin,"Stepper X"); // Creates a X Stepper object
Stepper stepperY(yDirPin, yStepPin,ms1Pin,ms2Pin,ms3Pin,"Stepper Y"); // Creates a Y stepper object

void movTo(int x, int y, int dt = 50); // Request to move at absolute x and y position within dt time in µs
SemaphoreHandle_t xMutex = NULL;  // Mutex object to safely perform the movToX bloc
SemaphoreHandle_t yMutex = NULL;  // Mutex object to safely perform the movToX bloc
SemaphoreHandle_t xDone = NULL; // Binary semaphore to indicate the the requestion x motion is done
SemaphoreHandle_t yDone = NULL; // Binary semaphore to indicate the the requestion y motion is done
sensors_event_t accel, g, temp; // Data from accelerometer
float xAccPerStep; // The gravity acceleration along X axis of the plate per one X step measured in the motor calibration function
float yAccPerStep; // The gravity acceleration along Y axis of the plate per one Y step measured in the motor calibration function
unsigned long millisLastMPUReadX;
unsigned long millisLastMPUReadY;
int xStepToDo; // The necessary steps to reach the requested absolute position
int yStepToDo; // The necessary steps to reach the requested absolute position

int pauseX; // a cycle motor step is made with step pin HIGH for 5 µs and LOW for pauseX or pauseY µs. pauseX and pauseY are computed to perform the X and Y displacement in the same time
int pauseY; // a cycle motor step is made with step pin HIGH for 5 µs and LOW for pauseX or pauseY µs. pauseX and pauseY are computed to perform the X and Y displacement in the same time 
// MPU sensor data are read in the readMPU6050 task.
float accX() 
  {
    return accel.acceleration.x;
  }
// MPU sensor data are read in the readMPU6050 task.  
float accY() 
  {
    return accel.acceleration.y;
  }
void setXPosition(int speed) // Move X motor to get X Horizontal
  {

    float gammaX=accX();
    stepperX.setSens(gammaX>0?LOW:HIGH);
    do
      {

        stepperX.movOneStep();
        delayMicroseconds(speed);
        gammaX=accX();
      } while (stepperX.sens==LOW?(gammaX)>0.01:(gammaX)<-0.010);
    Serial.println("Set X done!");
  }
void setYPosition(int speed) // Moves motor Y to get
  {

    float gammaY=accY();
    stepperY.setSens(gammaY>0?HIGH:LOW);
    Serial.println("Set Y position");
    do
      {

        stepperY.movOneStep();
        delayMicroseconds(speed);           
        gammaY=accY();
      } while (stepperY.sens==HIGH?(gammaY)>0.01:(gammaY)<-0.010);
    Serial.println("Set Y done!");
 }
void setPlateHorizontal() // Get the plate horizontal in 3 steps for a better accuracy
  {
    for (size_t i = 0; i < 3; i++)
      {
        setXPosition(500 + 1000*i);
        delay(500);
      }  
    for (size_t i = 0; i < 3; i++)
      {
        setYPosition(500 + 1000*i);
        delay(500);
      }
    stepperX.stepCounter=0; // When plate is horizontal the X and Y step counter are resetted
    stepperY.stepCounter=0;
  }
void motorCalibration() // Determines 2 coefficients giving the X and Y acceleration per step. // This allows to check validity of ball position by computing its acceleration and comparing with the one deduced from the step counter 
  {
    int step=0;
    int stepMax=1500;
    float sum=0;
    for (size_t i = 0; i < stepMax; i++)
      {
        stepperX.movOneStep();
        delayMicroseconds(200);
        step++;
        if (step==stepMax/10)
          {
            delay(1500);float x=accX();
            Serial.print("x= ");
            Serial.print(x,3);
            Serial.print(" i= ");
            Serial.print(i);
            Serial.print("  ");
            sum=sum+(float)stepMax/(float)i*x;
            step=0;
            Serial.print(stepMax*x/(float)i);
            Serial.print(" ");
            Serial.println(sum,3);
          }
      }
    Serial.print("Somme: ");
    Serial.println(sum);
    Serial.print("Accélération X:");
    xAccPerStep=sum/(float)stepMax/10.0;
    Serial.print(xAccPerStep,6);
    Serial.println(" m/s2 par step");
    movTo(0,0); // identical to setPlateHorizontal();
    delay(500);
    step=0;
    sum=0;
    for (size_t i = 0; i < stepMax; i++)
      {
        stepperY.movOneStep();
        delayMicroseconds(200);
        step++;
        if (step==stepMax/10)
          {
            delay(1500);float y=accY();
            Serial.print("y= ");
            Serial.print(y,3);
            Serial.print(" i= ");
            Serial.print(i);
            Serial.print("  ");
            sum=sum+(float)stepMax/(float)i*y;
            step=0;
            Serial.print(stepMax*y/(float)i);
            Serial.print(" ");
            Serial.println(sum,3);
          }
      }
    Serial.print("Somme: ");
    Serial.println(sum);
    Serial.print("Accélération Y:");
    yAccPerStep=sum/(float)stepMax/10.0;
    Serial.print(yAccPerStep,6);
    Serial.println(" m/s2 par step");
  }
void mov(int deltaStepX,int deltaStepY) // Move the plate by these incrementials steps
  {
    bool done=false;
    stepperX.setSens(LOW);
    stepperY.setSens(LOW);
    if(deltaStepX<0)
      {
        deltaStepX=-deltaStepX;
        stepperX.setSens(HIGH);
      }
    if(deltaStepY<0)
      {
        deltaStepY=-deltaStepY;
        stepperY.setSens(HIGH);
      }      
    do
      {
        done=true; 
        if(stepperX.stepCounter<deltaStepX)
          {
            stepperX.movOneStep();
            done=false;
          }
        if(stepperY.stepCounter<deltaStepY)
          {
            stepperY.movOneStep();
            done=false;
          }
        delay(10);
      } while (!done);/* condition */;
  }
// void readMPU6050(void * parameters)
//   {
//     unsigned long millisLastMPURead;
//     Serial.println("Démaragge de la task readMPU6050");
//     for(;;)
//       {
//         if(millis()>millisLastMPURead+50)
//           {
//             // mpu.setI2CBypass(true);
//             mpu.getEvent(&accel,&g,&temp);
//             millisLastMPURead=millis();
//           }
//         vTaskDelay(40/portTICK_PERIOD_MS); 
//       }        
//   }

// void readMPU6050(void * parameters)
// {
//     Serial.println("Démarrage de la task readMPU6050");
    
//     // Initialisation du tick pour la périodicité stricte
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     const TickType_t xFrequency = pdMS_TO_TICKS(40); // Période stricte de 50 ms (20 Hz)

//     for(;;)
//     {
//         // Bloque la tâche et la réveille précisément toutes les 50 ms
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);

//         // Lecture du MPU6050 (plus besoin de la structure 'if millis()')
//         // mpu.getEvent() utilise le bus I2C
//         mpu.getEvent(&accel, &g, &temp);
        
//         // Optionnel : Un léger yield si d'autres tâches de même priorité existent
//         taskYIELD(); 
//     }        
// }

void readMPU6050(void *parameters)
{
    TickType_t lastWake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));

        sensors_event_t a, g, t;

        if (mpu.getEvent(&a, &g, &t))
        {
            accel = a;
        }
    }
}

void movTo(int x, int y, int dt) // Move the plate to these absolute values (0,0 being plate horizontal)
  {
    if (xSemaphoreTake (xMutex, portMAX_DELAY))
      {
        xStepToDo=x-stepperX.stepCounter; // The necessary displacement to reach the requested absolute position
        if(xStepToDo!=0){pauseX=abs(dt/xStepToDo/1.1);}
        Serial.printf("%d %d %d ",xStepToDo,pauseX, micros());
        if(pauseX<50){pauseX=50;Serial.println("Pause minimum");}
        xSemaphoreGive (xMutex);  // release the mutex
        xSemaphoreTake(xDone,portMAX_DELAY);
      }
    if (xSemaphoreTake (yMutex, portMAX_DELAY))
      {
        yStepToDo=y-stepperY.stepCounter;
        if(yStepToDo!=0){pauseY=abs(dt/yStepToDo/1.1);}
        if(pauseY<50){pauseY=50;}
        // Serial.printf("       Step y %d   Pause y = %d\n",yStepToDo, pauseY);
        xSemaphoreGive (yMutex);  // release the mutex
        xSemaphoreTake(yDone,portMAX_DELAY);        
      }
    bool flagX=true; // true when requested x motion is done
    bool flagY=true; // true when requested y motion is done
    while (flagX || flagY) // Loop until x and y requested motion are done
      {
        if (xSemaphoreTake (xDone, (20 * portTICK_PERIOD_MS))) 
          {  
            flagX=false;
          }
        if (xSemaphoreTake (yDone, (20 * portTICK_PERIOD_MS))) 
          {  
            flagY=false;
          }
      }
  }
void movToX(void * parameters)
  {
    for(;;)
      {
        if (xSemaphoreTake (xMutex, portMAX_DELAY)) 
          {
            if(abs(xStepToDo)>0)
              {
                // Serial.printf(" %d  ",micros()); // 35 à 50 µs
                xStepToDo>0?stepperX.setSens(LOW):stepperX.setSens(HIGH);
                for (size_t i = 0; i < abs(xStepToDo); i++)
                  {
                    stepperX.movOneStep();
                    delayMicroseconds(pauseX);
                  }  
                xStepToDo=0;
                // Serial.println(micros());
              }
            xSemaphoreGive (xDone);  // Release the semaphore to indicate end of job

            xSemaphoreGive (xMutex);  // release the mutex
          }
        vTaskDelay(10/portTICK_PERIOD_MS); 
      }
  }
void movToY(void * parameters)
  {
    for(;;)
      {
        if (xSemaphoreTake (yMutex, portMAX_DELAY)) 
          {
            if(abs(yStepToDo)>0)
              {
                yStepToDo>0?stepperY.setSens(LOW):stepperY.setSens(HIGH);
                for (size_t i = 0; i < abs(yStepToDo); i++)
                  {
                    stepperY.movOneStep();
                    delayMicroseconds(pauseY);
                  }  
                yStepToDo=0;
              }
            xSemaphoreGive (yDone);  // Release the semaphore to indicate end of job

            xSemaphoreGive (yMutex);  // release the mutex
          }
        vTaskDelay(10/portTICK_PERIOD_MS); 
      }
  }
void turn()
  {
    int x1,y1;
    int dt=15000;
    for (float psi = 0; psi < 6.28 *5.0; psi+=0.1)
      {
        x1=1000 * cos(psi);
        y1=1000 * sin(psi);
         movTo(x1,y1,dt);
       }
  }
void setup() 
  {
    Serial.begin(115200);  
    delay(500);
    Serial.println("Adafruit MPU6050 test!");


    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000);

    if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
        Serial.println("Failed to find MPU6050");
        while (true) {
            delay(100);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

    // No cycle mode
    // No need to put temperature in standby



    
//     Wire.begin(SDA_PIN, SCL_PIN);
//     Wire.setClock(100000);
//     if (!mpu.begin(MPU6050_I2CADDR_DEFAULT, &Wire)) {
//         Serial.println("Failed to find MPU6050");
//         while (true) {
//             delay(100);
//         }
// }
//     Serial.println("MPU6050 Found!");
//     mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
//     mpu.setGyroRange(MPU6050_RANGE_250_DEG);
//     mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    // Serial.println("MPU Setting cycle rate");
    // mpu.setCycleRate(MPU6050_CYCLE_40_HZ);
    // Serial.println("Enabling cycle");
    // mpu.enableCycle(true);
    Serial.printf("Fréquence de mesure du MPU6050: %d\n", mpu.getCycleRate());
    mpu.setTemperatureStandby(true);
    xMutex = xSemaphoreCreateMutex();  // create a mutex object
    yMutex = xSemaphoreCreateMutex();  // crete a mutex object 
    xDone = xSemaphoreCreateBinary();  // Set the semaphore as binary
    yDone = xSemaphoreCreateBinary();  // Set the semaphore as binary
    // xTaskCreatePinnedToCore(readMPU6050,"Reading MPU data",4000,NULL,20,&taskReadMPU6050,0); // endless task to proceed data which are in ethe queue
    // xTaskCreatePinnedToCore(movToX,"Running motor X",4000,NULL,15,&taskMovToX,1); // endless task to proceed data which are in ethe queue
    // xTaskCreatePinnedToCore(movToY,"Running motor Y",4000,NULL,10,&taskMovToY,0); // endless task to proceed data which are in ethe queue

// Sur le Core 0 : On isole le moteur Y
xTaskCreatePinnedToCore(movToY, "Running motor Y", 4000, NULL, 15, &taskMovToY, 0);

// Sur le Core 1 : On place le moteur X ET la lecture MPU
xTaskCreatePinnedToCore(movToX, "Running motor X", 4000, NULL, 15, &taskMovToX, 0);
xTaskCreatePinnedToCore(readMPU6050, "Reading MPU data", 4000, NULL, 20, &taskReadMPU6050, 1);


    setPlateHorizontal();
    Serial.println("Après réglage horizontal...");
    for (size_t j = 0; j < 1; j++)
      {
        delay(500);
        movTo(2000,0); delay(500);// movTo(0,0);delay(500);
        movTo(0,2000); delay(500); //       movTo(0,0);delay(500);
        movTo(-2000,0); delay(500); //       movTo(0,0);delay(500);
        movTo(0,-2000); delay(500); //       movTo(0,0);
      }
    movTo(0,0);
    delay(500);
     motorCalibration();
    movTo(0,0);
}
bool turnDone=false;
void loop()
  {

    delay(500);
    if (!turnDone)
      {
        turn();
        turnDone=true;
      }
    movTo(0,0);
    delay(2000);
    Serial.println(accX(),4);
  delay(10);
  }