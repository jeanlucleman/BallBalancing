#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include "stepper.h"
#include <math.h>
#define sign(x) ((x > 0) - (x < 0))

// --- Configuration Mécanique et PID ---
// Ajustez ce facteur : combien de pas pour corriger 1 degré d'inclinaison ?
constexpr float DEGREES_TO_STEPS = 25.0; 
constexpr float ALPHA = 0.96; // Poids du gyroscope (0.95 à 0.98 classique)
// Consigne : On veut que la plaque reste parfaitement horizontale (0 degré)
constexpr float targetX = 0.0;
constexpr float targetY = 0.0;
float gyroOffsetX = 0.0;
float gyroOffsetY = 0.0;
float expectedAccel2 = 0.0;
float angleX = 0.0, angleY = 0.0;
// Paramètres PID (Axe X et Y)
// Conseil : commencez avec Ki=0 et Kd=0, augmentez Kp, puis ajoutez du Kd pour amortir.
float Kp_X =0.8, Ki_X = 0.05, Kd_X = 0.05;
float Kp_Y = 0.8, Ki_Y = 0.05, Kd_Y = 0.05;
// float Kp_X = 1.5, Ki_X = 0.02, Kd_X = 0.1;
// float Kp_Y = 1.5, Ki_Y = 0.02, Kd_Y = 0.1;
// Variables internes du PID
float errorIntegralX = 0, lastErrorX = 0;
float errorIntegralY = 0, lastErrorY = 0;
constexpr float dt = 0.040; // 40 ms (25 Hz)
unsigned long lastMicros;
// ============================================================
// MPU6050
// ============================================================

Adafruit_MPU6050 mpu;


// I²C pins - change if necessary
// Use your actual ESP32 SDA/SCL pins here.
constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;


// ============================================================
// Motors
// ============================================================

constexpr int xDirPin = 23;
constexpr int xStepPin = 18;

constexpr int yDirPin = 27;
constexpr int yStepPin = 19;

constexpr int ms1Pin = 2;
constexpr int ms2Pin = 4;
constexpr int ms3Pin = 15;


Stepper stepperX(
    xDirPin,
    xStepPin,
    ms1Pin,
    ms2Pin,
    ms3Pin,
    "Stepper X"
);

Stepper stepperY(
    yDirPin,
    yStepPin,
    ms1Pin,
    ms2Pin,
    ms3Pin,
    "Stepper Y"
);
// Structure pour envoyer les ordres aux tâches
struct MotorCommand {
    int steps;
    int speed;
    bool newCommand;
};

volatile MotorCommand cmdX = {0, 0, false};
volatile MotorCommand cmdY = {0, 0, false};
// ============================================================
// FreeRTOS tasks
// ============================================================

TaskHandle_t taskReadMPU6050 = nullptr;
TaskHandle_t taskStepperX = nullptr;
TaskHandle_t taskStepperY = nullptr;


// ============================================================
// MPU measurement
// ============================================================

struct MPUData
{
    float x;
    float y;
    float z;

    float gx;
    float gy;
    float gz;

    uint32_t timestamp;
};


// The MPU task writes this structure.
// Other tasks read it.
//
// Since floats are 32-bit on ESP32, individual values can be
// read atomically, but a structure containing several values
// can be read while it is being updated.
//
// Therefore we use a critical section when copying it.

MPUData mpuData = {};

portMUX_TYPE mpuDataMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool newMPUData = false;
volatile int counterMPU=0;

void moveBySimultaneous(int stepsX, int stepsY, int speed);
// ============================================================
// Motor commands
// ============================================================

volatile int xStepToDo = 0;
volatile int yStepToDo = 0;

// volatile int pauseX = 100;
// volatile int pauseY = 100;


// ============================================================
// Synchronisation
// ============================================================

// SemaphoreHandle_t xCommandMutex = nullptr;
// SemaphoreHandle_t yCommandMutex = nullptr;

// SemaphoreHandle_t xDone = nullptr;
// SemaphoreHandle_t yDone = nullptr;


// ============================================================
// Calibration
// ============================================================

float xAccPerStep = 0.0f;
float yAccPerStep = 0.0f;


// ============================================================
// MPU access
// ============================================================

MPUData getMPUData()
{
    MPUData result;

    portENTER_CRITICAL(&mpuDataMux);

    result = mpuData;

    portEXIT_CRITICAL(&mpuDataMux);

    return result;
}


float accX()
{
    return getMPUData().x;
}


float accY()
{
    return getMPUData().y;
}

// Fonction de secours pour débloquer l'ESP32 et l'MPU6050
// void reinitialiserI2C() {
//     Serial.println("-> Tentative de réinitialisation du bus I2C...");
//     Wire.end();
//     delay(10);
//     Wire.begin(); // Remettez vos broches si nécessaire : Wire.begin(SDA_PIN, SCL_PIN);
//     Wire.setTimeOut(50); 
    
//     // Réveiller l'MPU6050 (Registre PWR_MGMT_1 à 0) au cas où il aurait redémarré
//     Wire.beginTransmission(0x68);
//     Wire.write(0x6B); 
//     Wire.write(0);     
//     Wire.endTransmission();
// }
// ============================================================
// MPU task
// ============================================================

void readMPU6050(void *parameters)
{
    Serial.println("Starting MPU6050 task");
    TickType_t lastWakeTime = xTaskGetTickCount();
    // 25 Hz
    constexpr TickType_t period = pdMS_TO_TICKS(40);
    for (;;)
    {
        vTaskDelayUntil(&lastWakeTime, period);
        sensors_event_t accel;
        sensors_event_t gyro;
        sensors_event_t temperature;
        newMPUData = false;
        if (mpu.getEvent(&accel, &gyro, &temperature))
            {
                constexpr float tolerance = 0.15f;   // ±15 %

                float accel2 =
                    accel.acceleration.x * accel.acceleration.x +
                    accel.acceleration.y * accel.acceleration.y +
                    accel.acceleration.z * accel.acceleration.z;

                bool valid =
                    accel2 > expectedAccel2 * (1.0f - tolerance) &&
                    accel2 < expectedAccel2 * (1.0f + tolerance);
                if(valid)
                    {
                        portENTER_CRITICAL(&mpuDataMux);

                        mpuData.x = accel.acceleration.x;
                        mpuData.y = accel.acceleration.y;
                        mpuData.z = accel.acceleration.z;

                        mpuData.gx = gyro.gyro.x;
                        mpuData.gy = gyro.gyro.y;
                        mpuData.gz = gyro.gyro.z;

                        mpuData.timestamp = micros();
                        // On signale qu'une nouvelle donnée est disponible
                        newMPUData = true; 
                        counterMPU++;
                        portEXIT_CRITICAL(&mpuDataMux);
                    }
            }
        else
            {
                Serial.println("MPU6050 read failed");
            }
    }
}

void calibrateGyro() {
    Serial.println("Calibration du gyroscope... NE PAS BOUGER LA PLAQUE");
    float sumX = 0;
    float sumY = 0;
    int samples = 200;

    for (int i = 0; i < samples; i++) {
        sensors_event_t accel, gyro, temp;
        if (mpu.getEvent(&accel, &gyro, &temp)) {
            // Conversion immédiate en degrés/seconde (1 rad = 57.29578 degrés)
            sumX += gyro.gyro.x * 57.29578;
            sumY += gyro.gyro.y * 57.29578;
        }
        delay(10); // Petit délai entre les mesures
    }
    
    gyroOffsetX = sumX / samples;
    gyroOffsetY = sumY / samples;
    
    Serial.print("Offset X calculé : "); Serial.println(gyroOffsetX);
    Serial.print("Offset Y calculé : "); Serial.println(gyroOffsetY);
}
void calibrateX2Y2Z2()
{
    // expectedAccel2 = 9.81f * 9.81f;
    Serial.println("Mesure de l'accélération totale...");
    int samples = 200;
    float sumAcc=0;
    for (int i = 0; i < samples; i++) 
        {
            sensors_event_t accel, gyro, temp;
            if (mpu.getEvent(&accel, &gyro, &temp)) 
                {
                    // Conversion immédiate en degrés/seconde (1 rad = 57.29578 degrés)
                    sumAcc += accel.acceleration.x * accel.acceleration.x +
                            accel.acceleration.y * accel.acceleration.y +
                            accel.acceleration.z * accel.acceleration.z;
                }
            delay(10); // Petit délai entre les mesures
        }
    expectedAccel2=sumAcc/samples;
    Serial.printf("Accélération totale (sqrt(x2+y2+z2))= %3.3f m/s2\n",sqrt(expectedAccel2));
}
void moveToSimultaneous(int stepsX, int stepsY, int speed=25)
{
if(abs(stepsX)>stepperX.stepMax)
    {
        Serial.printf("Le déplacement absolu demandé (%d) est trop grand. Il est tronqué à %d\n",stepsX,stepperX.stepMax*stepsX/abs(stepsX));
        stepsX= stepperX.stepMax*stepsX/abs(stepsX);
    }

    moveBySimultaneous(stepsX-stepperX.stepCounter(), stepsY-stepperY.stepCounter(), speed);
    
}
// int maxStep(int steps, Stepper stepper)
//     {
//         if (sign(steps)*steps+)
//     }
void moveBySimultaneous(int stepsX, int stepsY, int speed=25) {
    if(abs(stepsX+stepperX.stepCounter())>stepperX.stepMax)
        {
            Serial.printf("Le déplacement relatif demandé (%d) est trop grand par rapport à la postion actuelle (%d)",stepsX,stepperX.stepCounter());
            stepsX=(stepperX.stepMax-abs(stepperX.stepCounter()))*sign(stepsX);
            Serial.printf("Il est tronqué à %d\n",stepsX);
        }
    // l'intercalation des pas (Bresenham).
    // Définir les directions
    stepsX > 0 ? stepperX.setSens(LOW) : stepperX.setSens(HIGH);
    stepsY > 0 ? stepperY.setSens(LOW) : stepperY.setSens(HIGH);

    int absX = abs(stepsX);
    int absY = abs(stepsY);
    int maxSteps = max(absX, absY);

    int counterX = 0;
    int counterY = 0;

    Serial.printf("Moving X: %d steps, Y: %d steps, speed: %d: \n", stepsX, stepsY, speed);
    for (int i = 0; i < maxSteps; i++) {
        // bool stepX_done = false;
        // bool stepY_done = false;

        counterX += absX;
        counterY += absY;

        // Est-ce le moment de faire un pas sur X ?
        if (counterX >= maxSteps) {
            stepperX.movOneStep(speed);
            counterX -= maxSteps;
        }

        // Est-ce le moment de faire un pas sur Y ?
        if (counterY >= maxSteps) {
            stepperY.movOneStep(speed);
            counterY -= maxSteps;
        }

   }
}


// ============================================================
// Move X motor until plate is horizontal
// ============================================================

void setXPosition(int speed)
{
    float gammaX = accX();

    stepperX.setSens(gammaX > 0 ? LOW : HIGH);

    do
    {
        stepperX.movOneStep(speed);

        // delayMicroseconds(speed);

        gammaX = accX();

    }
    while (
        stepperX.sens() == LOW
            ? gammaX > 0.01
            : gammaX < -0.010
    );

    Serial.println("Set X done!");
}


// ============================================================
// Move Y motor until plate is horizontal
// ============================================================

void setYPosition(int speed)
{
    float gammaY = accY();

    stepperY.setSens(gammaY > 0 ? HIGH : LOW);

    Serial.println("Set Y position");

    do
    {
        stepperY.movOneStep(speed);

        // delayMicroseconds(speed);

        gammaY = accY();

    }
    while (
        stepperY.sens() == HIGH
            ? gammaY > 0.01
            : gammaY < -0.010
    );

    Serial.println("Set Y done!");
}


// ============================================================
// Horizontal position
// ============================================================

void setPlateHorizontal()
{
    for (int i = 0; i < 3; i++)
    {
        setXPosition(500 + 1000 * i);
        stepperX.resetCounter();        
        delay(500);

    }

    for (int i = 0; i < 3; i++)
    {
        setYPosition(500 + 1000 * i);
        stepperY.resetCounter();
        delay(500);

    }

    // stepperX.stepCounter = 0;
    // stepperY.stepCounter = 0;
}



// Tâche pour le Moteur X
void cmdStepperX(void *pvParameters) {
    while(1) {
        if (cmdX.newCommand) {
            int stepsLeft = abs(cmdX.steps);
            stepperX.setSens(cmdX.steps > 0 ? LOW : HIGH);
            cmdX.newCommand = false; // Commande prise en compte

            while (stepsLeft > 0) {
                stepperX.movOneStep(cmdX.speed);
                stepsLeft--;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Laisse respirer le CPU si pas de mouvement
    }
}

// Tâche pour le Moteur Y
void cmdStepperY(void *pvParameters) {
    while(1) {
        if (cmdY.newCommand) {
            int stepsLeft = abs(cmdY.steps);
            stepperY.setSens(cmdY.steps > 0 ? LOW : HIGH);
            cmdY.newCommand = false;

            while (stepsLeft > 0) {
                stepperY.movOneStep(cmdY.speed);
                stepsLeft--;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================
// Motor calibration
// ============================================================

void motorCalibration()
{
    const int stepMax = 1500;
    float sum = 0;
    int step = 0;
    // --------------------------------------------------------
    // X
    // --------------------------------------------------------

    for (int i = 0; i < stepMax; i++)
    {
        stepperX.movOneStep(150);
        step++;
        if (step == stepMax / 10)
        {
            delay(1000);
            float x = accX();
            float value =
                (float)stepMax / (float)i * x;
            sum += value;
            Serial.printf(
                "X: x=%f i=%d value=%f sum=%f\n",
                x,
                i,
                value,
                sum
            );
            step = 0;
        }
    }

    xAccPerStep =
        sum / (float)(stepMax / 10);

    Serial.printf(
        "Acceleration X = %.6f m/s2/step\n",
        xAccPerStep
    );


    // Return horizontal

    moveToSimultaneous(0, 0, 25);

    delay(500);


    // --------------------------------------------------------
    // Y
    // --------------------------------------------------------

    sum = 0;
    step = 0;

    for (int i = 0; i < stepMax; i++)
    {
        stepperY.movOneStep(150);

        step++;

        if (step == stepMax / 10)
        {
            delay(1000);

            float y = accY();

            float value =
                (float)stepMax / (float)i * y;

            sum += value;

            Serial.printf(
                "Y: y=%f i=%d value=%f sum=%f\n",
                y,
                i,
                value,
                sum
            );

            step = 0;
        }
    }

    yAccPerStep =
        sum / (float)(stepMax / 10);

    Serial.printf(
        "Acceleration Y = %.6f m/s2/step\n",
        yAccPerStep
    );

}


// ============================================================
// Test rotation
// ============================================================


void turn()
  {
    int x1,y1;

    for (float psi = 0; psi < 6.28 *5.0; psi+=0.1)
      {
        x1=1000 * cos(psi);
        y1=1000 * sin(psi);
        moveToSimultaneous(x1,y1);
       }
  }

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(500);

    Serial.println();
    Serial.println("================================");
    Serial.println("Ball Balancer");
    Serial.println("================================");


    // --------------------------------------------------------
    // I²C
    // --------------------------------------------------------

    Wire.begin(I2C_SDA, I2C_SCL);

    // Start conservatively at 100 kHz.
    Wire.setClock(100000);
    Wire.setTimeOut(50);

    // --------------------------------------------------------
    // MPU6050
    // --------------------------------------------------------

    if (!mpu.begin(
            MPU6050_I2CADDR_DEFAULT,
            &Wire))
    {
        Serial.println(
            "Failed to find MPU6050"
        );

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("MPU6050 found");


    mpu.setAccelerometerRange(
        MPU6050_RANGE_2_G
    );

    mpu.setGyroRange(
        MPU6050_RANGE_250_DEG
    );

    mpu.setFilterBandwidth(
        MPU6050_BAND_10_HZ
    );


    // IMPORTANT:
    //
    // Do NOT use:
    //
    // mpu.setCycleRate(...);
    // mpu.enableCycle(true);
    //
    // The MPU should continuously operate
    // for this application.


    // --------------------------------------------------------
    // Synchronisation
    // --------------------------------------------------------

    // xCommandMutex =
    //     xSemaphoreCreateMutex();

    // yCommandMutex =
    //     xSemaphoreCreateMutex();

    // xDone =
    //     xSemaphoreCreateBinary();

    // yDone =
    //     xSemaphoreCreateBinary();


    // --------------------------------------------------------
    // Tasks
    // --------------------------------------------------------
Serial.println("Creating task MPU6050");    
    xTaskCreatePinnedToCore(
        readMPU6050,
        "MPU6050",
        4000,
        nullptr,
        20,
        &taskReadMPU6050,
        1
    );


    // Création des tâches sur les deux cœurs différents de l'ESP32
    xTaskCreatePinnedToCore(cmdStepperX, "TaskX", 2048, NULL, 1, &taskStepperX, 0); // Cœur 0
    xTaskCreatePinnedToCore(cmdStepperY, "TaskY", 2048, NULL, 1, &taskStepperY, 1); // Cœur 1


    calibrateGyro();
    calibrateX2Y2Z2();
    // --------------------------------------------------------
    // Give MPU task some time to obtain first measurement
    // --------------------------------------------------------

    delay(200);
    Serial.print("Starting set H plate");

    // --------------------------------------------------------
    // Initial horizontal positioning
    // --------------------------------------------------------

    setPlateHorizontal();

    Serial.println("Plate horizontal");
    Serial.printf("xCounter=%d yCounter=%d\n", stepperX.stepCounter(), stepperY.stepCounter());
    // --------------------------------------------------------
    // Test movements
    // --------------------------------------------------------
    // turn();
    moveToSimultaneous(0, 0, 25);
    Serial.printf("xCounter=%d yCounter=%d\n", stepperX.stepCounter(), stepperY.stepCounter());
    
    // --------------------------------------------------------
    // Calibration
    // --------------------------------------------------------
    // motorCalibration();
    moveToSimultaneous(0, 0, 25);
    Serial.printf("xCounter=%d yCounter=%d\n", stepperX.stepCounter(), stepperY.stepCounter());
 
    delay(1000);
    Serial.println("Moving to 1500,0");
    moveToSimultaneous(1500, -500, 25);
    delay(1000);
    Serial.println("Moving to 0,0");
    moveToSimultaneous(0, 0, 25);
    delay(1000);
    lastMicros=micros();
    Serial.println(
        "Setup completed"
    );
}

unsigned long microsNewMPUData;
// ============================================================
// LOOP
// ============================================================
int avgStepsX=0;
int avgStepsY=0;
// int counterLoop=0;
unsigned long lastMicrosNewMPUData;
float avgAngleX=0.0;
float avgAngleY=0.0;
void loop()
{
    bool localNewData = false;
    // counterLoop++;
    // unsigned long microsStartLoop=micros();
    // if(counterLoop>5000)
    //     {
    //         moveToSimultaneous(0,0,25);

    //         moveToSimultaneous(1000, 0, 25); 
    //         delay(1000);
    //         counterLoop=0;
    //     }
    MPUData localMpuData; // Copie locale pour le traitement
    // static uint32_t lastMov= 0;
    int localCounterMPU;
    // --- DEBUT ZONE PROTEGEE ---
    // 1. Récupération sécurisée de la dernière mesure (Section Critique)
    portENTER_CRITICAL(&mpuDataMux);
    if (newMPUData) 
    {
        localNewData = true;
        localMpuData = mpuData; // Copie rapide des données
        newMPUData = false;     // On réinitialise le drapeau
        localCounterMPU=counterMPU;
    }
    portEXIT_CRITICAL(&mpuDataMux);
    // --- FIN ZONE PROTEGEE ---

   
    if (localNewData) 
        {
        microsNewMPUData=micros();
        // Serial.printf("L%011d  %6d DM%4d %6d\n",microsStartLoop,counterLoop, (microsNewMPUData-lastMicrosNewMPUData)/1000,localCounterMPU);
        lastMicrosNewMPUData=microsNewMPUData;
        float angleX = -atan(localMpuData.x/localMpuData.z)*57.3;
        angleX=int(angleX*10)/10.0;
        float angleY = atan(localMpuData.y/localMpuData.z)*57.3;
        // Serial.printf("angleX= %3.3f  \n",angleX);
        // --- ÉTAPE B : Filtre Complémentaire (Fusion Accel + Gyro) ---
        // Le gyro mesure en rad/s, on convertit en deg/s (57.29578)
        float gyroRateX = localMpuData.gx * 57.29578 - gyroOffsetX;;
        float gyroRateY = localMpuData.gy * 57.29578 - gyroOffsetY;


// Serial.printf("gyroX= %3.3f, gyroY= %3.3f\n",gyroRateX,gyroRateY);
        float deltaT=float(micros()-lastMicros)/1000000.0;
        //  Serial.printf("DeltaT %3.3f\n",deltaT);
        //  Serial.printf("DtLoop= %3.3f\n",microsStartLoop-lastMicros);
        lastMicros=micros();
        // Intégration du gyro + recalage par l'accéléromètre pour éviter la dérive
        angleX = ALPHA * (angleX + gyroRateX * deltaT) + (1.0 - ALPHA) * angleX;
        angleY = ALPHA * (angleY + gyroRateY * deltaT) + (1.0 - ALPHA) * angleY;
        // angleX =  angleX;
        int n=3;
        // Serial.printf("Angle X = %3.3f\n",angleX);
        // --- ÉTAPE C : Calcul des erreurs ---
        avgAngleX=((n-1)*avgAngleX + angleX)/n;
        avgAngleY=((n-1)*avgAngleY + angleY)/n;
        float errorX = targetX - avgAngleX;
        float errorY = targetY - avgAngleY;



            // --- Calcul PID Axe X ---
            errorIntegralX += errorX * deltaT;
            errorIntegralX = constrain(errorIntegralX, -20.0, 20.0); // Anti-windup (limite l'intégration)
            float errorDerivativeX = (errorX - lastErrorX) / deltaT;
            float outputX = (Kp_X * errorX) + (Ki_X * errorIntegralX) + (Kd_X * errorDerivativeX);
            lastErrorX = errorX;

            // --- Calcul PID Axe Y ---
            errorIntegralY += errorY * deltaT;
            errorIntegralY = constrain(errorIntegralY, -20.0, 20.0); 
            float errorDerivativeY = (errorY - lastErrorY) / deltaT;
            float outputY = (Kp_Y * errorY) + (Ki_Y * errorIntegralY) + (Kd_Y * errorDerivativeY);
            lastErrorY = errorY;

    // 3. Conversion de la correction PID en nombre de pas
            // int k=10*abs(outputX)+10;
            // if (k>50){k=50;}
            // int stepsX = round(outputX * k);
            float k;
            k=(abs(outputX)-1)/(outputX*outputX+1)+1;
            k=1;
            int stepsX = round(outputX * DEGREES_TO_STEPS*k);
            // Serial.printf("stepsX= %d pour outputX= %3.3f\n",stepsX,outputX);
            int stepsY = round(outputY * DEGREES_TO_STEPS*k);

            avgStepsX=(avgStepsX*(n-1) + stepsX)/n;
            avgStepsY=(avgStepsY*(n-1) + stepsY)/n;           
            // if (abs(avgStepsX) < 3) avgStepsX = 0;
            // if (abs(stepsY) < 2) stepsY = 0;    
    // 4. Commande simultanée des moteurs
            if (avgStepsX != 0 || avgStepsY != 0) {
            // Note : si la plaque accentue l'erreur au lieu de la corriger, 
            // inversez simplement le signe ici (ex: -stepsX ou -stepsY)
                int speed;
                abs(avgStepsX)> abs(avgStepsY)?speed=abs(20000/avgStepsX):speed=abs(20000/avgStepsY);
                moveBySimultaneous(-avgStepsX, -avgStepsY, speed); 
            Serial.printf("P: %+4.3f  I: %+3.3f  D:%+3.3f  angleX=%+4.3f errorX=%+6.2f  k=%3.3f outputX= %+6.2f avgStepsX=%d speed=%d\n",Kp_X * errorX, Ki_X * errorIntegralX, Kd_X * errorDerivativeX,angleX, errorX,k,outputX,avgStepsX,speed);
                // Définir les directions
                // avgStepsX < 0 ? stepperX.setSens(LOW) : stepperX.setSens(HIGH);
                // int absX = abs(avgStepsX);
                // for (int i = 0; i < absX; i++) 
                //     {
                //         stepperX.movOneStep(speed);
                //     }
            }

   }
  

  // Permet de laisser du temps aux autres tâches FreeRTOS de l'ESP32 sur ce cœur
    vTaskDelay(pdMS_TO_TICKS(1)); 
}

    //     // Utilisez ici localMpuData sans risque de conflit
    //     Serial.print("Nouvelle valeur acceleration X : ");
    //     Serial.println(localMpuData.x);
    //     if ((abs(localMpuData.x))>0.1)
    //         {
    //             int xTarget=int(localMpuData.x*abs(localMpuData.x) * 150);
    //             if(xTarget>1500) xTarget=1500;
    //             if(xTarget<-1500) xTarget=-1500;
    //             Serial.printf(
    //                 "Count= %d AX=%+.3f  AY=%+.3f  "
    //                 "GX=%+.3f  GY=%+.3f\n",
    //                 localCounterMPU,
    //                 localMpuData.x,
    //                 localMpuData.y,
    //                 localMpuData.gx,
    //                 localMpuData.gy);
    //             Serial.printf("Moving to X: %d\n", xTarget);  
    //             int period = 20000; // 100 ms
    //             if (xTarget != 0) {

    //                 int speed = 2*period/abs(xTarget)/2;
    //                 moveBySimultaneous(xTarget, 0, speed);  
    //                 delay(1);
    //     }
    //     Serial.println();
    //         }
  

    // }




    // static uint32_t lastPrint = 0;

    // if (millis() - lastPrint >= 60)
    // {
    //     lastPrint = millis();

    //     MPUData data = getMPUData();
    //     int xTarget=int(data.x*500.0/2.43);
    //     Serial.printf("Moving to X: %d\n", xTarget);  
    //     Serial.printf(
    //         "AX=%+.3f  AY=%+.3f  "
    //         "GX=%+.3f  GY=%+.3f\n",
    //         data.x,
    //         data.y,
    //         data.gx,
    //         data.gy);


    //     moveBySimultaneous(xTarget, 0, 250);  
      
    // }



// }