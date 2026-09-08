#include "esp_camera.h"
#include <Arduino.h>
#include <Preferences.h>
#include <cstring>
#include <cmath>

// ============================================================
// AI-Thinker ESP32-CAM
// ============================================================

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define TX_PIN 1
#define RX_PIN 3
// ============================================================
// Image
// ============================================================

constexpr int WIDTH  = 320;
constexpr int HEIGHT = 240;


// ============================================================
// Plate
// ============================================================

constexpr float PLATE_WIDTH_MM  = 150.0f;
constexpr float PLATE_HEIGHT_MM = 150.0f;





// ============================================================
// Ball detection
// ============================================================

constexpr uint8_t THRESHOLD = 100;

constexpr uint16_t MIN_AREA = 350;
constexpr uint16_t MAX_AREA = 900;

constexpr uint16_t MIN_DIAMETER = 20;
constexpr uint16_t MAX_DIAMETER = 52;

constexpr float MAX_ASPECT_RATIO = 1.25f;


// ============================================================
// Connected component buffer
// ============================================================

uint8_t visited[WIDTH * HEIGHT];


// ============================================================
// Calibration
// ============================================================

struct Point2D
{
    float x;
    float y;
};
struct ROI // Region of interest (zone de la capture intéressante)
{
    int xMin;
    int yMin;
    int xMax;
    int yMax;
};

// Image coordinates of the four calibration points.
//
// Order:
//
// 0 = bottom-left
// 1 = bottom-right
// 2 = top-right
// 3 = top-left
//
Point2D imagePoints[4];

// ============================================================
// Region of interest
// ============================================================

ROI roiPoints ;//= {65, 30, 235, 205}; // Matches the order: xMin, yMin, xMax, yMax
// Les points sont mesurés en pixels dans l'image capturée par la caméra. 
// Ils définissent la zone de l'image où la balle est attendue, ce qui permet d'optimiser la détection en ignorant les zones non pertinentes de l'image.
// La zone est détectée automatiquement lors du calibrage (avec une plaque noir et un arrière plan blanc)
// Elle est stockée dans la mémoire non volatile de l'ESP32 pour être utilisée lors des exécutions suivantes du programme.

// Physical coordinates.
//
// These never change unless the plate dimensions change.
//
const Point2D platePoints[4] =
{
    {  0.0f,   0.0f },   // bottom-left
    {150.0f,   0.0f },   // bottom-right
    {150.0f, 150.0f },   // top-right
    {  0.0f, 150.0f }    // top-left
};


// Homography:
//
// H[0] = H11
// H[1] = H12
// H[2] = H13
// H[3] = H21
// H[4] = H22
// H[5] = H23
// H[6] = H31
// H[7] = H32
//
float H[8];


// ============================================================
// Preferences
// ============================================================

Preferences preferences;

constexpr uint32_t CALIBRATION_VERSION = 2;

// uint32_t positionSequence = 0;
uint32_t lastPositionTime = 0;
void sendBallPosition(float x, float y)
{
    static uint32_t sequence = 0;
    // uint32_t t0 = millis();
    Serial.printf("BALL,%lu,%7.1f,%7.1f\n",
        millis()-lastPositionTime,
                //    ++sequence,
                   x,
                   y);
    lastPositionTime = millis();
    // Serial.printf("  -> %lu\n",millis()-t0);
}
void sendMessage(const char* message)
{
    Serial.printf("MSG,%s\n", message);
}


// ============================================================
// Camera initialization
// ============================================================

bool initCamera()
{
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk  = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href  = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn  = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 16000000; // Normalement 20 MHz, mais 16 MHz est plus stable sur certaines cartes ESP32-CAM
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAMESIZE_QVGA;
    config.fb_count = 2;
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST; // Force la caméra à toujours vous donner l'image la plus fraîche
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf(
            "Camera initialization failed: 0x%x\n",
            err
        );

        return false;
    }
    return true;
}


// ============================================================
// Ball detection
// ============================================================

bool findBall(
    camera_fb_t *fb,
    float &centerX,
    float &centerY,
    uint32_t &bestArea
)
{
    memset(visited, 0, sizeof(visited));
    bool found = false;
    bestArea = 0;
    static uint16_t queue[10000];
    for (int startY = roiPoints.yMin;
         startY <= roiPoints.yMax;
         startY++)
    {
        for (int startX = roiPoints.xMin;
             startX <= roiPoints.xMax;
             startX++)
        {
            int startIndex =
                startY * WIDTH + startX;


            if (visited[startIndex])
                continue;


            if (fb->buf[startIndex] <= THRESHOLD)
                continue;


            // ------------------------------------------------
            // Start component
            // ------------------------------------------------

            uint32_t queueStart = 0;
            uint32_t queueEnd   = 0;

            queue[queueEnd++] =
                (uint16_t)startIndex;

            visited[startIndex] = 1;


            uint32_t area = 0;

            uint32_t sumX = 0;
            uint32_t sumY = 0;

            int minX = startX;
            int maxX = startX;
            int minY = startY;
            int maxY = startY;


            // ------------------------------------------------
            // Flood fill
            // ------------------------------------------------

            while (queueStart < queueEnd)
            {
                uint16_t index =
                    queue[queueStart++];

                int x = index % WIDTH;
                int y = index / WIDTH;


                area++;
                sumX += x;
                sumY += y;
                if (x < minX) minX = x;
                if (x > maxX) maxX = x;
                if (y < minY) minY = y;
                if (y > maxY) maxY = y;
                // Left
                if (x > roiPoints.xMin)
                {
                    int ni =
                        y * WIDTH + x - 1;
                    if (!visited[ni] &&
                        fb->buf[ni] > THRESHOLD)
                    {
                        visited[ni] = 1;

                        if (queueEnd < 10000)
                            queue[queueEnd++] = ni;
                    }
                }
                // Right
                if (x < roiPoints.xMax)
                {
                    int ni =
                        y * WIDTH + x + 1;
                    if (!visited[ni] &&
                        fb->buf[ni] > THRESHOLD)
                    {
                        visited[ni] = 1;
                        if (queueEnd < 10000)
                            queue[queueEnd++] = ni;
                    }
                }
                // Up
                if (y > roiPoints.yMin)
                {
                    int ni =
                        (y - 1) * WIDTH + x;

                    if (!visited[ni] &&
                        fb->buf[ni] > THRESHOLD)
                    {
                        visited[ni] = 1;
                        if (queueEnd < 10000)
                            queue[queueEnd++] = ni;
                    }
                }
                // Down
                if (y < roiPoints.yMax)
                {
                    int ni =
                        (y + 1) * WIDTH + x;

                    if (!visited[ni] &&
                        fb->buf[ni] > THRESHOLD)
                    {
                        visited[ni] = 1;
                        if (queueEnd < 10000)
                            queue[queueEnd++] = ni;
                    }
                }
            }
            // ------------------------------------------------
            // Component dimensions
            // ------------------------------------------------
            int diameterX = maxX - minX + 1;
            int diameterY = maxY - minY + 1;
            float aspectRatio = (float)diameterX / (float)diameterY;
            if (aspectRatio < 1.0f)
                aspectRatio = 1.0f / aspectRatio;
            // ------------------------------------------------
            // Is this the ball?
            // ------------------------------------------------
            if (area >= MIN_AREA &&
                area <= MAX_AREA &&
                diameterX >= MIN_DIAMETER &&
                diameterX <= MAX_DIAMETER &&
                diameterY >= MIN_DIAMETER &&
                diameterY <= MAX_DIAMETER &&
                aspectRatio <= MAX_ASPECT_RATIO)
            {
                if (!found || area > bestArea)
                {
                    found = true;
                    bestArea = area;
                    centerX =
                        (float)sumX / area;
                    centerY =
                        (float)sumY / area;
                //     Serial.printf(
                //     "Component accepted: "
                //     "area=%u "
                //     "diameterX=%d "
                //     "diameterY=%d "
                //     "aspect=%.3f\n",

                //     area,
                //     diameterX,
                //     diameterY,
                //     aspectRatio
                // );
                }
            }

        }
    }

    return found;
}
bool getBallPosition(
    float &x,
    float &y)
{  
    uint32_t t0 = millis();
    camera_fb_t *fb = esp_camera_fb_get();
    uint32_t t1 = millis();
    if (!fb)
        return false;


    uint32_t area;

    bool found =
        findBall(
            fb,
            x,
            y,
            area
        );
    uint32_t t2 = millis();
    esp_camera_fb_return(fb);
    uint32_t t3 = millis();


    // Serial.printf(
    //     "TIMING capture=%lu ms "
    //     "detect=%lu ms "
    //     "return=%lu ms\n",
    //     t1 - t0,
    //     t2 - t1,
    //     t3 - t2
    // );
    return found;
}

// ============================================================
// Calculate homography
// ============================================================

bool calculateHomography(
    const Point2D image[4],
    const Point2D plate[4],
    float Hout[8])
{
    float A[8][9];


    // --------------------------------------------------------
    // Build equations
    // --------------------------------------------------------

    for (int i = 0; i < 4; i++)
    {
        float u = image[i].x;
        float v = image[i].y;

        float X = plate[i].x;
        float Y = plate[i].y;

        int r = 2 * i;


        // X equation

        A[r][0] = u;
        A[r][1] = v;
        A[r][2] = 1.0f;

        A[r][3] = 0.0f;
        A[r][4] = 0.0f;
        A[r][5] = 0.0f;

        A[r][6] = -X * u;
        A[r][7] = -X * v;

        A[r][8] = X;


        // Y equation

        r++;

        A[r][0] = 0.0f;
        A[r][1] = 0.0f;
        A[r][2] = 0.0f;

        A[r][3] = u;
        A[r][4] = v;
        A[r][5] = 1.0f;

        A[r][6] = -Y * u;
        A[r][7] = -Y * v;

        A[r][8] = Y;
    }


    // --------------------------------------------------------
    // Gaussian elimination
    // --------------------------------------------------------

    for (int col = 0; col < 8; col++)
    {
        int pivot = col;

        float maxValue =
            fabsf(A[col][col]);


        for (int row = col + 1;
             row < 8;
             row++)
        {
            float value =
                fabsf(A[row][col]);

            if (value > maxValue)
            {
                maxValue = value;
                pivot = row;
            }
        }


        if (maxValue < 1e-8f)
            return false;


        // Swap rows

        if (pivot != col)
        {
            for (int j = col;
                 j < 9;
                 j++)
            {
                float temp =
                    A[col][j];

                A[col][j] =
                    A[pivot][j];

                A[pivot][j] =
                    temp;
            }
        }


        // Normalize

        float divisor =
            A[col][col];


        for (int j = col;
             j < 9;
             j++)
        {
            A[col][j] /= divisor;
        }


        // Eliminate

        for (int row = 0;
             row < 8;
             row++)
        {
            if (row == col)
                continue;


            float factor =
                A[row][col];


            if (fabsf(factor) < 1e-12f)
                continue;


            for (int j = col;
                 j < 9;
                 j++)
            {
                A[row][j] -=
                    factor * A[col][j];
            }
        }
    }


    // --------------------------------------------------------
    // Result
    // --------------------------------------------------------

    for (int i = 0; i < 8; i++)
        Hout[i] = A[i][8];


    return true;
}


// ============================================================
// Pixel -> plate coordinates
// ============================================================

void pixelToMM(
    float px,
    float py,
    float &x_mm,
    float &y_mm)
{
    float denominator =
        H[6] * px +
        H[7] * py +
        1.0f;


    x_mm =
        (H[0] * px +
         H[1] * py +
         H[2]) /
        denominator;


    y_mm =
        (H[3] * px +
         H[4] * py +
         H[5]) /
        denominator;
}


// ============================================================
// Calculate calibration error
// ============================================================

float calculateCalibrationError()
{
    float sumError = 0.0f;


    Serial.println();
    Serial.println("Calibration errors:");


    for (int i = 0; i < 4; i++)
    {
        float X;
        float Y;


        pixelToMM(
            imagePoints[i].x,
            imagePoints[i].y,
            X,
            Y
        );


        float dx =
            X - platePoints[i].x;

        float dy =
            Y - platePoints[i].y;


        float error =
            sqrtf(dx * dx + dy * dy);


        sumError += error;


        Serial.printf(
            "Point %d: measured=(%.2f,%.2f)"
            "  target=(%.2f,%.2f)"
            "  error=%.3f mm\n",

            i,

            X,
            Y,

            platePoints[i].x,
            platePoints[i].y,

            error
        );
    }


    return sumError / 4.0f;
}


// ============================================================
// Save calibration
// ============================================================

void saveCalibration()
{
    preferences.begin(
        "balltrack",
        false
    );


    preferences.putBytes(
        "points",
        imagePoints,
        sizeof(imagePoints)
    );

    preferences.putBytes(
        "roi",
        &roiPoints,
        sizeof(roiPoints)
    );

    preferences.putUInt(
        "version",
        CALIBRATION_VERSION
    );


    preferences.end();


    Serial.println(
        "Calibration saved."
    );
}


// ============================================================
// Load calibration
// ============================================================

bool loadCalibration()
{
    preferences.begin(
        "balltrack",
        true
    );


    uint32_t version =
        preferences.getUInt(
            "version",
            0
        );


    if (version != CALIBRATION_VERSION)
    {
        preferences.end();
        return false;
    }


    size_t lengthPoints =
        preferences.getBytes(
            "points",
            imagePoints,
            sizeof(imagePoints)
        );

    size_t lengthRoi =
        preferences.getBytes(
            "roi",
            &roiPoints,
            sizeof(roiPoints)
        );
        Serial.printf("Loaded ROI: xMin=%d, yMin=%d, xMax=%d, yMax=%d\n",
                  roiPoints.xMin,
                  roiPoints.yMin,
                  roiPoints.xMax,
                  roiPoints.yMax);
    preferences.end();


    return lengthPoints == sizeof(imagePoints) && lengthRoi == sizeof(roiPoints);
}


// ============================================================
// Acquire a stable ball position
// ============================================================

bool acquireCalibrationPoint(
    int pointNumber,
    Point2D &result)
{
    constexpr int SAMPLES = 20;

    constexpr float MAX_VARIATION = 2.0f;


    float samplesX[SAMPLES];
    float samplesY[SAMPLES];

// --------------------------------------------------------
// Discard old camera frames
// --------------------------------------------------------

    constexpr int DISCARD_FRAMES = 5;

    Serial.println("Waiting for camera...");

    for (int i = 0; i < DISCARD_FRAMES; i++)
    {
        camera_fb_t *fb = esp_camera_fb_get();

        if (fb)
            esp_camera_fb_return(fb);

        delay(50);
    }


// --------------------------------------------------------
// Start measurements
// --------------------------------------------------------

    Serial.println();
    Serial.printf(
        "Collecting %d measurements...\n",
        SAMPLES
    );


    int count = 0;


    while (count < SAMPLES)
    {
        float x;
        float y;


        if (getBallPosition(x, y))
        {
            samplesX[count] = x;
            samplesY[count] = y;

            count++;


            Serial.printf(
                "%2d: X=%7.2f Y=%7.2f\n",
                count,
                x,
                y
            );


            delay(50);
        }
        else
        {
            Serial.println(
                "Ball not detected..."
            );

            delay(100);
        }
    }


    // --------------------------------------------------------
    // Calculate mean
    // --------------------------------------------------------

    float meanX = 0.0f;
    float meanY = 0.0f;


    for (int i = 0; i < SAMPLES; i++)
    {
        meanX += samplesX[i];
        meanY += samplesY[i];
    }


    meanX /= SAMPLES;
    meanY /= SAMPLES;


    // --------------------------------------------------------
    // Check maximum deviation
    // --------------------------------------------------------

    float maxDeviation = 0.0f;


    for (int i = 0; i < SAMPLES; i++)
    {
        float dx =
            samplesX[i] - meanX;

        float dy =
            samplesY[i] - meanY;


        float deviation =
            sqrtf(dx * dx + dy * dy);


        if (deviation > maxDeviation)
            maxDeviation = deviation;
    }


    Serial.printf(
        "Mean: X=%.3f Y=%.3f"
        "   max deviation=%.3f px\n",

        meanX,
        meanY,
        maxDeviation
    );


    if (maxDeviation > MAX_VARIATION)
    {
        Serial.println(
            "Position not stable."
        );

        return false;
    }


    result.x = meanX;
    result.y = meanY;


    return true;
}
// ============================================================
// Measure ROI (size of plate in pixels)
// ============================================================
bool measureROI()
{
    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb)
        return false;

       static uint16_t columnSum[WIDTH];
    static uint16_t rowSum[HEIGHT];
    uint32_t sumValue=0;
    for (int startX = 0;startX <= WIDTH;++startX)
        {
            for (int startY = 0;startY <= HEIGHT;++startY)
                {
                    columnSum[startX] += fb->buf[startY * WIDTH + startX];
                    sumValue += fb->buf[startY * WIDTH + startX];

                }
            columnSum[startX] /= HEIGHT;
            // Serial.printf("Column %d average value: %d\n", startX, columnSum[startX]);
        }
    
    int avgValue=sumValue/ (WIDTH * HEIGHT);

    // Serial.printf("Average value of the image: %d\n", avgValue);
    // Serial.println("---Calcul des ROI X -------");
// Recherche roiPoints.xMin
        for(int startX=WIDTH/2;startX>=0;--startX)
        {
            if(columnSum[startX]>avgValue)
            {
                roiPoints.xMin = startX;
                Serial.printf("roiPoints.xMin found at column %d\n", startX);
                break;
            }
        }
// Recherche roiPoints.xMax
        for(int startX=WIDTH/2;startX<WIDTH;++startX)
        {
            if(columnSum[startX]>avgValue)
            {
                roiPoints.xMax = startX;
                Serial.printf("roiPoints.xMax found at column %d\n", startX);
                break;
            }
        }
        // memset(columnSum, 0, sizeof(columnSum));
    sumValue=0;
    for (int startY = 0;startY <= HEIGHT;++startY)
        {
            for (int startX = 0;startX <= WIDTH;++startX)
                {
                    rowSum[startY] += fb->buf[startY * WIDTH + startX];
                    sumValue += fb->buf[startY * WIDTH + startX];

                }
            rowSum[startY] /= WIDTH;
            // Serial.printf("Row %d average value: %d\n", startY, rowSum[startY]);
        }
    
    avgValue=sumValue/ (WIDTH * HEIGHT);
    // Serial.printf("Average value of the image: %d\n", avgValue);
    // Serial.println("---Calcul des ROI Y -------");

// Recherche roiPoints.yMin
        for(int startY=HEIGHT/2;startY>=0;--startY)
        {
            if(rowSum[startY]>avgValue)
            {
                roiPoints.yMin = startY;
                Serial.printf("roiPoints.yMin found at row %d\n", startY);
                break;
            }
        }
// Recherche roiPoints.yMax
        for(int startY=HEIGHT/2;startY<HEIGHT;++startY)
        {
            if(rowSum[startY]>avgValue)
            {
                roiPoints.yMax = startY;
                Serial.printf("roiPoints.yMax found at row %d\n", startY);
                break;
            }
        }
    esp_camera_fb_return(fb);
    return true;




}
// ============================================================
// Interactive calibration
// ============================================================

bool performCalibration()
{
    // Serial.println();
    // Serial.println("================================");
    // Serial.println("       BALL CALIBRATION");
    // Serial.println("================================");
    sendMessage("Starting calibration");
    sendMessage("Measuring ROI. Remove the ball from the plate...");
    measureROI();
    const char *names[4] =
    {
        "BOTTOM LEFT",
        "BOTTOM RIGHT",
        "TOP RIGHT",
        "TOP LEFT"
    };


    for (int i = 0; i < 4; i++)
    {
        String msg="Place ball at ";
        msg+=names[i];
        sendMessage(msg.c_str());
        // Serial.println();
        // Serial.println("--------------------------------");
        // Serial.printf(
        //     "Place ball at %s\n",
        //     names[i]
        // );
        sendMessage("Press ENTER when ready.");
                // Serial.println(
                //     "Press ENTER when ready."
                // );


        // Wait for ENTER

        while (!Serial.available())
        {
            delay(20);
        }


        // Clear input

        while (Serial.available())
            Serial.read();


        sendMessage("Detecting ball...");


        Point2D point;


        if (!acquireCalibrationPoint(
                i,
                point))
        {
            sendMessage("Calibration point failed.");
            // Serial.println(
            //     "Calibration point failed."
            // );

            return false;
        }


        imagePoints[i] = point;

        msg="Calibration point ";
        msg+=names[i];
        msg+=" acquired";
        sendMessage(msg.c_str());
        // Serial.printf(
        //     "%s saved: "
        //     "pixel=(%.3f, %.3f)\n",

        //     names[i],

        //     point.x,
        //     point.y
        // );
    }


    // --------------------------------------------------------
    // Calculate homography
    // --------------------------------------------------------

    sendMessage("Calculating homography..." );


    if (!calculateHomography(
            imagePoints,
            platePoints,
            H))
    {
        sendMessage("ERROR: homography calculation failed.");

        return false;
    }


    // --------------------------------------------------------
    // Display H
    // --------------------------------------------------------

    sendMessage("Homography:");

    // for (int i = 0; i < 8; i++)
    // {
    //     Serial.printf(
    //         "H[%d] = %.9f\n",
    //         i,
    //         H[i]
    //     );
    // }


    // --------------------------------------------------------
    // Check calibration error
    // --------------------------------------------------------

    float averageError =
        calculateCalibrationError();

    
    // Serial.printf(
    //     "\nAverage calibration error = %.3f mm\n",
    //     averageError
    // );


    if (averageError > 2.0f)
    {
        sendMessage("WARNING: calibration error is high.");
        sendMessage("Calibration NOT saved.");
        // Serial.println();
        // Serial.println(
        //     "WARNING: calibration error is high."
        // );

        // Serial.println(
        //     "Calibration NOT saved."
        // );

        return false;
    }


    saveCalibration();


    // sendMessage("================================");
    sendMessage("CALIBRATION SUCCESSFUL");        
    
    // Serial.println(
    //     "================================"
    // );


    return true;
}
void processCommands()
{
    static char buffer[32];
    static uint8_t index = 0;

    while (Serial.available())
    {
        char c = Serial.read();

        if (c == '\n')
        {
            buffer[index] = '\0';

            if (strcmp(buffer, "CAL") == 0)
            {
                performCalibration();
            }
            else if (strcmp(buffer, "PING") == 0)
            {
                sendMessage("PONG");
            }
            else if (strcmp(buffer, "STOP") == 0)
            {
                sendMessage("STOP received");
            }
            else
            {
                Serial.printf(
                    "MSG,Unknown command: %s\n",
                    buffer
                );
            }

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
                index = 0;
            }
        }
    }
}
void readSensorReg(sensor_t *s, uint8_t reg)
{
    uint8_t value = s->get_reg(s, reg, 0xFF);

    Serial.printf("Register 0x%02X = 0x%02X\n", reg, value);
}
// ============================================================
// Setup
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);


    Serial.println();
    Serial.println(
        "================================"
    );
    Serial.println(
        "       ESP32-CAM BALL TRACKER"
    );
    Serial.println(
        "================================"
    );


    if (!initCamera())
    {
        Serial.println(
            "Camera ERROR"
        );

        while (true)
            delay(1000);
    }

sensor_t * s = esp_camera_sensor_get();
if (s != NULL) {
    // Augmente la vitesse d'horloge interne du capteur (PCLK)
    // CLKRC registre : bit 0-5 contrôlent le diviseur de fréquence.
    s->set_reg(s, 0xff, 0x01, 0x01); // Bank select
    s->set_reg(s, 0x11, 0x01, 0x00); // Règle le diviseur au minimum (vitesse max)
    // s->set_gain_ctrl(s, 1); // Active le contrôle du gain (pour garder une image claire)
    // s->set_exposure_ctrl(s, 0); // Désactive l'auto-exposition (bloque le temps de capture à 30ms)
    readSensorReg(s, 0x11);  // CLKRC
    readSensorReg(s, 0x0C);  // COM3
    readSensorReg(s, 0x3E);  // COM14
    readSensorReg(s, 0x6B);  // DBLV
    s->set_reg(s, 0x11, 0xFF, 0x01);



    // // 1. Désactiver le contrôle d'exposition automatique (AEC)
    // s->set_exposure_ctrl(s, 0); 

    // // 2. Fixer un temps d'exposition très court (valeur basse = plus rapide)
    // // Essayez une valeur entre 50 et 200 selon la luminosité réelle
    // s->set_aec_value(s, 100); 

    // // 3. Activer le gain automatique (AGC) pour compenser la baisse de lumière
    // s->set_gain_ctrl(s, 1);
    // s->set_agc_gain(s, 2); // Ajuster le niveau de gain si l'image est trop sombre

    // // 4. Désactiver l'AEC2 (algorithme d'exposition secondaire agressif)
    // s->set_aec2(s, 0);



    // s->set_aec2(s, 0); // Désactive l'AEC perfectionné (0 = off)
// Vous pouvez aussi désactiver totalement l'exposition auto et la fixer bas :
    // s->set_exposure_ctrl(s, 0); 
}
    Serial.println(
        "Camera OK"
    );


    // --------------------------------------------------------
    // Try to load calibration
    // --------------------------------------------------------

    if (loadCalibration())
    {
        Serial.println();
        Serial.println(
            "Calibration found."
        );


        if (!calculateHomography(
                imagePoints,
                platePoints,
                H))
        {
            Serial.println(
                "Stored calibration invalid."
            );
        }
        else
        {
            float error =
                calculateCalibrationError();


            Serial.printf(
                "Average calibration error = "
                "%.3f mm\n",
                error
            );


            Serial.println(
                "Starting tracking."
            );
        }
    }
    else
    {
        Serial.println();
        Serial.println(
            "No calibration found."
        );


        if (!performCalibration())
        {
            Serial.println();
            Serial.println(
                "Calibration failed."
            );

            while (true)
                delay(1000);
        }
    }

}


// ============================================================
// Tracking
// ============================================================

void loop()
{
    // uint32_t start = millis();
    // Serial.printf("millis: %lu\n", millis());

    float px;
    float py;
    // uint32_t t0 = millis();


 

    if (getBallPosition(px, py))
    {
                // uint32_t t1 = millis();
        float x_mm;
        float y_mm;


        pixelToMM(
            px,
            py,
            x_mm,
            y_mm
        );


        // Serial.printf(
        //     "BALL  px=%6.1f py=%6.1f"
        //     "   X=%7.2f mm Y=%7.2f mm"
        //     "   %lu ms ",

        //     px,
        //     py,

        //     x_mm,
        //     y_mm,

        //     millis() - start
        // );
        sendBallPosition(x_mm, y_mm);
        // uint32_t t2 = millis();
    //     Serial.printf(
    //     "TIMING t1-t0=%lu ms \n ",
    //     t1 - t0
  
    // );
                // Debug éventuellement détaillé
    }
    else
    {
        sendMessage("No ball");
    }
        // uint32_t t1 = millis();
    processCommands();

    // delay(5);
}