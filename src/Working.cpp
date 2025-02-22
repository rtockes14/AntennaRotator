

#include <Arduino.h>
#include "MPU9250.h"
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <cmath>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

TwoWire I2C_1 = TwoWire(1);

LiquidCrystal_I2C lcd(0x27,20,4);

MPU9250 IMU(I2C_1,0x68);
Adafruit_LIS3MDL lis3mdl;

//#define LIS3MDL_CLK 18
//#define LIS3MDL_MISO 19 
//#define LIS3MDL_MOSI 23
//#define LIS3MDL_CS 2



// Motor 1
#define ENABLE_A 32
#define INPUT_1 27
#define INPUT_2 14

// Motor 2
#define ENABLE_B 26
#define INPUT_3 33
#define INPUT_4 25

unsigned long previousMillis = 0;
const long interval = 1000;

unsigned long lastReadTime = 0;
unsigned long readInterval = 10;

const float X_BIAS = -13.652010;
const float Y_BIAS = -2.466551;
const float Z_BIAS = -83.898040;

//float currentHeading = 0;

int status;

String data;
char d1;

int motorASpeed = 0;
int motorBSpeed = 0;

float roll;
float pitch;
float heading;

float currentAzimuth;
float currentElevation;

// Satellite info via app
const char *satName;
int satId;
float satElevation = 0;
float satAzimuth = 0;
double satLatitude = 0;
double satLongitude = 0;

float x;
float y;

int currentIndex = 0;
float elevation[5];

const char *date;

char shorterDate[19];

const int minimumSpeed = 50;
const int maxSpeed = 255;
const int maxRotation = 359;
const int maxElevation = 89;
const int minElevation = 0;

bool tiltingUp = false;
bool tiltingDown = false;
bool panningLeft = false;
bool panningRight = false;
bool alreadyTilting = false;
bool alreadyPanning = false;
bool flipped = true;

//float desiredElevation = 0; // test elevation for control
//float desiredHeading = 340; // test heading for control
float tolerance = 5;

// ===========================================================================================================

void rotateLeftThenRight()
{
    if (flipped)
    {
        digitalWrite(INPUT_1, HIGH);
        digitalWrite(INPUT_2, LOW);
        digitalWrite(INPUT_3, HIGH);
        digitalWrite(INPUT_4, LOW);

        analogWrite(ENABLE_A, 100);
        analogWrite(ENABLE_B, 70);
    }
    else
    {
        digitalWrite(INPUT_1, LOW);
        digitalWrite(INPUT_2, HIGH);
        digitalWrite(INPUT_3, LOW);
        digitalWrite(INPUT_4, HIGH);

        analogWrite(ENABLE_A, 100);
        analogWrite(ENABLE_B, 75);
    }
    flipped = !flipped;
}

float normalize_angle(double angle) {
    angle = fmod(angle, 360.0);  // Wrap angle within [0, 360)
    if (angle < 0) {
        angle += 360.0;  // Ensure it's positive
    }
    if (angle > 180.0) {
        angle -= 360.0;  // Normalize to [-180, 180)
    }
    return angle;
}

float get_shortest_rotation(double current, double target) 
{
    float delta = target - current;
    return normalize_angle(delta);  // Normalize the difference to [-180, 180]
}

void orientationTest(float currentElevation, float rotation)
{
    
    tiltingUp = false;
    tiltingDown = false;

    if ((currentElevation >= (0 - 5) && currentElevation <= (0 + 5)) && (satElevation <= minElevation)) 
    {
        //Serial.println("Don't move -- elevation = 0");
        tiltingUp = false;
        tiltingDown = false;
        //delay(25);
    }
    else 
    {
        // check if desired elevation is greater than current roll angle.  If so set tilting up to true
        if ((currentElevation <= (satElevation - tolerance)) && (currentElevation < maxElevation) || currentElevation < minElevation)
        {
            tiltingUp = true;
            tiltingDown = false;
            Serial.print("Begin Tilting UP  ");
            Serial.println(currentElevation);
        }
        
        if ((currentElevation >= (satElevation + tolerance)) && (currentElevation > minElevation))
        {
            tiltingDown = true;
            tiltingUp = false;
            Serial.print("Begin Tilting DOWN  ");
            Serial.println(currentElevation);
        }
    }

    panningLeft = false;
    panningRight = false;

    //// Determine whether difference in azimuth requires left or right pan
    //float rotation = get_shortest_rotation(currentAzimuth, satAzimuth);

    // TODO: REPLACE WITH ROTATION AND GETSHORTESTROTATION
    if (rotation < 0)
    {
        panningLeft = true;
        panningRight = false;
        Serial.println("Begin Panning LEFT  ");
        Serial.print("Current Azimuth: ");
        Serial.println(currentAzimuth);
        Serial.print("Sat Azimuth: ");
        Serial.println(satAzimuth);
    }
    //if (satAzimuth > currentAzimuth)
    if (rotation > 0)
    {
        panningRight = true;
        panningLeft = false;
        Serial.println("Begin Panning RIGHT  ");
        Serial.print("Current Azimuth: ");
        Serial.println(currentAzimuth);
        Serial.print("Sat Azimuth: ");
        Serial.println(satAzimuth);
    }
    if (rotation > -10 && rotation < 10)
    {
        panningLeft = false;
        panningRight = false;
    }
    // Start panning Left if not already panning TODO: LOGIC FOR ALREADY PANNING
    if ((panningLeft))
    {
        
        digitalWrite(INPUT_1, HIGH);
        digitalWrite(INPUT_2, LOW);
        if (rotation < -50)
        {
            analogWrite(ENABLE_A, 100); // pan
        }
        else if (rotation >= -50 && rotation < 30)
        {
            analogWrite(ENABLE_A, 60); // pan
        }
        else 
        {
            analogWrite(ENABLE_A, 45); // pan
        }
        
        Serial.println("Currently Panning Left");
    }
    // Start panning Right if not already panning
    if ((panningRight))
    {
        digitalWrite(INPUT_1, LOW);
        digitalWrite(INPUT_2, HIGH);
        if (rotation > 50)
        {
            analogWrite(ENABLE_A, 100); // pan
        }
        else if (rotation <= 50 && rotation > 30)
        {
            analogWrite(ENABLE_A, 60); // pan
        }
        else 
        {
            analogWrite(ENABLE_A, 45); // pan
        }
        Serial.println("Currently Panning Right");
    }
    // ===============================================================================================================================

    // if antenna should be tilting up, move motor in ccw direction.  ======== START TILTING UP
    if (tiltingUp)
    {
        digitalWrite(INPUT_3, LOW);
        digitalWrite(INPUT_4, HIGH);
        analogWrite(ENABLE_B, 55); // tilt
        Serial.println("Currently Tilting UP");
    }
   // if antenna should be tilting down, move motor in ccw direction.  ======== START TILTING DOWN
    if (tiltingDown)
    {
        digitalWrite(INPUT_3, HIGH);
        digitalWrite(INPUT_4, LOW);
        analogWrite(ENABLE_B, 55); // tilt
        Serial.println("Currently Tilting DOWN");
    }

    // ===============================================================================================================================

    // if neither up or down movement is needed, turn off motor
    if (tiltingDown == false && tiltingUp == false)
    {
        digitalWrite(INPUT_3, LOW);
        digitalWrite(INPUT_4, LOW);
        analogWrite(ENABLE_B, 0);
        alreadyTilting = false;
        Serial.println(" Quitting tilt condition");
    }
    // if neither left or right movement is needed, turn off motor
    if (panningLeft == false && panningRight == false)
    {
        digitalWrite(INPUT_1, LOW);
        digitalWrite(INPUT_2, LOW);
        analogWrite(ENABLE_A, 0);
        alreadyPanning = false;
        Serial.println(" Quitting pan condition");
    }
}

void azStop()
{
    digitalWrite(INPUT_1, LOW);
    digitalWrite(INPUT_2, LOW);
    analogWrite(ENABLE_A, 0);
}

void elStop()
{
    digitalWrite(INPUT_3, LOW);
    digitalWrite(INPUT_4, LOW);
    analogWrite(ENABLE_B, 0);
}

void parseJson(String jsonData)
{
  Serial.println("Received JSON: " + jsonData);

  // Create a JSON document to parse the incoming data
  JsonDocument doc;

  // Parse the JSON data
  DeserializationError error = deserializeJson(doc, jsonData);

  // Check for parsing errors
  if (error)
  {
    Serial.println("Failed to parse JSON");
    return;
  }

  satName = doc["SatName"];
  satId = doc["SatId"];
  satElevation = doc["SatElevation"];
  satAzimuth = doc["SatAzimuth"];
  satLatitude = doc["SatLatitude"];
  satLongitude = doc["SatLongitude"];
  date = doc["Date"];
  strcpy(shorterDate, date);
  shorterDate[19] = '\0';
}


void lcdPrint(float heading)
{
    lcd.clear();
    lcd.setCursor(0, 0);

    lcd.print("SAT: ");
    lcd.setCursor(5, 0);
    lcd.print(satName);
    lcd.setCursor(0, 1);
    lcd.print("ELV:");
    lcd.setCursor(4, 1);
    lcd.print(satElevation);
    lcd.setCursor(11, 1);
    lcd.print("AZI:");
    lcd.setCursor(15, 1);
    lcd.print(satAzimuth);
    lcd.setCursor(0, 2);
    lcd.print("LAT:");
    lcd.setCursor(4, 2);
    lcd.print(satLatitude);
    lcd.setCursor(10, 2);


    lcd.print("LNG:");
    lcd.setCursor(14, 2);
    lcd.print(satLongitude);
    lcd.setCursor(0, 3);
    lcd.print(currentElevation);
    lcd.print("   |   ");
    lcd.print(heading); 

}


void setup() {

    pinMode(ENABLE_A, OUTPUT);
    pinMode(ENABLE_B, OUTPUT);
    pinMode(INPUT_1, OUTPUT);
    pinMode(INPUT_2, OUTPUT);
    pinMode(INPUT_3, OUTPUT);
    pinMode(INPUT_4, OUTPUT);

    Serial.begin(9600);

    lcd.init();
    lcd.backlight();

    Wire.begin();
    I2C_1.begin(19,23);

    status = IMU.begin();
    if (status < 0)
    {
        Serial.println("IMU initialization unsuccessful");
        Serial.print("Status: ");
        Serial.println(status);
        while(1){}
    }

    delay(1000);

        // Try to initialize!
    if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
    //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
        Serial.println("Failed to find LIS3MDL chip");
        while (1) { delay(10); }
    }
    Serial.println("LIS3MDL Found!");

    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    Serial.print("Performance mode set to: ");
    switch (lis3mdl.getPerformanceMode()) {
        case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
        case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
        case LIS3MDL_HIGHMODE: Serial.println("High"); break;
        case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
    }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!




    lcd.setCursor(0, 0);
    lcd.print("  SAT TRACKER v1.0");
    delay(500);
    lcd.setCursor(0, 1);
    lcd.print("====================");
    lcd.setCursor(0, 2);
    lcd.print(" START APPLICATION");
    lcd.setCursor(0, 3);
    lcd.print(" TO BEGIN TRACKING->");

  delay(5000);
}


void loop() 
{
    unsigned long currentMillis = millis(); // get the current time
    

    if (Serial.available())
    {
        String jsonData = Serial.readString();
        parseJson(jsonData);
    }

    if (currentMillis - previousMillis >= interval) 
    {
        previousMillis = currentMillis;
        lis3mdl.read(); // get XYZ data all at once

        sensors_event_t event;
        lis3mdl.getEvent(&event);
        float Xm_off = event.magnetic.x - 7.325402; // X-axis compbined bias
        float Ym_off = event.magnetic.y - 17.031274; // Y -axis compbined bias
        float Zm_off = event.magnetic.z + 48.057196; // Z-axis compbined bias
    
        float sensorReadings[10];
        int numberOfReadings = 10; 
        float rotation;
        float sum = 0;
        //for (int i = 0; i < numberOfReadings; i++)
        //{
            //sensorReadings[i] = (atan2(event.magnetic.y,event.magnetic.x) * 180 / PI);
            heading = (atan2(event.magnetic.y - Y_BIAS,event.magnetic.x - X_BIAS) * 180 / PI);
            // Determine whether difference in azimuth requires left or right pan
            //rotation = get_shortest_rotation(sensorReadings[i], satAzimuth);
            rotation = get_shortest_rotation(heading, satAzimuth);
            //sum += rotation;
            //delay(10);
        //}
        delay(25);

        //float average = sum / numberOfReadings;
        //float heading = average;
        //float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180 / PI);
        //heading = (atan2(Ym_off, Xm_off));
        //heading = heading * 180 / PI;

        //if (heading < 0)
        //{
            //heading = 360 + heading;
        //}
        //currentAzimuth = heading;

        IMU.readSensor();
        
        float ax = IMU.getAccelX_mss();
        float ay = IMU.getAccelY_mss();
        float az = IMU.getAccelZ_mss();

        // Calculate roll angle in radians
        float roll = atan2(ax, az);  // atan2 handles the sign of both values

        // Convert radians to degrees
        roll = roll * 180.0 / PI;

        // Print roll angle
        Serial.print("Roll: ");
        Serial.println(roll); 
        
        currentElevation = roll;
        
        orientationTest(currentElevation, rotation);
        
        lcdPrint(heading);
    }
}