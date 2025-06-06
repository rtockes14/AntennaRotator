
#include <Arduino.h>
#include "MPU9250.h"
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include <cmath>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

#define MAGNETIC_DECLINATION 2.5

LiquidCrystal_I2C lcd(0x27,20,4);

// Motor 1
#define ENABLE_A 32
#define INPUT_1 27
#define INPUT_2 14

// Motor 2
#define ENABLE_B 26
#define INPUT_3 33
#define INPUT_4 25

unsigned long previousMillis = 0;
const long interval = 32000;

unsigned long lastReadTime = 0;
unsigned long readInterval = 10;

//MPU9250 mpu;

TwoWire I2C_1 = TwoWire(1);
Adafruit_LIS3MDL lis3mdl;
MPU9250 IMU(I2C_1,0x68);

const int numReadings = 1;
bool flipped = true;
int status;




// ===========================================================================================================


void rotateLeftThenRight()
{
    if (flipped)
    {
        digitalWrite(INPUT_1, HIGH);
        digitalWrite(INPUT_2, LOW);
        digitalWrite(INPUT_3, LOW);
        digitalWrite(INPUT_4, HIGH);

        analogWrite(ENABLE_A, 100);
        analogWrite(ENABLE_B, 0);
    }
    else
    {
        digitalWrite(INPUT_1, LOW);
        digitalWrite(INPUT_2, HIGH);
        digitalWrite(INPUT_3, HIGH);
        digitalWrite(INPUT_4, LOW);

        analogWrite(ENABLE_A, 100);
        analogWrite(ENABLE_B, 0);
    }
    flipped = !flipped;
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


    //if (!mpu.setup(0x68)) {  // change to your own address
        //while (1) {
            //Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            //delay(5000);
        //}
    //}

        // Try to initialize!
    if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    //if (! lis3mdl.begin_SPI(LIS3MDL_CS)) {  // hardware SPI mode
    //if (! lis3mdl.begin_SPI(LIS3MDL_CS, LIS3MDL_CLK, LIS3MDL_MISO, LIS3MDL_MOSI)) { // soft SPI
        Serial.println("Failed to find LIS3MDL chip");
        while (1) { delay(10); }
    }
    Serial.println("LIS3MDL Found!");

    lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
    //Serial.print("Performance mode set to: ");
    //switch (lis3mdl.getPerformanceMode()) {
        //case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
        //case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
        //case LIS3MDL_HIGHMODE: Serial.println("High"); break;
        //case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
    //}

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  //Serial.print("Operation mode set to: ");
  //// Single shot mode will complete conversion and go into power down
  //switch (lis3mdl.getOperationMode()) {
    //case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    //case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    //case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  //}

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  //Serial.print("Data rate set to: ");
  //switch (lis3mdl.getDataRate()) {
    //case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    //case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    //case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    //case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    //case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    //case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    //case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    //case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    //case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    //case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    //case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    //case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  //}
  
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  //Serial.print("Range set to: ");
  //switch (lis3mdl.getRange()) {
    //case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    //case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    //case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    //case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  //}

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

  delay(2000);
}


void loop() 
{
    unsigned long currentMillis = millis(); // get the current time

    float heading;
    float sumHeading = 0;

    //lis3mdl.read();      // get X Y and Z data at once
    //// Then print out the raw data
    ////Serial.print("\nX:  "); 
    //Serial.print(lis3mdl.x); Serial.print(",");

    ////Serial.print("  \tY:  "); 
    //Serial.print(lis3mdl.y); Serial.print(","); 
    ////Serial.print("  \tZ:  "); 
    //Serial.println(lis3mdl.z); //Serial.print(",");

    /* Or....get a new sensor event, normalized to uTesla */
    sensors_event_t event; 

    lis3mdl.getEvent(&event);

            // Transmit over serial
    Serial.print(event.magnetic.x, 3); Serial.print(",");
    Serial.print(event.magnetic.y, 3); Serial.print(",");
    Serial.println(event.magnetic.z, 3);

    //delay(250);  // 10 Hz

    //float Xm_off = event.magnetic.x - 7.325402; // X-axis compbined bias
    //float Ym_off = event.magnetic.y - 17.031274; // Y -axis compbined bias
    //float Zm_off = event.magnetic.z + 48.057196; // Z-axis compbined bias

    //float Xm_cal = 1.290962*Xm_off + -0.051106*Ym_off + 0.117649*Zm_off;
    //float Ym_cal = -0.051106*Xm_off + 1.294649*Ym_off + 0.109386*Zm_off;
    //float Zm_cal = 0.117649*Xm_off + 0.109386*Ym_off + 0.980242*Zm_off;

    if (currentMillis - previousMillis >= interval) 
    {
        previousMillis = currentMillis;
        rotateLeftThenRight();
    }

    //float heading = (atan2(event.magnetic.y,event.magnetic.x) * 180 / PI);
    //float heading = (atan2(event.magnetic.y,event.magnetic.x));
    //heading = (atan2(Ym_off, Xm_off));

    //heading = heading * 180.0 / PI;

    //if (heading < 0)
    //{
        //heading = 360 + heading;
    //}

    

    //Serial.println(heading + MAGNETIC_DECLINATION);

    /* Display the results (magnetic field is measured in uTesla) */
    //Serial.print(event.magnetic.x, 6); Serial.print(",");
    //Serial.print(event.magnetic.y, 6); Serial.print(",");
    //Serial.println(event.magnetic.z, 6); 

    delay(8.5); 


    //float avgHeading = sumHeading / numReadings;
    //Serial.print("Compass Heading: ");
    //Serial.println(heading);

    //delay(500);
}