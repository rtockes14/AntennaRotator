
#undef PI
#include <Arduino.h>
//#include <LiquidCrystal_I2C.h>
#include "MPU9250.h"

//LiquidCrystal_I2C lcd(0x27, 20, 4);
MPU9250 mpu;

unsigned long previousMillis = 0;  // Store the last time sensor was read
const long interval = 100;  // Set a time interval for sensor readings (in milliseconds)

TaskHandle_t sensorTaskHandle;
TaskHandle_t lcdTaskHandle;

void print_roll_pitch_yaw() {
    Serial.print("Yaw, Pitch, Roll: ");
    Serial.print(mpu.getYaw(), 2);
    Serial.print(", ");
    Serial.print(mpu.getPitch(), 2);
    Serial.print(", ");
    Serial.println(mpu.getRoll(), 2);
    delay(100);
}

void readSensorTask(void *pvParameters){

    //if (mpu.update()) 
    //{
        //static uint32_t prev_ms = millis();
        //if (millis() > prev_ms + 25) {
            //print_roll_pitch_yaw();
            //prev_ms = millis();
            ////lcd.write(mpu.getPitch());
            
        //}
    //}
}


void printLCDTask(void *pvParameters){

  //lcd.clear();
  //lcd.print(mpu.getRoll());
  //lcd.setCursor(0,2);
  //lcd.print(mpu.getMagZ());
  delay(200);
}


void setup() {
    //lcd.init();
    //lcd.backlight();
    Serial.begin(115200);
    Wire.begin(21, 22);
    delay(2000);

    if (!mpu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    
    xTaskCreatePinnedToCore(readSensorTask, "ReadSensor", 8192, NULL, 1, &sensorTaskHandle, 0);
    //xTaskCreatePinnedToCore(printLCDTask, "PrintLCD", 8192, NULL, 1, &lcdTaskHandle, 1);
}

void loop() {
  // Look pretty
    //if (mpu.update()) 
    //{
        //static uint32_t prev_ms = millis();
        //if (millis() > prev_ms + 25) {
            //print_roll_pitch_yaw();
            //prev_ms = millis();
            ////lcd.write(mpu.getPitch());
            
        //}
    //}
  }



