#include <ADXL345.h>
#include <Wire.h>
#include "Servo.h"
#include <SD.h>

#define ScaleFor2G 0.0039
#define ScaleFor4G 0.0078
#define ScaleFor8G 0.0156
#define ScaleFor16G 0.0312
#define BALL_VALVE_PWM_PORT 2
#define SD_CARD_PORT 53

unsigned long open_valve_time = 15000;
unsigned long close_valve_time = 395000;
unsigned long time;
int x;
int y;
int z;

File myfile;

boolean sd_connected;

Servo ball_valve;
unsigned int reading = 0;
int co2Value;

int co2Addr = 0x68; // This is the default address of the CO2 sensor, 7bits shifted
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

void setup() {  
  Serial.begin(9600);
  Serial.println("Initializing...");
  pinMode(53, OUTPUT);
  if(!(sd_connected = SD.begin(53))) {
    // sd card not found...
  } 
    Serial.println("Initializing...");
  ball_valve_setup();
    Serial.println("Initializing...");
  adxl.powerOn();

  adxl.setActivityThreshold(75); //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
  
  PORTD = (1 << PORTD1) | (1 << PORTD0);//enable pullups
}

void loop() {
  time = millis();
  Serial.print("Time: ");
  Serial.print(time);

  //Serial.print("A ");
  
  if(!sd_connected) {
    Serial.println("SDCARD NOT CONNECTED");
    sd_connected = SD.begin(53);
  } else {
    if (time > open_valve_time && time <= close_valve_time) {
      open_ball_valve();
    } else {
      close_ball_valve();
    }
    
    // The SD Card is connected - we can write data to it now...
    myfile = SD.open("test.txt", FILE_WRITE);

    
    adxl.readAccel(&x, &y, &z);
    x *= ScaleFor16G;
    y *= ScaleFor16G;
    z *= ScaleFor16G;

    co2Value = -1;
    if(reading % 10 == 0) {
      co2Value = readCO2(); 
    }

    myfile.print("Time: ");
    myfile.print(reading);
    myfile.print(" | x: ");
    myfile.print(x);
    myfile.print(" | y: ");
    myfile.print(y);
    myfile.print(" | z: ");
    myfile.print(z);
    myfile.print(" | temp: ");
    myfile.print("NA");
    myfile.print(" | CO2: ");
    myfile.print(co2Value);
    myfile.println();
    /*
    Serial.print("Time: ");
    Serial.print(reading);
    Serial.print(" | x: ");
    Serial.print(x);
    Serial.print(" | y: ");
    Serial.print(y);
    Serial.print(" | z: ");
    Serial.print(z);
    Serial.print(" | temp: ");
    Serial.print("NA");
    Serial.print(" | CO2: ");
    Serial.print(co2Value);

    Serial.println();
    */

    myfile.close();
    reading++;
  }
    Serial.println();
    delay(40);
}


/*
 * Ball Valve PWM functions
 */
void open_ball_valve() {
  Serial.println("Open valve!");
  ball_valve.write(150);
}

void close_ball_valve() {
  Serial.println("Close valve!");
  ball_valve.write(50); 
}
void ball_valve_setup() {
  ball_valve.attach(BALL_VALVE_PWM_PORT);
}

int readCO2()
{
  int co2_value = 0;

  Wire.beginTransmission(co2Addr);
  Wire.write(0x22);
  Wire.write((uint8_t)0x00);
  Wire.write(0x08);
  Wire.write(0x2A);
  Wire.endTransmission();

  delay(10);
  
  Wire.requestFrom(co2Addr, 4);
  byte i = 0;
  byte buffer[4] = {0, 0, 0, 0};

  while(Wire.available())
  {
    buffer[i] = Wire.read();
    i++; 
  }

  co2_value = 0;
  co2_value |= buffer[1] & 0xFF;
  co2_value = co2_value << 8;
  co2_value |= buffer[2] & 0xFF;
  byte sum = 0;
  sum = buffer[0] + buffer[1] + buffer[2];
  if(sum == buffer[3])
  {
    return co2_value; }
  else
  {
    return 0; 
  }
}
