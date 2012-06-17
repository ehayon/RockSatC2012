#include <ADXL345.h>
#include <Wire.h>
#include "Servo.h"
#include <SD.h>

// ADXL345 stuff...
#define ScaleFor2G 0.0039
#define ScaleFor4G 0.0078
#define ScaleFor8G 0.0156
#define ScaleFor16G 0.0312

#define BALL_VALVE_PWM_PORT 2
#define SD_CARD_PORT 53

// print out log messages through 9600 baud serial
boolean DEBUG = false;

Servo ball_valve;
// timings for the redundant safety valve
unsigned long open_valve_time = 15000;    // 15 seconds
unsigned long close_valve_time = 395000;  // 395 seconds

// current flight time
unsigned long time;

// acceleration
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
int x;
int y;
int z;

File myfile;

// keep track of sd card state, may need to attempt a reconnect mid-flight (we really don't want to though...)
boolean sd_connected;

// reading count
unsigned int reading = 0;

int co2Value;
int co2Addr = 0x68; // This is the default address of the CO2 sensor, 7bits shifted

void setup() {  
  if(DEBUG) Serial.begin(9600);
  if(DEBUG) Serial.println("Initializing...");
  pinMode(53, OUTPUT);
  if(!(sd_connected = SD.begin(53))) {
    // sd card not found...
    // we can do something here, but its in setup, pretty useless
    //   work inside loop() instead
  } 
  if(DEBUG) Serial.println("Initializing...");
  // ball_valve_setup();
  adxl.powerOn();

  adxl.setActivityThreshold(75);   //62.5mg per increment
  adxl.setInactivityThreshold(75); //62.5mg per increment
  adxl.setTimeInactivity(10);      // how many seconds of no activity is inactive?
  adxl.setActivityX(1);
  adxl.setActivityY(1);
  adxl.setActivityZ(1);
  
  PORTD = (1 << PORTD1) | (1 << PORTD0); //enable pullups - attempting to make IR temp work :(
}


/*
 * 50ms - runs at 20Hz
 * CO2 sensor is sampled at it's manufacturers recommended frequency of 2Hz
 *  mod-10 operation ignores all but every 10 samples
 */
void loop() {
  time = millis(); // get millis since boot - no sense of current datetime
  
  if(DEBUG) Serial.print("Time: ");
  if(DEBUG) Serial.print(time);
  
  if(!sd_connected) {
    Serial.println("SDCARD NOT CONNECTED");
    sd_connected = SD.begin(53);
  } else {
    
    // open and close the ball valve at the desired times
    
    /*
    if (time > open_valve_time && time <= close_valve_time) {
      open_ball_valve();
    } else {
      close_ball_valve();
    }
    */
    
    // The SD Card is connected - we can write data to it now...
    myfile = SD.open("rocksat.txt", FILE_WRITE);

    
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
    myfile.print(" | time: ");
    myfile.print(time);
    myfile.print(" | CO2: ");
    myfile.print(co2Value);
    myfile.println();
    if(DEBUG) {
      Serial.print("Time: ");
      Serial.print(reading);
      Serial.print(" | x: ");
      Serial.print(x);
      Serial.print(" | y: ");
      Serial.print(y);
      Serial.print(" | z: ");
      Serial.print(z);
      Serial.print(" | temp: ");
      Serial.print(time);
      Serial.print(" | CO2: ");
      Serial.print(co2Value);
      Serial.println();
    }
    
    myfile.close();
    reading++; // keep track of our current reading... we're going to need this to create a timeline of launch
  }
  
    Serial.println();
    delay(40);
}


/*
 * Ball Valve PWM functions
 */
 
/*
 * Open the ball valve
 */
void open_ball_valve() {
  if(DEBUG) Serial.println("Open valve!");
  ball_valve.write(150);
}
/*
 * Close the ball valve
 */
void close_ball_valve() {
  if(DEBUG) Serial.println("Close valve!");
  ball_valve.write(50); 
}
/*
 * Initialize the ball-valve servo - attach servo to BALL_VALVE_PWM_PORT
 */
void ball_valve_setup() {
  ball_valve.attach(BALL_VALVE_PWM_PORT);
}

/*
 * Communicate with the Senseair K-30 over 2-wire serial interface
 * returns: integer representation of the CO2 concentation in PPM
 * 10ms
 */
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

