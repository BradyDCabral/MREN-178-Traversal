#include "Odometry.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Cdrv8833.h>

// CONSTANTS

// Motor
#define IN1_PIN 12 // not accurate
#define IN2_PIN 13 // not accurate 
#define IN3_PIN 14 // not accurate
#define IN4_PIN 15 // not accurate


// Stage management
STAGE stage = START;



// Yaw
// stores previous YAW
float omegaZ = 0;
// stores absolute YAW
float zR = 0;
float Target_Z = 0;

// yaw from wheel odometry
float Z_wO = 0;


// Adafruit mpu
Adafruit_MPU6050 mpu;


// Time variables
int Delta_Millis; // might make float
unsigned long Total_Millis;
unsigned long Temp_Millis;


// Motor control
Cdrv8833 Lmotor;
const uint8_t Lchannel = 0; // not set
Cdrv8833 Rmotor;
const uint8_t Rchannel = 1; // not set


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  // Setup MPU
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Setup Motor
  Lmotor.init(IN1_PIN, IN2_PIN, Lchannel);
  Rmotor.init(IN3_PIN, IN4_PIN, Rchannel);


  delay(100);
  Total_Millis = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  // OVERALL updates all necessary info - ODOMETRY, MPU, RANGE
  
  // Get Delta time and total time
  Temp_Millis = millis();
  DTime(Temp_Millis, &Total_Millis);

  // MPU 
  // Gets MPU acceleration, gyro, temperature
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  UpdateYAW(g.gyro.z, &zR, &omegaZ, Delta_Millis);

  switch (stage) {
    case START:
      // evaluates if at a proper spawn ie. walls on both sides
      // can set up hallway movement
      // NEXT: FOLLOW_HALLWAY
      break;
    case FOLLOW_HALLWAY:
      // just use range sensors to determine how close to walls
      // use error L-R (sensor values) and make decision based on this 
      // might use wheel odometry not sure
      // need to also be checking if over magnet 
      // NEXT: ENTER_NODE_CENTRE || FOUND_EXIT
      break;
    case ENTER_NODE_CENTRE:
      /* make way to centre of the node trying to maintain angle and not stray to the side
      * (specific) use desired angle and position relative walls (use prev tick range) 
      * to approximate centre of node (need rough guess of cell size) then slowly move forward 
      * increasing one wheel to turn when deviation from desired angle is significant
      * (simple) straighten bot then move forward a slight amount just a rough amount (depends on time)
      * NEXT: DETERMINE_NEXT_STEP
      */
      break;
    case DETERMINE_NEXT_STEP:
      /* Use matrix or some other method based on if searching or not to determine next angle to move
      * ROTATE_TO_DESTINATION
      */
      break;
    case ROTATE_TO_DESTINATION:
      /* Use desired YAW and approximate global YAW to determine how much wheels should rotate 
      * rotate wheels in opposite directions to try to stay at centre when rotating
      * NEXT: EXIT_NODE_CENTRE
      */
      break;
    case EXIT_NODE_CENTRE:
      /* Similar to enter node centre except this time difference between angle and desired angle will be
      * used as error to determine action 
      * NEXT: FOLLOW_HALLWAY
      */
      break;
    case FOUND_EXIT:
      /* STOP EVERYTHING CONGRATS MAYBE DO CELEBRATORY MESSAGE
      */
      break;
    default:
      break;
  }
}


void DTime(unsigned long TempT, unsigned long *TotalT) {
  Delta_Millis = (float)(TempT - *TotalT)/1000;
  *TotalT = TempT;
}