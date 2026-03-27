#include "Odometry.h"
#include "MazeLogic.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Cdrv8833.h>

// CONSTANTS

// Motor
#define IN1_PIN 12 // not accurate
#define IN2_PIN 13 // not accurate 
#define IN3_PIN 14 // not accurate
#define IN4_PIN 15 // not accurate
// ENCODER
#define GEARING 50 // might not be accurate
#define ENCODERMULT 12 // definetly NOT ACCURATE
// LEFT
#define ENCODER_A_L 12 // NOT ACCURATE
#define ENCODER_B_L 13 // NOT ACCURATE
// RIGHT
#define ENCODER_A_R 12 // NOT ACCURATE
#define ENCODER_B_R 13 // NOT ACCURATE

// Stage management
STAGE stage = START;
#define HALLWAY_ERROR_DEADZONE 0.005 // NOT ACCURATE
bool Adjusting_Angle = false;
float Start_phs_X = 0;
float Start_phs_Y = 0;
uint8_t CW = 0;

// Entering Centre of node
#define DISTANCE_TO_CENTRE 0.1
#define CENTRE_DIST_GIVE 0.005
#define CENTRE_ANGLE_GIVE 1 // degree

// WHEEL Shit
// Yaw from wheel Odometry
float zR_W = 0;
// local x and y positions
float x_Whl = 0;
float y_Whl = 0;

// Yaw
// stores previous YAW
float omegaZ = 0;
// stores absolute YAW
float zR = 0;
float Target_Z = 0;
float M_zR = 0; // yaw used in error calculations

// yaw from wheel odometry
float Z_wO = 0;


// Adafruit mpu
// default address = 0x68
Adafruit_MPU6050 mpu;


// Time variables
int Delta_Millis; // might make float
unsigned long Total_Millis;
unsigned long Temp_Millis;


// Motor control
#define MOTOR_DELAY 20
#define MOTOR_TURN_POWER 50 // NOT ACCURATE
#define MOTOR_STNDRD_POWER 80 // NOT ACCURATE

Cdrv8833 Lmotor;
const uint8_t Lchannel = 0; // not set
Cdrv8833 Rmotor;
const uint8_t Rchannel = 1; // not set

// L motor SPEED info
volatile float RPS_L = 0; // rev / sec
volatile uint32_t lastA_L = 0; // last time since interupt
volatile bool motorDir_L = HIGH; // not sure how to treat this yet

// R motor SPEED info
volatile float RPS_R = 0; // rev / sec
volatile uint32_t lastA_R = 0; // last time since interupt
volatile bool motorDir_R = HIGH; // not sure how to treat this yet


// VL53L0X range sensor variables
Adafruit_VL53L0X Front_Range_S = Adafruit_VL53L0X();
const int Shut_X_Front = 12; // unknown
#define Front_Address 0x27

Adafruit_VL53L0X Right_Range_S = Adafruit_VL53L0X();
const int Shut_X_Right = 13; // unknown
#define Right_Address 0x26

Adafruit_VL53L0X Left_Range_S = Adafruit_VL53L0X();
const int Shut_X_Left = 13; // unknown
#define Left_Address 0x25

#define MAX_WALL_DIST 0.1 // this is a guess

// Range Sensor Data
float Front_Distance = 0;
float Right_Distance = 0;
float Left_Distance = 0;

// SCREEN
#define SCREEN_HEIGHT 128
#define SCREEN_WIDTH 64

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  // Setup MPU
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  // Setup Motor
  // will need to swap one to have consistency
  Lmotor.init(IN1_PIN, IN2_PIN, Lchannel);
  Rmotor.init(IN3_PIN, IN4_PIN, Rchannel);

  // Setup Motor Encoders
  pinMode(ENCODER_A_L, INPUT_PULLUP);
  pinMode(ENCODER_B_L, INPUT_PULLUP);
  pinMode(ENCODER_A_R, INPUT_PULLUP);
  pinMode(ENCODER_B_R, INPUT_PULLUP);

  // Setup interrupt functions 
  attachInterrupt(ENCODER_A_L, Interrupt_A_LMotor, RISING);
  attachInterrupt(ENCODER_A_R, Interrupt_A_RMotor, RISING);

  // Setup Range Sensors 
  pinMode(Shut_X_Front, OUTPUT);
  pinMode(Shut_X_Right, OUTPUT);
  pinMode(Shut_X_Left, OUTPUT);

  // Setup assuming all connected to the same I2C bus
  // Turn off ALL for reset
  digitalWrite(Shut_X_Front, LOW);
  digitalWrite(Shut_X_Right, LOW);
  digitalWrite(Shut_X_Left, LOW);
  delay(10);
  digitalWrite(Shut_X_Front, HIGH);
  digitalWrite(Shut_X_Right, HIGH); // might need to omit
  digitalWrite(Shut_X_Left, HIGH); // might need to omit
  // turn off all but front then assign address
  digitalWrite(Shut_X_Right, LOW);
  digitalWrite(Shut_X_Left, LOW);
  Front_Range_S.begin(Front_Address);
  digitalWrite(Shut_X_Right, HIGH);
  Right_Range_S.begin(Right_Address);
  digitalWrite(Shut_X_Left, HIGH);
  Left_Range_S.begin(Left_Address);

  // Setup Screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // check if address is correct
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 10);


  // not sure why this is here
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
  
  // updates YAW using mpu
  UpdateYAW(g.gyro.z, &zR, &omegaZ, Delta_Millis);

  // Updates position data using wheel encoders 
  // IMPORTANT motorDir_R is unsure what high means but will be assumed that high means the bot will move forward
  // if not the case then must swap something around 
  // also assumed both wheels going same direction if not the case don't use this function so there should be some check if the case
  // probably have it so this only updates in certain cases so this function might not be called in this spot in future
  // TLDR: might be moved to corresponding case
  Wheel_Tracking(RPS_L, RPS_R, &zR_W, &x_Whl, &y_Whl, motorDir_R, Delta_Millis);

  // Updates Range Sensors
  // BLAH BLAH BLAH

  // start with front data
  Front_Distance = ((float)Front_Range_S.readRange())*1000;
  uint8_t Front_Correct = Front_Range_S.readRangeStatus();

  // start with Right data
  Right_Distance = ((float)Right_Range_S.readRange())*1000;
  uint8_t Right_Correct = Right_Range_S.readRangeStatus();

  // start with Left data
  Left_Distance = ((float)Left_Range_S.readRange())*1000;
  uint8_t Left_Correct = Left_Range_S.readRangeStatus();

  // Motor wheels will be updated using an interrupt function shown in 
  // https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/examples/encoderMotorRPM/encoderMotorRPM.ino
  // then speeds will be used to determine distance travelled which can give approximates to positions
  
  // temp values
  float error;
  float error_Angle;
  float error_X;
  float error_Y;

  M_zR = zR; // change if odometry is more accurate 
  switch (stage) {
    case START:
      // evaluates if at a proper spawn ie. walls on both sides
      // can set up hallway movement
      // NEXT: FOLLOW_HALLWAY
      stage = FOLLOW_HALLWAY;
      display.clearDisplay();
      display.println("FOLLOW HALLWAY");
      display.display();
      break;
    case FOLLOW_HALLWAY:
      // just use range sensors to determine how close to walls
      // use error L-R (sensor values) and make decision based on this 
      // might use wheel odometry not sure
      // need to also be checking if over magnet 
      // NEXT: ENTER_NODE_CENTRE || FOUND_EXIT || DETERMINE_NEXT_STEP (if dead_end detected)
      error = Left_Distance - Right_Distance;
      // Dead end detected need to make decision
      if (Front_Distance < MAX_WALL_DIST && Left_Distance < MAX_WALL_DIST && Right_Distance < MAX_WALL_DIST) {
        BrakeMotors();
        delay(MOTOR_DELAY);
        stage = DETERMINE_NEXT_STEP;
        display.clearDisplay();
        display.println("MAKING DECISION");
        display.display();
      } /* Entering Node */ else if (Left_Distance >= MAX_WALL_DIST || Right_Distance >= MAX_WALL_DIST) {
        // not sure what prep must be done but switch to next stage
        BrakeMotors();
        stage = ENTER_NODE_CENTRE;
        display.clearDisplay();
        display.println("ENTERING NODE");
        display.display();
        // setup variables
        Adjusting_Angle = true;
        Start_phs_X = x_Whl;
        Start_phs_Y = y_Whl;
      } 
      /* To the left  */else if (error > HALLWAY_ERROR_DEADZONE) {
        Lmotor.stop(); // nmight brake
        Rmotor.move(MOTOR_TURN_POWER);
      } /* To the right */else if (error < - HALLWAY_ERROR_DEADZONE) {
        Rmotor.stop(); // might brake
        Lmotor.move(MOTOR_TURN_POWER);
      } /* Centre */ else {
        Lmotor.move(MOTOR_STNDRD_POWER);
        Rmotor.move(MOTOR_STNDRD_POWER);
      } // NEED TO ADD CONDITION IF AT EXIT or put this outside the switch statement
      break;
    case ENTER_NODE_CENTRE:
      /* make way to centre of the node trying to maintain angle and not stray to the side
      * (specific) use desired angle and position relative walls (use prev tick range) 
      * to approximate centre of node (need rough guess of cell size) then slowly move forward 
      * increasing one wheel to turn when deviation from desired angle is significant
      * (simple) straighten bot then move forward a slight amount just a rough amount (depends on time)
      * NEXT: DETERMINE_NEXT_STEP
      */
      // if true adjust angle to be close to target angle 
      if (Adjusting_Angle) { // degrees
        // might update Start_phs_X & Y after Angle reached
        error_Angle = fmod((Target_Z - M_zR + 360), 360);
        if ((abs(error_Angle) >= CENTRE_ANGLE_GIVE)){
          if (error_Angle >= 0 && error_Angle <= 180 && CW != 1) { // CW turn
            // Turn LeftWhl Back, Turn RghtWhl FRWRD
            CW = 1;
            BrakeMotors();
            delay(MOTOR_DELAY);
            Lmotor.move(-MOTOR_TURN_POWER);
            Rmotor.move(MOTOR_TURN_POWER);
          } else if (error_Angle > 180 && error_Angle < 360 && CW != 2){
            // Turn LeftWhl FRWD, Turn RghtWhl BCK
            CW = 2;
            BrakeMotors();
            delay(MOTOR_DELAY);
            Lmotor.move(MOTOR_TURN_POWER);
            Rmotor.move(-MOTOR_TURN_POWER);
          }
        } else {
          BrakeMotors();
          delay(MOTOR_DELAY);
          Adjusting_Angle = false;
          // might change these
          x_Whl = 0;
          y_Whl = 0;
          Start_phs_X = 0;
          Start_phs_Y = 0;
          CW = 0;
          Lmotor.move(MOTOR_STNDRD_POWER);
          Rmotor.move(MOTOR_STNDRD_POWER);
        }

      } /* Move forward a little */ else {
        error_X = x_Whl - Start_phs_X;
        error_Y = y_Whl - Start_phs_Y;
        error = DISTANCE_TO_CENTRE - hypotf(error_X, error_Y);
        if (error <= CENTRE_DIST_GIVE) {
          BrakeMotors();
          delay(MOTOR_DELAY);
          stage = DETERMINE_NEXT_STEP;
          display.clearDisplay();
          display.println("MAKING DECISION");
          display.display();
        }
      }
      
      break;
    case DETERMINE_NEXT_STEP:
      /* Use graph or some other method based on if searching or not to determine next angle to move
      * ROTATE_TO_DESTINATION
      */

      break;
    case ROTATE_TO_DESTINATION:
      /* Use desired YAW and approximate global YAW to determine how much wheels should rotate 
      * rotate wheels in opposite directions to try to stay at centre when rotating
      * NEXT: EXIT_NODE_CENTRE
      */
      error_Angle = fmod((Target_Z - M_zR + 360), 360);
      if ((abs(error_Angle) >= CENTRE_ANGLE_GIVE)){
        if (error_Angle >= 0 && error_Angle <= 180 && CW != 1) { // CW turn
          // Turn LeftWhl Back, Turn RghtWhl FRWRD
          CW = 1;
          BrakeMotors();
          delay(MOTOR_DELAY);
          Lmotor.move(-MOTOR_TURN_POWER);
          Rmotor.move(MOTOR_TURN_POWER);
        } else if (error_Angle > 180 && error_Angle < 360 && CW != 2){
          // Turn LeftWhl FRWD, Turn RghtWhl BCK
          CW = 2;
          BrakeMotors();
          delay(MOTOR_DELAY);
          Lmotor.move(MOTOR_TURN_POWER);
          Rmotor.move(-MOTOR_TURN_POWER);
        }
      } else {
        stage = EXIT_NODE_CENTRE;
        Lmotor.move(MOTOR_STNDRD_POWER);
        Rmotor.move(MOTOR_STNDRD_POWER);

        display.clearDisplay();
        display.println("EXIT NODE");
        display.display();
      }
      break;
    case EXIT_NODE_CENTRE:
      /* Similar to enter node centre except this time difference between angle and desired angle will be
      * used as error to determine action 
      * NEXT: FOLLOW_HALLWAY
      */
      if (Left_Distance < MAX_WALL_DIST && Right_Distance < MAX_WALL_DIST) {
        BrakeMotors();
        delay(MOTOR_DELAY);
        stage = FOLLOW_HALLWAY;
        display.clearDisplay();
        display.println("FOLLOW HALLWAY");
        display.display();
      }
      break;
    case FOUND_EXIT:
      /* STOP EVERYTHING CONGRATS MAYBE DO CELEBRATORY MESSAGE
      */
      display.clearDisplay();
      display.println("FUCK YEAH");
      display.display();
      break;
    case TEST:
      // in the name used for testing components whilst avoiding most of the regular flow of code
      break;
    default:
      break;
  }
}

// adapted from https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library/blob/master/examples/encoderMotorRPM/encoderMotorRPM.ino
void Interrupt_A_LMotor() {
  motorDir_L = digitalRead(ENCODER_B_L);


  uint32_t currA_L = micros();
  if (lastA_L < currA_L) {
    float rev = currA_L - lastA_L;
    rev = 1.0 / rev;
    rev *= 1000000;
    rev /= GEARING;
    rev /= ENCODERMULT;
    RPS_L = rev;
  }
  lastA_L = currA_L;
}

void Interrupt_A_RMotor() {
  motorDir_R = digitalRead(ENCODER_B_R);


  uint32_t currA_R = micros();
  if (lastA_R < currA_R) {
    float rev = currA_R - lastA_R;
    rev = 1.0 / rev;
    rev *= 1000000;
    rev /= GEARING;
    rev /= ENCODERMULT;
    RPS_R = rev;
  }
  lastA_R = currA_R;
}

void DTime(unsigned long TempT, unsigned long *TotalT) {
  Delta_Millis = (float)(TempT - *TotalT)/1000;
  *TotalT = TempT;
}

void BrakeMotors() {
  Lmotor.brake();
  Rmotor.brake();
}