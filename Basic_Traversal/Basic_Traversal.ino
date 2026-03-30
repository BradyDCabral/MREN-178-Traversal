#include "Odometry.h"
#include "MazeLogic.h"
#include "AStar.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>
#ifndef VL53L0X_ERROR_NONE
#define VL53L0X_ERROR_NONE 0 /* range valid (Adafruit VL53L0X) */
#endif
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Cdrv8833.h>
#include "M_CONSTANTS.h"
#include <cmath>
#include "esp_task_wdt.h"



#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// MPU-integrated zR is radians (gyro z is rad/s); maze / displays use degrees via M_zR
static float yaw_rad_to_deg_0_360(float rad) {
  float d = rad * (180.0f / static_cast<float>(M_PI));
  d = fmodf(d, 360.0f);
  if (d < 0.0f)
    d += 360.0f;
  return d;
}

// Shortest signed turn from current_deg toward target_deg, degrees in [-180, 180]
static float angle_error_deg_shortest(float target_deg, float current_deg) {
  float e = target_deg - current_deg;
  e = fmodf(e, 360.0f);
  if (e > 180.0f)
    e -= 360.0f;
  if (e < -180.0f)
    e += 360.0f;
  return e;
}

// CONSTANTS

// I2C
#define SDA_PIN 8
#define SCL_PIN 9

// Motor
#define IN1_PIN 1 // not accurate
#define IN2_PIN 2 // not accurate 
#define IN3_PIN 42
#define IN4_PIN 41

// ENCODER
#define GEARING 50 // might not be accurate
#define ENCODERMULT 12 // definetly NOT ACCURATE
// LEFT
#define ENCODER_A_L 6 // NOT ACCURATE
#define ENCODER_B_L 5 // NOT ACCURATE
// RIGHT
#define ENCODER_A_R 15 // NOT ACCURATE
#define ENCODER_B_R 7 // NOT ACCURATE

// Stage management
STAGE stage = START;
#define HALLWAY_ERROR_DEADZONE 0.010 // NOT ACCURATE
#define FRONT_BLOCK_DIST 0.15 // m, immediate front-stop threshold
bool Adjusting_Angle = false;
float Start_phs_X = 0;
float Start_phs_Y = 0;
uint8_t CW = 0;

// Entering Centre of node
#define DISTANCE_TO_CENTRE 0.1
#define CENTRE_DIST_GIVE 0.005
#define CENTRE_ANGLE_GIVE 4 // degree

// Wheel odometry heading (radians), used by Wheel_Tracking — not the same units as MPU zR (degrees)
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
float Delta_Millis;
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
const int Shut_X_Front = 10; // unknown
#define Front_Address 0x27

Adafruit_VL53L0X Right_Range_S = Adafruit_VL53L0X();
const int Shut_X_Right = 11; // unknown
#define Right_Address 0x26

Adafruit_VL53L0X Left_Range_S = Adafruit_VL53L0X();
// FIX #4: Shut_X_Left was 12, same as REED_SWITCH_PIN — changed to 13.
// Make sure physical wiring matches this new pin assignment.
const int Shut_X_Left = 12;
#define Left_Address 0x25

// #define MAX_WALL_DIST 0.1 // this is a guess

// Range Sensor Data
float Front_Distance = 0;
float prev_Front_Distance = 0;
float Right_Distance = 0;
float prev_Right_Distance = 0;
float Left_Distance = 0;
float prev_Left_Distance = 0;

// SCREEN
#define SCREEN_HEIGHT 64
#define SCREEN_WIDTH 128

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, -1);

// Reed Switch
#define REED_SWITCH_PIN 13
#define REED_MAX_TIME 300
// no magnet closed
// magnet open
unsigned long REED_TRIGGERED_TIME = 0;
unsigned long REED_DELTA_TIME = 0;


// Maze graph + planner
pGraph Maze = nullptr;
pVertex Current_Node = nullptr;
pVertex Goal_Node = nullptr;
pVertex Planned_Next_Node = nullptr;
int vertex_count = 0;
pVertex AStar_path[MAX_NODES];
int AStar_path_len = 0;

bool oled_init = false;

Adafruit_SSD1306* oled = nullptr;
unsigned long last_debug_ms = 0;


void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(IN1_PIN, OUTPUT); digitalWrite(IN1_PIN, LOW);
  pinMode(IN2_PIN, OUTPUT); digitalWrite(IN2_PIN, LOW);
  pinMode(IN3_PIN, OUTPUT); digitalWrite(IN3_PIN, LOW);
  pinMode(IN4_PIN, OUTPUT); digitalWrite(IN4_PIN, LOW);

  // Setup I2C
  Wire.end();  // release bus first
  delay(50);
  pinMode(SDA_PIN, OUTPUT); digitalWrite(SDA_PIN, HIGH);
  pinMode(SCL_PIN, OUTPUT);
  for(int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
  }
  // Send STOP condition
  digitalWrite(SDA_PIN, LOW); delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // FIX #2 & #3: Motor init and OLED init are now each done ONCE, in the
  // correct order. OLED must come first so UpdateDisplay() works during motor
  // test. The early duplicate motor init block (which ran before OLED was
  // ready) has been removed entirely.

  // --- OLED init (must come before any UpdateDisplay call) ---
  oled = new Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
  if (!oled) {
    Serial.println("OLED alloc failed -- out of heap");
  } else {
    // FIX #3: begin() called only once here; the second redundant begin()
    // call that followed in the original code has been removed.
    oled_init = oled->begin(SSD1306_SWITCHCAPVCC, 0x3C);
  }

  delay(500);
  oled->display();
  delay(2000);
  oled->clearDisplay();
  oled->setTextSize(1);
  oled->setTextColor(WHITE);
  oled->setCursor(0, 0);
  oled->println("WORKS");
  oled->display();

  // --- Motor init (now only once, after OLED is ready) ---
  Lmotor.init(IN1_PIN, IN2_PIN, Lchannel, false);
  if (Lmotor.move(50)) UpdateDisplay("LWorks");
  delay(500);

  Rmotor.init(IN3_PIN, IN4_PIN, Rchannel, true);
  if (Rmotor.move(50)) UpdateDisplay("RWorks");
  delay(500);
  BrakeMotors();

  // Setup MPU
  if (!mpu.begin()) {
    UpdateDisplay("MPU not work");
  } else {
    oled->clearDisplay();
    oled->println("MPUConnected");
    oled->display();
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

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
  digitalWrite(Shut_X_Right, HIGH);
  digitalWrite(Shut_X_Left, HIGH);
  // turn off all but front then assign address
  digitalWrite(Shut_X_Right, LOW);
  digitalWrite(Shut_X_Left, LOW);
  if (!Front_Range_S.begin(Front_Address)) UpdateDisplay("Front NOT WORKS");
  else UpdateDisplay("FRONT WORKS");
  delay(500);

  digitalWrite(Shut_X_Right, HIGH);
  if (!Right_Range_S.begin(Right_Address)) UpdateDisplay("Right NOT WORKS");
  else UpdateDisplay("Right WORKS");
  delay(500);

  digitalWrite(Shut_X_Left, HIGH);
  if (!Left_Range_S.begin(Left_Address)) UpdateDisplay("Left NOT WORKS");
  else UpdateDisplay("LEFT Works");
  delay(500);

  // Setup Motor Encoders
  pinMode(ENCODER_A_L, INPUT_PULLUP);
  pinMode(ENCODER_B_L, INPUT_PULLUP);
  pinMode(ENCODER_A_R, INPUT_PULLUP);
  pinMode(ENCODER_B_R, INPUT_PULLUP);

  UpdateDisplay("Motor encoder pin Setup");
  delay(500);

  // Setup interrupt functions 
  attachInterrupt(ENCODER_A_L, Interrupt_A_LMotor, RISING);
  attachInterrupt(ENCODER_A_R, Interrupt_A_RMotor, RISING);

  UpdateDisplay("Motor Encoders Setup");
  delay(500);

  // Setup Reed switch
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);

  UpdateDisplay("Reed switch setup");
  delay(500);

  // Initial range (m); only use good samples
  {
    uint16_t mm = Front_Range_S.readRange();
    if (Front_Range_S.readRangeStatus() == VL53L0X_ERROR_NONE)
      Front_Distance = (float)mm / 1000.0f;
  }
  {
    uint16_t mm = Right_Range_S.readRange();
    if (Right_Range_S.readRangeStatus() == VL53L0X_ERROR_NONE)
      Right_Distance = (float)mm / 1000.0f;
  }
  {
    uint16_t mm = Left_Range_S.readRange();
    if (Left_Range_S.readRangeStatus() == VL53L0X_ERROR_NONE)
      Left_Distance = (float)mm / 1000.0f;
  }

  Maze = createGraph();
  if (Maze) {
    vertex_count = 0;
    if (CreateEmptyVertex(&Maze->Root, &vertex_count) == 0 && Maze->Root) {
      Current_Node = Maze->Root;
      Maze->Root->row = 0;
      Maze->Root->col = 0;
    }
  }

  delay(100);
  Total_Millis = millis();
}

void loop() {
  // Get Delta time and total time
  Temp_Millis = millis();
  DTime(Temp_Millis, &Total_Millis);

  // MPU 
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // updates YAW using mpu
  UpdateYAW(g.gyro.z, &zR, &omegaZ, Delta_Millis);

  // Wheel odometry
  Wheel_Tracking(RPS_L, RPS_R, &zR_W, &x_Whl, &y_Whl, motorDir_L, motorDir_R, Delta_Millis);

  // Range: keep previous value if sample failed (VL53L0X range status 0 = valid)
  prev_Front_Distance = Front_Distance;
  {
    uint16_t mm = Front_Range_S.readRange();
    uint8_t st = Front_Range_S.readRangeStatus();
    if (st == VL53L0X_ERROR_NONE)
      Front_Distance = (float)mm / 1000.0f;
  }

  prev_Right_Distance = Right_Distance;
  {
    uint16_t mm = Right_Range_S.readRange();
    uint8_t st = Right_Range_S.readRangeStatus();
    if (st == VL53L0X_ERROR_NONE)
      Right_Distance = (float)mm / 1000.0f;
  }

  prev_Left_Distance = Left_Distance;
  {
    uint16_t mm = Left_Range_S.readRange();
    uint8_t st = Left_Range_S.readRangeStatus();
    if (st == VL53L0X_ERROR_NONE)
      Left_Distance = (float)mm / 1000.0f;
  }

  if (millis() - last_debug_ms > 200) {
    last_debug_ms = millis();
    Serial.print("F:");
    Serial.print(Front_Distance, 3);
    Serial.print(" L:");
    Serial.print(Left_Distance, 3);
    Serial.print(" R:");
    Serial.print(Right_Distance, 3);
    Serial.print(" stage:");
    Serial.println((int)stage);
  }

  // REED Switch detector
  if (digitalRead(REED_SWITCH_PIN) == LOW) {
    if (REED_TRIGGERED_TIME == 0)
      REED_TRIGGERED_TIME = millis();
    else {
      REED_DELTA_TIME = millis();
      if (REED_DELTA_TIME - REED_TRIGGERED_TIME >= REED_MAX_TIME) {
        stage = FOUND_EXIT;
        Rmotor.move(0);
        Lmotor.move(0);
      }
    }
  } else if (REED_TRIGGERED_TIME > 0) {
    REED_TRIGGERED_TIME = 0;
  }

  float error;
  float error_Angle;
  float error_X;
  float error_Y;

  M_zR = yaw_rad_to_deg_0_360(zR);
  switch (stage) {
    case START:
      stage = FOLLOW_HALLWAY;
      oled->clearDisplay();
      oled->println("FOLLOW HALLWAY");
      oled->display();
      break;

    case FOLLOW_HALLWAY:
      error = Left_Distance - Right_Distance;
      // Front blocked: do not continue driving forward.
      if (Front_Distance < FRONT_BLOCK_DIST) {
        BrakeMotors();
        delay(MOTOR_DELAY);
        stage = DETERMINE_NEXT_STEP;
        oled->clearDisplay();
        oled->println("FRONT BLOCK");
        oled->display();
      }
      // Dead end detected — need to make decision
      else if (Front_Distance < MAX_WALL_DIST && Left_Distance < MAX_WALL_DIST && Right_Distance < MAX_WALL_DIST) {
        BrakeMotors();
        delay(MOTOR_DELAY);
        stage = DETERMINE_NEXT_STEP;
        oled->clearDisplay();
        oled->println("MAKING DECISION");
        oled->display();
      }
      /* Entering Node */
      else if (Left_Distance >= MAX_WALL_DIST || Right_Distance >= MAX_WALL_DIST) {
        BrakeMotors();
        Target_Z = ReturnProperAngleFromIndex(ReturnProperIndex(M_zR));
        stage = ENTER_NODE_CENTRE;
        oled->clearDisplay();
        oled->println("ENTERING NODE");
        oled->display();
        Adjusting_Angle = true;
        Start_phs_X = x_Whl;
        Start_phs_Y = y_Whl;
      }
      /* To the left */
      else if (error > HALLWAY_ERROR_DEADZONE) {
        Lmotor.move(MOTOR_STNDRD_POWER - 10);
        Rmotor.move(MOTOR_STNDRD_POWER);
      }
      /* To the right */
      else if (error < -HALLWAY_ERROR_DEADZONE) {
        Rmotor.move(MOTOR_STNDRD_POWER - 10);
        Lmotor.move(MOTOR_STNDRD_POWER);
      }
      /* Centre */
      else {
        Lmotor.move(MOTOR_STNDRD_POWER);
        Rmotor.move(MOTOR_STNDRD_POWER);
      }
      break;

    case ENTER_NODE_CENTRE:
      if (Adjusting_Angle) {
        error_Angle = angle_error_deg_shortest(Target_Z, M_zR);
        if (fabsf(error_Angle) >= CENTRE_ANGLE_GIVE) {
          if (error_Angle > 0 && CW != 1) {
            CW = 1;
            Lmotor.move(-MOTOR_TURN_POWER);
            Rmotor.move(MOTOR_TURN_POWER);
          } else if (error_Angle < 0 && CW != 2) {
            CW = 2;
            Lmotor.move(MOTOR_TURN_POWER);
            Rmotor.move(-MOTOR_TURN_POWER);
          }
        } else {
          Adjusting_Angle = false;
          x_Whl = 0;
          y_Whl = 0;
          Start_phs_X = 0;
          Start_phs_Y = 0;
          CW = 0;
          Lmotor.move(MOTOR_STNDRD_POWER);
          Rmotor.move(MOTOR_STNDRD_POWER);
        }
      }
      /* Move forward a little */
      else {
        error_X = x_Whl - Start_phs_X;
        error_Y = y_Whl - Start_phs_Y;
        error = DISTANCE_TO_CENTRE - hypotf(error_X, error_Y);
        if (error <= CENTRE_DIST_GIVE) {
          BrakeMotors();
          delay(MOTOR_DELAY);
          stage = DETERMINE_NEXT_STEP;
          oled->clearDisplay();
          oled->println("MAKING DECISION");
          oled->display();
        }
      }
      break;

    case DETERMINE_NEXT_STEP:
      if (Current_Node) {
        CreateVertex(Current_Node, M_zR, Left_Distance, Front_Distance, Right_Distance, &vertex_count);
        Planned_Next_Node = nullptr;
        AStar_path_len = 0;

        if (Goal_Node &&
            AStar_Search(Current_Node, Goal_Node, AStar_heuristic_zero,
                         AStar_path, MAX_NODES, &AStar_path_len) == 0 &&
            AStar_path_len >= 2) {
          int dir = NeighborIndexOf(Current_Node, AStar_path[1]);
          if (dir >= 0) {
            Target_Z = ReturnProperAngleFromIndex(dir);
            Planned_Next_Node = AStar_path[1];
            stage = ROTATE_TO_DESTINATION;
            oled->clearDisplay();
            oled->println("ASTAR");
            oled->display();
            break;
          }
        }

        int fidx = ReturnProperIndex(M_zR);
        if (Front_Distance >= MAX_WALL_DIST) {
          Target_Z = ReturnProperAngleFromIndex(fidx);
        } else if (Left_Distance >= MAX_WALL_DIST) {
          Target_Z = ReturnProperAngleFromIndex((fidx + 3) % 4);
        } else if (Right_Distance >= MAX_WALL_DIST) {
          Target_Z = ReturnProperAngleFromIndex((fidx + 1) % 4);
        } else {
          Target_Z = M_zR;
        }
        stage = ROTATE_TO_DESTINATION;
        oled->clearDisplay();
        oled->println("EXPLORE");
        oled->display();
      }
      break;

    case ROTATE_TO_DESTINATION:
      error_Angle = angle_error_deg_shortest(Target_Z, M_zR);
      if (fabsf(error_Angle) >= CENTRE_ANGLE_GIVE) {
        if (error_Angle > 0 && CW != 1) {
          CW = 1;
          Lmotor.move(-MOTOR_TURN_POWER);
          Rmotor.move(MOTOR_TURN_POWER);
        } else if (error_Angle < 0 && CW != 2) {
          CW = 2;
          Lmotor.move(MOTOR_TURN_POWER);
          Rmotor.move(-MOTOR_TURN_POWER);
        }
      } else {
        CW = 0;
        stage = EXIT_NODE_CENTRE;
        Lmotor.move(MOTOR_STNDRD_POWER);
        Rmotor.move(MOTOR_STNDRD_POWER);
        oled->clearDisplay();
        oled->println("EXIT NODE");
        oled->display();
      }
      break;

    case EXIT_NODE_CENTRE:
      if (Left_Distance < MAX_WALL_DIST && Right_Distance < MAX_WALL_DIST) {
        BrakeMotors();
        delay(MOTOR_DELAY);
        if (Planned_Next_Node) {
          Current_Node = Planned_Next_Node;
          Planned_Next_Node = nullptr;
        }
        stage = FOLLOW_HALLWAY;
        oled->clearDisplay();
        oled->println("FOLLOW HALLWAY");
        oled->display();
      } else {
        // FIX #5: Drive motors forward while waiting for walls to appear.
        // Without this the robot just sat still after rotating.
        Lmotor.move(MOTOR_STNDRD_POWER);
        Rmotor.move(MOTOR_STNDRD_POWER);
      }
      break;

    case FOUND_EXIT:
      BrakeMotors();
      oled->clearDisplay();
      oled->println("YEAH");
      oled->display();
      break;

    case TEST:
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
  Delta_Millis = (float)(TempT - *TotalT) / 1000.0f;
  *TotalT = TempT;
}

void UpdateDisplay(const char c[]) {
  if (!oled) return;
  oled->clearDisplay();
  oled->setCursor(0, 0);
  oled->println(c);
  oled->display();
}

void BrakeMotors() {
  Lmotor.move(0);
  Rmotor.move(0);
}

void i2cRecover() {
  Wire.end();
  delay(20);
  pinMode(SDA_PIN, OUTPUT); digitalWrite(SDA_PIN, HIGH);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
    digitalWrite(SCL_PIN, LOW);  delayMicroseconds(10);
  }
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(10);
  delay(20);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
}
