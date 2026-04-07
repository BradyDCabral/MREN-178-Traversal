// please note distance measurements are not converted and are in milimeters
// Claude used to help get basic PID math and debug code, large amounts of code were taken from traversal.ino


#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#ifndef VL53L0X_ERROR_NONE
#define VL53L0X_ERROR_NONE 0
#endif

#define SDA_PIN 8 
#define SLC_PIN 9


int left_distance;
int right_distance;
float error;
const float Kp = 0.4;
float correction;
int left_motor, right_motor; // pwm values for motors
const int base_speed = 511;

Adafruit_VL53L0X Front_Range_S = Adafruit_VL53L0X();
const int Shut_X_Front = 10; // unknown
#define Front_Address 0x27

Adafruit_VL53L0X Right_Range_S = Adafruit_VL53L0X();
const int Shut_X_Right = 11; // unknown
#define Right_Address 0x26

Adafruit_VL53L0X Left_Range_S = Adafruit_VL53L0X();
// Make sure physical wiring matches this new pin assignment.
const int Shut_X_Left = 12;
#define Left_Address 0x25

void setup() {
  Serial.begin(115200);
  Wire.setClock(50000);
  Wire.begin(SDA_PIN,SLC_PIN);
  // put your setup code here, to run once:
  pinMode(1,OUTPUT); // right forward
  pinMode(2,OUTPUT); // right reverse
  pinMode(41,OUTPUT); // left forward
  pinMode(42,OUTPUT); // left reverse

  pinMode(Shut_X_Front, OUTPUT);
  pinMode(Shut_X_Right, OUTPUT);
  pinMode(Shut_X_Left, OUTPUT);

  digitalWrite(Shut_X_Front, LOW);
  digitalWrite(Shut_X_Right, LOW);
  digitalWrite(Shut_X_Left, LOW);
  delay(10);
  digitalWrite(Shut_X_Front, HIGH);
  digitalWrite(Shut_X_Right, HIGH);
  digitalWrite(Shut_X_Left, HIGH);
  delay(10);
  digitalWrite(Shut_X_Right, LOW);
  digitalWrite(Shut_X_Left, LOW);
  if (!Front_Range_S.begin(Front_Address)) Serial.println("Front NOT WORKS");
  else Serial.println("FRONT WORKS");
  //Front_Range_S.startRangeSingle();
  delay(500);

  digitalWrite(Shut_X_Right, HIGH);
  if (!Right_Range_S.begin(Right_Address)) Serial.println("Right NOT WORKS");
  else Serial.println("Right WORKS");
  //Right_Range_S.startRangeSingle();
  delay(500);

  digitalWrite(Shut_X_Left, HIGH);
  if (!Left_Range_S.begin(Left_Address)) Serial.println("Left NOT WORKS");
  else Serial.println("LEFT Works");
  //Left_Range_S.startRangeSingle();
  delay(500);
  Serial.println("Scanning I2C...");
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }
}
// functions

void go_forwards(int miliseconds){
  digitalWrite(1,HIGH);
  digitalWrite(41,HIGH);
  delay(miliseconds);
  digitalWrite(1,LOW);
  digitalWrite(41,LOW);
  delay(100);
}

void go_forwards_proportional(){
  correction = Kp * error;

  left_motor = base_speed - correction;
  right_motor = base_speed + correction;
  clamp_values();
  analogWrite(1,(int)right_motor);
  analogWrite(41,(int)left_motor);
  analogWrite(2,0);
  analogWrite(42,0);
}

void clamp_values(){
  if (left_motor > 1023){
    left_motor = 1023;
  }
  if (right_motor > 1023){
    right_motor = 1023;
  }
  if (left_motor < 0){
    left_motor = 0;
  }
  if (right_motor < 0){
    right_motor = 0;
  }
}

void read_sensors() {
    VL53L0X_RangingMeasurementData_t measure;

    Left_Range_S.rangingTest(&measure, false);
    if (measure.RangeStatus != 4)
        left_distance = measure.RangeMilliMeter;

    Right_Range_S.rangingTest(&measure, false);
    if (measure.RangeStatus != 4)
        right_distance = measure.RangeMilliMeter;
}

void loop() {
  // put your main code here, to run repeatedly:
  read_sensors();
  Serial.printf("%d %d \n", left_motor, right_motor);
  if (left_distance < 500 && right_distance < 500){
    error = right_distance-left_distance;
  }

  
  go_forwards_proportional();
  delay(5);
  

}
