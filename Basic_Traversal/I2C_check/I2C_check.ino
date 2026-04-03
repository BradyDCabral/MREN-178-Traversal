// ai generated code to test if the devices on I2C communication work

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SDA_PIN 8
#define SLC_PIN 9

const int Shut_X_Front = 10;
const int Shut_X_Right = 11;
const int Shut_X_Left = 12;

Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SLC_PIN);

  pinMode(Shut_X_Front, OUTPUT);
  pinMode(Shut_X_Right, OUTPUT);
  pinMode(Shut_X_Left, OUTPUT);

  // disable all
  digitalWrite(Shut_X_Front, LOW);
  digitalWrite(Shut_X_Right, LOW);
  digitalWrite(Shut_X_Left, LOW);
  delay(10);

  // test front only
  Serial.println("Testing FRONT sensor...");
  digitalWrite(Shut_X_Front, HIGH);
  delay(10);
  if (!sensor.begin()) {
    Serial.println("FRONT FAILED");
  } else {
    for (int i = 0; i < 5; i++) {
      Serial.printf("Front: %d mm\n", sensor.readRange());
      delay(200);
    }
  }

  // disable front, test right
  digitalWrite(Shut_X_Front, LOW);
  delay(10);
  Serial.println("Testing RIGHT sensor...");
  digitalWrite(Shut_X_Right, HIGH);
  delay(10);
  if (!sensor.begin()) {
    Serial.println("RIGHT FAILED");
  } else {
    for (int i = 0; i < 5; i++) {
      Serial.printf("Right: %d mm\n", sensor.readRange());
      delay(200);
    }
  }

  // disable right, test left
  digitalWrite(Shut_X_Right, LOW);
  delay(10);
  Serial.println("Testing LEFT sensor...");
  digitalWrite(Shut_X_Left, HIGH);
  delay(10);
  if (!sensor.begin()) {
    Serial.println("LEFT FAILED");
  } else {
    for (int i = 0; i < 5; i++) {
      Serial.printf("Left: %d mm\n", sensor.readRange());
      delay(200);
    }
  }

  Serial.println("Done.");
}

void loop() {}