#include <Wire.h>
#include <MPU6050.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
MPU6050 mpu;

// LDR pin
const int LDR_PIN = 34;

// MPU raw
int16_t ax, ay, az, gx, gy, gz;

// Control values
int pitch = 0, roll = 0, throttle = 0;

// Calibration offsets
int pitch_offset = 0;
int roll_offset  = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  SerialBT.begin("ESP32_GESTURE");

  pinMode(LDR_PIN, INPUT);

  mpu.initialize();
  Serial.println("ESP32 started");

  // -------- CALIBRATION --------
  Serial.println("Calibrating... Keep hand steady");

  long p_sum = 0, r_sum = 0;

  for (int i = 0; i < 200; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    int p = map(ay, -12000, 12000, -30, 30);
    int r = map(ax, -12000, 12000, -30, 30);

    p_sum += p;
    r_sum += r;

    delay(5);
  }

  pitch_offset = p_sum / 200;
  roll_offset  = r_sum / 200;

  Serial.print("Pitch offset = ");
  Serial.println(pitch_offset);
  Serial.print("Roll offset  = ");
  Serial.println(roll_offset);

  Serial.println("Calibration done");
}

void loop() {
  // Read MPU
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // -------- PITCH & ROLL --------
  pitch = map(ay, -12000, 12000, -30, 30) - pitch_offset;
  roll  = map(ax, -12000, 12000, -30, 30) - roll_offset;

  pitch = constrain(pitch, -30, 30);
  roll  = constrain(roll,  -30, 30);

  // -------- THROTTLE (LDR) --------
  int ldrValue = analogRead(LDR_PIN);
  ldrValue = constrain(ldrValue, 0, 4095);

  throttle = map(ldrValue, 800, 3000, 0, 100);
  throttle = constrain(throttle, 0, 100);

  // -------- SEND: THR,PIT,ROL --------
  SerialBT.print(throttle);
  SerialBT.print(",");
  SerialBT.print(pitch);
  SerialBT.print(",");
  SerialBT.println(roll);

  // USB debug (optional)
  Serial.print("THR=");
  Serial.print(throttle);
  Serial.print(" PIT=");
  Serial.print(pitch);
  Serial.print(" ROL=");
  Serial.println(roll);

  delay(40); // ~25 Hz
}
